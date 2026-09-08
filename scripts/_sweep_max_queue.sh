#!/usr/bin/env bash
# Sweep --max-queue on holdout @ 10 FPS; find smallest queue with zero HTTP 503s.
set -euo pipefail

ROOT=/mnt/d/agent-workspaces/Rover-SLAM
HOLD=/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/datasets/2026-04-02_13-42-53/Frames
PORT=18080
OUT_DIR=$HOME/build/holdout_queue_sweep
LOG=$HOME/build/rover-slam-queue-sweep.log

export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
export CMAKE_PREFIX_PATH="${HOME}/.local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PATH="/usr/local/cuda/bin:${HOME}/bin:${PATH}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true

mkdir -p "$OUT_DIR"
cd "$ROOT"

stop_port() {
  local port="$1"
  local pid
  pid=$(ss -ltnp 2>/dev/null | awk -v p=":$port" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1 || true)
  if [[ -n "${pid:-}" ]]; then
    kill "$pid" 2>/dev/null || true
    sleep 0.8
    kill -9 "$pid" 2>/dev/null || true
  fi
}

run_one() {
  local q="$1"
  local minb="$2"
  local tag="q${q}_b${minb}"
  local status_out="$OUT_DIR/${tag}_status.json"
  local replay_out="$OUT_DIR/${tag}_replay.json"

  echo ""
  echo "======== max-queue=$q min-buffer=$minb ========"
  stop_port "$PORT"
  : > "$LOG"
  ./Examples/Monocular/slam_server \
    Vocabulary/voc_binary_tartan_8u_6.yml.gz \
    Examples/Monocular/iphoneSLAM.yaml \
    --prior-pack testdata/prior_pack_fixture \
    --port "$PORT" \
    --max-queue "$q" \
    --min-buffer "$minb" \
    >"$LOG" 2>&1 &
  local spid=$!
  echo "started pid=$spid"

  local ok=0
  for i in $(seq 1 90); do
    if curl -sf -m 2 "http://127.0.0.1:$PORT/api/v1/status" >/tmp/slam_boot_q.json 2>/dev/null; then
      if python3 -c 'import json; d=json.load(open("/tmp/slam_boot_q.json")); raise SystemExit(0 if "timing" in d else 1)'; then
        ok=1
        break
      fi
    fi
    if ! kill -0 "$spid" 2>/dev/null; then
      echo "server died during boot"
      tail -40 "$LOG"
      return 2
    fi
    sleep 1
  done
  if [[ "$ok" -ne 1 ]]; then
    echo "server not ready"
    tail -40 "$LOG"
    return 2
  fi

  # Quiet paced replay; count HTTP status codes (503 = queue full).
  python3 - "$HOLD" "$PORT" "$replay_out" <<'PY'
import json, sys, time
from pathlib import Path
import requests

hold = Path(sys.argv[1])
port = int(sys.argv[2])
out = Path(sys.argv[3])
url = f"http://127.0.0.1:{port}/api/v1/track"
exts = {".jpg", ".jpeg", ".png", ".bmp"}
images = sorted(p for p in hold.iterdir() if p.is_file() and p.suffix.lower() in exts)
fps = 10.0
interval = 1.0 / fps
intr = "1335.984,1335.984,963.1064,716.9096"
counts = {}
t0 = time.time()
for i, p in enumerate(images):
    t_send = time.time()
    data = p.read_bytes()
    headers = {
        "Content-Type": "image/jpeg" if p.suffix.lower() in (".jpg", ".jpeg") else "application/octet-stream",
        "X-Client-ID": "queue_sweep",
        "X-Intrinsics": intr,
    }
    try:
        r = requests.post(url, data=data, headers=headers, timeout=30)
        code = r.status_code
    except Exception as e:
        code = -1
        counts["exc"] = counts.get("exc", 0) + 1
        if counts.get("exc", 0) <= 3:
            print("exc", e, flush=True)
    counts[code] = counts.get(code, 0) + 1
    if (i + 1) % 100 == 0 or code == 503:
        print(f"  frame {i+1}/{len(images)} status={code} counts={counts}", flush=True)
    elapsed = time.time() - t_send
    sleep = interval - elapsed
    if sleep > 0:
        time.sleep(sleep)

# drain: wait until queue empty + a little slack
for _ in range(120):
    try:
        st = requests.get(f"http://127.0.0.1:{port}/api/v1/status", timeout=2).json()
        if int(st.get("queue_size") or 0) == 0:
            time.sleep(1.0)
            break
    except Exception:
        pass
    time.sleep(0.5)

st = requests.get(f"http://127.0.0.1:{port}/api/v1/status", timeout=5).json()
summary = {
    "n_images": len(images),
    "http_counts": {str(k): v for k, v in sorted(counts.items(), key=lambda kv: str(kv[0]))},
    "http_503": int(counts.get(503, 0)),
    "http_200": int(counts.get(200, 0)),
    "wall_s": time.time() - t0,
    "total_requests": st.get("total_requests"),
    "successful_tracks": st.get("successful_tracks"),
    "failed_tracks": st.get("failed_tracks"),
    "max_queue_size": st.get("max_queue_size"),
    "queue_size_end": st.get("queue_size"),
    "timing_p50": (st.get("timing") or {}).get("p50_ms"),
    "timing_p95": (st.get("timing") or {}).get("p95_ms"),
}
out.write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2), flush=True)
# exit 0 if no 503
raise SystemExit(0 if summary["http_503"] == 0 and summary["http_200"] == summary["n_images"] else 1)
PY
  local rc=$?
  curl -sf -m 5 "http://127.0.0.1:$PORT/api/v1/status" -o "$status_out" || true
  echo "tag=$tag replay_rc=$rc"
  return "$rc"
}

# Ascending search: find min that passes, confirm neighbors.
# Floor: with default min_buffer=3, queue is raised to 3 — so also try min-buffer=1.
QUEUE_CANDIDATES=(10 5 3 2 1)
declare -a RESULTS=()

for q in "${QUEUE_CANDIDATES[@]}"; do
  minb=1
  if run_one "$q" "$minb"; then
    RESULTS+=("PASS q=$q minb=$minb")
  else
    RESULTS+=("FAIL q=$q minb=$minb")
  fi
done

echo ""
echo "======== SUMMARY ========"
printf '%s\n' "${RESULTS[@]}"
python3 - "$OUT_DIR" <<'PY'
import json, sys
from pathlib import Path
d = Path(sys.argv[1])
rows = []
for p in sorted(d.glob("q*_b*_replay.json")):
    j = json.loads(p.read_text())
    rows.append((p.stem, j))
print(f"{'tag':22} {'q':>3} {'503':>5} {'200':>5} {'N':>5} {'fail_trk':>8} {'tot_req':>8} {'p50':>6} {'p95':>6}")
min_pass = None
for stem, j in rows:
    q = j.get("max_queue_size")
    s503 = j.get("http_503", -1)
    s200 = j.get("http_200", -1)
    n = j.get("n_images", -1)
    ok = (s503 == 0 and s200 == n)
    p50 = (j.get("timing_p50") or {}).get("total")
    p95 = (j.get("timing_p95") or {}).get("total")
    print(f"{stem:22} {q!s:>3} {s503:5} {s200:5} {n:5} {j.get('failed_tracks'):>8} {j.get('total_requests'):>8} "
          f"{(p50 if p50 is not None else float('nan')):6.1f} {(p95 if p95 is not None else float('nan')):6.1f}"
          f"  {'PASS' if ok else 'FAIL'}")
    if ok:
        try:
            qi = int(q)
        except Exception:
            continue
        if min_pass is None or qi < min_pass:
            min_pass = qi
print("MIN_PASS_MAX_QUEUE", min_pass)
PY

# leave server on last config for live inspection
echo "SWEEP_DONE"
