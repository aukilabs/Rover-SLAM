#!/usr/bin/env bash
# Confirm min --max-queue with N repeats on borderline sizes.
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
HOLD=/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/datasets/2026-04-02_13-42-53/Frames
PORT=18080
OUT=$HOME/build/holdout_queue_confirm
LOG=$HOME/build/rover-slam-queue-confirm.log

export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
export CMAKE_PREFIX_PATH="${HOME}/.local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PATH="/usr/local/cuda/bin:${HOME}/bin:${PATH}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true

mkdir -p "$OUT"
cd "$ROOT"

stop_port() {
  local pid
  pid=$(ss -ltnp 2>/dev/null | awk -v p=":$PORT" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1 || true)
  if [[ -n "${pid:-}" ]]; then
    kill "$pid" 2>/dev/null || true
    sleep 0.8
    kill -9 "$pid" 2>/dev/null || true
  fi
}

run_one() {
  local q="$1" rep="$2"
  local tag="q${q}_r${rep}"
  stop_port
  : > "$LOG"
  ./Examples/Monocular/slam_server \
    Vocabulary/voc_binary_tartan_8u_6.yml.gz \
    Examples/Monocular/iphoneSLAM.yaml \
    --prior-pack testdata/prior_pack_fixture \
    --port "$PORT" \
    --max-queue "$q" \
    --min-buffer 1 \
    >"$LOG" 2>&1 &
  local spid=$!
  for i in $(seq 1 90); do
    if curl -sf -m 2 "http://127.0.0.1:$PORT/api/v1/status" >/tmp/slam_boot_q.json 2>/dev/null; then
      python3 -c 'import json; d=json.load(open("/tmp/slam_boot_q.json")); raise SystemExit(0 if "timing" in d else 1)' && break
    fi
    kill -0 "$spid" 2>/dev/null || { echo die; tail -30 "$LOG"; return 2; }
    sleep 1
  done
  # warm: optional short pause after boot so CUDA kernels settle
  sleep 2
  python3 - "$HOLD" "$PORT" "$OUT/${tag}.json" "$q" <<'PY'
import json, sys, time
from pathlib import Path
import requests
hold = Path(sys.argv[1]); port=int(sys.argv[2]); out=Path(sys.argv[3]); q=int(sys.argv[4])
url=f"http://127.0.0.1:{port}/api/v1/track"
images=sorted(p for p in hold.iterdir() if p.is_file() and p.suffix.lower() in {".jpg",".jpeg",".png",".bmp"})
interval=0.1
intr="1335.984,1335.984,716.9096,963.1064"
counts={}
for i,p in enumerate(images):
    t0=time.time()
    headers={"Content-Type":"image/jpeg","X-Client-ID":"queue_confirm","X-Intrinsics":intr}
    try:
        code=requests.post(url, data=p.read_bytes(), headers=headers, timeout=30).status_code
    except Exception:
        code=-1
    counts[code]=counts.get(code,0)+1
    sleep=interval-(time.time()-t0)
    if sleep>0: time.sleep(sleep)
for _ in range(60):
    st=requests.get(f"http://127.0.0.1:{port}/api/v1/status", timeout=2).json()
    if int(st.get("queue_size") or 0)==0:
        time.sleep(0.5); break
    time.sleep(0.25)
st=requests.get(f"http://127.0.0.1:{port}/api/v1/status", timeout=5).json()
summary={"q":q,"http_503":counts.get(503,0),"http_200":counts.get(200,0),"n":len(images),
         "total_requests":st.get("total_requests"),"failed_tracks":st.get("failed_tracks"),
         "p50_total":(st.get("timing") or {}).get("p50_ms",{}).get("total"),
         "p95_total":(st.get("timing") or {}).get("p95_ms",{}).get("total")}
out.write_text(json.dumps(summary, indent=2))
print(json.dumps(summary), flush=True)
raise SystemExit(0 if summary["http_503"]==0 and summary["http_200"]==summary["n"] else 1)
PY
}

# Confirm 3,4,5,10 twice each (ascending interest around floor)
for q in 3 4 5 10; do
  for r in 1 2; do
    echo "=== confirm q=$q rep=$r ==="
    if run_one "$q" "$r"; then
      echo "PASS q=$q r=$r"
    else
      echo "FAIL q=$q r=$r"
    fi
  done
done

python3 - "$OUT" <<'PY'
import json, collections
from pathlib import Path
d=Path(__import__('sys').argv[1])
by=collections.defaultdict(list)
for p in sorted(d.glob('q*_r*.json')):
    j=json.loads(p.read_text()); by[j['q']].append(j)
print(f"{'q':>3} {'pass':>4}/{'n':<2} {'503s':>12} {'p50':>8} {'p95':>8}")
for q in sorted(by):
    js=by[q]
    npass=sum(1 for j in js if j['http_503']==0 and j['http_200']==j['n'])
    s503=','.join(str(j['http_503']) for j in js)
    p50=','.join(f"{j['p50_total']:.0f}" if j['p50_total'] is not None else '?' for j in js)
    p95=','.join(f"{j['p95_total']:.0f}" if j['p95_total'] is not None else '?' for j in js)
    print(f"{q:3} {npass:4}/{len(js):<2} {s503:>12} {p50:>8} {p95:>8}")
PY
echo CONFIRM_DONE
