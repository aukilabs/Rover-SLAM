#!/usr/bin/env bash
# Restart instrumented slam_server and replay holdout @ 10 FPS; dump timing.
set -euo pipefail

ROOT=/mnt/d/agent-workspaces/Rover-SLAM
HOLD=/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/datasets/2026-04-02_13-42-53/Frames
PORT=18080
LOG=$HOME/build/rover-slam-holdout-timing.log
STATUS_OUT=$HOME/build/holdout_timing_status.json
SPIKES_OUT=$HOME/build/holdout_timing_spikes.txt

export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
export CMAKE_PREFIX_PATH="${HOME}/.local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PATH="/usr/local/cuda/bin:${HOME}/bin:${PATH}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true

mkdir -p "$HOME/build"
cd "$ROOT"

echo "=== stop old slam_server on :$PORT ==="
if pid=$(ss -ltnp | awk -v p=":$PORT" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1); then
  echo "killing pid $pid"
  kill "$pid" || true
  for i in $(seq 1 30); do
    ss -ltn | grep -q ":$PORT" || break
    sleep 0.5
  done
  if ss -ltn | grep -q ":$PORT"; then
    echo "force kill"
    kill -9 "$pid" || true
    sleep 1
  fi
else
  echo "no listener on :$PORT"
fi

BIN="$ROOT/Examples/Monocular/slam_server"
if [[ ! -x "$BIN" ]]; then
  echo "ERROR: missing $BIN"
  exit 1
fi
echo "=== binary ==="
ls -la "$BIN"
strings "$BIN" | grep -E 'over_100ms_count|p95_ms|\[timing\]' | head -10

echo "=== start slam_server ==="
: > "$LOG"
./Examples/Monocular/slam_server \
  Vocabulary/voc_binary_tartan_8u_6.yml.gz \
  Examples/Monocular/iphoneSLAM.yaml \
  --prior-pack testdata/prior_pack_fixture \
  --port "$PORT" \
  --max-queue 100 \
  >"$LOG" 2>&1 &
SPID=$!
echo "started pid=$SPID log=$LOG"

echo "=== wait for /status ==="
ok=0
for i in $(seq 1 60); do
  if curl -sf -m 2 "http://127.0.0.1:$PORT/api/v1/status" >/tmp/slam_boot.json; then
    if python3 -c 'import json; d=json.load(open("/tmp/slam_boot.json")); raise SystemExit(0 if "timing" in d else 1)'; then
      echo "ready with timing after ${i}s"
      ok=1
      break
    else
      echo "up but no timing key yet (unexpected); keys=$(python3 -c 'import json;print(sorted(json.load(open("/tmp/slam_boot.json")).keys()))')"
      ok=2
      break
    fi
  fi
  sleep 1
done
if [[ $ok -eq 0 ]]; then
  echo "ERROR: server did not become ready"
  tail -80 "$LOG" || true
  exit 1
fi

echo "=== pre-replay status timing ==="
python3 - <<'PY'
import json
d=json.load(open('/tmp/slam_boot.json'))
print(json.dumps(d.get('timing'), indent=2))
PY

N=$(find "$HOLD" -maxdepth 1 -type f \( -name '*.jpg' -o -name '*.png' \) | wc -l)
echo "=== replay holdout frames N=$N @ 10 FPS ==="
# Use a modest subset first if full run is too long? Plan says holdout @ 10 FPS.
# 580 frames @ 10 FPS ~= 58s of send pacing; processing may stretch wall time.
# Run full holdout as planned.
python3 Examples/Monocular/send_frame.py "$HOLD" \
  --server "http://127.0.0.1:$PORT" \
  --fps 10 \
  --raw \
  --intrinsics 1335.984 1335.984 963.1064 716.9096 \
  2>&1 | tee "$HOME/build/holdout_replay.log"

echo "=== post-replay /status ==="
curl -sf -m 5 "http://127.0.0.1:$PORT/api/v1/status" -o "$STATUS_OUT"
python3 - <<PY
import json
d=json.load(open("$STATUS_OUT"))
print("total_requests", d.get("total_requests"))
print("successful_tracks", d.get("successful_tracks"))
print("failed_tracks", d.get("failed_tracks"))
print("avg_processing_time_ms", d.get("avg_processing_time_ms"))
print("queue_size", d.get("queue_size"))
t=d.get("timing")
print("TIMING:")
print(json.dumps(t, indent=2))
if t:
    p95=t.get("p95_ms") or {}
    p50=t.get("p50_ms") or {}
    print("COMPARE pose_pred sub-paths:")
    print("  p50 pose_pred", p50.get("pose_pred"),
          "mm", p50.get("pose_mm"),
          "mm_match", p50.get("pose_mm_match"),
          "mm_opt", p50.get("pose_mm_opt"),
          "lg_fwd", p50.get("lg_fwd"),
          "lg_drain", p50.get("lg_drain"),
          "refkf", p50.get("pose_refkf"),
          "reloc", p50.get("pose_reloc"))
    print("  p95 pose_pred", p95.get("pose_pred"),
          "mm", p95.get("pose_mm"),
          "mm_match", p95.get("pose_mm_match"),
          "mm_opt", p95.get("pose_mm_opt"),
          "lg_fwd", p95.get("lg_fwd"),
          "lg_drain", p95.get("lg_drain"),
          "refkf", p95.get("pose_refkf"),
          "reloc", p95.get("pose_reloc"))
    print("  max_pose_mm_ms", t.get("max_pose_mm_ms"),
          "max_pose_mm_match_ms", t.get("max_pose_mm_match_ms"),
          "max_pose_mm_opt_ms", t.get("max_pose_mm_opt_ms"),
          "max_lg_fwd_ms", t.get("max_lg_fwd_ms"),
          "max_lg_drain_ms", t.get("max_lg_drain_ms"),
          "max_pose_refkf_ms", t.get("max_pose_refkf_ms"),
          "max_pose_reloc_ms", t.get("max_pose_reloc_ms"))
    print("  over_100ms_count", t.get("over_100ms_count"))
PY

echo "=== spike lines from server log ==="
grep '\[timing\] spike' "$LOG" | tee "$SPIKES_OUT" | tail -40
echo "spike_count=$(grep -c '\[timing\] spike' "$LOG" || true)"
python3 - <<'PY'
import re
from pathlib import Path
from collections import Counter
log = Path.home() / "build" / "rover-slam-holdout-timing.log"
text = log.read_text(errors="replace")
spikes = []
pat = re.compile(
    r"\[timing\] spike frame (\d+) total=([\d.]+)ms.*?"
    r" pose=([\d.]+) mm=([\d.]+)"
    r"(?: mm_match=([\d.]+) mm_opt=([\d.]+)"
    r" lg_prep=([\d.]+) lg_fwd=([\d.]+) lg_drain=([\d.]+) lg_post=([\d.]+))?"
    r".*? refkf=([\d.]+) reloc=([\d.]+)"
)
for m in pat.finditer(text):
    g = m.groups()
    spikes.append({
        "frame": int(g[0]),
        "total": float(g[1]),
        "pose": float(g[2]),
        "mm": float(g[3]),
        "mm_match": float(g[4]) if g[4] is not None else None,
        "mm_opt": float(g[5]) if g[5] is not None else None,
        "lg_prep": float(g[6]) if g[6] is not None else None,
        "lg_fwd": float(g[7]) if g[7] is not None else None,
        "lg_drain": float(g[8]) if g[8] is not None else None,
        "lg_post": float(g[9]) if g[9] is not None else None,
        "refkf": float(g[10]),
        "reloc": float(g[11]),
    })
print("parsed_spikes", len(spikes))
if spikes:
    def dominant_pose(row):
        return max(("mm", row["mm"]), ("refkf", row["refkf"]), ("reloc", row["reloc"]), key=lambda x: x[1])[0]
    print("spike_dominant_pose_subpath", dict(Counter(dominant_pose(s) for s in spikes)))
    deep = [s for s in spikes if s["mm_match"] is not None]
    if deep:
        def dominant_mm(row):
            return max(("match", row["mm_match"]), ("opt", row["mm_opt"]), key=lambda x: x[1])[0]
        print("spike_dominant_mm_split", dict(Counter(dominant_mm(s) for s in deep)))
        def dominant_lg(row):
            return max(
                ("prep", row["lg_prep"]),
                ("fwd", row["lg_fwd"]),
                ("drain", row["lg_drain"]),
                ("post", row["lg_post"]),
                key=lambda x: x[1],
            )[0]
        print("spike_dominant_lg_stage", dict(Counter(dominant_lg(s) for s in deep)))
        worst = max(deep, key=lambda r: r["pose"])
        print("worst_pose_spike", worst)
PY
echo "DONE status=$STATUS_OUT spikes=$SPIKES_OUT"
