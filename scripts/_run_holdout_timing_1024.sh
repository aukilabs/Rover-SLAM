#!/usr/bin/env bash
# Replay holdout at full LG MM input (1024) and dump spike analysis.
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
HOLD=/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/datasets/2026-04-02_13-42-53/Frames
PORT=18080
LOG=$HOME/build/rover-slam-holdout-timing-1024.log
STATUS_OUT=$HOME/build/holdout_timing_1024.json
SPIKES_OUT=$HOME/build/holdout_timing_spikes_1024.txt
ANALYSIS_OUT=$HOME/build/holdout_spike_analysis_1024.json

export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true
unset ROVER_LG_MM_TOPK || true

mkdir -p "$HOME/build"
cd "$ROOT"

echo "=== ensure ROVER_LG_MM_TOPK unset (full 1024 into LightGlue) ==="
echo "ROVER_LG_MM_TOPK=${ROVER_LG_MM_TOPK:-<unset>}"

echo "=== stop old slam_server on :$PORT ==="
if pid=$(ss -ltnp | awk -v p=":$PORT" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1); then
  echo "killing pid $pid"
  kill "$pid" || true
  for i in $(seq 1 30); do
    ss -ltn | grep -q ":$PORT" || break
    sleep 0.5
  done
  if ss -ltn | grep -q ":$PORT"; then
    kill -9 "$pid" || true
    sleep 1
  fi
else
  echo "no listener on :$PORT"
fi

BIN="$ROOT/Examples/Monocular/slam_server"
echo "=== start slam_server ==="
: > "$LOG"
"$BIN" \
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
      echo "ready after ${i}s"
      ok=1
      break
    fi
  fi
  sleep 1
done
if [[ $ok -eq 0 ]]; then
  echo "ERROR: server not ready"
  tail -80 "$LOG" || true
  exit 1
fi

N=$(find "$HOLD" -maxdepth 1 -type f \( -name '*.jpg' -o -name '*.png' \) | wc -l)
echo "=== replay holdout frames N=$N @ 10 FPS ==="
python3 Examples/Monocular/send_frame.py "$HOLD" \
  --server "http://127.0.0.1:$PORT" \
  --fps 10 \
  --raw \
  --intrinsics 1335.984 1335.984 963.1064 716.9096 \
  2>&1 | tee "$HOME/build/holdout_replay_1024.log"

curl -sf -m 5 "http://127.0.0.1:$PORT/api/v1/status" -o "$STATUS_OUT"
grep '\[timing\] spike' "$LOG" | tee "$SPIKES_OUT" >/dev/null || true

python3 "$ROOT/scripts/_analyze_holdout_spikes.py" \
  --log "$LOG" \
  --status "$STATUS_OUT" \
  --out "$ANALYSIS_OUT"

echo "DONE status=$STATUS_OUT spikes=$SPIKES_OUT analysis=$ANALYSIS_OUT"
