#!/usr/bin/env bash
# Repeat holdout A/B N times: baseline (EMPTY_CACHE+MM_RETRY) vs new (defaults).
# Usage: scripts/_ab_sync_noretry_repeat.sh [N=3]
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
BUILD="${BUILD_DIR:-$ROOT/build}"
N="${1:-3}"
export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true
unset ROVER_LG_MM_TOPK || true

OUT_ROOT="${HOME}/build/holdout_ab_repeat"
mkdir -p "$OUT_ROOT"

cd "$BUILD"
echo "=== build slam_server ==="
cmake --build . --target slam_server -j"$(nproc)" 2>&1 | tee /tmp/slam_server_ab_repeat_build.log | tail -40

stop_port() {
  local pid
  pid=$(ss -ltnp | awk -v p=":18080" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1 || true)
  if [[ -n "${pid:-}" ]]; then
    kill "$pid" || true
    sleep 2
  fi
}

run_one() {
  local label="$1"
  local out="$2"
  mkdir -p "$out"
  stop_port
  sed -i 's/\r$//' "$ROOT/scripts/eval_holdout_pose_accuracy.py"
  python3 "$ROOT/scripts/eval_holdout_pose_accuracy.py" --port 18080 --fps 10 --out-dir "$out"
  curl -sf -m 5 "http://127.0.0.1:18080/api/v1/status" -o "$out/status.json" || true
  stop_port
  python3 "$ROOT/scripts/_print_holdout_ab_summary.py" "$label" "$out/holdout_pose_metrics.json" "$out/status.json" \
    | tee "$out/summary.txt"
}

sed -i 's/\r$//' "$ROOT/scripts/_print_holdout_ab_summary.py" "$ROOT/scripts/_summarize_ab_repeat.py"

for i in $(seq 1 "$N"); do
  echo "======== ROUND $i / $N — baseline (EMPTY_CACHE+MM_RETRY) ========"
  export ROVER_LG_EMPTY_CACHE=1
  export ROVER_LG_MM_RETRY=1
  run_one "baseline_r${i}" "$OUT_ROOT/r${i}_baseline"

  echo "======== ROUND $i / $N — new (sync + no retry) ========"
  unset ROVER_LG_EMPTY_CACHE || true
  unset ROVER_LG_MM_RETRY || true
  run_one "new_r${i}" "$OUT_ROOT/r${i}_new"
done

echo "======== AGGREGATE ========"
python3 "$ROOT/scripts/_summarize_ab_repeat.py" "$OUT_ROOT" "$N" | tee "$OUT_ROOT/aggregate.txt"
echo "AB_REPEAT_DONE"
