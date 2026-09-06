#!/usr/bin/env bash
# Rebuild slam_server after sync-only + no-MM-retry, run holdout pose+timing.
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
BUILD="${BUILD_DIR:-$ROOT/build}"
export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true
unset ROVER_LG_EMPTY_CACHE || true
unset ROVER_LG_NO_EMPTY_CACHE || true
unset ROVER_LG_MM_TOPK || true

OUT="${HOME}/build/holdout_pose_eval_sync_noretry"
STATUS="${HOME}/build/holdout_timing_sync_noretry.json"
mkdir -p "$OUT"

cd "$BUILD"
echo "=== build slam_server ==="
cmake --build . --target slam_server -j"$(nproc)" 2>&1 | tee /tmp/slam_server_sync_noretry_build.log | tail -50
strings "$ROOT/Examples/Monocular/slam_server" | grep -E 'ROVER_LG_EMPTY_CACHE|wider window' | head -5 || true

# free port
pid=$(ss -ltnp | awk -v p=":18080" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1 || true)
if [[ -n "${pid:-}" ]]; then kill "$pid" || true; sleep 2; fi

sed -i 's/\r$//' "$ROOT/scripts/eval_holdout_pose_accuracy.py" "$ROOT/scripts/_print_holdout_ab_summary.py"
python3 "$ROOT/scripts/eval_holdout_pose_accuracy.py" --port 18080 --fps 10 --out-dir "$OUT"
curl -sf -m 5 "http://127.0.0.1:18080/api/v1/status" -o "$STATUS" || true
python3 "$ROOT/scripts/_print_holdout_ab_summary.py" sync_noretry "$OUT/holdout_pose_metrics.json" "$STATUS"

pid=$(ss -ltnp | awk -v p=":18080" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1 || true)
if [[ -n "${pid:-}" ]]; then kill "$pid" || true; fi
echo "NEW_DONE"
