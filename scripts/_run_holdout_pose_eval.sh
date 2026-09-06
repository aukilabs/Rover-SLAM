#!/usr/bin/env bash
# Rebuild slam_server (all_maps trajectory) and run holdout pose accuracy eval.
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
BUILD="$ROOT/build"
export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true

cd "$BUILD"
echo "=== build slam_server ==="
cmake --build . --target slam_server -j"$(nproc)" 2>&1 | tee /tmp/slam_server_pose_eval_build.log | tail -40
strings "$ROOT/Examples/Monocular/slam_server" | grep -E 'all_atlas_maps|all_maps' | head -5

echo "=== eval ==="
sed -i 's/\r$//' "$ROOT/scripts/eval_holdout_pose_accuracy.py"
python3 "$ROOT/scripts/eval_holdout_pose_accuracy.py" --port 18080 --fps 10 \
  --out-dir "$HOME/build/holdout_pose_eval"
echo "EVAL_DONE"
