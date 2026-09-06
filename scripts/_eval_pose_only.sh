#!/usr/bin/env bash
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true
python3 -c 'import matplotlib, pycolmap, requests, numpy; print("imports_ok")'
sed -i 's/\r$//' "$ROOT/scripts/eval_holdout_pose_accuracy.py"
python3 "$ROOT/scripts/eval_holdout_pose_accuracy.py" --port 18080 --fps 10 \
  --out-dir "$HOME/build/holdout_pose_eval"
echo EVAL_DONE
