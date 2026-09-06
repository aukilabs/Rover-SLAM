#!/usr/bin/env bash
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
BUILD="${BUILD_DIR:-$ROOT/build}"
export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"

FRAMES=/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/datasets/2026-04-02_13-42-53/Frames
N="${1:-60}"

cd "$BUILD"
cmake -S "$ROOT" -B "$BUILD" >/tmp/bench_batch_cmake.log 2>&1 || {
  tail -40 /tmp/bench_batch_cmake.log; exit 1
}
cmake --build "$BUILD" --target bench_batch_aliked_lg -j"$(nproc)" 2>&1 | tee /tmp/bench_batch_build.log | tail -50

BIN="$ROOT/Examples/Monocular/bench_batch_aliked_lg"
echo "BIN=$BIN N=$N"
"$BIN" "$FRAMES" "$N" cuda
echo "BENCH_OK"
