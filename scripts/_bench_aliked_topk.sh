#!/usr/bin/env bash
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
BUILD="${BUILD_DIR:-$ROOT/build}"
export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${LD_LIBRARY_PATH:-}"

FRAMES=/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/datasets/2026-04-02_13-42-53/Frames
echo "LIBTORCH_ROOT=$LIBTORCH_ROOT"
echo "BUILD=$BUILD"
echo "FRAMES=$FRAMES"
python3 -c "import cv2,glob; im=cv2.imread(sorted(glob.glob('$FRAMES/*.jpg'))[99]); print('shape', im.shape if im is not None else None)"

# Ensure target is registered (new executable in CMakeLists).
cmake -S "$ROOT" -B "$BUILD" >/tmp/bench_aliked_cmake.log 2>&1 || {
  echo "cmake configure failed; see /tmp/bench_aliked_cmake.log"
  tail -40 /tmp/bench_aliked_cmake.log
  exit 1
}
cmake --build "$BUILD" --target bench_aliked_topk -j"$(nproc)" 2>&1 | tee /tmp/bench_aliked_build.log | tail -40

BIN="$ROOT/Examples/Monocular/bench_aliked_topk"
if [[ ! -x "$BIN" ]]; then
  BIN="$BUILD/Examples/Monocular/bench_aliked_topk"
fi
echo "BIN=$BIN"
# Pass Frames dir so we get multi-frame keypoint stats + timing on one frame.
"$BIN" "$FRAMES" 5 40 cuda
echo "BENCH_OK"
