#!/usr/bin/env bash
# Rebuild slam_server with MM match / LightGlue stage timers.
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
BUILD="$ROOT/build"
export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${LD_LIBRARY_PATH:-}"
cd "$BUILD"
echo "=== building slam_server in $BUILD ==="
cmake --build . --target slam_server -j"$(nproc)" 2>&1 | tee /tmp/slam_server_mm_match_build.log | tail -60
echo "=== binary ==="
ls -la "$ROOT/Examples/Monocular/slam_server" "$BUILD/Examples/Monocular/slam_server" 2>/dev/null || true
strings "$ROOT/Examples/Monocular/slam_server" | grep -E 'mm_match|lg_fwd|lg_drain|pose_mm_match' | head -20 || true
echo "BUILD_OK"
