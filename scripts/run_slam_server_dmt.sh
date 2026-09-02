#!/usr/bin/env bash
# Launch stream-mode slam_server with DMT.yaml (ALIKED + LightGlue), same stack as offline DMT.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIBTORCH_ROOT="${LIBTORCH_ROOT:-${HOME}/build/libtorch-cu128}"
ONNX="${ROOT}/onnxruntime-linux-x64-gpu-1.16.3/lib"
PORT="${PORT:-8000}"
VIEWER="${VIEWER:-0}"
BUILD_FIRST="${BUILD_FIRST:-1}"
LOG="${HOME}/build/rover-slam-server-dmt.log"

mkdir -p "${HOME}/build"
export LIBTORCH_ROOT
export CMAKE_PREFIX_PATH="${HOME}/.local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PATH="/usr/local/cuda/bin:${HOME}/bin:${PATH}"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE=1

if [[ "${BUILD_FIRST}" == "1" ]]; then
  echo "=== BUILD slam_server ==="
  cmake --build "${ROOT}/build" --target slam_server -j"${JOBS:-2}"
fi

ARGS=(
  "${ROOT}/Vocabulary/voc_binary_tartan_8u_6.yml.gz"
  "${ROOT}/Examples/Monocular-Inertial/DMT.yaml"
  --port "${PORT}"
)
if [[ "${VIEWER}" == "1" ]]; then
  unset ROVER_SLAM_HEADLESS || true
  export DISPLAY="${DISPLAY:-:0}"
  ARGS+=(--viewer)
else
  # Stream UI still works; Pangolin viewer off (safer under WSL / no display)
  export ROVER_SLAM_HEADLESS=1
  unset DISPLAY || true
fi

cd "${ROOT}"
echo "=== START slam_server port=${PORT} viewer=${VIEWER} $(date -Iseconds) ==="
echo "Log: ${LOG}"
echo "UI:  http://localhost:${PORT}/"
exec ./Examples/Monocular/slam_server "${ARGS[@]}" 2>&1 | tee "${LOG}"
