#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROVER_ROOT:-${HOME}/rover-slam-work}"
SMOKE="${ROOT}/Examples/Monocular/smoke_aliked_lightglue"
IMG0="${1:-${ROOT}/frame_100_original.png}"
IMG1="${2:-${ROOT}/frame_101_original.png}"

export LD_LIBRARY_PATH="${ROOT}/lib:${ROOT}/onnxruntime-linux-x64-gpu-1.16.3/lib:${HOME}/libtorch-cu129/lib:/usr/local/cuda-12.9/lib64:${LD_LIBRARY_PATH:-}"
export PATH="/usr/local/cuda-12.9/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

cd "${ROOT}/Examples/Monocular"
exec "${SMOKE}" "${IMG0}" "${IMG1}" cuda
