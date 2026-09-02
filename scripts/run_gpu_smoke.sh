#!/usr/bin/env bash
set -euo pipefail

BUILD="${HOME}/build/rover-slam-s1"
ROOT="/mnt/d/agent-workspaces/Rover-SLAM"
DEPS="${HOME}/rover-s1-deps"
LIBTORCH="${HOME}/libtorch-native"
CUDA_ROOT="/usr/local/cuda-12.9"
JOBS="${ROVER_BUILD_JOBS:-2}"

export PATH="${CUDA_ROOT}/bin:${PATH}"
export CUDA_HOME="${CUDA_ROOT}"

rm -rf "${BUILD}"
mkdir -p "${BUILD}"
cd "${BUILD}"

cmake "${ROOT}" \
  -DLIBTORCH_ROOT="${LIBTORCH}" \
  -DCMAKE_C_COMPILER=/usr/bin/cc \
  -DCMAKE_CXX_COMPILER=/usr/bin/c++ \
  -DCMAKE_CUDA_COMPILER="${CUDA_ROOT}/bin/nvcc" \
  -DCUDA_TOOLKIT_ROOT_DIR="${CUDA_ROOT}" \
  -DCUDAToolkit_ROOT="${CUDA_ROOT}" \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -Dyaml-cpp_DIR=/home/robin/miniconda3/lib/cmake/yaml-cpp \
  -DFETCHCONTENT_BASE_DIR="${DEPS}" \
  -DFETCHCONTENT_SOURCE_DIR_ZXING_CPP="${DEPS}/zxing_cpp-src"

test -f CMakeCache.txt
echo "=== CMake configure OK in $(pwd) ==="

cmake --build . --target smoke_aliked_lightglue -j"${JOBS}"
echo "=== Build OK ==="

SMOKE="${BUILD}/Examples/Monocular/smoke_aliked_lightglue"
[ -x "${SMOKE}" ] || SMOKE="${ROOT}/Examples/Monocular/smoke_aliked_lightglue"
echo "=== Running ${SMOKE} ==="
export LD_LIBRARY_PATH="${LIBTORCH}/lib:${ROOT}/lib:${ROOT}/Thirdparty/g2o/lib:${ROOT}/Thirdparty/DBoW3/lib:${CUDA_ROOT}/lib64:${LD_LIBRARY_PATH:-}"
(cd "${ROOT}" && "${SMOKE}" "${ROOT}/frame_100_original.png" "${ROOT}/frame_101_original.png" cuda)
