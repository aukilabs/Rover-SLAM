#!/usr/bin/env bash
set -euo pipefail

ROOT="/mnt/d/agent-workspaces/Rover-SLAM"
BUILD="${BUILD_DIR:-${HOME}/build/rover-slam-s1}"
DEPS="${ROVER_FETCHCONTENT_DIR:-${HOME}/rover-s1-deps}"
LIBTORCH="${LIBTORCH_ROOT:-${HOME}/libtorch-native}"
JOBS="${ROVER_BUILD_JOBS:-2}"
CUDA_ROOT="${CUDA_HOME:-/usr/local/cuda-12.9}"
YAML_CPP_DIR="${YAML_CPP_DIR:-/home/robin/miniconda3/lib/cmake/yaml-cpp}"
LOCK="${BUILD}/.configure.lock"

export PATH="${CUDA_ROOT}/bin:${PATH}"
export CUDA_HOME="${CUDA_ROOT}"

mkdir -p "${BUILD}" "${DEPS}"
cd "${BUILD}"

need_configure=0
if [ ! -f CMakeCache.txt ] || [ ! -f Makefile ]; then
  need_configure=1
fi

if [ "${need_configure}" -eq 1 ]; then
  exec 9>"${LOCK}"
  if ! flock -n 9; then
    echo "Another configure is running; waiting..." >&2
    flock 9
  fi
  if [ ! -f CMakeCache.txt ] || [ ! -f Makefile ]; then
    rm -rf "${BUILD:?}/"*
    cmake "${ROOT}" \
      -DLIBTORCH_ROOT="${LIBTORCH}" \
      -DCMAKE_C_COMPILER=/usr/bin/cc \
      -DCMAKE_CXX_COMPILER=/usr/bin/c++ \
      -DCMAKE_CUDA_COMPILER="${CUDA_ROOT}/bin/nvcc" \
      -DCUDA_TOOLKIT_ROOT_DIR="${CUDA_ROOT}" \
      -DCUDAToolkit_ROOT="${CUDA_ROOT}" \
      -DCMAKE_CUDA_ARCHITECTURES=120 \
      -Dyaml-cpp_DIR="${YAML_CPP_DIR}" \
      -DFETCHCONTENT_BASE_DIR="${DEPS}" \
      -DFETCHCONTENT_SOURCE_DIR_ZXING_CPP="${DEPS}/zxing_cpp-src"
  fi
fi

cmake --build . --target smoke_aliked_lightglue -j"${JOBS}"

SMOKE="${BUILD}/Examples/Monocular/smoke_aliked_lightglue"
if [ ! -x "${SMOKE}" ]; then
  SMOKE="${BUILD}/smoke_aliked_lightglue"
fi

echo "=== Running ${SMOKE} ==="
(cd "${ROOT}" && "${SMOKE}")
