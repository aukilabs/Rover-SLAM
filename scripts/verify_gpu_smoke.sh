#!/usr/bin/env bash
# GPU verify: configure (if needed), build smoke target, run smoke on native WSL paths.
set -euo pipefail

ROOT="${ROVER_ROOT:-${HOME}/rover-slam-work}"
BUILD="${BUILD_DIR:-${HOME}/build/rover-slam-s1}"
DEPS="${ROVER_FETCHCONTENT_DIR:-${HOME}/rover-s1-deps}"
LIBTORCH="${LIBTORCH_ROOT:-${HOME}/libtorch-cu129}"
CUDA_ROOT="${CUDA_HOME:-/usr/local/cuda-12.9}"
YAML_CPP_DIR="${YAML_CPP_DIR:-${HOME}/miniconda3/lib/cmake/yaml-cpp}"
JOBS="${ROVER_BUILD_JOBS:-1}"

export PATH="${CUDA_ROOT}/bin:${PATH}"
export CUDA_HOME="${CUDA_ROOT}"
export CUDACXX="${CUDA_ROOT}/bin/nvcc"

# Clean PATH of Windows /mnt paths that confuse nvcc under WSL.
export PATH="${CUDA_ROOT}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

mkdir -p "${DEPS}" "${BUILD}"
cd "${BUILD}"

if [ ! -f "${BUILD}/CMakeCache.txt" ]; then
  echo "=== CMake configure ==="
  cmake -S "${ROOT}" -B "${BUILD}" \
    -DLIBTORCH_ROOT="${LIBTORCH}" \
    -DCMAKE_C_COMPILER=/usr/bin/cc \
    -DCMAKE_CXX_COMPILER=/usr/bin/c++ \
    -DCMAKE_CUDA_COMPILER="${CUDA_ROOT}/bin/nvcc" \
    -DCUDA_TOOLKIT_ROOT_DIR="${CUDA_ROOT}" \
    -DCMAKE_CUDA_ARCHITECTURES=120 \
    -Dyaml-cpp_DIR="${YAML_CPP_DIR}" \
    -DFETCHCONTENT_BASE_DIR="${DEPS}" \
    -DFETCHCONTENT_SOURCE_DIR_ZXING_CPP="${DEPS}/zxing_cpp-src"
fi

echo "=== Building smoke_aliked_lightglue (-j${JOBS}) ==="
cmake --build "${BUILD}" --target smoke_aliked_lightglue -j"${JOBS}"

SMOKE="${BUILD}/smoke_aliked_lightglue"
[ -x "${SMOKE}" ] || SMOKE="${BUILD}/Examples/Monocular/smoke_aliked_lightglue"
[ -x "${SMOKE}" ] || SMOKE="${ROOT}/Examples/Monocular/smoke_aliked_lightglue"

if [ ! -x "${SMOKE}" ]; then
  echo "ERROR: smoke binary not found under ${BUILD}" >&2
  exit 1
fi

echo "=== Running smoke ==="
cd "${ROOT}"
IMG0="${SMOKE_IMG0:-${ROOT}/frame_100_original.png}"
IMG1="${SMOKE_IMG1:-${ROOT}/frame_101_original.png}"
export LD_LIBRARY_PATH="${LIBTORCH}/lib:${ROOT}/lib:${ROOT}/Thirdparty/g2o/lib:${ROOT}/Thirdparty/DBoW3/lib:${CUDA_ROOT}/lib64:${LD_LIBRARY_PATH:-}"
"${SMOKE}" "${IMG0}" "${IMG1}" cuda
echo "=== GPU smoke PASS ==="
