#!/usr/bin/env bash
set -euo pipefail

# S1 GPU verification: configure + build auki_lightglue_lib (+ link into ORB_SLAM3 if full build succeeds).
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${BUILD_DIR:-${HOME}/build/rover-slam-s1}"
DEPS="${ROVER_FETCHCONTENT_DIR:-/tmp/rover-s1-deps}"
LIBTORCH="${LIBTORCH_ROOT:-${HOME}/libtorch-cu129}"
CUDA_ROOT="${CUDA_HOME:-/usr/local/cuda-12.9}"
YAML_CPP_DIR="${YAML_CPP_DIR:-/home/robin/miniconda3/lib/cmake/yaml-cpp}"

export PATH="${CUDA_ROOT}/bin:/usr/bin:${PATH}"
export CUDA_HOME="${CUDA_ROOT}"
export CC=/usr/bin/cc
export CXX=/usr/bin/c++

rm -rf "${BUILD}" "${DEPS}"
mkdir -p "${BUILD}" "${DEPS}"
cd "${BUILD}"

cmake "${ROOT}" \
  -DLIBTORCH_ROOT="${LIBTORCH}" \
  -DCMAKE_CUDA_COMPILER="${CUDA_ROOT}/bin/nvcc" \
  -DCUDA_TOOLKIT_ROOT_DIR="${CUDA_ROOT}" \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -Dyaml-cpp_DIR="${YAML_CPP_DIR}" \
  -DFETCHCONTENT_BASE_DIR="${DEPS}"

echo "=== Configure OK ==="
grep -E 'auki_lightglue_lib|AUKI_LIGHTGLUE' CMakeFiles/ORB_SLAM3.dir/link.txt 2>/dev/null || \
  grep -E 'auki_lightglue_lib' CMakeFiles/Export/*/ORB_SLAM3* 2>/dev/null || true

mkdir -p \
  CMakeFiles/auki_lightglue_lib.dir/third_party/Light_Glue_CPP/src/feature \
  CMakeFiles/auki_lightglue_lib.dir/third_party/Light_Glue_CPP/src/matcher/lightglue

cmake --build . --target auki_lightglue_lib -j1
echo "=== auki_lightglue_lib built ==="
ls -la libauki_lightglue_lib.a

if cmake --build . --target ORB_SLAM3 -j"$(nproc)"; then
  echo "=== ORB_SLAM3 linked ==="
  ls -la "${ROOT}/lib/libORB_SLAM3.so" 2>/dev/null || ls -la lib/libORB_SLAM3.so
else
  echo "=== ORB_SLAM3 build skipped/failed (S1 lib target OK) ==="
fi
