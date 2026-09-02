#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROVER_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
BUILD="${BUILD_DIR:-${HOME}/build/rover-slam-s1}"
DEPS="${ROVER_FETCHCONTENT_DIR:-${HOME}/rover-s1-deps}"
TP_BUILD_ROOT="${HOME}/build/rover-slam-thirdparty"
JOBS="${ROVER_BUILD_JOBS:-4}"

# Prefer a native WSL LibTorch tree (avoid /mnt/d I/O under heavy parallel builds).
LIBTORCH_DL="${HOME}/build/libtorch-cu128"
LIBTORCH_ZIP="${HOME}/build/libtorch-dl/libtorch.zip"
LIBTORCH_URL="https://download.pytorch.org/libtorch/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu128.zip"
NATIVE_LIBTORCH="${HOME}/libtorch-native"

pick_libtorch() {
  if [ -n "${LIBTORCH_ROOT:-}" ] && [ -f "${LIBTORCH_ROOT}/share/cmake/Torch/TorchConfig.cmake" ]; then
    echo "${LIBTORCH_ROOT}"
    return
  fi
  for candidate in "${LIBTORCH_DL}" "${NATIVE_LIBTORCH}"; do
    if [ -f "${candidate}/share/cmake/Torch/TorchConfig.cmake" ]; then
      echo "${candidate}"
      return
    fi
  done
  local fallback="/mnt/d/gaussian-splatting-cuda/external/libtorch"
  if [ -f "${fallback}/share/cmake/Torch/TorchConfig.cmake" ]; then
    echo "=== Copying LibTorch ${fallback} -> ${NATIVE_LIBTORCH} ===" >&2
    rm -rf "${NATIVE_LIBTORCH}"
    cp -a "${fallback}" "${NATIVE_LIBTORCH}"
    echo "${NATIVE_LIBTORCH}"
    return
  fi
  if [ -f "${LIBTORCH_ZIP}" ] && unzip -tq "${LIBTORCH_ZIP}" >/dev/null 2>&1; then
    echo "=== Extracting ${LIBTORCH_ZIP} -> ${LIBTORCH_DL} ===" >&2
    rm -rf "${LIBTORCH_DL}"
    unzip -q "${LIBTORCH_ZIP}" -d "${HOME}/build/libtorch-dl"
    mv "${HOME}/build/libtorch-dl/libtorch" "${LIBTORCH_DL}"
    echo "${LIBTORCH_DL}"
    return
  fi
  echo "ERROR: set LIBTORCH_ROOT or fix LibTorch at ${fallback} / ${LIBTORCH_ZIP}" >&2
  exit 1
}

LIBTORCH="$(pick_libtorch)"

CUDA_ROOT="${CUDA_HOME:-/usr/local/cuda-12.9}"
YAML_CPP_DIR="${YAML_CPP_DIR:-/home/robin/miniconda3/lib/cmake/yaml-cpp}"

export PATH="${CUDA_ROOT}/bin:${PATH}"
export CUDA_HOME="${CUDA_ROOT}"
export CUDACXX="${CUDA_ROOT}/bin/nvcc"
export CUDAHOME="${CUDA_ROOT}"
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-12.0}"

build_thirdparty() {
  local name="$1"
  local dir="$2"
  local lib_rel="$3"
  local lib_path="${dir}/${lib_rel}"
  if [ -f "${lib_path}" ] 2>/dev/null; then
    echo "=== Thirdparty/${name}: using existing ${lib_rel} ==="
    return 0
  fi
  echo "=== Building Thirdparty/${name} (native build dir) ==="
  local tp_build="${TP_BUILD_ROOT}/${name}"
  rm -rf "${tp_build}"
  mkdir -p "${tp_build}" "${dir}/lib"
  cmake -S "${dir}" -B "${tp_build}" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  cmake --build "${tp_build}" -j"${JOBS}"
  find "${tp_build}" -name '*.so' -exec cp -a {} "${dir}/lib/" \;
}

build_thirdparty DBoW3 "${ROOT}/Thirdparty/DBoW3" "lib/libDBoW3.so"
build_thirdparty g2o "${ROOT}/Thirdparty/g2o" "lib/libg2o.so"

seed_zxing() {
  rm -rf "${DEPS}/zxing_cpp-src"
  for zxing_src in /mnt/d/Rover-SLAM/build/_deps/zxing_cpp-src \
                   /mnt/d/agent-workspaces/Rover-SLAM/build/_deps/zxing_cpp-src; do
    if [ -f "${zxing_src}/CMakeLists.txt" ]; then
      echo "=== Seeding zxing_cpp from ${zxing_src} (tar) ===" >&2
      if (cd "${zxing_src}/.." && tar cf - zxing_cpp-src) | (cd "${DEPS}" && tar xf -); then
        return 0
      fi
      rm -rf "${DEPS}/zxing_cpp-src"
    fi
  done
  echo "=== Cloning zxing-cpp (shallow) into ${DEPS} ===" >&2
  git clone --depth 1 https://github.com/zxing-cpp/zxing-cpp.git "${DEPS}/zxing_cpp-src"
}

mkdir -p "${DEPS}"
if [ ! -f "${DEPS}/zxing_cpp-src/CMakeLists.txt" ]; then
  seed_zxing
fi

rm -rf "${BUILD}"
mkdir -p "${BUILD}"
cd "${BUILD}"

cmake_args=(
  "${ROOT}"
  -DLIBTORCH_ROOT="${LIBTORCH}"
  -DCMAKE_C_COMPILER=/usr/bin/cc
  -DCMAKE_CXX_COMPILER=/usr/bin/c++
  -DCMAKE_CUDA_COMPILER="${CUDA_ROOT}/bin/nvcc"
  -DCUDA_TOOLKIT_ROOT_DIR="${CUDA_ROOT}"
  -DCUDAToolkit_ROOT="${CUDA_ROOT}"
  -DCMAKE_CUDA_ARCHITECTURES=120
  -Dyaml-cpp_DIR="${YAML_CPP_DIR}"
  -DFETCHCONTENT_BASE_DIR="${DEPS}"
  -DFETCHCONTENT_SOURCE_DIR_ZXING_CPP="${DEPS}/zxing_cpp-src"
)

cmake "${cmake_args[@]}"

echo "=== Configure OK ==="
cmake --build . --target smoke_aliked_lightglue -j"${JOBS}"
echo "=== smoke_aliked_lightglue built ==="

SMOKE="${BUILD}/smoke_aliked_lightglue"
if [ ! -x "${SMOKE}" ]; then
  SMOKE="${BUILD}/Examples/Monocular/smoke_aliked_lightglue"
fi
if [ -x "${SMOKE}" ]; then
  echo "=== Running smoke_aliked_lightglue ==="
  (cd "${ROOT}" && "${SMOKE}" frame_100_original.png frame_101_original.png cuda)
else
  echo "ERROR: smoke binary not found under ${BUILD}" >&2
  exit 1
fi
