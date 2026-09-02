#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="${HOME}/build/rover-dmt-clean.log"
BUILD="${ROOT}/build"
JOBS="${JOBS:-2}"
LIBTORCH_ROOT="${LIBTORCH_ROOT:-${HOME}/build/libtorch-cu128}"

mkdir -p "${HOME}/build"
export LIBTORCH_ROOT
export CMAKE_PREFIX_PATH="${HOME}/.local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PATH="/usr/local/cuda/bin:${HOME}/bin:${PATH}"

{
  echo "=== START $(date -Iseconds) ==="
  echo "ROOT=${ROOT}"
  echo "LIBTORCH_ROOT=${LIBTORCH_ROOT}"
  echo "JOBS=${JOBS}"

  rm -rf "${BUILD}"
  mkdir -p "${BUILD}"
  cd "${BUILD}"

  echo "=== CMAKE ==="
  cmake -S "${ROOT}" -B . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="${HOME}/.local" \
    -DLIBTORCH_ROOT="${LIBTORCH_ROOT}"

  echo "=== BUILD mono_inertial_dmt JOBS=${JOBS} ==="
  cmake --build . --target mono_inertial_dmt -j"${JOBS}"

  echo "=== DONE $(date -Iseconds) ==="
  ls -la "${ROOT}/Examples/Monocular-Inertial/mono_inertial_dmt" "${ROOT}/lib/libORB_SLAM3.so"
} 2>&1 | tee "${LOG}"
