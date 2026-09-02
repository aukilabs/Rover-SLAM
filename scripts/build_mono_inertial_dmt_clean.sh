#!/usr/bin/env bash
set -euo pipefail

PREFIX="${HOME}/.local"
YAML_CMAKE="${PREFIX}/lib/cmake/yaml-cpp/yaml-cpp-config.cmake"

install_yaml_cpp() {
  if [[ -f "${YAML_CMAKE}" ]]; then
    echo "yaml-cpp already at ${PREFIX}"
    return 0
  fi
  echo "=== Building yaml-cpp into ${PREFIX} ==="
  rm -rf /tmp/yaml-cpp-build
  git clone --depth 1 --branch 0.8.0 https://github.com/jbeder/yaml-cpp.git /tmp/yaml-cpp-build
  cmake -S /tmp/yaml-cpp-build -B /tmp/yaml-cpp-build/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DYAML_BUILD_SHARED_LIBS=ON \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  cmake --build /tmp/yaml-cpp-build/build -j"${JOBS:-4}"
  cmake --install /tmp/yaml-cpp-build/build
}

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${ROOT}/build"
JOBS="${JOBS:-4}"

install_yaml_cpp

echo "=== Clean build: mono_inertial_dmt ==="
echo "ROOT=${ROOT}"

rm -rf "${BUILD}"
mkdir -p "${BUILD}"
cd "${BUILD}"

export CMAKE_PREFIX_PATH="${PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"

cmake -S "${ROOT}" -B . \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${PREFIX}"

cmake --build . --target mono_inertial_dmt -j"${JOBS}"

BIN="${ROOT}/Examples/Monocular-Inertial/mono_inertial_dmt"
echo "=== Done ==="
ls -la "${BIN}"
