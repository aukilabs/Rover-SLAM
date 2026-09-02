#!/usr/bin/env bash
set -euo pipefail

SRC="${1:-/mnt/d/agent-workspaces/Rover-SLAM}"
DST="${2:-${HOME}/rover-slam-work}"

mkdir -p "${DST}/third_party" "${DST}/cmake/modules"

echo "=== Copying Light_Glue_CPP + cmake modules ==="
cp -a "${SRC}/third_party/Light_Glue_CPP" "${DST}/third_party/"
cp -a "${SRC}/cmake/modules/"*.cmake "${DST}/cmake/modules/"
cp "${SRC}/CMakeLists.txt" "${DST}/"

echo "=== Copying src, include, Thirdparty ==="
mkdir -p "${DST}/src" "${DST}/include" "${DST}/Thirdparty" "${DST}/cmake_modules"
cp -a "${SRC}/src/." "${DST}/src/"
cp -a "${SRC}/include/." "${DST}/include/"
cp -a "${SRC}/Thirdparty/." "${DST}/Thirdparty/"
cp -a "${SRC}/cmake_modules/." "${DST}/cmake_modules/" 2>/dev/null || true
cp "${SRC}/Examples/Monocular/smoke_aliked_lightglue.cc" "${DST}/Examples/Monocular/"
cp -a "${SRC}/scripts/"*.sh "${DST}/scripts/"

# Test frames for smoke (small subset)
for f in frame_100_original.png frame_101_original.png; do
  if [ -f "${SRC}/${f}" ]; then
    cp "${SRC}/${f}" "${DST}/"
  fi
done

echo "=== Verify ==="
ls -la "${DST}/third_party/Light_Glue_CPP/models/aliked-n16.pt"
ls -la "${DST}/include/Extractors/AlikedExtractor.h"
echo "SYNC_OK"
