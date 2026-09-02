#!/usr/bin/env bash
set -euo pipefail

LT="${HOME}/libtorch-cu129"

if [ ! -f "${LT}/lib/libtorch.so" ] || [ "$(wc -c < "${LT}/lib/libtorch.so")" -lt 1000000 ]; then
  rm -rf "${LT}" "${HOME}/libtorch-dl"
  mkdir -p "${HOME}/libtorch-dl"
  cd "${HOME}/libtorch-dl"
  echo "Downloading libtorch cu129..."
  curl -L -o libtorch.zip "https://download.pytorch.org/libtorch/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcu128.zip"
  unzip -q libtorch.zip
  mv libtorch "${LT}"
  echo "DOWNLOAD_DONE"
else
  echo "DOWNLOAD_SKIP"
fi

ls -la "${LT}/share/cmake/Torch/TorchConfig.cmake"
wc -c < "${LT}/lib/libtorch.so"
