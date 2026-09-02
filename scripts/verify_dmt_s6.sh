#!/usr/bin/env bash
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
LIBTORCH_ROOT="${LIBTORCH_ROOT:-${HOME}/build/libtorch-cu128}"
ONNX="${ROOT}/onnxruntime-linux-x64-gpu-1.16.3/lib"
SCAN="${ROOT}/datasets/dmt_euroc/85a134bb-ecbd-40c0-bff8-e0381f6ef580/2024-12-08_19-32-19"
OUT="${ROOT}/outputs/dmt_offline_s6_verify_$(date +%Y%m%d_%H%M%S)"
LOG="${HOME}/build/rover-dmt-s6-verify.log"

export DISPLAY="${DISPLAY:-:0}"
export LIBGL_ALWAYS_SOFTWARE=1
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"

mkdir -p "${OUT}" "${HOME}/build"
cd "${ROOT}"

set +e
timeout 240 ./Examples/Monocular-Inertial/mono_inertial_dmt \
  Vocabulary/voc_binary_tartan_8u_6.yml.gz \
  ./Examples/Monocular-Inertial/DMT.yaml \
  "${SCAN}" \
  "${OUT}" 2>&1 | tee "${LOG}"
EXIT=${PIPESTATUS[0]}
set -e

echo "=== exit code: ${EXIT} ==="
if grep -q "double free or corruption" "${LOG}"; then
  echo "FAIL: double-free detected"
  exit 1
fi
if grep -q "BA: End of the optimization" "${LOG}"; then
  echo "PASS: BA completed without g2o double-free"
else
  echo "WARN: BA end message not seen (timeout or early exit ${EXIT})"
  exit 1
fi
