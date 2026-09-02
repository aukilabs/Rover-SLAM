#!/usr/bin/env bash
set -eu
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
LIBTORCH_ROOT="${LIBTORCH_ROOT:-${HOME}/build/libtorch-cu128}"
ONNX="${ROOT}/onnxruntime-linux-x64-gpu-1.16.3/lib"
SCAN="${ROOT}/datasets/dmt_euroc/85a134bb-ecbd-40c0-bff8-e0381f6ef580/2024-12-08_19-32-19"
OUT="${ROOT}/outputs/dmt_gdb_s7_$(date +%Y%m%d_%H%M%S)"
LOG="${HOME}/build/rover-dmt-s7-gdb.log"
BIN="${ROOT}/Examples/Monocular-Inertial/mono_inertial_dmt"

export DISPLAY="${DISPLAY:-:0}"
export ROVER_SLAM_HEADLESS=1
export LIBGL_ALWAYS_SOFTWARE=1
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"

mkdir -p "${OUT}" "${HOME}/build"
cd "${ROOT}"

gdb -batch \
  -ex "set pagination off" \
  -ex "run Vocabulary/voc_binary_tartan_8u_6.yml.gz ./Examples/Monocular-Inertial/DMT.yaml ${SCAN} ${OUT}" \
  -ex "thread apply all bt" \
  -ex "quit" \
  "${BIN}" 2>&1 | tee "${LOG}"

echo "=== GDB log: ${LOG} ==="
