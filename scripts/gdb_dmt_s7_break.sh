#!/usr/bin/env bash
set -eu
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
LIBTORCH_ROOT="${LIBTORCH_ROOT:-${HOME}/build/libtorch-cu128}"
ONNX="${ROOT}/onnxruntime-linux-x64-gpu-1.16.3/lib"
SCAN="${ROOT}/datasets/dmt_euroc/85a134bb-ecbd-40c0-bff8-e0381f6ef580/2024-12-08_19-32-19"
OUT="${ROOT}/outputs/dmt_gdb_break"
GDBSCRIPT="${HOME}/build/gdb_s7_break.gdb"

export ROVER_SLAM_HEADLESS=1
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"

cat > "${GDBSCRIPT}" <<EOF
set pagination off
set breakpoint pending on
break ORB_SLAM3::LoopClosing::NewDetectCommonRegions
commands
  silent
  printf "HIT NewDetectCommonRegions mbActiveLC=%d\n", mbActiveLC
  continue
end
break ORB_SLAM3::KeyFrameDatabase::add
commands
  silent
  printf "HIT KeyFrameDatabase::add\n"
  continue
end
break ORB_SLAM3::LoopClosing::DetectCommonRegionsFromBoW_sp
commands
  silent
  printf "HIT DetectCommonRegionsFromBoW_sp\n"
  continue
end
run Vocabulary/voc_binary_tartan_8u_6.yml.gz ./Examples/Monocular-Inertial/DMT.yaml ${SCAN} ${OUT}
thread apply all bt
quit
EOF

cd "${ROOT}"
gdb -batch -x "${GDBSCRIPT}" Examples/Monocular-Inertial/mono_inertial_dmt 2>&1 | tee "${HOME}/build/rover-dmt-s7-gdb-break.log"
grep -E 'HIT |SIGSEGV|BA:|mlNew' "${HOME}/build/rover-dmt-s7-gdb-break.log" || true
