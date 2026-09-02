set pagination off
set print thread-events off
break ORB_SLAM3::LoopClosing::NewDetectCommonRegions
commands
  silent
  printf "HIT NewDetectCommonRegions\n"
  continue
end
break ORB_SLAM3::LoopClosing::DetectCommonRegionsFromBoW_sp
commands
  silent
  printf "HIT DetectCommonRegionsFromBoW_sp\n"
  continue
end
run Vocabulary/voc_binary_tartan_8u_6.yml.gz ./Examples/Monocular-Inertial/DMT.yaml SCAN_PLACEHOLDER OUT_PLACEHOLDER
thread apply all bt
quit
