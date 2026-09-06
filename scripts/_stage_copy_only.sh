#!/usr/bin/env bash
set -euo pipefail
SRC="${HOME}/build/holdout_pose_eval_sync_noretry"
DST="/mnt/d/agent-workspaces/Rover-SLAM/out/holdout_pose_eval"
cp -f "$SRC/holdout_pose_metrics.json" "$SRC/holdout_pose_topdown.png" "$SRC/holdout_pose_rot_rel.png" "$SRC/trajectory_all_maps.json" "$DST/"
ls -la "$DST"
echo STAGE_OK
