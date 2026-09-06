#!/usr/bin/env bash
# Stage baseline vs new holdout A/B artifacts into out/holdout_pose_eval and serve on :8080.
# Live phone stream + timing/trajectory plots stay on slam_server /index.html (separate process/port).
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
BASE_SRC="${BASE_SRC:-$HOME/build/holdout_pose_eval_baseline_drain_retry}"
NEW_SRC="${NEW_SRC:-$HOME/build/holdout_pose_eval_sync_noretry}"
AGG_SRC="${AGG_SRC:-$HOME/build/holdout_ab_repeat/aggregate.txt}"
DST="$ROOT/out/holdout_pose_eval"
PORT="${PORT:-8080}"

if [[ ! -d "$BASE_SRC" ]]; then
  echo "missing baseline src: $BASE_SRC" >&2
  exit 1
fi
if [[ ! -d "$NEW_SRC" ]]; then
  echo "missing new src: $NEW_SRC" >&2
  exit 1
fi
if [[ ! -f "$DST/index.html" ]]; then
  echo "missing $DST/index.html" >&2
  exit 1
fi

mkdir -p "$DST/baseline" "$DST/new"
for f in holdout_pose_metrics.json holdout_pose_topdown.png holdout_pose_rot_rel.png trajectory_all_maps.json; do
  cp -f "$BASE_SRC/$f" "$DST/baseline/" 2>/dev/null || true
  cp -f "$NEW_SRC/$f" "$DST/new/" 2>/dev/null || true
done
# Keep root copies pointing at "new" for older links.
cp -f "$NEW_SRC/holdout_pose_metrics.json" "$DST/" 2>/dev/null || true
cp -f "$NEW_SRC/holdout_pose_topdown.png" "$DST/" 2>/dev/null || true
cp -f "$NEW_SRC/holdout_pose_rot_rel.png" "$DST/" 2>/dev/null || true
cp -f "$NEW_SRC/trajectory_all_maps.json" "$DST/" 2>/dev/null || true
if [[ -f "$AGG_SRC" ]]; then
  cp -f "$AGG_SRC" "$DST/aggregate.txt"
fi

echo "Staged baseline=$BASE_SRC new=$NEW_SRC -> $DST"
ls -la "$DST" "$DST/baseline" "$DST/new"

pkill -f serve_holdout_pose_eval.py 2>/dev/null || true
sleep 0.3

sed -i 's/\r$//' "$ROOT/scripts/serve_holdout_pose_eval.py"
exec python3 "$ROOT/scripts/serve_holdout_pose_eval.py" --host 0.0.0.0 --port "$PORT" --dir "$DST"
