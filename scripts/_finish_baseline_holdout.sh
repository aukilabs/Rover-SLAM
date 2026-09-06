#!/usr/bin/env bash
# Capture status from running slam_server and print baseline summary; stop port.
set -euo pipefail
STATUS="${HOME}/build/holdout_timing_baseline_drain_retry.json"
METRICS="${HOME}/build/holdout_pose_eval_baseline_drain_retry/holdout_pose_metrics.json"
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
curl -sf -m 5 "http://127.0.0.1:18080/api/v1/status" -o "$STATUS" && echo "STATUS_OK" || echo "STATUS_FAIL"
sed -i 's/\r$//' "$ROOT/scripts/_print_holdout_ab_summary.py"
python3 "$ROOT/scripts/_print_holdout_ab_summary.py" baseline "$METRICS" "$STATUS"
pid=$(ss -ltnp | awk -v p=":18080" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1 || true)
if [[ -n "${pid:-}" ]]; then
  echo "stopping pid=$pid"
  kill "$pid" || true
  sleep 1
fi
echo "BASELINE_SUMMARY_DONE"
