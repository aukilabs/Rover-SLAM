#!/usr/bin/env bash
# A/B: motion-model LightGlue input top-k (extract unchanged).
# Usage: scripts/_ab_lg_mm_topk.sh [512]
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
BUILD="$ROOT/build"
TOPK="${1:-512}"
export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true

cd "$BUILD"
echo "=== build slam_server ==="
cmake --build . --target slam_server -j"$(nproc)" 2>&1 | tee /tmp/slam_server_lg_mm_topk_build.log | tail -40
strings "$ROOT/Examples/Monocular/slam_server" | grep -E 'ROVER_LG_MM_TOPK|MM LightGlue' | head -5 || true

run_mode() {
  local label="$1"
  local topk_env="${2:-}"
  local out="$HOME/build/holdout_pose_eval_${label}"
  local status_out="$HOME/build/holdout_timing_${label}.json"
  mkdir -p "$out"

  if [[ -n "$topk_env" ]]; then
    export ROVER_LG_MM_TOPK="$topk_env"
    echo "=== mode=$label ROVER_LG_MM_TOPK=$ROVER_LG_MM_TOPK ==="
  else
    unset ROVER_LG_MM_TOPK || true
    echo "=== mode=$label ROVER_LG_MM_TOPK=<unset> (full extract kpts into LG) ==="
  fi

  sed -i 's/\r$//' "$ROOT/scripts/eval_holdout_pose_accuracy.py"
  python3 "$ROOT/scripts/eval_holdout_pose_accuracy.py" --port 18080 --fps 10 \
    --out-dir "$out"

  # Capture timing from the still-running server started by the eval script.
  curl -sf -m 5 "http://127.0.0.1:18080/api/v1/status" -o "$status_out" || true
  echo "--- timing summary ($label) ---"
  python3 - <<PY
import json
from pathlib import Path
p = Path("$status_out")
if not p.exists():
    print("no status file")
else:
    d = json.loads(p.read_text())
    t = d.get("timing") or {}
    print("successful_tracks", d.get("successful_tracks"), "failed_tracks", d.get("failed_tracks"))
    print("p50", json.dumps(t.get("p50_ms"), indent=2))
    print("p95", json.dumps(t.get("p95_ms"), indent=2))
    last = (t.get("last_frame") or {})
    print("last lg_kpts", last.get("lg_kpts0"), last.get("lg_kpts1"),
          "pose_mm_match", last.get("pose_mm_match_ms"),
          "lg_fwd", last.get("lg_fwd_ms"))
m = json.loads(Path("$out/holdout_pose_metrics.json").read_text())
print("pose metrics file:", "$out/holdout_pose_metrics.json")
# print compact
keys = [k for k in m if any(x in k.lower() for x in ("ate","rot","match","rmse","median","mean","n_"))]
print(json.dumps({k: m[k] for k in keys}, indent=2)[:3000])
PY

  # Stop server before next mode
  if pid=$(ss -ltnp | awk -v p=":18080" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1); then
    echo "stopping server pid=$pid"
    kill "$pid" || true
    sleep 2
  fi
}

run_mode "full" ""
run_mode "topk${TOPK}" "$TOPK"

echo "=== A/B DONE ==="
echo "Results: $HOME/build/holdout_pose_eval_full  and  $HOME/build/holdout_pose_eval_topk${TOPK}"
echo "Timing:  $HOME/build/holdout_timing_full.json  and  $HOME/build/holdout_timing_topk${TOPK}.json"
