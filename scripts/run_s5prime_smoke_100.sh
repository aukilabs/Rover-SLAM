#!/usr/bin/env bash
# S5′ revise smoke: first 100 holdout frames, check prior_soft_edges_accepted > 0.
set -euo pipefail

ROOT=/mnt/d/agent-workspaces/Rover-SLAM
PACK="${PACK:-$ROOT/out/prior_pack_map_refined}"
LINUX_PACK="${LINUX_PACK:-$HOME/build/prior_pack_map_refined}"
HOLD="${HOLD:-/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/datasets/2026-04-02_13-42-53/Frames}"
PORT="${PORT:-18079}"
LOG="${LOG:-$HOME/build/rover-slam-s5prime-smoke.log}"
OUT="${OUT:-$ROOT/out/holdout_prior_pack_replay_s5prime_smoke}"
MAX_FRAMES="${MAX_FRAMES:-100}"

export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
export CMAKE_PREFIX_PATH="${HOME}/.local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PATH="/usr/local/cuda/bin:${HOME}/bin:${PATH}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true
unset ROVER_PRIOR_SOFT_EDGE_FIXTURE_SMOKE || true

mkdir -p "$OUT"
cd "$ROOT"
python3 "$ROOT/scripts/ensure_prior_pack_for_slam_server.py" --src "$PACK" --dst "$LINUX_PACK"

if pid=$(ss -ltnp 2>/dev/null | awk -v p=":$PORT" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1); then
  kill "$pid" || true
  sleep 1
fi

: > "$LOG"
"$ROOT/Examples/Monocular/slam_server" \
  Vocabulary/voc_binary_tartan_8u_6.yml.gz \
  Examples/Monocular/iphoneSLAM.yaml \
  --prior-pack "$LINUX_PACK" \
  --portal-detect-every 1 \
  --prior-match-every 10 \
  --port "$PORT" \
  --max-queue 100 >"$LOG" 2>&1 &
SPID=$!

ok=0
for i in $(seq 1 90); do
  if curl -sf -m 2 "http://127.0.0.1:$PORT/api/v1/status" >/tmp/slam_boot_smoke.json; then
    if python3 -c 'import json; d=json.load(open("/tmp/slam_boot_smoke.json")); raise SystemExit(0 if d.get("prior_pack",{}).get("loaded") else 1)'; then
      ok=1
      break
    fi
  fi
  sleep 1
done
if [[ $ok -eq 0 ]]; then
  echo "ERROR: server not ready"
  tail -40 "$LOG" || true
  exit 1
fi

SMOKE_DIR=$(mktemp -d)
mapfile -t SMOKE_FRAMES < <(find "$HOLD" -maxdepth 1 -type f \( -name '*.jpg' -o -name '*.png' \) | sort | head -n "$MAX_FRAMES")
for f in "${SMOKE_FRAMES[@]}"; do
  ln -sf "$f" "$SMOKE_DIR/$(basename "$f")"
done
echo "smoke frames: ${#SMOKE_FRAMES[@]}"

python3 "$ROOT/Examples/Monocular/send_frame.py" "$SMOKE_DIR" \
  --server "http://127.0.0.1:$PORT" \
  --fps 10 \
  --raw \
  --intrinsics 1335.984 1335.984 963.1064 716.9096

curl -sf -m 5 "http://127.0.0.1:$PORT/api/v1/status" -o "$OUT/holdout_prior_pack_status.json"
kill "$SPID" || true
cp "$LOG" "$OUT/rover-slam-holdout-prior-pack.log"

python3 - <<PY
import json
from pathlib import Path
log = Path("$LOG").read_text(errors="replace")
status = json.loads(Path("$OUT/holdout_prior_pack_status.json").read_text())
ov = status.get("overlay", {})
accepted = ov.get("prior_soft_edges_accepted", 0)
rejected = ov.get("prior_soft_edges_rejected", 0)
accept_lines = [ln for ln in log.splitlines() if "[prior] soft edge accepted" in ln]
skip_lines = [ln for ln in log.splitlines() if "[prior] soft edge skip" in ln]
print("prior_soft_edges_accepted", accepted)
print("prior_soft_edges_rejected", rejected)
print("accept_log_lines", len(accept_lines))
for ln in accept_lines[:5]:
    print(ln)
for ln in skip_lines[:3]:
    print(ln)
for ln in log.splitlines():
    if "Feature extraction resolution" in ln or "SLAM input resolution" in ln:
        print(ln)
if accepted <= 0:
    raise SystemExit(2)
PY

rm -rf "$SMOKE_DIR"
echo "SMOKE_OK accepted=$accepted"
