#!/usr/bin/env bash
# Slice D: holdout replay with real prior pack + portal QR detection (no fixture smoke).
set -euo pipefail

ROOT=/mnt/d/agent-workspaces/Rover-SLAM
PACK="${PACK:-$ROOT/out/prior_pack_map_refined}"
LINUX_PACK="${LINUX_PACK:-$HOME/build/prior_pack_map_refined}"
HOLD="${HOLD:-/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/datasets/2026-04-02_13-42-53/Frames}"
PORT="${PORT:-18080}"
LOG="${LOG:-$HOME/build/rover-slam-holdout-prior-pack.log}"
STATUS_OUT="${STATUS_OUT:-$HOME/build/holdout_prior_pack_status.json}"
EVIDENCE_OUT="${EVIDENCE_OUT:-$HOME/build/holdout_prior_pack_evidence.json}"
REPLAY_LOG="${REPLAY_LOG:-$HOME/build/holdout_prior_pack_replay.log}"

export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
export CMAKE_PREFIX_PATH="${HOME}/.local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PATH="/usr/local/cuda/bin:${HOME}/bin:${PATH}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true
unset ROVER_PRIOR_SOFT_EDGE_FIXTURE_SMOKE || true

mkdir -p "$HOME/build"
cd "$ROOT"

if [[ ! -d "$PACK" ]]; then
  echo "ERROR: prior pack missing: $PACK"
  exit 1
fi
if [[ ! -d "$HOLD" ]]; then
  echo "ERROR: holdout Frames missing: $HOLD"
  exit 1
fi

echo "=== prepare pack for slam_server (Linux fs + uncompressed npz) ==="
python3 "$ROOT/scripts/ensure_prior_pack_for_slam_server.py" --src "$PACK" --dst "$LINUX_PACK"
PACK="$LINUX_PACK"

if [[ ! -d "$PACK" ]]; then
  echo "ERROR: prepared prior pack missing: $PACK"
  exit 1
fi

echo "=== stop old slam_server on :$PORT ==="
if pid=$(ss -ltnp 2>/dev/null | awk -v p=":$PORT" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1); then
  echo "killing pid $pid"
  kill "$pid" || true
  for _ in $(seq 1 30); do
    ss -ltn | grep -q ":$PORT" || break
    sleep 0.5
  done
  if ss -ltn | grep -q ":$PORT"; then
    kill -9 "$pid" || true
    sleep 1
  fi
else
  echo "no listener on :$PORT"
fi

BIN="$ROOT/Examples/Monocular/slam_server"
if [[ ! -x "$BIN" ]]; then
  echo "ERROR: missing $BIN"
  exit 1
fi

echo "=== prior pack ==="
python3 - <<PY
import json
from pathlib import Path
pack = Path("$PACK")
manifest = json.loads((pack / "manifest.json").read_text())
portals = json.loads((pack / "portals.json").read_text())["portals"]
print("loaded", manifest.get("version"), "images", manifest.get("counts", {}).get("images"))
print("portals", [p["shortId"] for p in portals])
PY

echo "=== start slam_server (real pack, portal detect every frame) ==="
: > "$LOG"
./Examples/Monocular/slam_server \
  Vocabulary/voc_binary_tartan_8u_6.yml.gz \
  Examples/Monocular/iphoneSLAM.yaml \
  --prior-pack "$PACK" \
  --portal-detect-every 1 \
  --prior-match-every 10 \
  --port "$PORT" \
  --max-queue 100 \
  >"$LOG" 2>&1 &
SPID=$!
echo "started pid=$SPID log=$LOG"

echo "=== wait for /status ==="
ok=0
for i in $(seq 1 90); do
  if curl -sf -m 2 "http://127.0.0.1:$PORT/api/v1/status" >/tmp/slam_boot_prior.json; then
    if python3 -c 'import json; d=json.load(open("/tmp/slam_boot_prior.json")); pp=d.get("prior_pack",{}); raise SystemExit(0 if pp.get("loaded") else 1)'; then
      echo "ready with prior_pack after ${i}s"
      ok=1
      break
    fi
  fi
  sleep 1
done
if [[ $ok -eq 0 ]]; then
  echo "ERROR: server did not become ready"
  tail -80 "$LOG" || true
  exit 1
fi

python3 - <<'PY'
import json
d=json.load(open("/tmp/slam_boot_prior.json"))
pp=d.get("prior_pack",{})
al=d.get("alignment",{})
print("prior_pack.loaded", pp.get("loaded"))
print("prior_pack.counts", pp.get("counts"))
print("alignment", json.dumps(al, indent=2))
PY

N=$(find "$HOLD" -maxdepth 1 -type f \( -name '*.jpg' -o -name '*.png' \) | wc -l)
echo "=== replay holdout frames N=$N @ 10 FPS (raw + intrinsics) ==="
python3 Examples/Monocular/send_frame.py "$HOLD" \
  --server "http://127.0.0.1:$PORT" \
  --fps 10 \
  --raw \
  --intrinsics 1335.984 1335.984 963.1064 716.9096 \
  2>&1 | tee "$REPLAY_LOG"

echo "=== post-replay /status ==="
curl -sf -m 5 "http://127.0.0.1:$PORT/api/v1/status" -o "$STATUS_OUT"

python3 - <<PY
import json, re, sys
from pathlib import Path

status_path = Path("$STATUS_OUT")
log_path = Path("$LOG")
d = json.loads(status_path.read_text())
pp = d.get("prior_pack", {})
al = d.get("alignment", {})
ov = d.get("overlay", {})

portal_obs = al.get("portal_observations", 0)
aligned = al.get("aligned", False)
active_pinned = ov.get("active_map_pinned", False)
soft_edges = ov.get("prior_soft_edges_accepted", 0)
pinned_map_ids = ov.get("pinned_map_ids") or []
pinned_maps = [m for m in ov.get("maps", []) if m.get("pinned")]
overlay_portal_obs = max(
    (int(m.get("portal_factor_count", 0)) for m in pinned_maps), default=0
)

log_text = log_path.read_text(errors="replace")
portal_lines = [ln for ln in log_text.splitlines() if "[portal]" in ln]
aligned_lines = [ln for ln in log_text.splitlines() if "[portal] aligned via" in ln]
prior_lines = [ln for ln in log_text.splitlines() if "[prior] soft edge accepted" in ln]
skip_lines = [ln for ln in log_text.splitlines() if "[prior] soft edge skip" in ln]

portal_obs_effective = max(portal_obs, overlay_portal_obs, len(aligned_lines))
domain_pinned_effective = bool(
    aligned or active_pinned or pinned_map_ids or pinned_maps
)

print("total_requests", d.get("total_requests"))
print("successful_tracks", d.get("successful_tracks"))
print("failed_tracks", d.get("failed_tracks"))
print("prior_pack.loaded", pp.get("loaded"))
print("alignment.status", al.get("status"))
print("alignment.aligned", aligned)
print("alignment.portal_observations", portal_obs)
print("overlay.portal_factor_count_max", overlay_portal_obs)
print("log.portal_aligned_lines", len(aligned_lines))
print("portal_observations_effective", portal_obs_effective)
print("domain_pinned_effective", domain_pinned_effective)
print("alignment.last_portal_short_id", al.get("last_portal_short_id"))
print("alignment.scale", al.get("scale"))
print("alignment.last_reprojection_error_px", al.get("last_reprojection_error_px"))
print("overlay.active_map_pinned", active_pinned)
print("overlay.prior_soft_edges_accepted", soft_edges)
print("overlay.pinned_map_ids", ov.get("pinned_map_ids"))

evidence = {
    "status_path": str(status_path),
    "log_path": str(log_path),
    "checks": {
        "prior_pack_loaded": pp.get("loaded") is True,
        "portal_observations_ge_1": portal_obs_effective >= 1,
        "domain_pinned": domain_pinned_effective,
        "prior_soft_edges_accepted_gt_0": soft_edges > 0,
    },
    "portal_observations_effective": portal_obs_effective,
    "domain_pinned_effective": domain_pinned_effective,
    "overlay_pinned_maps": pinned_maps,
    "alignment": al,
    "overlay": ov,
    "prior_pack": pp,
    "portal_log_tail": portal_lines[-20:],
    "aligned_log_tail": aligned_lines[-10:],
    "prior_soft_edge_log_tail": prior_lines[-10:],
    "prior_soft_edge_skip_tail": skip_lines[-15:],
}
out_path = Path("$EVIDENCE_OUT")
out_path.write_text(json.dumps(evidence, indent=2))
print("EVIDENCE", out_path)

failed = []
if not evidence["checks"]["prior_pack_loaded"]:
    failed.append("prior_pack not loaded")
if not evidence["checks"]["portal_observations_ge_1"]:
    failed.append("portal_observations < 1 (no real QR pin)")
if not evidence["checks"]["domain_pinned"]:
    failed.append("not domain-pinned (alignment.aligned and overlay.active_map_pinned both false)")
if not evidence["checks"]["prior_soft_edges_accepted_gt_0"]:
    failed.append("prior_soft_edges_accepted == 0")

if failed:
    print("ASSERT_FAIL:", "; ".join(failed))
    sys.exit(2)

print("ASSERT_OK: portal pin + prior soft edges validated")
PY

echo "DONE status=$STATUS_OUT evidence=$EVIDENCE_OUT"
