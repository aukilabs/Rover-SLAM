#!/usr/bin/env bash
# Slice E (fuse-prior-into-ba): holdout replay + domain ATE vs overlay baseline.
# Atlas domainized: raw trajectory pose_source=slam (identity vs GT).
# Overlay-Sim3 baseline: historical S9 from --baseline-metrics (same-run aligned=1
#   has pose_source=slam after domainize → overlay_sim3_direct matched=0).
# Exit codes: 2=plumbing, 3=meters-scale domain ATE, 4=active-map worse than S9.
set -euo pipefail

ROOT=/mnt/d/agent-workspaces/Rover-SLAM
PACK_SRC="${PACK_SRC:-$ROOT/out/prior_pack_map_refined}"
PACK="${PACK:-$HOME/build/prior_pack_map_refined}"
PORT="${PORT:-18080}"
OUT="${OUT:-$ROOT/out/holdout_fuse_ba_verify}"
BASELINE_METRICS="${BASELINE_METRICS:-$ROOT/out/holdout_pose_eval/compare_summary.json}"
LOG="${LOG:-$HOME/build/rover-slam-fuse-ba-holdout.log}"
JOB=/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete
GT="${GT:-$JOB/refined/global/refined_sfm_combined}"
SCAN=2026-04-02_13-42-53
MAX_FRAMES="${MAX_FRAMES:-0}"
EVAL_PORT="${EVAL_PORT:-8081}"

export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
export PATH="/usr/local/cuda/bin:${HOME}/bin:${PATH}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true
unset ROVER_PRIOR_SOFT_EDGE_FIXTURE_SMOKE || true

mkdir -p "$OUT" "$HOME/build"
cd "$ROOT"

stop_port() {
  local pid
  pid=$(ss -ltnp 2>/dev/null | awk -v p=":$1" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1 || true)
  if [[ -n "${pid:-}" ]]; then
    echo "killing pid $pid on :$1"
    kill "$pid" 2>/dev/null || true
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
  fi
}

echo "=== prepare prior pack ==="
python3 "$ROOT/scripts/ensure_prior_pack_for_slam_server.py" --src "$PACK_SRC" --dst "$PACK"

stop_port "$PORT"
pkill -f 'send_frame.py' 2>/dev/null || true
sleep 1

BIN="$ROOT/Examples/Monocular/slam_server"
if [[ ! -x "$BIN" ]]; then
  echo "ERROR: missing $BIN — run scripts/_build_slam_server.sh"
  exit 1
fi

: > "$LOG"
echo "=== start slam_server (portal detect every frame) ==="
"$BIN" \
  Vocabulary/voc_binary_tartan_8u_6.yml.gz \
  Examples/Monocular/iphoneSLAM.yaml \
  --prior-pack "$PACK" \
  --portal-detect-every 1 \
  --prior-match-every 10 \
  --port "$PORT" \
  --max-queue 200 \
  >"$LOG" 2>&1 &
SPID=$!
echo "started pid=$SPID log=$LOG"

ok=0
for i in $(seq 1 300); do
  if curl -sf -m 5 "http://127.0.0.1:$PORT/api/v1/status" >/tmp/slam_boot_fuse_ba.json 2>/dev/null; then
    if python3 -c 'import json; d=json.load(open("/tmp/slam_boot_fuse_ba.json")); ok=d.get("prior_pack",{}).get("loaded") or d.get("prior_pack_loaded"); raise SystemExit(0 if ok and "timing" in d else 1)'; then
      echo "ready after ${i}s"
      ok=1
      break
    fi
  fi
  if ! kill -0 "$SPID" 2>/dev/null; then
    echo "slam_server died"; tail -50 "$LOG"; exit 1
  fi
  sleep 1
done
[[ $ok -eq 1 ]] || { echo "not ready"; tail -50 "$LOG"; exit 1; }

echo "=== holdout replay (MAX_FRAMES=${MAX_FRAMES:-all}) ==="
cd "$ROOT/scripts"
python3 - <<PY
import eval_holdout_pose_accuracy as ev
from pathlib import Path
import os, shutil, tempfile

job = Path("$JOB")
scan = "$SCAN"
frames_dir = job / "datasets" / scan / "Frames"
ts = ev.load_frames_csv(job / "datasets" / scan / "Frames.csv")
intr_csv = job / "datasets" / scan / "CameraIntrinsics.csv"
intr = (
    ev.load_camera_intrinsics_csv(intr_csv)
    if intr_csv.is_file()
    else [1335.984, 1335.984, 963.1064, 716.9096]
)
images = ev.list_images(frames_dir)
max_f = int("$MAX_FRAMES")
if max_f > 0:
    images = images[:max_f]
    ts = ts[:max_f]
    if isinstance(intr[0], (list, tuple)):
        intr = intr[:max_f]
    tmpdir = Path(tempfile.mkdtemp())
    for p in images:
        os.symlink(p, tmpdir / p.name)
    replay_dir = tmpdir
else:
    replay_dir = frames_dir
print(f"replaying {len(images)} frames @ 10 FPS from {replay_dir}")
ev.replay("http://127.0.0.1:$PORT", replay_dir, ts, intr, 10.0)
if max_f > 0:
    shutil.rmtree(replay_dir)
print("REPLAY_DONE")
PY
cd "$ROOT"

echo "=== drain queue ==="
stable=0
prev_tracked=""
for i in $(seq 1 900); do
  if st=$(curl -sf -m 2 "http://127.0.0.1:$PORT/api/v1/status"); then
    qs=$(python3 -c 'import json,sys; print(json.load(sys.stdin).get("queue_size", -1))' <<<"$st")
    tracked=$(python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("successful_tracks", 0))' <<<"$st")
    echo "drain t=${i}s queue=$qs tracked=$tracked"
    if [[ "$qs" == "0" ]]; then
      sleep 3
      break
    fi
    if [[ "$tracked" == "$prev_tracked" && "$qs" -gt 0 && "$qs" -lt 3 ]]; then
      stable=$((stable + 1))
      [[ "$stable" -ge 5 ]] && { sleep 3; break; }
    else
      stable=0
    fi
    prev_tracked=$tracked
  fi
  sleep 1
done

sleep 2
curl -sf -m 10 "http://127.0.0.1:$PORT/api/v1/status" -o "$OUT/status.json"
curl -sf -m 60 "http://127.0.0.1:$PORT/api/v1/trajectory?aligned=0&dense=1&all_maps=1" -o "$OUT/trajectory_raw.json"
curl -sf -m 60 "http://127.0.0.1:$PORT/api/v1/trajectory?aligned=1&dense=1&all_maps=1" -o "$OUT/trajectory_overlay.json"
cp -f "$LOG" "$OUT/rover-slam-holdout.log"

echo "=== evaluate atlas domain vs overlay baseline ==="
cd "$ROOT/scripts"
EVAL_RC=0
python3 eval_portal_domain_vs_vo.py \
  --raw-traj "$OUT/trajectory_raw.json" \
  --domain-traj "$OUT/trajectory_raw.json" \
  --overlay-traj "$OUT/trajectory_overlay.json" \
  --status "$OUT/status.json" \
  --baseline-metrics "$BASELINE_METRICS" \
  --gt-colmap "$GT" \
  --gt-name-contains "$SCAN" \
  --atlas-domain \
  --out-dir "$OUT" || EVAL_RC=$?
cd "$ROOT"

python3 - <<PY
import json, sys
from pathlib import Path

out = Path("$OUT")
status = json.loads((out / "status.json").read_text())
cmp = json.loads((out / "compare_summary.json").read_text())
evidence = json.loads((out / "evidence.json").read_text()) if (out / "evidence.json").is_file() else {}
log = (out / "rover-slam-holdout.log").read_text(errors="replace")
eval_rc = int("$EVAL_RC")

print("=== FUSE-BA SLICE E RESULTS ===")
print("domainized", status.get("domainized"))
print("portal_ba_edges", status.get("portal_ba_edges"))
print("atlas_domain_pooled", cmp.get("atlas_domain_direct"))
print("atlas_domain_map0", cmp.get("atlas_domain_direct_map0"))
print("atlas_domain_active", cmp.get("atlas_domain_direct_active_map"))
print("overlay_sim3_same_run", cmp.get("overlay_sim3_direct"))
print("s9_overlay_baseline", cmp.get("s9_overlay_domain_baseline"))
print("vo_umeyama", cmp.get("vo_umeyama"))
print("checks", evidence.get("checks"))
print("overlay_baseline", evidence.get("overlay_baseline"))
print("accuracy_pass", evidence.get("accuracy_pass"))
domain_lines = [ln for ln in log.splitlines() if "[domainize]" in ln]
print("domainize_log_lines", len(domain_lines))
for ln in domain_lines[:5]:
    print(" ", ln)

checks = evidence.get("checks") or {}
failed = []
if not checks.get("prior_pack_loaded"):
    failed.append("prior_pack not loaded")
if not checks.get("domainized"):
    failed.append("domainized != true")
if checks.get("portal_observations_ge_1") is False:
    failed.append("no portal observations")
pba = (status.get("portal_ba_edges") or {}).get("map", 0)
if pba <= 0:
    failed.append(f"portal_ba_edges.map={pba} (expected >0 after portals)")
if checks.get("domain_ate_not_meters_scale") is False:
    per_map = evidence.get("domain_ate_per_map_median_m") or {}
    failed.append(
        "meters-scale domain ATE "
        f"(pooled={per_map.get('pooled_domainized_maps')}, "
        f"map0={per_map.get('map0')}, active={per_map.get('active_map')})"
    )
if checks.get("domain_active_map_not_worse_than_s9_overlay") is False:
    active = (cmp.get("atlas_domain_direct_active_map") or {}).get("ate_median_m")
    s9 = (cmp.get("s9_overlay_domain_baseline") or {}).get("ate_median_m")
    failed.append(f"active-map domain {active:.3f}m worse than S9 overlay {s9:.3f}m")

if failed:
    print("ASSERT_FAIL:", "; ".join(failed))
else:
    print("ASSERT_OK (plumbing only; accuracy_pass=false)")

# Propagate eval script exit (2=plumbing, 3=meters-scale, 4=worse-than-S9).
sys.exit(eval_rc if eval_rc else (2 if failed and any(
    "domainized" in f or "portal" in f or "prior_pack" in f for f in failed
) else (3 if checks.get("domain_ate_not_meters_scale") is False else (
    4 if checks.get("domain_active_map_not_worse_than_s9_overlay") is False else 0
))))
PY

# Optional glance server
stop_port "$EVAL_PORT"
nohup python3 "$ROOT/scripts/serve_holdout_pose_eval.py" \
  --host 0.0.0.0 --port "$EVAL_PORT" --dir "$OUT" \
  >"$HOME/build/fuse_ba_eval_serve.log" 2>&1 &
sleep 0.5
curl -sf -m 2 -o /dev/null -w "EVAL_HTTP:%{http_code}\n" "http://127.0.0.1:$EVAL_PORT/" || true

echo "DONE out=$OUT viewer=http://127.0.0.1:$EVAL_PORT/ evidence=$OUT/evidence.json"
