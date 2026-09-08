#!/usr/bin/env bash
# Full holdout prior-pack replay + portal-domain eval + refresh :8080 A/B page.
# Soft-edge gate revise (S5′b) — 2026-09-07.
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
PACK_SRC="${PACK_SRC:-$ROOT/out/prior_pack_map_refined}"
PACK="${PACK:-$HOME/build/prior_pack_map_refined}"
PORT="${PORT:-18080}"
OUT="${OUT:-$HOME/build/holdout_pose_eval_s5prime_softgate}"
OUT_BASE="${OUT_BASE:-$HOME/build/holdout_pose_eval_baseline_global_gt}"
EVIDENCE_DST="${EVIDENCE_DST:-$ROOT/out/holdout_prior_pack_replay_s5prime_verify}"
DST="$ROOT/out/holdout_pose_eval"
EVAL_PORT="${EVAL_PORT:-8080}"
LOG="$HOME/build/rover-slam-holdout-s5prime-softgate.log"
JOB=/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete
GT=$JOB/refined/global/refined_sfm_combined
SCAN=2026-04-02_13-42-53

export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
export PATH="/usr/local/cuda/bin:${HOME}/bin:${PATH}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true
unset ROVER_PRIOR_SOFT_EDGE_FIXTURE_SMOKE || true

mkdir -p "$OUT" "$EVIDENCE_DST" "$HOME/build"
cd "$ROOT"

stop() {
  local pid
  pid=$(ss -ltnp 2>/dev/null | awk -v p=":$1" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1 || true)
  if [[ -n "${pid:-}" ]]; then
    echo "killing pid $pid on :$1"
    kill "$pid" 2>/dev/null || true
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
  fi
}

echo "=== prepare pack ==="
python3 "$ROOT/scripts/ensure_prior_pack_for_slam_server.py" --src "$PACK_SRC" --dst "$PACK"

stop "$PORT"
pkill -f 'eval_holdout_pose_accuracy.py' 2>/dev/null || true
pkill -f 'send_frame.py' 2>/dev/null || true
pkill -f '_replay_holdout_timestamps.py' 2>/dev/null || true
sleep 1

BIN="$ROOT/Examples/Monocular/slam_server"
if [[ ! -x "$BIN" ]]; then
  echo "ERROR: missing $BIN"
  exit 1
fi

: > "$LOG"
echo "=== start slam_server ==="
"$BIN" \
  Vocabulary/voc_binary_tartan_8u_6.yml.gz \
  Examples/Monocular/iphoneSLAM.yaml \
  --prior-pack "$PACK" \
  --portal-detect-every 1 \
  --prior-match-every 5 \
  --port "$PORT" \
  --max-queue 200 \
  >"$LOG" 2>&1 &
SPID=$!
echo "started $SPID log=$LOG"

ok=0
for i in $(seq 1 180); do
  if curl -sf -m 2 "http://127.0.0.1:$PORT/api/v1/status" >/tmp/slam_boot_s5prime.json; then
    if python3 -c 'import json; d=json.load(open("/tmp/slam_boot_s5prime.json")); raise SystemExit(0 if d.get("prior_pack",{}).get("loaded") else 1)'; then
      echo "ready after ${i}s"
      ok=1
      break
    fi
  fi
  if ! kill -0 "$SPID" 2>/dev/null; then
    echo "died"; tail -40 "$LOG"; exit 1
  fi
  sleep 1
done
[[ $ok -eq 1 ]] || { echo "not ready"; tail -40 "$LOG"; exit 1; }

sed -i 's/\r$//' "$ROOT/scripts/"*.py "$ROOT/scripts/_holdout_portal_ab_index.html" || true

echo "=== timestamped holdout replay ==="
cd "$ROOT/scripts"
python3 - <<PY
import eval_holdout_pose_accuracy as ev
from pathlib import Path
job=Path("$JOB"); scan="$SCAN"
frames=job/"datasets"/scan/"Frames"
ts=ev.load_frames_csv(job/"datasets"/scan/"Frames.csv")
intr_csv=job/"datasets"/scan/"CameraIntrinsics.csv"
if intr_csv.is_file():
    intr=ev.load_camera_intrinsics_csv(intr_csv)
    print("loaded", len(intr), "per-frame intrinsics rows from", intr_csv)
else:
    intr=[1335.984,1335.984,963.1064,716.9096]
    print("WARNING: missing", intr_csv, "using fallback intrinsics")
print("replaying", len(ev.list_images(frames)))
ev.replay("http://127.0.0.1:$PORT", frames, ts, intr, 10.0)
print("REPLAY_DONE")
PY
cd "$ROOT"

echo "=== drain processing queue ==="
# Wait until image queue is empty (prior-match-every=1 can lag 10 FPS ingest by minutes).
for i in $(seq 1 900); do
  if st=$(curl -sf -m 2 "http://127.0.0.1:$PORT/api/v1/status"); then
    qs=$(python3 -c 'import json,sys; print(json.load(sys.stdin).get("queue_size", -1))' <<<"$st")
    tracked=$(python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("successful_tracks", d.get("tracked_frames", 0)))' <<<"$st")
    echo "drain t=${i}s queue_size=$qs tracked=$tracked"
    if [[ "$qs" == "0" ]]; then
      # brief slack so the last frame finishes overlay work
      sleep 3
      break
    fi
  else
    echo "drain: status not ready"
  fi
  sleep 1
done

sleep 2
curl -sf -m 10 "http://127.0.0.1:$PORT/api/v1/status" -o "$OUT/status.json"
curl -sf -m 60 "http://127.0.0.1:$PORT/api/v1/trajectory?aligned=0&dense=1&all_maps=1" -o "$OUT/trajectory_raw.json"
curl -sf -m 60 "http://127.0.0.1:$PORT/api/v1/trajectory?aligned=1&dense=1&all_maps=1" -o "$OUT/trajectory_domain.json"
cp -f "$LOG" "$OUT/rover-slam-holdout.log"

echo "=== evaluate portal-domain vs VO (global GT) ==="
cd "$ROOT/scripts"
python3 eval_portal_domain_vs_vo.py \
  --raw-traj "$OUT/trajectory_raw.json" \
  --domain-traj "$OUT/trajectory_domain.json" \
  --baseline-metrics "$OUT_BASE/holdout_pose_metrics.json" \
  --status "$OUT/status.json" \
  --gt-colmap "$GT" \
  --gt-name-contains "$SCAN" \
  --out-dir "$OUT"
cd "$ROOT"

python3 - <<PY
import json,re
from pathlib import Path
st=json.loads(Path("$OUT/status.json").read_text())
cmp=json.loads(Path("$OUT/compare_summary.json").read_text())
log=Path("$LOG").read_text(errors="replace")
print("=== S5prime SOFTGATE RESULTS ===")
print("map0_direct", cmp.get("portal_domain_direct_map0"))
print("overlay_direct", cmp.get("portal_domain_direct"))
print("vo_umeyama", cmp.get("vo_umeyama"))
ov=st.get("overlay") or {}
al=st.get("alignment") or {}
print("soft_edges_accepted", ov.get("prior_soft_edges_accepted"))
print("soft_edges_rejected", ov.get("prior_soft_edges_rejected"))
print("scale", al.get("scale"), al.get("scale_source"))
print("pinned", ov.get("pinned_map_ids"))
skips={}
for ln in log.splitlines():
    if "soft edge accepted" in ln:
        skips["ACCEPTED"]=skips.get("ACCEPTED",0)+1
    if "soft edge skip" in ln and ": " in ln:
        # reason may include extra fields; keep first token after colon if present
        part=ln.split("soft edge skip",1)[-1]
        if "no_gated_match" in part: skips["no_gated_match"]=skips.get("no_gated_match",0)+1
        elif "slam_disagreement" in part: skips["slam_disagreement"]=skips.get("slam_disagreement",0)+1
print("soft_edge_counts", skips)
print("unit_fallback_log", len(re.findall(r"unit_fallback", log)))
PY

echo "=== evidence copy ==="
cp -f "$OUT/status.json" "$OUT/compare_summary.json" "$OUT/trajectory_raw.json" "$OUT/trajectory_domain.json" \
  "$OUT/holdout_pose_metrics.json" "$OUT/holdout_pose_topdown.png" "$OUT/holdout_pose_rot_rel.png" \
  "$OUT/rover-slam-holdout.log" "$EVIDENCE_DST/" 2>/dev/null || true

echo "=== stage A/B page :$EVAL_PORT ==="
mkdir -p "$DST/baseline" "$DST/new"
cp -f "$OUT_BASE/"holdout_pose_* "$DST/baseline/" 2>/dev/null || true
cp -f "$OUT_BASE/trajectory_all_maps.json" "$DST/baseline/" 2>/dev/null || true
cp -f "$OUT/"holdout_pose_* "$OUT/status.json" "$OUT/compare_summary.json" "$DST/new/" 2>/dev/null || true
cp -f "$OUT/trajectory_domain.json" "$DST/new/trajectory_all_maps.json"
cp -f "$OUT/compare_summary.json" "$DST/"
cp -f "$OUT/holdout_pose_metrics.json" "$OUT/holdout_pose_topdown.png" "$OUT/holdout_pose_rot_rel.png" "$DST/"
cp -f "$OUT/trajectory_domain.json" "$DST/trajectory_all_maps.json"
cp -f "$ROOT/scripts/_holdout_portal_ab_index.html" "$DST/index.html"

stop "$EVAL_PORT"
pkill -f serve_holdout_pose_eval.py 2>/dev/null || true
sleep 0.3

# Prefer Tailscale IP on the Windows host so phones/other machines can open the page.
# WSL 0.0.0.0 is only reliably reachable via Windows localhost port-forward, not Tailscale.
TS_IP=""
if command -v tailscale >/dev/null 2>&1; then
  TS_IP=$(tailscale ip -4 2>/dev/null | head -1 || true)
fi
if [[ -z "${TS_IP}" && -x "/mnt/c/Program Files/Tailscale/tailscale.exe" ]]; then
  TS_IP=$(/mnt/c/Program\ Files/Tailscale/tailscale.exe ip -4 2>/dev/null | head -1 | tr -d '\r' || true)
fi

EVAL_HOST="${EVAL_HOST:-}"
if [[ -z "${EVAL_HOST}" ]]; then
  if [[ -n "${TS_IP}" ]]; then
    EVAL_HOST="$TS_IP"
  else
    EVAL_HOST="0.0.0.0"
  fi
fi

# Bind in Windows when serving a Tailscale address (WSL cannot own the TS iface).
SERVE_PID=""
if [[ -n "${TS_IP}" && "${EVAL_HOST}" == "${TS_IP}" && -x "/mnt/c/Users/robin/AppData/Local/hermes/hermes-agent/venv/Scripts/python.exe" ]]; then
  WIN_DIR=$(echo "$DST" | sed -e 's#^/mnt/\([a-z]\)/#\1:/#' -e 's#/#\\#g')
  WIN_PY='/mnt/c/Users/robin/AppData/Local/hermes/hermes-agent/venv/Scripts/python.exe'
  WIN_SCRIPT=$(echo "$ROOT/scripts/serve_holdout_pose_eval.py" | sed -e 's#^/mnt/\([a-z]\)/#\1:/#' -e 's#/#\\#g')
  mkdir -p "$HOME/build"
  nohup "$WIN_PY" "$WIN_SCRIPT" --host "$EVAL_HOST" --port "$EVAL_PORT" --dir "$WIN_DIR" \
    >"$HOME/build/holdout_eval_serve.log" 2>&1 &
  SERVE_PID=$!
else
  nohup python3 "$ROOT/scripts/serve_holdout_pose_eval.py" --host "${EVAL_HOST}" --port "$EVAL_PORT" --dir "$DST" \
    >"$HOME/build/holdout_eval_serve.log" 2>&1 &
  SERVE_PID=$!
fi
sleep 0.8
curl -sf -m 2 -o /dev/null -w "EVAL_HTTP:%{http_code}\n" "http://127.0.0.1:$EVAL_PORT/" \
  || curl -sf -m 2 -o /dev/null -w "EVAL_HTTP:%{http_code}\n" "http://${EVAL_HOST}:$EVAL_PORT/" \
  || echo "EVAL_FAIL"
if [[ -n "${TS_IP}" ]]; then
  echo "DONE page=http://${TS_IP}:$EVAL_PORT/ (tailscale) local=http://127.0.0.1:$EVAL_PORT/ out=$OUT evidence=$EVIDENCE_DST"
else
  echo "DONE page=http://127.0.0.1:$EVAL_PORT/ out=$OUT evidence=$EVIDENCE_DST"
fi
