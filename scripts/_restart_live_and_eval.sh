#!/usr/bin/env bash
# Restart slam_server (live HTML) on :18080 and A/B eval HTML on :8080.
set -euo pipefail
ROOT=/mnt/d/agent-workspaces/Rover-SLAM
LIVE_PORT=18080
EVAL_PORT=8080
LOG=$HOME/build/rover-slam-live.log

export LIBTORCH_ROOT="${LIBTORCH_ROOT:-$HOME/build/libtorch-cu128}"
export CMAKE_PREFIX_PATH="${HOME}/.local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PATH="/usr/local/cuda/bin:${HOME}/bin:${PATH}"
ONNX="$ROOT/onnxruntime-linux-x64-gpu-1.16.3/lib"
export LD_LIBRARY_PATH="${LIBTORCH_ROOT}/lib:${ONNX}:/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROVER_SLAM_HEADLESS=1
unset DISPLAY || true

stop_port() {
  local port="$1"
  local pid
  pid=$(ss -ltnp 2>/dev/null | awk -v p=":$port" '$4 ~ p {print}' | grep -oP 'pid=\K[0-9]+' | head -1 || true)
  if [[ -n "${pid:-}" ]]; then
    echo "killing pid $pid on :$port"
    kill "$pid" 2>/dev/null || true
    sleep 0.8
    kill -9 "$pid" 2>/dev/null || true
  else
    echo "no listener on :$port"
  fi
}

cd "$ROOT"
stop_port "$LIVE_PORT"

BIN="$ROOT/Examples/Monocular/slam_server"
if [[ ! -x "$BIN" ]]; then
  echo "missing $BIN" >&2
  exit 1
fi

mkdir -p "$HOME/build"
: > "$LOG"
echo "=== start slam_server :$LIVE_PORT ==="
"$BIN" \
  Vocabulary/voc_binary_tartan_8u_6.yml.gz \
  Examples/Monocular/iphoneSLAM.yaml \
  --prior-pack testdata/prior_pack_fixture \
  --port "$LIVE_PORT" \
  --max-queue 100 \
  >"$LOG" 2>&1 &
SPID=$!
echo "slam_server pid=$SPID log=$LOG"

for i in $(seq 1 90); do
  if curl -sf -m 1 "http://127.0.0.1:$LIVE_PORT/api/v1/health" >/dev/null 2>&1; then
    echo "LIVE_READY http://127.0.0.1:$LIVE_PORT/index.html"
    if curl -sf -m 2 "http://127.0.0.1:$LIVE_PORT/index.html" | grep -q timingPlot; then
      echo "LIVE_HTML_HAS_PLOTS"
    else
      echo "LIVE_HTML_MISSING_PLOTS"
    fi
    grep -E "Phone stream|Listening|prior pack|error|Error" "$LOG" | head -15 || true
    break
  fi
  if ! kill -0 "$SPID" 2>/dev/null; then
    echo "slam_server died"
    tail -50 "$LOG"
    exit 1
  fi
  sleep 1
  if [[ "$i" -eq 90 ]]; then
    echo "LIVE_NOT_READY"
    tail -50 "$LOG"
    exit 1
  fi
done

# Restart A/B static serve on 8080 (pkill only the python serve script)
stop_port "$EVAL_PORT"
pkill -f 'serve_holdout_pose_eval.py' 2>/dev/null || true
sleep 0.3
sed -i 's/\r$//' "$ROOT/scripts/serve_holdout_pose_eval.py"
DST="$ROOT/out/holdout_pose_eval"
python3 "$ROOT/scripts/serve_holdout_pose_eval.py" --host 0.0.0.0 --port "$EVAL_PORT" --dir "$DST" \
  >"$HOME/build/holdout_eval_serve.log" 2>&1 &
echo "eval_serve pid=$!"
sleep 0.5
curl -sf -m 2 -o /dev/null -w "EVAL_HTTP:%{http_code}\n" "http://127.0.0.1:$EVAL_PORT/" || echo "EVAL_FAIL"
echo DONE
