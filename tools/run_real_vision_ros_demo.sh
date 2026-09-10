#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
GPU_VENV=${GPU_VENV:-/root/v05-gpu-venv}
VISION_ROOT=${VISION_ROOT:-/root/vision-upstream}
ARTIFACT_ROOT=${ARTIFACT_ROOT:-/root/autodl-tmp/v05-acceptance}
MODEL_MANIFEST=${MODEL_MANIFEST:-$ARTIFACT_ROOT/model-manifest.json}
HUMBLE_INSTALL=${HUMBLE_INSTALL:-$ARTIFACT_ROOT/humble/final/install}
RUN_ID=${RUN_ID:-real-vision-ros-$(date -u +%Y%m%dT%H%M%SZ)}
RUN="$ARTIFACT_ROOT/ros-gpu/$RUN_ID"
SOURCE="$VISION_ROOT/runs/current/cargo_7/01_proposals/source.png"
PROPOSALS="$VISION_ROOT/runs/current/cargo_7/01_proposals/boxes_merged.json"
PEOPLE="$VISION_ROOT/runs/current/cargo_7/02_people/person_masks.npz"

test -f /opt/ros/humble/setup.bash
test -f "$HUMBLE_INSTALL/setup.bash"
test -x "$GPU_VENV/bin/python"
test -f "$MODEL_MANIFEST"
mkdir -p "$RUN/worker-runs"
set +u
source /opt/ros/humble/setup.bash
source "$HUMBLE_INSTALL/setup.bash"
set -u
export PATH="/usr/bin:/bin:$PATH"
export PYTHONPATH="$ROOT/src:$ROOT/packages/unloading_contracts/src${PYTHONPATH:+:$PYTHONPATH}"

SAM_MODEL=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["sam"]["snapshot_path"])' "$MODEL_MANIFEST")
MOGE_MODEL=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["moge"]["model_path"])' "$MODEL_MANIFEST")
WORKER_COMMAND="['$GPU_VENV/bin/python','$ROOT/tools/vision_worker_entry.py','--upstream-root','$VISION_ROOT','--proposal-json','$PROPOSALS','--person-masks','$PEOPLE','--output-root','$RUN/worker-runs','--input-root','$VISION_ROOT','--sam-model','$SAM_MODEL','--moge-model','$MOGE_MODEL']"
WORKER_ROOTS="['$VISION_ROOT','$ROOT','$RUN']"

pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run unloading_ros_bridge mock_state_publisher >"$RUN/mock-state.log" 2>&1 &
pids+=("$!")
ros2 run unloading_ros_bridge world_bridge_node >"$RUN/world-bridge.log" 2>&1 &
pids+=("$!")
/usr/bin/python3 "$ROOT/tools/real_vision_ros_probe.py" --output "$RUN/summary.json" --timeout 1900 >"$RUN/probe.log" 2>&1 &
probe_pid=$!
pids+=("$probe_pid")
sleep 2
ros2 run unloading_ros_bridge perception_node --ros-args \
  -p backend:=pipeline -p input_image:="$SOURCE" -p input_width:=1290 -p input_height:=1333 \
  -p worker_cwd:="$ROOT" -p worker_timeout_seconds:=1800.0 \
  -p worker_command:="$WORKER_COMMAND" -p worker_allowed_roots:="$WORKER_ROOTS" \
  >"$RUN/perception-node.log" 2>&1 &
pids+=("$!")
wait "$probe_pid"
cat "$RUN/probe.log"
