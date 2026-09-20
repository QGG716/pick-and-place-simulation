#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "${HUMBLE_INSTALL:?set HUMBLE_INSTALL to the existing colcon install}/setup.bash"
set -u
export PATH="/usr/bin:/bin:$PATH"
export PYTHONPATH="$ROOT/src:$ROOT/packages/unloading_contracts/src:$ROOT/ros2_ws/src/unloading_ros_bridge${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 "$ROOT/tools/run_rgbd_video_demo.py" "$@"
