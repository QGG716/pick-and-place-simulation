#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ARTIFACT_ROOT=${ARTIFACT_ROOT:-/root/autodl-tmp/v05-acceptance}
RUN_ID=${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}
RUN="$ARTIFACT_ROOT/humble/$RUN_ID"
test -f /opt/ros/humble/setup.bash
set +u
source /opt/ros/humble/setup.bash
set -u
export PATH="/usr/bin:/bin:$PATH"
[[ $ROS_DISTRO == humble ]]
[[ $(/usr/bin/python3.10 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")') == 3.10 ]]
export PYTHONPATH="$ROOT/src:$ROOT/packages/unloading_contracts/src${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$RUN"
colcon --log-base "$RUN/build-log" build --base-paths "$ROOT/ros2_ws/src" \
  --build-base "$RUN/build" --install-base "$RUN/install" --symlink-install \
  2>&1 | tee "$RUN/colcon-build.log"
set +u
source "$RUN/install/setup.bash"
set -u
colcon --log-base "$RUN/test-log" test --base-paths "$ROOT/ros2_ws/src" \
  --build-base "$RUN/build" --install-base "$RUN/install" --return-code-on-test-failure \
  2>&1 | tee "$RUN/colcon-test.log"
colcon test-result --test-result-base "$RUN/build" --verbose \
  2>&1 | tee "$RUN/colcon-test-result.log"
