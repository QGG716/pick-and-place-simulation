#!/usr/bin/env bash
set -eo pipefail
# ROS/ament setup scripts read optional unset variables.
source /opt/ros/humble/setup.bash
repo="$(cd "$(dirname "$0")/../.." && pwd)"
# CMake install prefixes carry the pluginlib resource index as well as the
# shared library. A build-tree executable alone cannot discover custom OMPL.
# The same supported launcher is used with tracing enabled or disabled.
prefix="${M710_MOVEIT_INSTALL_PREFIX:-$repo/install/moveit}"
if [[ -f "$prefix/local_setup.bash" ]]; then
  source "$prefix/local_setup.bash"
fi
set -u
if [[ ! -f "$prefix/share/ament_index/resource_index/moveit_core__pluginlib__plugin/m710_moveit_backend" ]]; then
  echo "Missing MoveIt plugin installation at $prefix; run cmake --install for this build." >&2
  exit 2
fi
export AMENT_PREFIX_PATH="$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export RCUTILS_LOGGING_USE_STDOUT=0
exec "$prefix/lib/m710_moveit_backend/m710_moveit_worker"
