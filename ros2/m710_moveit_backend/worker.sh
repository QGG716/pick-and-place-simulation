#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
repo="$(cd "$(dirname "$0")/../.." && pwd)"
export RCUTILS_LOGGING_USE_STDOUT=0
exec "$repo/build/moveit/m710_moveit_worker"
