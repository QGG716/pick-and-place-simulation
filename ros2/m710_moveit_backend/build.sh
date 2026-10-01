#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
repo="$(cd "$(dirname "$0")/../.." && pwd)"
cmake -S "$repo/ros2/m710_moveit_backend" -B "$repo/build/moveit" -DCMAKE_BUILD_TYPE=Release
cmake --build "$repo/build/moveit" -j "${M710_BUILD_JOBS:-2}"
