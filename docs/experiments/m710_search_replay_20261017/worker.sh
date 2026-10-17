#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/tmp/search-trace-20261017/install:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/tmp/search-trace-20261017/install/lib:$LD_LIBRARY_PATH
exec /tmp/search-trace-20261017/build/m710_moveit_worker
