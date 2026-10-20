#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/tmp/junction-hmin-20261020/formal-install:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/tmp/junction-hmin-20261020/formal-install/lib:$LD_LIBRARY_PATH
exec /tmp/junction-hmin-20261020/formal-build/m710_moveit_worker
