#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_HOME=/tmp/planning-opt-20261003/ros
exec /tmp/planning-opt-20261003/build-baseline/m710_moveit_worker
