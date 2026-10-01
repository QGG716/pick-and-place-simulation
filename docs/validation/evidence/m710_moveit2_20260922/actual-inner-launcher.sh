#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_HOME=/tmp/ros RCUTILS_LOGGING_USE_STDOUT=0
exec /work/build/moveit/m710_moveit_worker
