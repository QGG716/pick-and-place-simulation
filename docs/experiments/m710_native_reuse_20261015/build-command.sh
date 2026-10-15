#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-native-reuse2-20261015
cmake -S ros2/m710_moveit_backend -B /tmp/native-reuse-20261015/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/native-reuse-20261015/build -j2
