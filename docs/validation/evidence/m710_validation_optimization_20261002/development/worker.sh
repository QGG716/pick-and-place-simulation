#!/bin/bash
exec chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /bin/bash -c 'source /opt/ros/humble/setup.bash; export ROS_HOME=/tmp/ros-validation-opt-20261002; exec /work-native-cold-20261002/build/m710_moveit_worker'
