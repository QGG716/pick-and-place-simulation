#!/bin/bash
exec chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /bin/bash -c 'source /opt/ros/humble/setup.bash; export ROS_HOME=/tmp/ros; exec /work-round3/build-perf/m710_moveit_worker'
