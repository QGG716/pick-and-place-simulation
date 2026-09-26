#!/bin/bash
exec chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /bin/bash -c 'source /opt/ros/humble/setup.bash; export ROS_HOME=/tmp/ros; exec /work-round2/build-round2/m710_moveit_worker'
