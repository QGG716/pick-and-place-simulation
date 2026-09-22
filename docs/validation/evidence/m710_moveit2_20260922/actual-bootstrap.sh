#!/bin/bash
set -euxo pipefail
cd /root/autodl-tmp/m710-moveit2-20260922
mkdir -p rootfs
curl -L --retry 2 -o ubuntu-base.tar.gz https://cdimage.ubuntu.com/ubuntu-base/releases/22.04/release/ubuntu-base-22.04.5-base-amd64.tar.gz
tar -xzf ubuntu-base.tar.gz -C rootfs
cp /etc/resolv.conf rootfs/etc/resolv.conf
printf 'deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main universe\ndeb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main universe\ndeb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-security main universe\ndeb [trusted=yes] http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main\n' > rootfs/etc/apt/sources.list
printf '#!/bin/sh\nexit 101\n' > rootfs/usr/sbin/policy-rc.d
chmod +x rootfs/usr/sbin/policy-rc.d
for spec in 'null 1 3' 'zero 1 5' 'random 1 8' 'urandom 1 9'; do set -- $spec; test -e rootfs/dev/$1 || mknod -m 666 rootfs/dev/$1 c $2 $3; done
chroot rootfs /bin/bash -c 'export DEBIAN_FRONTEND=noninteractive; apt-get update && apt-get install -y --no-install-recommends build-essential cmake python3-ament-package python3-empy python3-catkin-pkg ros-humble-moveit-task-constructor-core ros-humble-pilz-industrial-motion-planner ros-humble-moveit-planners-ompl ros-humble-moveit-kinematics nlohmann-json3-dev'
chroot rootfs dpkg-query -W > evidence/rootfs-packages.lock
