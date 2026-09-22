#!/usr/bin/env bash
# Optional Ubuntu 22.04 rootfs; never installs/upgrades host packages.
# Usage: sudo bash bootstrap_rootfs.sh /absolute/empty/experiment/rootfs
set -euo pipefail
rootfs="${1:?absolute empty rootfs directory required}"
[[ "$rootfs" = /* && "$rootfs" != / ]] || exit 2
mkdir -p "$rootfs"
[[ -z "$(ls -A "$rootfs")" ]] || { echo 'rootfs must be empty' >&2; exit 2; }
curl --fail -L https://cdimage.ubuntu.com/ubuntu-base/releases/22.04/release/ubuntu-base-22.04.5-base-amd64.tar.gz -o "$rootfs/../ubuntu-base.tar.gz"
tar -xzf "$rootfs/../ubuntu-base.tar.gz" -C "$rootfs"
cp /etc/resolv.conf "$rootfs/etc/resolv.conf"
mkdir -p "$rootfs/dev/shm"
chmod 1777 "$rootfs/dev/shm"
for spec in 'null 1 3' 'zero 1 5' 'random 1 8' 'urandom 1 9'; do
  read -r name major minor <<< "$spec"
  [[ -e "$rootfs/dev/$name" ]] || mknod -m 666 "$rootfs/dev/$name" c "$major" "$minor"
done
printf '#!/bin/sh\nexit 101\n' > "$rootfs/usr/sbin/policy-rc.d"
chmod +x "$rootfs/usr/sbin/policy-rc.d"
# Use the official ROS signing key; pin installed versions after bootstrap.
curl --fail -L https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$rootfs/usr/share/keyrings/ros-archive-keyring.gpg"
cat > "$rootfs/etc/apt/sources.list" <<'APT'
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main universe
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main universe
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-security main universe
deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main
APT
cp "$(dirname "$0")/ros-packages.lock" "$rootfs/ros-packages.lock"
chroot "$rootfs" /bin/bash -c 'export DEBIAN_FRONTEND=noninteractive; mapfile -t packages < /ros-packages.lock; apt-get update && apt-get install -y --no-install-recommends build-essential cmake python3-ament-package python3-empy python3-catkin-pkg nlohmann-json3-dev "${packages[@]}"'
chroot "$rootfs" dpkg-query -W > "$rootfs/../rootfs-packages.lock"
