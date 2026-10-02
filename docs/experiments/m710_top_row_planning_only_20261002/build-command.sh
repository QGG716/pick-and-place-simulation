#!/bin/bash
set -eu
D=/root/autodl-tmp/m710-planning-only-20261002
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs
mkdir -p "$D/repo" "$R/work-planning-only-20261002"
tar -xzf "$D/source.tar.gz" -C "$D/repo"
cp -a /root/autodl-tmp/m710-validation-opt-20261002/repo/assets "$D/repo/"
tar -xzf "$D/source.tar.gz" -C "$R/work-planning-only-20261002"
cp -a "$D/repo/assets" "$R/work-planning-only-20261002/"
chroot "$R" /bin/bash -c 'source /opt/ros/humble/setup.bash; cmake -S /work-planning-only-20261002/ros2/m710_moveit_backend -B /work-planning-only-20261002/build -DCMAKE_BUILD_TYPE=Release; cmake --build /work-planning-only-20261002/build --target m710_moveit_worker -j2'
