#!/bin/bash
set -eu
D=/root/autodl-tmp/m710-validation-opt-20261002
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs
chroot "$R" /usr/bin/dpkg-query -W '-f=${binary:Package}\t${Version}\n' > "$D/evidence/native-packages-current.lock"
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pip freeze > "$D/evidence/cpu-python-current.lock"
/root/autodl-tmp/envs/isaacsim-clean/bin/python -m pip freeze > "$D/evidence/isaac-python-current.lock"
cp "$R/work-native-cold-20261002/build/m710_moveit_worker" "$D/evidence/m710_moveit_worker"
sha256sum "$D/evidence/m710_moveit_worker"
cat > "$D/worker.sh" <<'WORKER'
#!/bin/bash
exec chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /bin/bash -c 'source /opt/ros/humble/setup.bash; export ROS_HOME=/tmp/ros-validation-opt-20261002; exec /work-native-cold-20261002/build/m710_moveit_worker'
WORKER
chmod +x "$D/worker.sh"
cp "$D/worker.sh" "$D/evidence/worker.sh"
uname -a > "$D/evidence/hardware-environment.txt"
lscpu >> "$D/evidence/hardware-environment.txt"
nvidia-smi >> "$D/evidence/hardware-environment.txt"
chroot "$R" cat /etc/os-release > "$D/evidence/native-os-release.txt"
cmp /root/autodl-tmp/m710-native-cold-20261002/evidence/packages.lock "$D/evidence/native-packages-current.lock"
echo 'Native package lock unchanged'
test ! -e "$D/repo/outputs/native-cold-validation-opt-once"
echo 'ENVIRONMENT_AUDIT_COMPLETE_NO_FORMAL_WORLD_STARTED'
