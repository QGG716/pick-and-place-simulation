#!/usr/bin/env bash
set -euo pipefail

OUTPUT=${1:-./artifacts/server-preflight.txt}
mkdir -p "$(dirname "$OUTPUT")"
exec > >(tee "$OUTPUT") 2>&1

cat /etc/os-release
uname -m
id
pwd
systemd-detect-virt || true
printf 'cpu_limit='; cat /sys/fs/cgroup/cpu.max 2>/dev/null || echo unavailable
printf 'memory_limit='; cat /sys/fs/cgroup/memory.max 2>/dev/null || echo unavailable
nproc
free -h
df -h / "${PERSISTENT_ROOT:-/root/autodl-tmp}"
nvidia-smi --query-gpu=name,memory.total,memory.free,driver_version,compute_cap --format=csv,noheader
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader
command -v docker || true
command -v podman || true
command -v apptainer || true
/usr/bin/python3.10 --version
test -f /opt/ros/humble/setup.bash
bash -c 'source /opt/ros/humble/setup.bash && printf "ROS_DISTRO=%s\n" "$ROS_DISTRO" && ros2 --help >/dev/null'
