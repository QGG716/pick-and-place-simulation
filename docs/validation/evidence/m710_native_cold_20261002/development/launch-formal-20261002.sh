#!/bin/bash
set -eu
D=/root/autodl-tmp/m710-native-cold-20261002
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs
cd "$D/repo"
export OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1
export XDG_RUNTIME_DIR=/root/autodl-tmp/runtime-root
export XDG_CACHE_HOME=/root/autodl-tmp/cache/xdg
export XDG_CONFIG_HOME=/root/autodl-tmp/config/xdg
export XDG_DATA_HOME=/root/autodl-tmp/data/xdg
exec /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -u tools/run_m710_native_cold_once.py \
  --output "$D/repo/outputs/native-cold-20261002-once" \
  --isaac-python /root/autodl-tmp/envs/isaacsim-clean/bin/python \
  --worker-command "$D/worker.sh" \
  --worker-binary "$R/work-native-cold-20261002/build/m710_moveit_worker" \
  --native-asset-root /work-native-cold-20261002 \
  --source-commit f94a1ca4c48b4b4bf5805466549c941ed07dd830 \
  --task-budget-s 3600 --stage-budget-s 300 --ipc-timeout-s 360
