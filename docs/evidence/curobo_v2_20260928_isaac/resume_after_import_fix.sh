#!/bin/bash
set -u
cd /root/autodl-tmp/curobo-v2-20260928-isaac/repo
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=src
export OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1
export XDG_RUNTIME_DIR=/root/autodl-tmp/runtime-root
export XDG_CACHE_HOME=/root/autodl-tmp/cache/xdg
export XDG_CONFIG_HOME=/root/autodl-tmp/config/xdg
export XDG_DATA_HOME=/root/autodl-tmp/data/xdg
/root/autodl-tmp/envs/isaacsim-clean/bin/python scripts/isaacsim_fanuc_replay.py \
 --bundle /root/autodl-tmp/curobo-v2-20260928-isaac/trial/execution/replay_bundle.json \
 --project-root /root/autodl-tmp/curobo-v2-20260928-isaac/repo \
 --usd-directory /root/autodl-tmp/curobo-v2-20260928-isaac/trial/usd_after_import_fix \
 --reuse-usd-entrypoint /root/autodl-tmp/m710-official-dynamics-20260910/isaac_usd/m710id_70_official_8/m710id_70_official.usda \
 --reuse-usd-run-evidence /root/autodl-tmp/m710-official-dynamics-20260910/isaac_outputs/initialization_render_sync_logged_final/run_status.json \
 --reuse-usd-source-contract /root/autodl-tmp/m710-official-dynamics-20260910/repo/outputs/m710_official_dynamics_20260910_final/initialization_contract.json \
 --output /root/autodl-tmp/curobo-v2-20260928-isaac/trial/isaac_after_import_fix \
 --record-video --video-preview-speed 1 --maximum-segments 1
code=$?
printf '%s\n' "$code" > /root/autodl-tmp/curobo-v2-20260928-isaac/isaac_exit_code.txt
exit "$code"
