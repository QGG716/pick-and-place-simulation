#!/bin/bash
set -euo pipefail
cd /root/autodl-tmp/m710-poc-step31-v3-20260918
export PYTHONPATH=src
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python - <<'PY'
import json
from pathlib import Path
assert not Path('outputs/isaac_step31_once').exists(), 'single-trial output already exists'
d=json.loads(Path('outputs/recovery_delivery/status.json').read_text());assert d['status']=='COMPLETE_PREFLIGHT_EXPORT_READBACK_PASS'
s=json.loads(Path('outputs/suffix_sensitivity.json').read_text())
assert all(v['runtime_5mm_pass'] and v['release_region']['accepted'] for v in s['scenarios'].values())
v=json.loads(Path('outputs/source_verification.json').read_text());assert v['production_sources_exact_git_match']
PY
export OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1
export XDG_RUNTIME_DIR=/root/autodl-tmp/runtime-root XDG_CACHE_HOME=/root/autodl-tmp/cache/xdg XDG_CONFIG_HOME=/root/autodl-tmp/config/xdg XDG_DATA_HOME=/root/autodl-tmp/data/xdg
exec /root/autodl-tmp/envs/isaacsim-clean/bin/python scripts/isaacsim_fanuc_replay.py \
 --project-root /root/autodl-tmp/m710-poc-step31-v3-20260918 \
 --bundle outputs/recovery_delivery/first_feasible/replay_bundle.json \
 --usd-directory outputs/isaac_step31_once_usd \
 --reuse-usd-entrypoint /root/autodl-tmp/m710-official-dynamics-20260910/isaac_usd/m710id_70_official_8/m710id_70_official.usda \
 --reuse-usd-run-evidence /root/autodl-tmp/m710-official-dynamics-20260910/isaac_outputs/initialization_render_sync_logged_final/run_status.json \
 --reuse-usd-source-contract /root/autodl-tmp/m710-official-dynamics-20260910/repo/outputs/m710_official_dynamics_20260910_final/initialization_contract.json \
 --output outputs/isaac_step31_once --continuation-dir outputs/isaac_step31_once_control \
 --maximum-segments 1 --record-video --video-preview-speed 1
