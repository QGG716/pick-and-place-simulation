#!/bin/bash
set -euo pipefail
D=/root/autodl-tmp/m710-tcp-lin-20260928
E=$D/evidence/final2
cd "$D/repo"
export PYTHONPATH=src
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python - <<'PY'
import json,time
from pathlib import Path
from unloading_sim.m710_replay_contract import verify_m710_replay_bundle
D=Path('/root/autodl-tmp/m710-tcp-lin-20260928');E=D/'evidence/final2'
assert json.loads((E/'full-task.json').read_text())['complete_trajectory_status']=='PASS'
assert json.loads((E/'preflight.json').read_text())['simulation_execution_ready']
assert json.loads((E/'executor-tcp.json').read_text())['status']=='PASS'
b=json.loads((E/'replay-bundle.json').read_text());verify_m710_replay_bundle(b,project_root=D/'repo')
m=b['metadata'];assert m['target']=='carton_l07_c02'
assert not m.get('completed_carton_ids') and not m.get('ideal_received_ids') and not m.get('handed_off_ids')
assert sum(bool(p.get('dynamic')) for p in m['scene_primitives'])==40
assert not (D/'isaac-single-box').exists()
with (E/'ISAAC_STARTED.json').open('x') as f:json.dump(dict(started_unix_s=time.time(),run_id='m710-tcp-lin-20260928-single-box',maximum_segments=1,new_world=True),f)
PY
export OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1
export XDG_RUNTIME_DIR=/root/autodl-tmp/runtime-root XDG_CACHE_HOME=/root/autodl-tmp/cache/xdg XDG_CONFIG_HOME=/root/autodl-tmp/config/xdg XDG_DATA_HOME=/root/autodl-tmp/data/xdg
exec /root/autodl-tmp/envs/isaacsim-clean/bin/python scripts/isaacsim_fanuc_replay.py \
 --project-root "$D/repo" --bundle "$E/replay-bundle.json" \
 --usd-directory "$D/isaac-single-box-usd" \
 --reuse-usd-entrypoint /root/autodl-tmp/m710-official-dynamics-20260910/isaac_usd/m710id_70_official_8/m710id_70_official.usda \
 --reuse-usd-run-evidence /root/autodl-tmp/m710-official-dynamics-20260910/isaac_outputs/initialization_render_sync_logged_final/run_status.json \
 --reuse-usd-source-contract /root/autodl-tmp/m710-official-dynamics-20260910/repo/outputs/m710_official_dynamics_20260910_final/initialization_contract.json \
 --output "$D/isaac-single-box" --maximum-segments 1 \
 --record-video --video-preview-speed 1 --width 640 --height 360 --output-fps 5
