set -e
D=/root/autodl-tmp/m710-tcp-lin-20260928
E=$D/evidence/final2
cd "$D/repo"
PY=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
# This script is launched only once a complete plan exists; it independently
# verifies readiness before export. It never starts Isaac.
"$PY" - <<'PY'
import json
from pathlib import Path
p=Path('/root/autodl-tmp/m710-tcp-lin-20260928/evidence/final2/full-task.json')
d=json.loads(p.read_text());assert d['complete_trajectory_status']=='PASS'
q=p.with_name('first-accepted-full-task.json')
if q.exists(): assert q.read_bytes()==p.read_bytes()
else:
 with q.open('x') as f:f.write(p.read_text())
PY
time -p "$PY" tools/prepare_m710id70_dynamic_execution.py --config configs/simulation/m710id70_proof_of_concept.yaml --motion-result "$E/full-task.json" --output "$E/preflight.json" > "$E/preflight.log" 2>&1
export PYTHONPATH=src
time -p "$PY" scripts/export_isaac_fanuc_replay.py --preflight "$E/preflight.json" --output "$E/replay-bundle.json" > "$E/export.log" 2>&1
"$PY" tools/check_m710_moveit_execution_tcp.py --bundle "$E/replay-bundle.json" --requests "$E/task-requests.jsonl" --output "$E/executor-tcp.json" > "$E/executor-tcp.log" 2>&1
