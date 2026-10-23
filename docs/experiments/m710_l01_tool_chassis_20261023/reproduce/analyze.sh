#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-l01-tool-e9dfef3/src:/work-l01-tool-e9dfef3/tools PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-l01-tool-e9dfef3 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cd /l01-tool-chassis-v2-20261023
test "$(cat formal-v1.exit)" = 0
/opt/planning-venv/bin/python /work-l01-tool-e9dfef3/tools/verify_m710_population.py formal-v1 --prepare-replay replay-v1
/opt/planning-venv/bin/python analyze.py
/opt/planning-venv/bin/python - <<'PY'
import json
from pathlib import Path
from export_m710_top_layer_replay import export
if json.loads(Path('formal-v1/summary.json').read_text())['completed_count']:
    export(Path('replay-v1'),71070,Path('analysis-v1/accepted-cycles.html'))
PY
INSIDE
tar czf /root/autodl-tmp/m710-l05-x500-20261022/l01-tool-analysis.tar.gz -C "$rootfs/l01-tool-chassis-v2-20261023" analysis-v1 formal-v1/summary.json formal-v1/protocol.json formal-v1/verification.json formal-v1/initial-state-check.json formal-v1/predecessor.json focused-tests.log production-check.log formal-v1.log formal-v1.exit raw-receipt.json build.log native-policy-tests.log native-binary-sha256.txt
