#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-layer-top-8770a4a/src:/work-layer-top-8770a4a/tools PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-layer-top-8770a4a OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cd /layer-top-20261023
test "$(cat formal-v1.exit)" = 0
/opt/planning-venv/bin/python /work-layer-top-8770a4a/tools/verify_m710_population.py formal-v1 --prepare-replay replay-v1
/opt/planning-venv/bin/python analyze.py
if /opt/planning-venv/bin/python -c 'import json;exit(0 if json.load(open("formal-v1/summary.json"))["completed_count"] else 1)'; then
 /opt/planning-venv/bin/python /work-layer-top-8770a4a/tools/export_m710_top_layer_replay.py replay-v1 --seed 71070 --output analysis-v1/accepted-cycles.html
fi
/opt/planning-venv/bin/python export-top-ik-failures.py formal-v1 analysis-v1/failures.html --source-root /work-layer-top-8770a4a
INSIDE
tar czf /root/autodl-tmp/m710-l05-x500-20261022/layer-top-analysis.tar.gz -C "$rootfs/layer-top-20261023" analysis-v1 formal-v1/summary.json formal-v1/protocol.json formal-v1/verification.json formal-v1/initial-state-check.json formal-v1/predecessor.json focused-tests.log production-check.log formal-v1.log formal-v1.exit raw-receipt.json
