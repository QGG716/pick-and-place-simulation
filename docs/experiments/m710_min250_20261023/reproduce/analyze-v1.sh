#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-min250-a300d23/src:/work-min250-a300d23/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-min250-a300d23
cd /min250-20261023
/opt/planning-venv/bin/python summarize-cycles-v1.py
/opt/planning-venv/bin/python /work-min250-a300d23/tools/export_m710_top_layer_replay.py replay-v1 --seed 71070 --output analysis-v1/accepted-cycles.html
INSIDE
tar czf /root/autodl-tmp/m710-l05-x500-20261022/min250-v1-analysis.tar.gz -C "$rootfs/min250-20261023" analysis-v1 formal-v1/summary.json formal-v1/protocol.json formal-v1/prefix-verification.json formal-v1/initial-state-check.json formal-v1/world-drift-diagnosis.json focused-tests.log focused-tests-v2.log production-check/initial-state-check.json formal-v1.log formal-v1.exit
