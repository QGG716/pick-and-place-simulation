#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
work=/work-l00-top-xplus200-b05652e
export PYTHONPATH="$work/src:$work/tools" M710_MOVEIT_ASSET_ROOT="$work" PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cd /l00-top-xplus200-v2-20261024
/opt/planning-venv/bin/python export_m710_top_ik_failures.py formal-v1 analysis-v1/failures.html --source-root "$work"
tar czf failure-viewer.tar.gz analysis-v1/failures.html analysis-v1/failures-checks.json export_m710_top_ik_failures.py
INSIDE
