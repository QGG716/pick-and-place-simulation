#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-min250-3f449e8/src:/work-min250-3f449e8/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-min250-3f449e8
/opt/planning-venv/bin/python -u /min250-20261023/sync-regression.py > /min250-20261023/sync-regression.log 2>&1
INSIDE
