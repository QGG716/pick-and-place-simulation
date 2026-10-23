#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-bottom-top-4337f91/src:/work-bottom-top-4337f91/tools PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-bottom-top-4337f91 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cd /bottom-two-top-20261023
/opt/planning-venv/bin/python export-top-failures.py formal-v1 analysis-v1/top-ik-failures.html --source-root /work-bottom-top-4337f91
INSIDE
tar czf /root/autodl-tmp/m710-l05-x500-20261022/bottom-top-viewer.tar.gz -C "$rootfs/bottom-two-top-20261023/analysis-v1" top-ik-failures.html top-ik-failures-checks.json
