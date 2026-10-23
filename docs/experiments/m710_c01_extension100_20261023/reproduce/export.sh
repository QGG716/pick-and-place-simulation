#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-c01-ext100-v2-2f6e1ea/src:/work-c01-ext100-v2-2f6e1ea/tools PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-c01-ext100-v2-2f6e1ea OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cd /c01-ext100-v2-20261023
/opt/planning-venv/bin/python geometry.py > geometry.log
/opt/planning-venv/bin/python export.py formal-v1 analysis-v1/c01-extension100.html --source-root /work-c01-ext100-v2-2f6e1ea --overlay selected_routes.js --geometry-preview geometry.json
INSIDE
tar czf /root/autodl-tmp/m710-l05-x500-20261022/c01-ext100-v2-viewer.tar.gz -C "$rootfs/c01-ext100-v2-20261023" analysis-v1/c01-extension100.html analysis-v1/c01-extension100-checks.json geometry.json geometry.log
