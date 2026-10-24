#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /usr/bin/env PYTHONPATH=/work-current-c04-c3b815d/src:/work-current-c04-c3b815d/tools M710_MOVEIT_ASSET_ROOT=/work-current-c04-c3b815d OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 /opt/planning-venv/bin/python /current-c04-comparison-v2-20261024/export-front.py
tar czf /root/autodl-tmp/m710-l05-x500-20261022/current-c04-front-view.tar.gz -C "$rootfs/current-c04-comparison-v2-20261024" analysis-front/failure.html analysis-front/failure-checks.json
