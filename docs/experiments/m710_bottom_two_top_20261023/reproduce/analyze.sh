#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-bottom-top-4337f91/src:/work-bottom-top-4337f91/tools PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-bottom-top-4337f91 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cd /bottom-two-top-20261023
test "$(cat formal-v1.exit)" = 0
/opt/planning-venv/bin/python /work-bottom-top-4337f91/tools/verify_m710_population.py formal-v1 --prepare-replay replay-v1
/opt/planning-venv/bin/python analyze.py
INSIDE
tar czf /root/autodl-tmp/m710-l05-x500-20261022/bottom-top-analysis.tar.gz -C "$rootfs/bottom-two-top-20261023" analysis-v1 formal-v1/summary.json formal-v1/protocol.json formal-v1/verification.json formal-v1/initial-state-check.json formal-v1/predecessor.json focused-tests.log production-check.log formal-v1.log formal-v1.exit raw-receipt.json native-build-reuse.json
