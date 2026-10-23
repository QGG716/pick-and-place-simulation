#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-lowprofile-f17d636/src:/work-lowprofile-f17d636/tools PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-lowprofile-f17d636 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cd /lowprofile250-20261023
test "$(cat formal-v1.exit)" = 0
/opt/planning-venv/bin/python analyze-v2.py
/opt/planning-venv/bin/python /work-lowprofile-f17d636/tools/export_m710_top_layer_replay.py replay-v1 --seed 71070 --output analysis-v2/accepted-cycles.html
INSIDE
tar czf /root/autodl-tmp/m710-l05-x500-20261022/lowprofile250-analysis.tar.gz -C "$rootfs/lowprofile250-20261023" analysis-v2 formal-v1/summary.json formal-v1/protocol.json formal-v1/verification.json formal-v1/initial-state-check.json formal-v1/predecessor.json focused-tests.log production-check.log formal-v1.log formal-v1.exit raw-receipt.json build.log native-policy-tests.log native-binary-sha256.txt native-build-reuse.json focused-tests-v2.log production-check-v2.log
