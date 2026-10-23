#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-lowprofile-ab491d2/src:/work-lowprofile-ab491d2/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-lowprofile-ab491d2
test -z "${LD_PRELOAD:-}"
out="${1:-/lowprofile250-20261023/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-lowprofile-ab491d2/tools/plan_m710_xy_top_population.py \
 --output "$out" --worker-command '/bin/bash /lowprofile-worker.sh' --worker-binary /lowprofile-build/m710_moveit_worker \
 --source-commit ab491d2314e5b08eff6a2f3f84b469bb25a215ae --source-manifest /work-lowprofile-ab491d2/source-manifest.json \
 --mounting-variant x_minus_500_y_minus_100 --continue-lowprofile-from /min250-20261023/formal-v2 \
 "${@:2}"
