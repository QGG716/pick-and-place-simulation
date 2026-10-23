#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-bottom-top-4337f91/src:/work-bottom-top-4337f91/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-bottom-top-4337f91
test -z "${LD_PRELOAD:-}"
out="${1:-/bottom-two-top-20261023/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-bottom-top-4337f91/tools/plan_m710_xy_top_population.py \
 --output "$out" --worker-command '/bin/bash /lowprofile-worker.sh' --worker-binary /lowprofile-build/m710_moveit_worker \
 --source-commit 4337f9115d193645bfaf6122c1ca5673ebf3e7ac --source-manifest /work-bottom-top-4337f91/source-manifest.json \
 --mounting-variant x_minus_500_y_minus_100 --continue-lowprofile-top-from /lowprofile250-20261023/formal-v1 \
 "${@:2}"
