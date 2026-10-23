#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-l01-routes-8058c17/src:/work-l01-routes-8058c17/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-l01-routes-8058c17
test -z "${LD_PRELOAD:-}"
out="${1:-/l01-routes-20261023/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-l01-routes-8058c17/tools/plan_m710_xy_top_population.py \
 --output "$out" --worker-command '/bin/bash /l01-tool-v2-worker.sh' --worker-binary /l01-tool-v2-build/m710_moveit_worker \
 --source-commit 8058c17ca721646ec5bed1cdd22805a2633eeeba --source-manifest /work-l01-routes-8058c17/source-manifest.json \
 --mounting-variant x_minus_500_y_minus_100 --continue-l01-selected-routes-from /l01-tool-chassis-v2-20261023/formal-v1 \
 "${@:2}"
