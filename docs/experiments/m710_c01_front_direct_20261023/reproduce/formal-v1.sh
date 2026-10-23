#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-c01-direct-75f98fc/src:/work-c01-direct-75f98fc/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-c01-direct-75f98fc
test -z "${LD_PRELOAD:-}"
out="${1:-/c01-direct-20261023/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-c01-direct-75f98fc/tools/plan_m710_xy_top_population.py \
 --output "$out" --worker-command '/bin/bash /l01-tool-v2-worker.sh' --worker-binary /l01-tool-v2-build/m710_moveit_worker \
 --source-commit 75f98fce8e68bd807c84590097782c638eb36629 --source-manifest /work-c01-direct-75f98fc/source-manifest.json \
 --mounting-variant x_minus_500_y_minus_100 --continue-l01-selected-routes-from /l01-tool-chassis-v2-20261023/formal-v1 --c01-finite-side-channel --c01-direct-suction --tool-connector-extension-m 0.1 \
 "${@:2}"
