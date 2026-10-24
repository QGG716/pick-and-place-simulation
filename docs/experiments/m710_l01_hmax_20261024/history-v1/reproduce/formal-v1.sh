#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-l01-hmax-48dcea9/src:/work-l01-hmax-48dcea9/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-l01-hmax-48dcea9
test -z "${LD_PRELOAD:-}"
out="${1:-/l01-hmax-20261024/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-l01-hmax-48dcea9/tools/plan_m710_xy_top_population.py  --output "$out" --worker-command '/bin/bash /l01-hmax-worker.sh' --worker-binary /l01-hmax-build/m710_moveit_worker  --source-commit 48dcea934c7eb909673561ba7a3a2fc62e471261 --source-manifest /work-l01-hmax-48dcea9/source-manifest.json  --mounting-variant x_minus_500_y_minus_100 --continue-l01-selected-routes-from /l01-tool-chassis-v2-20261023/formal-v1  --l01-all-hmax --tool-connector-extension-m 0.1 "${@:2}"
