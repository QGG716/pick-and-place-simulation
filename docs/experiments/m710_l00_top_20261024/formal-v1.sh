#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-l00-top-0398dc4/src:/work-l00-top-0398dc4/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-l00-top-0398dc4
test -z "${LD_PRELOAD:-}"
out="${1:-/l00-top-20261024/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-l00-top-0398dc4/tools/plan_m710_xy_top_population.py  --output "$out" --worker-command '/bin/bash /l01-hmax-worker.sh' --worker-binary /l01-hmax-build/m710_moveit_worker  --source-commit 0398dc4756df9b97ecbc855fb3fe7f89e26051c9 --source-manifest /work-l00-top-0398dc4/source-manifest.json  --mounting-variant x_minus_500_y_minus_100 --continue-l00-top-from /l01-hmax-v2-20261024/formal-v1  --tool-connector-extension-m 0.2 "${@:2}"
