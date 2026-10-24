#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-c04-side-94ff464/src:/work-c04-side-94ff464/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-c04-side-94ff464
test -z "${LD_PRELOAD:-}"
out="${1:-/c04-side-20261024/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-c04-side-94ff464/tools/plan_m710_xy_top_population.py  --output "$out" --worker-command '/bin/bash /l01-hmax-worker.sh' --worker-binary /l01-hmax-build/m710_moveit_worker  --source-commit 94ff46486f4c2a929a9c46323699198a14d18af4 --source-manifest /work-c04-side-94ff464/source-manifest.json  --mounting-variant x_minus_500_y_minus_100 --continue-l01-c04-side-from /l01-hmax-v2-20261024/formal-v1  --tool-connector-extension-m 0.1 "${@:2}"
