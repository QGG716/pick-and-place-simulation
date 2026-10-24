#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-l01-hmax-163eb5e/src:/work-l01-hmax-163eb5e/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-l01-hmax-163eb5e
test -z "${LD_PRELOAD:-}"
out="${1:-/l01-hmax-v2-20261024/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-l01-hmax-163eb5e/tools/plan_m710_xy_top_population.py  --output "$out" --worker-command '/bin/bash /l01-hmax-worker.sh' --worker-binary /l01-hmax-build/m710_moveit_worker  --source-commit 163eb5ec25f348263957e9e0764b43c8eb93ee5d --source-manifest /work-l01-hmax-163eb5e/source-manifest.json  --mounting-variant x_minus_500_y_minus_100 --continue-l01-selected-routes-from /l01-tool-chassis-v2-20261023/formal-v1  --l01-all-hmax --tool-connector-extension-m 0.1 "${@:2}"
