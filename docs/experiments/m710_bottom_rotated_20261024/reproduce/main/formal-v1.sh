#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-bottom-rotated-a4552c3/src:/work-bottom-rotated-a4552c3/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-bottom-rotated-a4552c3
test -z "${LD_PRELOAD:-}"
out="${1:-/bottom-rotated-20261024/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-bottom-rotated-a4552c3/tools/plan_m710_xy_top_population.py  --output "$out" --worker-command '/bin/bash /l01-hmax-worker.sh' --worker-binary /l01-hmax-build/m710_moveit_worker  --source-commit a4552c363c0e2cce56be4d4b28958dd44ebbe3e8 --source-manifest /work-bottom-rotated-a4552c3/source-manifest.json  --mounting-variant x_minus_300_y_minus_100 --continue-bottom-rotated-from /l00-top-xplus200-v2-20261024/formal-v1  --tool-connector-extension-m 0.2 "${@:2}"
