#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-current-c04-dea9b2b/src:/work-current-c04-dea9b2b/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-current-c04-dea9b2b
test -z "${LD_PRELOAD:-}"
out="${1:-/current-c04-comparison-20261024/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-current-c04-dea9b2b/tools/plan_m710_xy_top_population.py  --output "$out" --worker-command '/bin/bash /l01-hmax-worker.sh' --worker-binary /l01-hmax-build/m710_moveit_worker  --source-commit dea9b2bce8b5fc770efc48db703b8d0cf6ba0a3e --source-manifest /work-current-c04-dea9b2b/source-manifest.json  --mounting-variant x_minus_300_y_minus_100 --continue-l01-c04-face-from /bottom-rotated-20261024/formal-v1 --c04-current-prefix --c04-face "${C04_FACE:?}"  --tool-connector-extension-m 0.2 "${@:2}"
