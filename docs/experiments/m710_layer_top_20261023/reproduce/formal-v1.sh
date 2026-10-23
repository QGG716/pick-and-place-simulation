#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-layer-top-8770a4a/src:/work-layer-top-8770a4a/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-layer-top-8770a4a
test -z "${LD_PRELOAD:-}"
out="${1:-/layer-top-20261023/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-layer-top-8770a4a/tools/plan_m710_xy_top_population.py \
 --output "$out" --worker-command '/bin/bash /worker.sh' --worker-binary /mount-build/m710_moveit_worker \
 --source-commit 8770a4ad99bf35f9346ac321b940c880089c097c --source-manifest /work-layer-top-8770a4a/source-manifest.json \
 --mounting-variant x_minus_500_y_minus_100 --continue-layer-top-from /min250-20261023/formal-v2 \
 --top-layer-anchor carton_l02_c01 "${@:2}"
