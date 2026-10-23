#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-l01-tool-e9dfef3/src:/work-l01-tool-e9dfef3/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-l01-tool-e9dfef3
test -z "${LD_PRELOAD:-}"
out="${1:-/l01-tool-chassis-v2-20261023/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-l01-tool-e9dfef3/tools/plan_m710_xy_top_population.py \
 --output "$out" --worker-command '/bin/bash /l01-tool-v2-worker.sh' --worker-binary /l01-tool-v2-build/m710_moveit_worker \
 --source-commit e9dfef3215b55bb9905ed0671d43ca4b9716a71a --source-manifest /work-l01-tool-e9dfef3/source-manifest.json \
 --mounting-variant x_minus_500_y_minus_100 --continue-l01-tool-chassis-from /lowprofile250-20261023/formal-v1 \
 "${@:2}"
