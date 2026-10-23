#!/bin/bash
set -eo pipefail
export PYTHONPATH=/work-min250-a300d23/src PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-min250-a300d23
test -z "${LD_PRELOAD:-}"
out="${1:-/min250-20261023/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-min250-a300d23/tools/plan_m710_xy_top_population.py \
  --output "$out" --worker-command '/bin/bash /worker.sh' \
  --worker-binary /mount-build/m710_moveit_worker --source-commit a300d234633e122c6d8328dddb259f3c9c38ad91 \
  --source-manifest /work-min250-a300d23/source-manifest.json --mounting-variant x_minus_500_y_minus_100 \
  --continue-min250-from /min250-20261023/previous-formal-v1
