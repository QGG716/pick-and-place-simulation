#!/bin/bash
set -eo pipefail
export PYTHONPATH=/work-population-x500-66604f4/src PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-population-x500-66604f4
test -z "${LD_PRELOAD:-}"
out="${1:-/population-x500-20261022/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-population-x500-66604f4/tools/plan_m710_xy_top_population.py \
  --output "$out" --worker-command '/bin/bash /worker.sh' \
  --worker-binary /mount-build/m710_moveit_worker --source-commit 66604f4fd4a346be8d1947f3d3d44c544a5ad407 \
  --source-manifest /work-population-x500-66604f4/source-manifest.json --mounting-variant x_minus_500_y_minus_100
