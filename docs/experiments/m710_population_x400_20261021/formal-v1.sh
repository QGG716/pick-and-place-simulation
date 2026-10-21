#!/bin/bash
set -eo pipefail
export PYTHONPATH=/work-population-x400-90c8304/src PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-population-x400-90c8304
exec /opt/planning-venv/bin/python -u /work-population-x400-90c8304/tools/plan_m710_xy_top_population.py \
  --output "${1:-/population-x400-20261021/formal-v1}" \
  --worker-command '/bin/bash /worker.sh' --worker-binary /mount-build/m710_moveit_worker \
  --source-commit 90c83043005797e07c8671fccf9da862f6484ac7 \
  --source-manifest /work-population-x400-90c8304/source-manifest.json \
  --mounting-variant x_minus_400_y_minus_100
