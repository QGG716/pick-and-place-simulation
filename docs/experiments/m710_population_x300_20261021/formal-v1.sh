#!/bin/bash
set -eo pipefail
export PYTHONPATH=/work-population-x300-cb6eb01/src PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-population-x300-cb6eb01
exec /opt/planning-venv/bin/python -u /work-population-x300-cb6eb01/tools/plan_m710_xy_top_population.py \
  --output "${1:-/population-x300-20261021/formal-v1}" \
  --worker-command '/bin/bash /worker.sh' --worker-binary /mount-build/m710_moveit_worker \
  --source-commit cb6eb0199fb07f10bff632edf3aaa1234f679c00 \
  --source-manifest /work-population-x300-cb6eb01/source-manifest.json \
  --mounting-variant x_minus_300_y_minus_100
