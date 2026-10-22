#!/bin/bash
set -eo pipefail
export PYTHONPATH=/work/src PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work
test -z "${LD_PRELOAD:-}"
out="${1:-/formal-v1}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work/tools/plan_m710_extraction_retreat.py \
  --trial l05_extra_x100 --previous-run /previous --output "$out" \
  --source-commit e0c33a6b599eff6493e320b9f7242e927b1e7834 \
  --source-manifest /work/source-manifest.json \
  --worker-command '/bin/bash /worker.sh' --worker-binary /mount-build/m710_moveit_worker
