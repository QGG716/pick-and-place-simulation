#!/bin/bash
set -euo pipefail
cd /work-xy-top-f37448f
export PYTHONPATH=src PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-xy-top-f37448f
/opt/planning-venv/bin/python -u tools/plan_m710_xy_top_population.py \
 --output /xy-top-formal-v1 --worker-command '/bin/bash /worker.sh' \
 --worker-binary /mount-build/m710_moveit_worker \
 --source-commit f37448f998d0e82401c096a72c198b28e58fd2c2 --source-manifest source-manifest.json
/opt/planning-venv/bin/python tools/verify_m710_population.py /xy-top-formal-v1 --prepare-replay /xy-top-formal-replay-v1
/opt/planning-venv/bin/python tools/export_m710_top_layer_replay.py /xy-top-formal-replay-v1 --seed 71070 --output /xy-top-accepted-cycles.html
