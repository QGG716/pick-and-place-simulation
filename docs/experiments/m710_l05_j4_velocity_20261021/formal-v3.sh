#!/bin/bash
set -e
export PYTHONPATH=/work-j4-velocity-b6abb7a/src
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-j4-velocity-b6abb7a
/opt/planning-venv/bin/python -m pytest /work-j4-velocity-b6abb7a/tests/test_mount_comparison.py /work-j4-velocity-b6abb7a/tests/test_carton_population.py -q -k 'not paired_inputs_rebuild' > /l05-j4-velocity-20261021/focused-tests.log
exec /opt/planning-venv/bin/python -u /work-j4-velocity-b6abb7a/tools/plan_m710_xy_top_population.py \
 --output /l05-j4-velocity-20261021/formal-v3 \
 --worker-command '/bin/bash /worker.sh' --worker-binary /mount-build/m710_moveit_worker \
 --source-commit b6abb7ac39fe67fa6d82d3673aaad13d45ea1ae3 \
 --source-manifest /work-j4-velocity-b6abb7a/source-manifest.json \
 --mounting-variant x_minus_400_y_minus_100 \
 --l05-j4-velocity-restart /population-x400-20261021/formal-v1 \
 --l05-j4-velocity-worker-command '/bin/bash /l05-j4-velocity-20261021/worker-j4-only.sh' \
 --l05-j4-velocity-library /l05-j4-velocity-20261021/libl05_j4_velocity_only.so
