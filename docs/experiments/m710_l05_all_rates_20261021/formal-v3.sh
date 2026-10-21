#!/bin/bash
set -e
export PYTHONPATH=/work-l05-rates-bounds-655b495/src
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-l05-rates-bounds-655b495
/opt/planning-venv/bin/python -m pytest /work-l05-rates-bounds-655b495/tests/test_mount_comparison.py /work-l05-rates-bounds-655b495/tests/test_carton_population.py -q -k 'not paired_inputs_rebuild' > /l05-all-rates-20261021/focused-tests.log
exec /opt/planning-venv/bin/python -u /work-l05-rates-bounds-655b495/tools/plan_m710_xy_top_population.py \
 --output /l05-all-rates-20261021/formal-v3 \
 --worker-command '/bin/bash /worker.sh' --worker-binary /mount-build/m710_moveit_worker \
 --source-commit 655b49571f4422b2a86a4c2a02c0df830aedf0e8 \
 --source-manifest /work-l05-rates-bounds-655b495/source-manifest.json \
 --mounting-variant x_minus_400_y_minus_100 \
 --l05-limit-restart /population-x400-20261021/formal-v1 \
 --l05-limit-worker-command '/bin/bash /l05-all-rates-20261021/worker-rates-bounds.sh' \
 --l05-limit-library /l05-all-rates-20261021/libl05_joint_rates_bounds.so \
 --l05-motion-limit-policy all_joint_rates
