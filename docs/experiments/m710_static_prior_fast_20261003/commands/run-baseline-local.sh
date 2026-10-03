#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-planning-opt-20261003-source
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1 ROS_HOME=/tmp/static-prior-20261003/ros
for carton in carton_l07_c03 carton_l07_c04; do
 /opt/planning-opt-20261003-venv/bin/python -u tools/plan_m710_top_row_only.py --output /tmp/static-prior-20261003/baseline-$carton --worker-command /tmp/planning-opt-20261003/worker-optimized.sh --worker-binary /tmp/planning-opt-20261003/build-optimized/m710_moveit_worker --source-commit f7846bee960d290de695312b0e2ded0ef4975034 --stage-budget-s 90 --box-budget-s 300 --batch-budget-s 300 --ipc-timeout-s 100 --seed 71070 --diagnostic-input /tmp/static-prior-20261003/$carton-task-input.json
done
