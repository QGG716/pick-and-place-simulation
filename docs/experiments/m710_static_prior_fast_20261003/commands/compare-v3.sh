#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-static-prior-20261003
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1 ROS_HOME=/tmp/static-prior-20261003/ros
/opt/planning-opt-20261003-venv/bin/python -u tools/benchmark_m710_prior_connection.py --archive /tmp/planning-opt-20261003/formal-once --output /tmp/static-prior-20261003/fixed-v3-rest --static-prior /tmp/static-prior-20261003/static-prior-v3.json --worker-command /tmp/static-prior-20261003/worker-current.sh --worker-binary /tmp/static-prior-20261003/build/m710_moveit_worker --source-commit 30c6f3bf058563fcd2ffe23c2523ef71db265809 --case carton_l07_c03-pregrasp --case carton_l07_c04-transit --case carton_l07_c03-transit-heldout-j1-plus-0p005 --case carton_l07_c04-transit-heldout-j1-plus-0p005
for carton in carton_l07_c03 carton_l07_c04; do
 /opt/planning-opt-20261003-venv/bin/python -u tools/plan_m710_top_row_only.py --output /tmp/static-prior-20261003/optimized-$carton --planning-mode static_prior_fast --static-prior /tmp/static-prior-20261003/static-prior-v3.json --fast-phase-budget-s 300 --slow-completion --worker-command /tmp/static-prior-20261003/worker-current.sh --worker-binary /tmp/static-prior-20261003/build/m710_moveit_worker --source-commit 30c6f3bf058563fcd2ffe23c2523ef71db265809 --stage-budget-s 90 --box-budget-s 300 --batch-budget-s 300 --ipc-timeout-s 100 --seed 71070 --diagnostic-input /tmp/static-prior-20261003/$carton-task-input.json
done
