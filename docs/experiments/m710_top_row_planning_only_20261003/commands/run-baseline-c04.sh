#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-planning-opt-20261003-baseline-snapshot
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1 ROS_HOME=/tmp/planning-opt-20261003/ros
exec /opt/planning-opt-20261003-venv/bin/python -u tools/plan_m710_top_row_only.py --output /tmp/planning-opt-20261003/baseline-c04 --worker-command /tmp/planning-opt-20261003/worker-baseline.sh --worker-binary /tmp/planning-opt-20261003/build-baseline/m710_moveit_worker --source-commit 26658f2b426d2d827d2429f6c2d7e47d152bb1e8 --diagnostic-input /tmp/planning-opt-20261003/c04-fixed-input.json --stage-budget-s 300 --box-budget-s 1800 --batch-budget-s 7200 --ipc-timeout-s 360 --seed 71070
