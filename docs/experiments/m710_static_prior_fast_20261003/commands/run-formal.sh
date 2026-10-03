#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-static-prior-20261003
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1 ROS_HOME=/tmp/static-prior-20261003/ros
exec /opt/planning-opt-20261003-venv/bin/python -u tools/plan_m710_top_row_only.py --output /tmp/static-prior-20261003/formal-once --planning-mode static_prior_fast --static-prior /tmp/static-prior-20261003/static-prior-v3.json --fast-phase-budget-s 1 --slow-completion --worker-command /tmp/static-prior-20261003/worker-frozen.sh --worker-binary /tmp/static-prior-20261003/worker-frozen --source-commit 00352b6ff2ab3114b5248b266a0f3e955dd30f08 --stage-budget-s 90 --box-budget-s 300 --batch-budget-s 900 --ipc-timeout-s 100 --seed 71070
