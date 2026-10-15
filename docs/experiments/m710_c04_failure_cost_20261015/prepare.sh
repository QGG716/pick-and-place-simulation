#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-c04-cost-20261015
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1 ROS_HOME=/tmp/c04-cost-20261015/ros
export M710_MOVEIT_ASSET_ROOT=/work-static-prior-20261003
exec taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python -u tools/plan_m710_top_row_only.py --output /tmp/c04-cost-20261015/prepare-only --prepare-only --diagnostic-input /tmp/c04-cost-20261015/diagnostic-input.json --planning-mode static_prior_fast --static-prior /tmp/static-prior-20261003/static-prior-v3.json --fast-phase-budget-s 1 --slow-completion --worker-command /tmp/c04-cost-20261015/worker.sh --worker-binary /tmp/c04-cost-20261015/build/m710_moveit_worker --source-commit c7457aed7e6afd2e11ac861afa6474489fe85f97 --stage-budget-s 90 --box-budget-s 300 --batch-budget-s 900 --ipc-timeout-s 100 --seed 71070
