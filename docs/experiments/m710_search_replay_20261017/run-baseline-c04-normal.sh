#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-single-contact-20261016
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1 M710_MOVEIT_ASSET_ROOT=/work-single-contact-20261016
export ROS_HOME=/tmp/search-trace-20261017/ros-baseline-c04-normal
mkdir -p "$ROS_HOME"
taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py --through-preplace --target carton_l07_c04 --initial-state /tmp/preplace-ik-20261017/c04-input.json --output /tmp/search-trace-20261017/baseline-c04-normal --worker-command /tmp/search-trace-20261017/worker.sh --worker-binary /tmp/search-trace-20261017/build/m710_moveit_worker --source-commit 1a2407a18961aa2729bdc96dc2dd1a7c27402acd --seed 71070 --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100
