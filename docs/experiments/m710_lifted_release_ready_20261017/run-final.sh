#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-single-contact-20261016
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=$PWD
taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py \
  --through-lifted-release-ready --target "carton_l07_$1" --seed 71070 \
  --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100 \
  --worker-command /tmp/lift-native-20261017/worker.sh \
  --worker-binary /tmp/lift-native-20261017/build/m710_moveit_worker \
  --source-commit f896a9d84675125678dd32208de1183b2fb8dadb --output "/tmp/lift-native-20261017/$1-final"
