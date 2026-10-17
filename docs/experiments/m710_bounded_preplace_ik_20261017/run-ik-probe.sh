#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-single-contact-20261016
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-single-contact-20261016 ROS_HOME=/tmp/preplace-ik-20261017/ros-ik-probe
mkdir -p "$ROS_HOME"
exec taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python /tmp/preplace-ik-20261017/ik-probe.py
