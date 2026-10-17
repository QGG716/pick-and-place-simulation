#!/bin/bash
source /opt/ros/humble/setup.bash
set -u
cd /work-single-contact-20261016
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-single-contact-20261016
O=/tmp/preplace-ik-20261017
for c in c02 c04; do
 export ROS_HOME=$O/ros-$c
 mkdir -p "$ROS_HOME"
 extra=()
 if [ "$c" = c04 ]; then extra=(--initial-state $O/c04-input.json); fi
 date -u +%FT%TZ
 echo CASE_START=$c
 taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py --through-preplace --target carton_l07_$c --output $O/$c --worker-command $O/worker.sh --worker-binary $O/build/m710_moveit_worker --source-commit 9a2856be05ee45560b8e3c719e1964824089d2c2 --seed 71070 --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100 "${extra[@]}" > $O/$c-stdout.txt 2>&1
 rc=$?
 echo CASE_EXIT=$c:$rc
 cat $O/$c-stdout.txt
 if [ "$rc" != 0 ]; then echo IMPLEMENTATION_OR_ENVIRONMENT_ERROR_STOP; exit "$rc"; fi
 done
 echo CASES_COMPLETE
