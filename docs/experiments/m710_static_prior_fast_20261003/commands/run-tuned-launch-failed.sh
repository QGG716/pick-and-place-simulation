#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-static-prior-20261003
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1 ROS_HOME=/tmp/static-prior-20261003/ros
SOURCE_COMMIT=$(git rev-parse HEAD)
/opt/planning-opt-20261003-venv/bin/python -m pytest -q tests/test_static_prior_adapter.py tests/test_static_prior_binding.py tests/test_static_prior_extraction_exit.py tests/test_planning_only.py tests/test_planning_only_timing.py tests/test_moveit2_backend.py tests/test_extraction_boundary_guard.py
for carton in carton_l07_c03 carton_l07_c04; do
 /opt/planning-opt-20261003-venv/bin/python -u tools/plan_m710_top_row_only.py --output /tmp/static-prior-20261003/tuned-$carton --planning-mode static_prior_fast --static-prior /tmp/static-prior-20261003/static-prior-v3.json --fast-phase-budget-s 300 --slow-completion --worker-command /tmp/static-prior-20261003/worker-current.sh --worker-binary /tmp/static-prior-20261003/build/m710_moveit_worker --source-commit "$SOURCE_COMMIT" --stage-budget-s 90 --box-budget-s 300 --batch-budget-s 300 --ipc-timeout-s 100 --seed 71070 --diagnostic-input /tmp/static-prior-20261003/$carton-task-input.json
done
