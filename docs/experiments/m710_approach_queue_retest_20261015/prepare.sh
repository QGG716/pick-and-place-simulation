#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-approach-queue-20261015
export PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1
/opt/planning-opt-20261003-venv/bin/python -m pytest -q tests/test_approach_connection_tiers.py tests/test_static_prior_adapter.py tests/test_moveit2_native_cold.py tests/test_planning_only.py tests/test_planning_only_timing.py tests/test_top_row_retest_summary.py > /tmp/approach-queue-20261015/python-tests.log 2>&1
cmake -S ros2/m710_moveit_backend -B /tmp/approach-queue-20261015/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/approach-queue-20261015/build --target m710_moveit_worker m710_static_prior_test m710_request_evidence_test m710_native_semantic_event_test -j2
for test in m710_static_prior_test m710_request_evidence_test m710_native_semantic_event_test; do
 /tmp/approach-queue-20261015/build/$test > /tmp/approach-queue-20261015/$test.log 2>&1
done
echo PREPARE_COMPLETE
