#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-single-contact-20261016
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
/opt/planning-opt-20261003-venv/bin/python -m pytest -q tests/test_single_contact.py tests/test_approach_connection_tiers.py tests/test_moveit2_native_cold.py tests/test_planning_only.py tests/test_independent_cups.py > /tmp/single-contact-20261016/python-tests.log 2>&1
cmake -S ros2/m710_moveit_backend -B /tmp/single-contact-20261016/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/single-contact-20261016/build --target m710_moveit_worker m710_native_semantic_event_test m710_request_evidence_test m710_clearance_test -j2
for test in m710_native_semantic_event_test m710_request_evidence_test m710_clearance_test; do
 /tmp/single-contact-20261016/build/$test > /tmp/single-contact-20261016/$test.log 2>&1
done
echo PREPARE_COMPLETE
