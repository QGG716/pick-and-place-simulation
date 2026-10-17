#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-single-contact-20261016
cmake -S ros2/m710_moveit_backend -B /tmp/preplace-ik-20261017/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/preplace-ik-20261017/build --target m710_bounded_ik_test m710_native_ik_probe m710_moveit_worker m710_native_semantic_event_test m710_request_evidence_test -j2
for name in m710_bounded_ik_test m710_native_semantic_event_test m710_request_evidence_test; do
 /tmp/preplace-ik-20261017/build/$name > /tmp/preplace-ik-20261017/$name.log 2>&1
done
echo BUILD_NATIVE_TESTS_COMPLETE
cd /work-single-contact-20261016
PYTHONPATH=src /opt/planning-opt-20261003-venv/bin/python -m pytest -q tests/test_single_contact.py tests/test_single_extraction.py tests/test_single_preplace.py tests/test_bounded_preplace_ik.py > /tmp/preplace-ik-20261017/python-tests.log 2>&1
echo ALL_DONE
