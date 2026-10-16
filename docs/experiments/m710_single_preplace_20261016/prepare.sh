#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-single-contact-20261016
cmake -S ros2/m710_moveit_backend -B /tmp/single-preplace-20261016/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/single-preplace-20261016/build --target m710_moveit_worker m710_native_semantic_event_test m710_request_evidence_test -j2
for name in m710_native_semantic_event_test m710_request_evidence_test; do
 /tmp/single-preplace-20261016/build/$name > /tmp/single-preplace-20261016/$name.log 2>&1
done
echo BUILD_NATIVE_TESTS_COMPLETE
