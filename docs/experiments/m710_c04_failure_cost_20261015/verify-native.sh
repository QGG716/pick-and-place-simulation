set -e
source /opt/ros/humble/setup.bash
cmake --build /tmp/c04-cost-20261015/build --target m710_moveit_worker m710_request_evidence_test m710_static_prior_test m710_native_semantic_event_test -j2
/tmp/c04-cost-20261015/build/m710_request_evidence_test
/tmp/c04-cost-20261015/build/m710_static_prior_test
/tmp/c04-cost-20261015/build/m710_native_semantic_event_test
echo NATIVE_TESTS_COMPLETE
