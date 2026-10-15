set -e
source /opt/ros/humble/setup.bash
cmake -S /work-c04-cost-20261015/ros2/m710_moveit_backend -B /tmp/c04-cost-20261015/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/c04-cost-20261015/build --target m710_moveit_worker m710_request_evidence_test m710_static_prior_test m710_native_semantic_event_test -j2
