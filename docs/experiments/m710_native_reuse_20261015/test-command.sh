#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-native-reuse2-20261015
E=docs/experiments/m710_native_reuse_20261015
B=/tmp/native-reuse-20261015/build
mkdir -p "$E/tests"
for t in "m710_process_geometry_test" "m710_process_policy_test reference" "m710_process_policy_test optimized" "m710_fixed_reuse_test" "m710_clearance_test optimized" "m710_clearance_test optimized reuse" "m710_native_zero_motion_test" "m710_native_semantic_event_test" "m710_static_prior_test"; do
  $B/$t > "$E/tests/${t// /-}.log" 2>&1
  echo "PASS $t"
done
