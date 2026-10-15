set -e
source /opt/ros/humble/setup.bash
/tmp/native-pair-profile-20261015/build/m710_narrow_reuse_test
/tmp/native-pair-profile-20261015/build/m710_fixed_reuse_test
/tmp/native-pair-profile-20261015/build/m710_clearance_test optimized reuse
/tmp/native-pair-profile-20261015/build/m710_pair_profile_test
python3 /work-native-pair-profile-20261015/docs/experiments/m710_native_pair_profile_20261015/worker-probe.py
