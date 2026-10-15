set -e
source /opt/ros/humble/setup.bash
cmake -S /work-native-pair-profile-20261015/ros2/m710_moveit_backend -B /tmp/native-pair-profile-20261015/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/native-pair-profile-20261015/build --target m710_narrow_reuse_test m710_fixed_reuse_test m710_clearance_test m710_pair_profile_test m710_process_benchmark m710_moveit_worker -j2
