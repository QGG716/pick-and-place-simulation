# Ubuntu 22.04 / ROS 2 Humble setup

This is the only formal acceptance environment. Do not substitute Jazzy or change `/usr/bin/python3`.

```bash
source /opt/ros/humble/setup.bash
test "$ROS_DISTRO" = humble
python3 -c 'import sys, rclpy; assert sys.version_info[:2] == (3, 10); print(sys.executable)'

python3 -m venv --system-site-packages .venv-humble
source .venv-humble/bin/activate
python -m pip install ./packages/unloading_contracts
python -m pip install -e '.[dev]'

cd ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Source `/opt/ros/humble/setup.bash` before activating the `--system-site-packages` venv so its interpreter can see apt-provided `rclpy`. The startup guard verifies both `ROS_DISTRO=humble` and Python 3.10.

Launch the replay/mock graph after providing a real path and publishing `sensor_msgs/JointState` plus the required capture-time TF:

```bash
ros2 launch unloading_bringup replay_mock.launch.py \
  replay_path:=$(pwd)/../tests/fixtures/vision_upstream/cargo7_minimal.json
```

The replay is expected to publish a complete but non-plannable snapshot because the real monocular sample lacks independently verified scale/extrinsics/uncertainty. Use `python -m unloading_perception.demo --mode synthetic` for the explicitly synthetic contract/mock execution demonstration; it is not physical planning evidence.

The visual worker environment is separate. At fixed SHA `1d208f2`, its declared geometry set pins NumPy 1.26.4, SciPy 1.14.1, OpenCV 4.10.0.84, Torch 2.5.1, and Torchvision 0.20.1. Do not install these into the ROS interpreter merely to run replay.
