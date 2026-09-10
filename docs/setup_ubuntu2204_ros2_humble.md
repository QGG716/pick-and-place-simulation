# Ubuntu 22.04 / ROS 2 Humble setup

This is the only formal ROS acceptance environment. Do not substitute Jazzy or change `/usr/bin/python3`. The GPU worker is a separate Python 3.10 venv and never imports `rclpy`.

```bash
tools/server_preflight.sh /persistent/path/server-preflight.txt
ARTIFACT_ROOT=/persistent/path/v05-acceptance tools/run_server_cpu_acceptance.sh
ARTIFACT_ROOT=/persistent/path/v05-acceptance RUN_ID=final tools/run_humble_acceptance.sh
```

`run_humble_acceptance.sh` explicitly sources `/opt/ros/humble/setup.bash`, restores `/usr/bin` ahead of Conda, checks Humble/Python 3.10, creates isolated build/install/log roots, runs `colcon build`, runs the discovered tests with failure propagation, and finishes with verbose `colcon test-result`. Missing ROS packages are a hard failure.

Prepare the worker only after placing the exact MoGe v2 source at its audited commit. CUDA wheels and ordinary packages use separate fixed requirement files so pip does not resolve every ordinary package against the CUDA index:

```bash
MOGE_SOURCE=/path/to/MoGe-at-b942f00 \
GPU_VENV=/persistent/path/v05-gpu-venv \
HF_CACHE=/persistent/path/huggingface \
ARTIFACT_ROOT=/persistent/path/v05-acceptance \
tools/prepare_server_env.sh

ARTIFACT_ROOT=/persistent/path/v05-acceptance \
VISION_ROOT=/path/to/vision-at-1d208f2 \
RUN_ID=smoke-final tools/run_gpu_vision_smoke.sh
```

The preparation script refuses a different MoGe source SHA, installs PyTorch 2.7.1 CUDA 12.8 and the bounded worker dependencies in the venv, checks installed dependency consistency, downloads only the fixed SAM/MoGe files, verifies a real CUDA tensor operation, and records a model manifest plus `pip freeze`. MoGe is exposed from the exact source checkout because its application metadata includes unrelated Gradio/CLI dependencies that the headless model API does not use.

Launch the replay/mock graph after providing a real path and publishing `sensor_msgs/JointState` plus the required capture-time TF:

```bash
ros2 launch unloading_bringup replay_mock.launch.py \
  replay_path:=$(pwd)/../tests/fixtures/vision_upstream/cargo7_minimal.json
```

The replay is expected to publish a complete but non-plannable snapshot because the real monocular sample lacks independently verified scale/extrinsics/uncertainty. Use `python -m unloading_perception.demo --mode synthetic` for the explicitly synthetic contract/mock execution demonstration; it is not physical planning evidence.

For the finite performance run, use `WARMUPS` and `MEASUREMENTS`; do not infer population percentiles from a tiny repeated sample. The checked-in default is 3 warm-ups and 10 measurements, one GPU and batch size 1:

```bash
WARMUPS=3 MEASUREMENTS=10 RUN_ID=benchmark-final tools/run_gpu_vision_benchmark.sh
HUMBLE_INSTALL=/persistent/path/v05-acceptance/humble/final/install \
RUN_ID=real-final tools/run_real_vision_ros_demo.sh
```

Do not install Torch, SAM or MoGe into the ROS interpreter merely to run replay or bridge tests. Model caches and full logs remain outside Git in the configured persistent artifact root.
