# GPU server deployment record

The requested deployment target was described as an RTX 4090 Ubuntu server. The instance inspected on 2026-09-10 did not expose that GPU: both `nvidia-smi` and PyTorch identified one **NVIDIA RTX PRO 6000 Blackwell Server Edition** with 97,887 MiB VRAM and compute capability 12.0. Acceptance results in this repository therefore apply to that actual device and must not be represented as RTX 4090 measurements.

## Inspected boundary

- Ubuntu 22.04.5 LTS, amd64, system Python 3.10.12 and ROS 2 Humble from `/opt/ros/humble`.
- NVIDIA driver 595.71.05. The host-reported CUDA capability is not treated as an installed toolkit version.
- The session runs in a restricted container. The host presents 208 logical CPUs, while cgroup `cpu.max` limits the container to 25 CPU equivalents; memory is capped at 120 GiB.
- No Docker, Podman or Apptainer command is available inside the instance, so a nested container is neither required nor used.
- ROS uses the apt/system Python boundary. GPU inference uses `/root/v05-gpu-venv`; it contains no `rclpy`. Model/cache/log artifacts live under the configurable persistent root (the acceptance run used `/root/autodl-tmp/v05-acceptance` and `/root/autodl-tmp/huggingface`).
- No firewall, driver, `/usr/bin/python3`, kernel, or unrelated process was changed. No reboot or hardware command was issued.

## Fixed inputs and software

- Main vision source: `1d208f2ed380a207e6e46b4a62d2ac640edfe477`.
- MoGe v2 source: `b942f00bdc2a2a23ebb474fbe034d487e6dcceec`; utils3d source dependency: `3fab839f0be9931dac7c8488eb0e1600c236e183`.
- SAM: `facebook/sam-vit-base@70c1a07f894ebb5b307fd9eaaee97b9dfc16068f`.
- MoGe weights: `Ruicheng/moge-2-vits-normal@26b477f41595707c5db6770294c0d1721e8ed4ed`.
- PyTorch 2.7.1 + CUDA 12.8 and Torchvision 0.22.1 + CUDA 12.8 are pinned separately from the ordinary Python dependency set. This version was selected for actual Blackwell `sm_120` execution; the driver was not modified.
- Mode B image SHA-256: `4ae030b9e29edb83976830011a3e221ffc375a98aaa20228a062dfdd1f8830d7`.
- Frozen proposal SHA-256: `43b44525eb8de42cbe7be77921025b9d5ec7311abc7146b27f9f0e55389c76c8`; frozen person-mask SHA-256: `6029140ff28a11a40f005b0ecc75d8406212ce00e9949ce85e3149695903cbf5`.

The proposal and person-mask stages are fixed inputs matched to that image. SAM segmentation, 2-D geometry, MoGe point-map inference, 3-D cuboid recovery and assembly run anew. Consequently `RAW_IMAGE_AUTOMATIC=false`; the run is real GPU inference with disclosed frozen prerequisites, not an automatic detector benchmark.

## Reproduction

Use the configurable commands in `docs/setup_ubuntu2204_ros2_humble.md`. `tools/server_preflight.sh` is read-only. `tools/prepare_server_env.sh` checks exact source identities, dependency consistency, model hashes and a synchronized CUDA tensor calculation before any model smoke test. GPU stages run sequentially, batch size 1, to bound residency. The benchmark is finite and records cold/warm status, stage wall time and sampled per-process GPU memory.

The real-image ROS demonstration intentionally supplies no invented camera-to-world transform or metric calibration. Its success condition is fresh GPU output reaching `perception_node` and `world_bridge_node`, followed by a non-plannable world snapshot with explicit blocking reasons. It is not a grasp-success or physical-planner test.

`enable_hardware` remains false. Setting it true is rejected because this branch contains no verified FANUC hardware adapter; real robot motion is outside this acceptance.
