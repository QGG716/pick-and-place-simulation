# Perception / ROS 2 GPU acceptance record

Date: 2026-09-10 (Asia/Shanghai)

## Decision

The tested code commit is `9e61c4f305fe2e6859d2a286ce048cc86a0eb4b2` on
`feat/v0.5-perception-ros2`. It passes the server CPU suite, both frozen
consumer compatibility suites, the ROS 2 Humble build and launch tests, a real
CUDA inference smoke test, a real-image GPU-to-ROS round trip, and a bounded
GPU benchmark.

The real-image result is deliberately **not execution-admissible**. It has
model-estimated monocular geometry, no validated metric scale, no camera-to-world
extrinsics, no geometry uncertainty, and unknown/untransformed regions. The
world bridge preserves those limitations and rejects planning/execution instead
of silently converting the image into trusted free space.

No physical planner, robot, PLC, or end effector was exercised. Hardware remains
disabled (`enable_hardware=false`); the action server used below is a ROS mock.

## Frozen references

| Item | Frozen value |
|---|---|
| Tested branch | `feat/v0.5-perception-ros2` |
| Tested code commit | `9e61c4f305fe2e6859d2a286ce048cc86a0eb4b2` |
| Visual upstream | `1d208f2ed380a207e6e46b4a62d2ac640edfe477` |
| MoGe source | `b942f00bdc2a2a23ebb474fbe034d487e6dcceec` |
| utils3d source | `3fab839f0be9931dac7c8488eb0e1600c236e183` |
| Feasibility consumer | `a372c7f117e61509229b2c15eef344cd7908d499` |
| Online consumer | `31f160d3f47d0b801ab5698e16dc7267e89434f1` |
| Shared schema/package | `1.1.0` |
| Tested-code Git bundle SHA-256 | `d9a5792d5948330ec11275539b7653dad7c9080a429a39861d129abf3c9874ab` |

The bundle was expanded to `/root/v05-exact-9e61c4f`; `git rev-parse HEAD`
returned the tested commit. The tracked checkout was clean after the runs, and
`git diff --check` passed.

## Server facts

The rented host exposed Ubuntu 22.04.5 LTS, ROS 2 Humble, `/usr/bin/python3.10`
3.10.12, 208 logical CPUs, a cgroup quota equivalent to 25 CPU cores, and a
120 GiB memory limit. All CPU and GPU validations in this record ran on that
host.

The supplied machine is not the advertised RTX 4090. `nvidia-smi` reported:

```text
NVIDIA RTX PRO 6000 Blackwell Server Edition
97887 MiB
driver 595.71.05
compute capability 12.0
```

The isolated GPU environment used PyTorch `2.7.1+cu128` with CUDA `12.8`.
`pip check` reported no broken requirements, and a synchronized CUDA tensor
operation completed on the named device (result `22898102272.0`). No CPU or
synthetic inference fallback was allowed by the smoke scripts.

## Pinned inputs and models

`RAW_IMAGE_AUTOMATIC=false`: the source image is real, while the proposal JSON
and person masks are frozen upstream artifacts. SAM mask cross-validation,
2-D geometry, MoGe inference, 3-D geometry, and final assembly were actually
rerun. This run must not be described as fully automatic raw-image inference.

| Artifact | Revision / SHA-256 |
|---|---|
| Input image | `4ae030b9e29edb83976830011a3e221ffc375a98aaa20228a062dfdd1f8830d7` |
| Frozen proposal JSON (Linux Git blob bytes) | `43b44525eb8de42cbe7be77921025b9d5ec7311abc7146b27f9f0e55389c76c8` |
| Frozen person masks | `6029140ff28a11a40f005b0ecc75d8406212ce00e9949ce85e3149695903cbf5` |
| `facebook/sam-vit-base` revision | `70c1a07f894ebb5b307fd9eaaee97b9dfc16068f` |
| SAM `model.safetensors` | `892c410e496344e527255ccdcb2cb7244a609acb5389c7c4fdbaa1288f861c579` |
| SAM `config.json` | `5ebd0d8643b486f3a716bf17c2a15531eb818b2b96ff0c6c5dcc88fa0115161af` |
| SAM `preprocessor_config.json` | `225545a743c654e3c495ec6f545a0eaba57c8ba3fbbd88483b3cb1c0fc58db517` |
| `Ruicheng/moge-2-vits-normal` revision | `26b477f41595707c5db6770294c0d1721e8ed4ed` |
| MoGe `model.pt` | `79a16621928c2bf0ed04659218c55c01075e950507f40bb3332fb4c873d3e1dc` |

Model manifest:
`/root/autodl-tmp/v05-acceptance/model-manifest.json`
(SHA-256 `8960f5dc7af364a06b6e0537d0d80527e9556b01da484647f6cd73d247941c30`).

## Results actually obtained

### CPU and frozen consumers

`tools/run_server_cpu_acceptance.sh` was run against the exact checkout with an
isolated Python 3.10 environment.

| Suite | Result |
|---|---|
| This repository | `276 passed, 1 deselected in 5.05s` |
| Frozen feasibility consumer plus minimal adapter patch | `9 passed in 0.24s` |
| Frozen online consumer plus minimal re-export patch | `138 passed in 2.98s` |

The consumers remained detached at their fixed commits; neither consumer branch
was modified or pushed. The feasibility patch SHA-256 is
`3ff19a1acafe91ddbf8b61ff6dedd7eab51a727a0a89a9740d8cc8d241a9edab`;
the online patch SHA-256 is
`c16a72732871d03d370c3e852ebb553d1d975b41d6c503eb56f1706540507bfa`.

Machine-readable report:
`/root/autodl-tmp/v05-acceptance/exact-9e61c4f/cpu/three-branch-contracts.json`
(SHA-256 `392167611a803afb0e83100b4e900e3646e7a524f0d02c8d29c9d9f850f74781`).

### ROS 2 Humble and synthetic cancel/stop chain

`tools/run_humble_acceptance.sh` built `unloading_interfaces`,
`unloading_ros_bridge`, and `unloading_bringup`, then ran the installed tests:

```text
Summary: 3 packages finished
Summary: 5 tests, 0 errors, 0 failures, 0 skipped
```

The full launch integration publishes a synthetic metric observation, constructs
the world/context/grant, submits a timed trajectory to a mock
`FollowJointTrajectory` controller, observes `STARTED` and feedback, accepts a
cancel request, receives a `CANCELED` terminal result, and requires a separately
correlated controller stop acknowledgement. It asserts that the canceled goal
never becomes `SUCCEEDED`.

JUnit XML:
`/root/autodl-tmp/v05-acceptance/humble/exact-9e61c4f/build/unloading_ros_bridge/pytest.xml`
(SHA-256 `5133689b93e97e8fb3f7cef65059c652a9747b89059c7dff2b8d153874da3f77`).

### Real image, actual GPU, and ROS rejection

The exact-checkout GPU smoke completed all stages and produced a `PARTIAL`
observation with 39 cargo records and 39 unknown regions in 39.614 s. The peak
sampled process GPU allocation was 3790 MiB (SAM); MoGe peaked at 1602 MiB.

| Stage | Seconds |
|---|---:|
| SAM | 20.890 |
| 2-D geometry | 2.185 |
| MoGe | 6.102 |
| 3-D geometry | 7.070 |
| Assembly | 0.634 |
| CUDA probe | 0.181 |

The same real-image path was then passed through the actual Humble perception
and world bridges. It completed in 43.296 s, retained 39 cargo records and 39
unknown regions, and verified an identical domain-to-ROS-to-domain world fingerprint:
`e3f924e58ddb23fd42c31ef40e4abbebf3db97a6faece750ad14a93ffb3c55228`.
`planning_admissible` was `false`; the world gate reported
`OBSERVATION_STALE` and `UNKNOWN_OR_UNTRANSFORMED_REGIONS`. Cargo eligibility
also retains `MONOCULAR_SCALE_UNVERIFIED`, `CAMERA_EXTRINSICS_MISSING`, and
`GEOMETRY_UNCERTAINTY_MISSING`.

Evidence files:

| File | SHA-256 |
|---|---|
| `/root/autodl-tmp/v05-acceptance/gpu/exact-9e61c4f/result.json` | `7b272b56a086bb6a17f0f3ae117952554ad23433cf6a757416b5edda136ff781f` |
| `/root/autodl-tmp/v05-acceptance/gpu/exact-9e61c4f/worker-runs/mode-b-a0b549b2-e5f0-449d-b127-605cedb241fd-000000-172ef2e2-3a0e-495d-bdd0-cb05c4d9a7ba/metrics.json` | `32d25dd4223200867e49147584795281abd68550f6c7112ce2a052ffdc9b35f7` |
| `/root/autodl-tmp/v05-acceptance/ros-gpu/exact-9e61c4f/summary.json` | `2acf58bb8d859be544f1ce9c3d8c876afc94455495ee7f940e73f0a06f6999a9` |

The worker recorded and verified returned-artifact hashes before conversion:

| Worker artifact | SHA-256 |
|---|---|
| `cargo_masks.npz` | `f15d65e001704a1519e8b94671b09f0832294996c9230850b2d152e1ebe179cc8` |
| `cargo_instances.json` | `3706bc479fec3dfb5f10777020c4dc77157eef208ec55d4598390976d318acee9` |
| `sam_crossvalidated.jpg` | `3ad1aa558762f0784dbe3fc8997442e005354bb4e84a304e1ae60a9d1c18c0bbc` |
| `box_geometry_2d.json` | `44f8631f877f44868d71fa619375298382690e3c86babb6e6bf449e323f8ce99e` |
| `moge2_pointmap.npz` | `9d403edc3c5af65a212bf73c76e34d388169cbafc5117382c6051f6889de4044d` |
| `box_cuboids_instance_aware.json` | `b6d7efa23502ee7cd31b638ee670cbe96e32b62ce22cd233bdab7a248ec59e669` |
| `final_instance_aware.jpg` | `136674de2ba269d17f72b01ad44682c9cf23c052ad28937d80e257b9375ee5009` |
| `final_instance_aware.json` | `4a0c6bb661db66a6202bf35b16479f12c9459fc1e6370620217114939d0716008` |

### Bounded GPU benchmark

The benchmark ran on the exact commit with one warmup and three measured
repetitions of one fixed image. Each request used one bounded worker process and
sequential model stages.

| Metric | Result |
|---|---:|
| Measured repetitions | 3 |
| Minimum | 39.474 s |
| Median | 40.000 s |
| Maximum | 40.482 s |
| Mean | 39.985 s |
| Peak sampled process GPU memory | 3790 MiB |

Measured mean stage times were SAM 20.965 s, MoGe 6.238 s, 2-D geometry
2.049 s, 3-D geometry 7.268 s, assembly 0.654 s, and CUDA probe 0.175 s.
Three repetitions of one image are not enough for defensible P95/P99,
throughput, or dataset-generalization claims; none are reported.

Benchmark summary:
`/root/autodl-tmp/v05-acceptance/gpu/exact-9e61c4f-benchmark/benchmark-summary.json`
(SHA-256 `de73ae2b75c321f4605e07db0b2ffa933b8f55e2a3163fc13cd6b3e7b13605eb`).

## Fail-closed regression coverage

The server-run suites include negative coverage for:

- pose/rotation changes affecting scene fingerprints while heartbeat-only
  changes do not; bounded serialization roundoff is normalized, but materially
  non-orthogonal or left-handed axes are rejected;
- unknown-only and failed observations never becoming free space, missed and
  ambiguous detections retaining conservative occupancy, and stale epochs or
  duplicate observations being rejected;
- missing TF, missing/stale robot state, missing/stale mechanism state, missing
  metric calibration, and missing uncertainty blocking admission;
- absent, mismatched, expired, duplicate, stale-generation, changed-world, or
  changed-trajectory execution grants being rejected;
- NaN values, wrong joint ordering, missing/non-increasing time, and provenance
  mismatches being rejected at the trajectory boundary;
- rejected goals cleaning up state, cancel acceptance remaining distinct from
  physical stop, and wrong goal/controller epoch/sequence or excessive velocity
  being unable to acknowledge stop;
- action cancel/result reordering mapping to `CANCELED`, never `SUCCEEDED`;
- worker timeout, crash, bad schema, unavailable GPU/model, injected OOM,
  out-of-root artifact paths, late results, and hash mismatches failing closed.

The OOM case is deterministic fault-injection coverage; the acceptance did not
attempt to exhaust the server GPU.

## Limits and remaining evidence

This record establishes software-contract, ROS communication, mock execution,
and actual GPU inference behavior for the tested commit. It does not validate
perception accuracy over a representative dataset, calibrated metric geometry,
camera/world registration, collision-free physical planning, payload/dynamics
limits, controller following accuracy, stop-category performance, or real robot
safety. Those facts remain prerequisites before any hardware enablement.
