# Isaac perception / ROS 2 validation

Validation date: 2026-09-11 (Asia/Shanghai)

This record covers the `feat/v0.5-perception-ros2` work that started at
`c66b97ff2d967f8b21e9311f521d99574486309b`. The rendered capture was produced
at `1b9b9764365ae17dd02ff7cc222be4e30ea252b0`; the final resident Mode B1 run
used `9153266a25b5a4e044af9a94a707ce62d75b55f1`, and the live ROS time probe
used `0bde7ad542e42d722f1e3b4aab01efe0dcf4ce0e`.

## Environment and frozen identities

- Host: Ubuntu 22.04.5 LTS.
- GPU: NVIDIA RTX PRO 6000 Blackwell Server Edition, driver 595.71.05.
- Isaac Sim: 6.0.1.0, isolated Python 3.12 environment.
- ROS: ROS 2 Humble, isolated system Python 3.10 environment; `vision_msgs`
  4.1.1 was installed from the official Humble package repository.
- Vision worker: isolated GPU environment and upstream commit
  `1d208f2ed380a207e6e46b4a62d2ac640edfe477`.
- Feasibility reference: detached worktree at
  `f84578a3c9a91364b747560b994f2e6e9b5d65a2`. It was never modified, merged,
  committed, or pushed by this validation.
- Layout: `m710id70_unloading_layout_v1`, fingerprint
  `dada3b46c570791cf49745e2fb61003b9aac39c9529e8be94c9caf27a7938804`.
- Frozen initial scene fingerprint:
  `9ef799617486ee8986777d96d0cdde5cf46c807782ac8cd8f6ad72a90a2a8698`.
- Contract fingerprint:
  `33ca659b731678a3f8fa0784b413c152054c07f54af046b54a9465d456758863`.

The current feasibility exporter was also attempted. It failed closed with
`TOOL_SELF_COLLISION` between `tool_rigid_13` and `J5_link`; therefore this run
uses the archived, previously validated bundle from the exact reference SHA
above. The failed current export was not relabelled as a pass.

## Frames and camera

World frame `W` has its origin at the projection of the carton-stack front-face
centre onto the trailer floor. `+X` points into the trailer, `+Y` points left,
and `+Z` points up. Internal geometry is SI. `T_W_C` maps camera optical
coordinates (`+X` right, `+Y` down, `+Z` forward) into `W`:

```text
[[ 0.0, -0.7863183388,  0.6178215519, -0.8],
 [-1.0,  0.0,           0.0,           0.0],
 [ 0.0, -0.6178215519, -0.7863183388,  2.6],
 [ 0.0,  0.0,           0.0,           1.0]]
```

The validation camera is `front_validation_camera` /
`front_validation_camera_optical`, 640x480, `K = [560, 0, 319.5, 0, 560,
239.5, 0, 0, 1]`, `plumb_bob` with five zero distortion coefficients, near/far
0.05/12.0 m, 10 Hz, 24 mm focal length and 27.4285714286 mm horizontal
aperture. The camera-change scene translates the camera by +0.15 m in `W.Y`
and starts `sim-epoch-camera-b`; the old calibration is not reused.

## Acceptance result

Mode A (`ISAAC_GROUND_TRUTH`) passed in all six deterministic scenes. Each
manifest contains 40 stable cartons. RGB, metric depth (finite fraction 1.0),
CameraInfo, world point cloud, camera/world TF, object/semantic identities,
2D boxes, instance masks, 3D poses and full dimensions were captured. All six
`PlanningWorldSnapshot` and feasibility handoff artifacts passed. Mode A
unknown-region counts were 2, 2, 2, 2, 4 and 3. Ordered snapshot scene revisions
were 0 through 5; the 50 mm translation and 10 degree rotation changed dynamic
scene and world identities without changing the layout fingerprint.

Mode B1 (`ISAAC_SENSOR_WITH_ORACLE_PROPOSALS`) also passed in all six scenes.
One resident GPU worker loaded SAM and MoGe once and then ran the actual pinned
SAM, 2D geometry, MoGe, 3D geometry and assembly stages for every RGB keyframe.
Proposal source is explicitly `ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL` and
`RAW_IMAGE_AUTOMATIC=false`; the proposal bbox IoU of 1.0 is therefore not a
detector metric. GT was used only for proposals, evaluation and display—never
to replace depth, pose, dimensions, rejected geometry, or eligibility.

| Scene | GT / proposals / predictions | mask IoU | predicted-mask bbox IoU | world centre error m | orientation error deg | runtime s |
|---|---:|---:|---:|---:|---:|---:|
| static_full_stack | 40 / 38 / 35 | 0.6229 | 0.7428 | 0.8198 | 121.73 | 13.50 |
| single_target_focus | 40 / 38 / 34 | 0.6144 | 0.7447 | 0.7829 | 134.92 | 12.84 |
| known_translation | 40 / 38 / 33 | 0.6212 | 0.7479 | 1.1052 | 131.52 | 12.64 |
| known_rotation | 40 / 38 / 35 | 0.6253 | 0.7381 | 1.1185 | 138.84 | 12.77 |
| partial_occlusion | 40 / 36 / 33 | 0.6362 | 0.7852 | 0.8957 | 130.83 | 12.48 |
| camera_transform_change | 40 / 37 / 34 | 0.4955 | 0.6831 | 1.0138 | 135.62 | 11.94 |

World errors use `T_W_object_prediction = T_W_C @ T_C_object_prediction`.
They are deliberately large because MoGe output is monocular pseudo-3D:
absolute depth MAE is 0.5131–0.7785 m, while scale-aligned MAE is
0.0436–0.0967 m. The scale-aligned number is not advertised as metric accuracy.
Mean per-axis full-dimension absolute errors across individual scenes are
approximately 0.492–0.534 m, 0.202–0.277 m and 0.063–0.155 m. Every B1 result
remains ineligible with `MONOCULAR_SCALE_UNVERIFIED` and related evidence
reasons; B1 unknown-region counts are 33–35. Late GPU results remain eligible
for their bound historical evaluation record but cannot replace a newer live
world snapshot.

The external Humble adapter published `/clock`, RGB, depth, CameraInfo, point
cloud, `/tf`, `/tf_static`, `/joint_states`, GT Detection2D/3D and the external
domain observation. A live isolated-DDS probe observed five advancing frames
in `sim-epoch-a`, seven duplicate clock samples while paused, and a reset to
frame/time zero in the new `sim-epoch-camera-b`. Every observed RGB, depth,
CameraInfo, point-cloud and TF stamp matched its `/clock`/domain capture stamp.
The adapter is the clock authority and consequently uses a wall timer; consuming
perception/world nodes use `use_sim_time=true`.

The handoff checker loaded every generated handoff against the detached
feasibility worktree at the stated SHA. All six passed with zero differences
for axes, units, transforms, robot/assets, carton IDs/poses/dimensions,
quaternion convention, layout/dynamic identity separation, unknown regions and
eligibility. This is interface compatibility only; it does not claim IK,
collision, grasp, dynamics, trajectory or unloading-task success.

## Visual and machine-readable evidence

- [Isaac overview](evidence/isaac-evidence-9153266/static_full_stack_overview.png)
- [Sensor RGB](evidence/isaac-evidence-9153266/single_target_focus_rgb.png)
- [Metric-depth visualization](evidence/isaac-evidence-9153266/single_target_focus_depth.png)
- [GT overlay](evidence/isaac-evidence-9153266/single_target_focus_gt.png)
- [Prediction mask/status overlay](evidence/isaac-evidence-9153266/single_target_focus_prediction.png)
- [Four-panel RGB/GT/prediction/3D comparison](evidence/isaac-evidence-9153266/single_target_focus_comparison.png)
- [Occlusion comparison](evidence/isaac-evidence-9153266/partial_occlusion_comparison.png)
- [Translation comparison](evidence/isaac-evidence-9153266/known_translation_comparison.png)
- [Rotation comparison](evidence/isaac-evidence-9153266/known_rotation_comparison.png)
- [Mode B1 summary](evidence/isaac-evidence-9153266/mode_b1_summary.json)
- [ROS time probe](evidence/isaac-evidence-9153266/ros_time_sequence_report.json)
- [SHA-256 manifest](evidence/isaac-evidence-9153266/SHA256SUMS)

Large artifacts remain at
`/root/autodl-tmp/pick-and-place-simulation-v0.5-perception-ros2/outputs/isaac_perception_validation_1b9b976`.
The Mode A MP4 is `isaac_perception_validation.mp4`, SHA-256
`124b4606b1ad5b56f9d7b11cd7d3278fc668fc4f314c29101073d19cc37a88216`.
The four-panel Mode B1 MP4 is `isaac_perception_mode_b1_validation.mp4`, SHA-256
`ec7bba8af2a747c07e4a794016108a1d091c05ef013001b06c9bb05c04d0c7b2`.

## Test boundary

- CPU: `301 passed, 1 deselected` (Isaac marker excluded from ordinary runs).
- Humble: 3 packages built; 5 tests, 0 errors, failures or skips.
- Isaac/GPU: six-scene capture PASS, six-scene Mode A PASS, six-scene resident
  Mode B1 PASS, ROS time-sequence probe PASS, six handoff probes PASS, clean
  shutdown confirmed.
- Not implemented by design: Mode B2/raw-image automatic detection, online
  continuous planning, IK/collision/grasp/dynamics, robot trajectory or real
  robot/PLC execution. Synthetic-rendered-image measurements do not establish
  real-camera accuracy or real-robot safety.
