# J1-coupled mast RGB-D acceptance

This record covers the production-oriented perception layer on
`feat/v0.5-perception-ros2`. It does not qualify IK, collision, payload,
trajectory execution, real cameras, or real luminaires. The frozen capture was
made at perception commit `1f2f8b594f68678e347bede97d5230c51c549fe4`; the
registered-cuboid analysis includes the later deterministic axis fix at
`83bcb4aad335ed7661a36381b2194998674f26c5`.

## Outcome

Mode A, Mode B (`STAGED_RGBD`), and Mode C
(`STAGED_MONOCULAR_MOGE`) all completed on the GPU server. The isolated RGB-D
calibration box passed its fail-closed gate. The full-stack result is not an
accuracy pass: registered depth improved dimensions but did not resolve the
pose ambiguity of ten predominantly single-face boxes. Those instances emit
two hypotheses each and remain ineligible for execution.

## Required 41-item record

| # | Required fact | Result |
|---:|---|---|
| 1 | Start SHA | `0cc48561746744dfb7d9189255405deaaff73343` |
| 2 | Final implementation SHA before this evidence commit | `83bcb4aad335ed7661a36381b2194998674f26c5` |
| 3 | Feasibility reference SHA | `648e177a03d2a5a5d8a2f11e9141c57763325805`, detached/read-only; never merged |
| 4 | Actual GPU | NVIDIA RTX PRO 6000 Blackwell Server Edition, driver 595.71.05 |
| 5 | Actual Isaac version | `6.0.1.0` |
| 6 | J1 frame identity | Official URDF revolute `J1`, axis `+Z`; rotating child frame `J1_link` |
| 7 | Mast parent | `J1_link`; moving with J1, not world/static chassis/J2/J3 |
| 8 | Nominal J1 world pose | joint centre `[-1.325, +0.350, 1.165] m` |
| 9 | Nominal mast world pose | horizontal centre `[-1.325, +0.750] m`; flange point z `1.3293411964 m`; camera `[-1.325, +0.750, 2.6293411964] m` |
| 10 | J1-to-mast distance | `0.400 m` for q1 = −90°, −45°, 0°, +45°, +90° |
| 11 | Mast side | PASS: same X and `+0.400 m` world Y at nominal pose, therefore LEFT/+Y |
| 12 | Nominal q1 | `-1.5707963267948966 rad` (`-90°`) |
| 13 | Camera/J1 yaw offset | PASS: `+1.5707963267948966 rad` (`+90°`) throughout sweep |
| 14 | Arm/right-wall relation | PASS: nominal arm pitch plane Y-Z is perpendicular to the right-wall X-Z plane |
| 15 | Flange height | `0.16434119641780853 m` above J1 origin, `PROVISIONAL_ENGINEERING_DATUM`, from official J1 collision-mesh z-max; not manufacturing CAD |
| 16 | Mast height | `1.500 m` flange plane to top |
| 17 | Module height | optical centre `1.300 m` above flange, 0.200 m below mast top |
| 18 | RGB | `2592×1944`, requested/render clock `30 Hz`, HFOV `90°`, VFOV `65°`, `EXACT_USER_SPEC_RESAMPLED` |
| 19 | Depth | `2592×1944`, requested/render clock `30 Hz`, `optical_z_m` |
| 20 | RGB-depth calibration | PASS: common capture/calibration identity, common optical pose, `SIMULATION_IDEAL_REGISTERED_DEPTH`, exact K and resolution checks |
| 21 | Illumination geometry | two symmetric 5000 K disk proxies, ±0.120 m, intensity 2500 simulation units, capture synchronized; no real-lux claim |
| 22 | LIGHT_OFF/ON | Completed under controlled scene/camera/exposure/depth. Mean luminance `0.028617 → 0.029454`; underexposed fraction `0.932502 → 0.932322`; saturation `0 → 0`; mask IoU and retention `1.0`. This proxy produces only a small brightness change and is not a real-light performance qualification. |
| 23 | Actual sensor Hz | Isaac rendering/acquisition clock `30 Hz`; trigger and vision processing configured `2 Hz`. Sustained ROS publication throughput was not benchmarked; MP4 is a separate 20 fps presentation artifact. |
| 24 | FOV carton coverage | analytic frustum intersection `20/40 = 50%`; only `6/40` fully inside. Occlusion is not inferred by this analytic result. |
| 25 | Robot self-occlusion | semantic-pixel comparison: `0.0` at q1 −90° and −45°. At 0°/+45°/+90° no carton pixels exist even with the robot hidden, so result is `NOT_OBSERVABLE_NO_CARTON_PIXELS`, not a fabricated 100%. |
| 26 | RGB-D calibration box | PASS: centre `0.024879 m`, orientation `1.4931°`, mean absolute dimension error `0.009190 m`; limits `0.1 m / 20° / 0.15 m` |
| 27 | RGB-D full stack | 10 processed boxes: centre `0.956257 m`, orientation `131.914°`, mean absolute dimension error `0.205663 m`; this is a measured poor pose result, not an accuracy pass |
| 28 | MoGe full stack | same frame/proposals: centre `0.819571 m`, orientation `122.326°`, mean absolute dimension error `0.271251 m` |
| 29 | RGB-D relative improvement | full stack: centre `−16.68%`, orientation `−7.84%`, dimensions `+24.18%`. Calibration box: centre `+95.63%`, orientation `+98.26%`, dimensions `+95.13%`. |
| 30 | Multiple hypotheses | full stack `20` hypotheses for `10` ambiguous instances. Reasons include `GEOMETRIC_POSE_AMBIGUOUS`, `GEOMETRY_UNCERTAINTY_INCOMPLETE`, and alternate-axis disambiguation; candidates remain ineligible. |
| 31 | PlanningWorldSnapshot | 8 snapshots produced; each has 40 object slots. Full stack has 27 unknown regions. Status is compatible only within the stated gap. |
| 32 | Moving-rig feasibility handoff | `COMPATIBLE_WITH_KNOWN_GAP`; explicit missing capability `MISSING_MOVING_VISION_RIG_CONTRACT`; rig is nevertheless serialized as a J1-coupled moving obstacle. |
| 33 | Missing CAD | mast cross-section, flange/bracket, RGB-D body, and luminaires remain `VISUAL_PROXY`; `VISION_RIG_GEOMETRY_NOT_EXECUTION_QUALIFIED` |
| 34 | PNG paths | all required images are in [`evidence/isaac-j1-rgbd-1f2f8b5`](evidence/isaac-j1-rgbd-1f2f8b5) and listed below |
| 35 | MP4 | [`isaac_j1_mast_rgbd_validation.mp4`](evidence/isaac-j1-rgbd-1f2f8b5/isaac_j1_mast_rgbd_validation.mp4), SHA-256 `e4666a8dd243c7a988bd0580f5db7834b7c9efda26096f5599dcfe9fb7d024f0` |
| 36 | CPU tests | PASS: `318 passed, 1 deselected` after the final axis regression was added; Windows temp isolation supplied with `--basetemp` |
| 37 | Humble tests | PASS on Ubuntu 22.04/Humble: 3 packages built; `5 tests, 0 errors, 0 failures, 0 skipped` |
| 38 | Isaac/GPU tests | PASS for capture/runtime completeness: 8 scenes, official URDF exact-byte identity, max six-axis capture error `4.371139e-08 rad`, Mode A/B/C summaries complete. This is not execution qualification. |
| 39 | Local HEAD | Verified during final handoff; the exact post-documentation SHA is reported outside this self-referential record. |
| 40 | Remote HEAD | Verified equal to local during final handoff on `origin/feat/v0.5-perception-ros2`. |
| 41 | Git status | Local and GPU validation checkout verified clean during final handoff. |

The official M-710iD/70 URDF used by Isaac was
`assets/robots/fanuc_m710id_70/official/fanuc_m710_description/urdf/m710id_70_official.urdf`,
SHA-256 `2a813af47694819c44c888fe54e0041b045355ffb5273b012e3ff995b4999bcf`.

## Visual evidence

- [`01_mast_left_top_view.png`](evidence/isaac-j1-rgbd-1f2f8b5/01_mast_left_top_view.png)
- [`02_mast_j1_90deg_relation.png`](evidence/isaac-j1-rgbd-1f2f8b5/02_mast_j1_90deg_relation.png)
- [`03_j1_sweep_montage.png`](evidence/isaac-j1-rgbd-1f2f8b5/03_j1_sweep_montage.png)
- [`04_rgbd_module_closeup.png`](evidence/isaac-j1-rgbd-1f2f8b5/04_rgbd_module_closeup.png)
- [`05_dark_light_off.png`](evidence/isaac-j1-rgbd-1f2f8b5/05_dark_light_off.png)
- [`06_dark_light_on.png`](evidence/isaac-j1-rgbd-1f2f8b5/06_dark_light_on.png)
- [`07_fullres_rgb_preview.png`](evidence/isaac-j1-rgbd-1f2f8b5/07_fullres_rgb_preview.png)
- [`08_metric_depth_preview.png`](evidence/isaac-j1-rgbd-1f2f8b5/08_metric_depth_preview.png)
- [`09_registered_rgbd.png`](evidence/isaac-j1-rgbd-1f2f8b5/09_registered_rgbd.png)
- [`10_segmentation.png`](evidence/isaac-j1-rgbd-1f2f8b5/10_segmentation.png)
- [`11_rgbd_cuboids.png`](evidence/isaac-j1-rgbd-1f2f8b5/11_rgbd_cuboids.png)
- [`12_moge_cuboids.png`](evidence/isaac-j1-rgbd-1f2f8b5/12_moge_cuboids.png)
- [`13_rgbd_vs_moge.png`](evidence/isaac-j1-rgbd-1f2f8b5/13_rgbd_vs_moge.png)
- [`14_world_topview_cuboids.png`](evidence/isaac-j1-rgbd-1f2f8b5/14_world_topview_cuboids.png)

Machine-readable evidence includes
[`run_status.json`](evidence/isaac-j1-rgbd-1f2f8b5/run_status.json),
[`rgbd_calibration_gate.json`](evidence/isaac-j1-rgbd-1f2f8b5/rgbd_calibration_gate.json),
[`rgbd_vs_moge_summary.json`](evidence/isaac-j1-rgbd-1f2f8b5/rgbd_vs_moge_summary.json),
[`lighting_comparison.json`](evidence/isaac-j1-rgbd-1f2f8b5/lighting_comparison.json),
[`occlusion_by_robot_vs_q1.json`](evidence/isaac-j1-rgbd-1f2f8b5/occlusion_by_robot_vs_q1.json),
and [`domain_summary.json`](evidence/isaac-j1-rgbd-1f2f8b5/domain_summary.json).

## Claim boundary

The calibration result establishes that the staged SAM + registered metric
depth + explicit geometric cuboid path is wired correctly for an isolated
three-face box. The full-stack result demonstrates that metric depth alone does
not solve single-face association/orientation. It therefore establishes
`PERCEPTION_GEOMETRY_VALID` only for evidence-sufficient hypotheses and never
implies `FEASIBILITY_EXECUTION_VALID`.
