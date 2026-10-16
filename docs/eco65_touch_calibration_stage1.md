# Known-tip touch calibration, simulation stage 1

The following records the original stage-one experiment. Its 0.1 mm general
static-clearance acceptance is superseded by [stage 1.1](eco65_touch_calibration_stage11.md):
the image estimate remains valid, but the existing probe bracket has only 1 mm
clearance to tool part_001 and fails the applicable 2.4 mm policy. Do not reuse
the historical static PASS as a motion endpoint qualification.

This separate entry estimates **T_link_1_camera_optical**, mapping optical-frame
points into link_1. The existing suction TCP and unloading entry are unchanged.
It does not calibrate the probe tip, execute touch motion, evaluate forces or
connect hardware. All joint states are independent static settings, with the
Isaac timeline stopped. The board is a fixed simulation fixture; its physical
support and manufacturing/fastener design are not qualified.

The reference is the latest connected J1 mast snapshot, including its conformal
seat. Private inputs are located under the repository root, not a drive letter:
`assets/camera/XD500.stp` and the existing mast/tool/robot assets. The STEP is a
single assembly, millimetres, 21 solids, with CAD-axis extents approximately
83 × 42 × 67.5 mm. XCAF leaf instances are converted once with source/mesh hashes.
The assembly contains V0 and V1 front-shell/internal-bracket variants. The
calibration derivative selects V1 and retains the other supplied parts, while
explicitly omitting the two V0 variants. The raw STEP is never changed.

The new rear-cover bounds register to the existing camera pocket. Only old
camera part instances are deactivated in the derivative USD; mast_part_001,
mast_part_010, the original pole/U-shaped pocket (mast_part_019), and J1 seat
remain. This nominal installation is a simulation assumption, not confirmation
of hardware hole correspondence.

The nominal optical origin uses the identified RGB-window centre in its two
in-plane directions and a virtual entrance plane 0.05 mm ahead of the housing
front. It is **not** the STEP origin or a measured optical centre. An explicit
`--optical-offset-m` can change this assumption in a new run. Unknown device
resolution, factory K/distortion, RGB/depth extrinsics, depth range/noise and
optical-centre offset are recorded. The normal optical axis remains horizontal,
image +Y points down, and the signed heading from link_1 local -X remains -90°.

The actual render-camera hierarchy is
`Robot/link_1/Mast/CameraMount/camera_optical/RenderCamera`.
Optical axes are +X right, +Y down, +Z forward. The fixed optical-from-USD-camera
matrix is diag(1,-1,-1,1). Only external diagnostic cameras use look-at.
Final composed USD readbacks check J1 changes and each J2–J6 independently,
including original robot visual/collision child transforms and the added tip.

The configurable simulation default is 640×480, centred principal point
((W-1)/2,(H-1)/2), zero distortion, with pixel boundaries at -0.5 and N-0.5.
fx and fy independently follow the supplied 64°/50° FOV. Isaac's square-pixel
aperture coupling is documented in the [camera API](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/deprecated/isaacsim.sensors.camera/docs/index.html).
The adapter therefore renders 1024×1024 at 64°/64° and explicitly resamples
the same rays into the requested rectangular image/K. Native images are retained.
Depth is ideal registered optical-Z distance from `distance_to_image_plane`,
with the same remapping and a finite/positive validity mask. It is not a real
device depth algorithm, range model or RGB/depth calibration result.

Five separately rendered optical-ray targets verify image directions, principal
point and FOV. Renderer instance IDs identify their image masks; these are used
only for projection diagnostics and are absent from calibration photographs.
Their [annotator semantics](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.13.30/source/extensions/omni.replicator.core/docs/API.html)
avoid confusing coloured warehouse objects with the targets. No segmentation
or scene truth is used to detect the calibration board.

The board uses OpenCV's runtime ChArUco geometry: DICT_4X4_50, 6×5 squares,
25 mm squares, 18 mm markers, 150×125 mm printed area. Six mechanical datums
lie on its surrounding 200×175 mm rigid plate. The coordinate array from
`getChessboardCorners()` is stored and checked when solving; this handles the
[OpenCV board-coordinate convention](https://docs.opencv.org/4.12.0/df/d4a/tutorial_charuco_detection.html)
explicitly. Images are actually rendered from this pattern.

The probe is attached to the measured side edge of the original rigid suction
plate. Its 2.5 mm radius rod is outside the measured tool X envelope; its
ideal zero-radius tip extends 85 mm past the measured tool Z envelope.
The resulting known simulation input is
`p_F_tip = [0.10212736613382173, 0, 0.22775036000000004] m`, F=link_6.
Original `T_flange_tcp` is retained. Native IK receives the equivalent suction
TCP goal computed from the desired flange/known-tip pose, never the tip goal
as an unconverted suction goal. Samples are reconstructed from accepted q and
FK, not copied from board world coordinates.

Finite fixture search uses three candidate centres, two wrist-roll choices,
and finite seed lists. It records rejected IK/geometry attempts. Static checks
cover 1,115 geometries, including 220 conservative conveyor components, all
original obstacles, robot/tool convexes, new camera, mast, seat, probe and board.
No trajectory is planned. The exact-tip/finite-datum pair additionally uses a
separating-plane check of every cone vertex and the known finite disk centre.
One MuJoCo EPA query at an exactly coincident pointed contact returned an
unstable negative distance; the original value and independent plane result
are both retained. There is no whole-probe or whole-board collision exemption.

The observation-only solver performs planar-compatible SVD rigid registration
of touch points to obtain T_D_P. It detects real raster ChArUco corners, retains
IPPE planar candidates, checks positive depth/printed face and ambiguity, and
refines PnP. For each capture q it computes
`X = inv(T_D_L(q_capture)) @ T_D_P @ inv(T_C_P)`.
It checks per-observation consistency, then averages rotations on SO(3) and
translations. It never averages Euler angles or accepts a full scene/snapshot.
Truth is read only by the separate evaluator after solving. Loading the result
changes coordinate conversion only; the simulated camera is never repositioned
to match an estimate.

Reproduction uses the existing authorized Linux Python/worker. Every output
directory must be new; private prerequisites remain local:

```bash
set -e
export PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
R=outputs/eco65_native/touch_calibration_new_run
"$P" tools/prepare_eco65_touch_calibration.py \
  --snapshot outputs/eco65_native/j1_mast_20261016_connected_run01/build/snapshot.json \
  --pose outputs/eco65_native/j1_mast_20261016_connected_run01/build/pose_A.json \
  --camera-manifest assets/camera/derived/20261016_xd500_v1/cad.json \
  --composed-usd outputs/eco65_native/j1_mast_20261016_connected_run01/isaac_A/eco65_desktop_warehouse.usda \
  --geometry-manifest outputs/eco65_native/j1_mast_20261016_connected_run01/build/usd_A/workstation_manifest.json \
  --worker-command /root/autodl-tmp/eco65-native-20261015/worker.sh --output "$R"
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y OMNI_KIT_ALLOW_ROOT=1
"$P" scripts/isaacsim_capture_eco65_touch.py --prepared "$R" --output "$R/capture"
"$P" -c 'import json,sys; assert json.load(open(sys.argv[1]))["status"] == "RENDERED_SENSOR_IMAGES"' "$R/capture/capture_report.json"
"$P" tools/solve_touch_calibration.py --observations "$R/capture/observations" --output "$R/capture/solve"
"$P" tools/evaluate_touch_calibration.py --observations "$R/capture/observations" \
  --result "$R/capture/solve/extrinsics.json" --truth "$R/capture/truth/truth.json" --output "$R/capture/validate"
# Separate nonzero optical-extrinsic recovery experiment:
"$P" scripts/isaacsim_capture_eco65_touch.py --prepared "$R" --output "$R/perturbed" --perturb
# Apply the same observation-only solve and separate evaluation to perturbed/.
"$P" scripts/isaacsim_review_eco65_touch.py --prepared "$R" --capture "$R/capture" --output "$R/review"
"$P" -m unittest discover -s tests -p test_touch_calibration.py -v
```

The private successful run is `outputs/eco65_native/touch_calibration_20261016_run08/`:
normal `capture02`, nonzero perturbation `perturb01`, results under each
`solve03`/`validate03`, and external diagnostics `review01`.
Training has six touch datums and three J1 views (82°,90°,98°); held-out data
has another board placement with three touch checks and three views. The
perturbed run re-renders six images with a nonzero 8/-5/6 mm optical-frame
translation and 2/-1/1.5° xyz rotation. No nominal -90° assertion is applied to
the perturbed solution.

Predeclared working thresholds are 0.1 mm touch registration/arrival, 0.8 px
reprojection RMS, 5 mm/1° extrinsics, 5 mm held-out errors/consistency, 1 px
projection targets and 0.15° FOV. They were written before rendering or solving.
Measured normal results: 0.00337 mm touch registration RMS; 20 detected corners
per training image; 0.048–0.103 px reprojection RMS; 1.440 mm translation and
0.1431° rotation error; held-out maximum 0.799 mm; multi-J1 fixed-point spread
at most 1.160 mm. FOV is 63.9853°×50.0000°, maximum projection-target error
0.115 px. Perturbation recovery: 0.621 mm / 0.0550°, held-out maximum 0.529 mm.

Eight targeted tests passed on the existing server environment (no dependency
upgrade). They include planar registration/recovery, degenerate points, improper
transforms, pixel/frame conventions, wrong capture q/optical flip, truth schema
rejection, insufficient image features and board-placement mismatch. Portable
FK was checked in ten states; worker FK differed by at most 6.7e-16 in matrix
entries. Preparation took 14.86 s; normal and perturbed acquisitions took
43.71 s and 30.55 s after app initialization; installation/tip review 23.87 s.
Logs retain the initial IK/fixture rejections and the first projection-display
failure. No historical result is overwritten.

The stage-one claim is limited to fixed-extrinsic estimation from rendered
images and synthetic geometric touches with the specified independent checks.
Probe self-calibration, physical contact, continuous motion, real RGB-D depth
accuracy and hardware qualification remain unperformed.
