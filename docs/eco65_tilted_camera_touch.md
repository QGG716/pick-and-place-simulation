# ECO65 30? camera and one-point physical touch adapter

This round starts from `f9adc0e10da7e1ecc9020a16e19235a652f5b122` on
`feat/v0.6-eco65-moveit2-native-single-carton`. The original ECO65-B,
12-carton workstation, CAD suction tool/TCP and J1 mast/seat remain fixed.
Private CAD, meshes, USD, images, measurements and videos stay under ignored
`outputs/eco65_native/tilt30_touch_20261017_run02/`.

## Mechanical and visual changes

`build_eco65_tilted_fixture.py` creates a source-bound installation derivative.
The whole XD500 component, optical frame and collision representation rotate
about a documented mast-local mechanical pivot. Two side journals and a tilted
saddle are explicit project-derived simulation parts. They do not invent a hole
through the original camera housing. CAD-to-optical transformation is unchanged.
The optical axis is 30? below horizontal; the image right axis is horizontal;
the horizontal heading remains ?90? from link_1 local ?X. Legacy horizontal
configurations remain supported and cannot pass a 30? installation check.

`--target-longitudinal` solves both J1 endpoints using the camera's orbit and
the existing receiving-surface centre. It does not equate conveyor flow direction
with camera heading, or force a 90? turn. The actual endpoints are ?86.597128?
and ?157.987119?, a ?71.389991? turn. J2?J6 stay fixed while every link FK updates.
The two scan videos contain 130 frames at 5 fps, 26 seconds at 1?. The optical
640?480 output uses the existing same-ray 64?H?50?V imaging adaptation.

All 12 cartons have their eight projected corners inside the image and have
visible instance pixels. The longitudinal receiving surface has 28/35 visible
samples; its near portion falls below the image. This is partial receiver
coverage, not a claim that the entire conveyor fits in the fixed view.

A base, column, back arm and rear mounting plate support the temporary board.
The final USD base/table gap is approximately ?1.1e?8 m (float representation).
The temporary training and held-out placements differ, but each stays locked
within its acquisition group. Support geometry is shared by USD, native FCL,
independent MuJoCo and the physical stage. Named fixed joints do not remove
moving robot/fixture clearance checks.

Only the added probe bracket's thickness changes, from 8 mm to 5 mm; the measured
part_001 gap is approximately 2.500 mm under the unchanged 2.4 mm rule. Probe
point, rod axis, source tool and original suction TCP are unchanged.

## Fresh calibration

New rendered images and new installation identities feed the existing stage 1.1
estimator and strict validation. Six training touches/three views and three
held-out touches/three views are retained. Truth remains outside the solver.
Validation passes: extrinsic translation error 0.599 mm, rotation error 0.0580?,
held-out maximum point error 0.622 mm, cross-J1 spread 0.960 mm. These are
calibration results, not physical touch accuracy.

## Native and physics boundary

`prepare_eco65_touch_native.py` builds a cold probe-TCP model from the bound
geometry and verifies copied rootfs mesh hashes. The original suction TCP remains
in the model. Fixed world mesh identity includes vertices, faces and transforms;
the finite datum cannot move or attach. `eco65_touch_process.h` handles only the
named probe/datum contact neighbourhood and explicitly named assembly contacts.
Actual FCL distance queries run; independent MuJoCo checks validate states and
sampled paths. These discrete checks are not a continuous swept-volume proof.

`isaacsim_eco65_look_touch.py` uses finite official effort/position limits and
actual articulation position/velocity/contact feedback. There is one initial
state authoring operation and no per-frame FK or pose teleport during physics.
Self collisions are enabled. PhysX controls adjacent articulation-body response;
native/MuJoCo checks additionally preserve exact tool/neighbor clearances.
The original 1 kg tool input and an explicit 1 kg mast/camera mass assumption
are simulation inputs, not measured hardware inertia. New metal probe density
is 2700 kg/m?. Contact offset is 20 ?m, rest offset zero; the datum radius stays
4 mm. The physical controller and contact monitor run at 1000 Hz; video is 5 fps.

The target module receives only new RGB, actual capture q, observed board-local
geometry and frozen validated estimated extrinsics. It fixes the target in D
before planning. No truth transform, world test-point coordinate or taught touch
q enters target formation. Only two probe-axis rolls and 12 native IK seeds per
roll are considered; free transfer tries PTP then OMPL with an 8-second planning
budget. Equivalent joint windings stay within official limits. IPC has a separate
180-second cap for native generation plus geometry checks. Controlled moves use
probe-tip LIN. The guarded search is limited to 6 mm, 8 simulated seconds and a
5 N force stop; the 5 mm end-to-end working threshold and actual 4 mm disk are
locked before control. These are simulation settings, not manufacturer limits.

Failures return nonzero and retain their own images, feedback, native calls and
videos. No run is combined with another run to claim completion.

## Entries

Use a new ignored output directory and the existing isolated Isaac environment;
`PYTHONPATH=src:tools`, `OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1`.

1. `tools/build_eco65_tilted_fixture.py --source ... --output ...`
2. `tools/prepare_eco65_touch_calibration.py --installation ...` with the explicit
   snapshot, composed USD and geometry manifest inputs.
3. `scripts/isaacsim_capture_eco65_touch.py`, then the existing solve/evaluate tools.
4. `scripts/isaacsim_visualize_eco65_j1.py --target-longitudinal` and the report tool.
5. `tools/prepare_eco65_touch_native.py` with the current rootfs and worker.
6. `scripts/isaacsim_eco65_look_touch.py` with explicit configuration, native context,
   scene/hash, observations, solver result, validated extrinsics and worker command.

Exact invocations, timings, source hashes and per-run outcomes are in the private
delivery report. No hardware, unloading task, six-point continuous motion or
historical task sweep is authorized by these entries.


## Measured execution outcome

A?D above completed. E **did not reach contact or retreat**. Four distinct physical
debug runs retain their own failures. The first could not meet the prelocked
settled-velocity threshold at 1/240 s; using 1 ms steps passed settling without
relaxing that threshold. A fresh image and actual capture q produced a fixed
visual target about 0.761 mm from the independent truth point in physical04 (physical02: 0.603 mm). This is only
visual-target error. No physical touch precision was measured.

The initial IK branch failed actual FCL mast/link_2 clearance. A further bounded
branch reached native PTP; it collided with link_2/link_3 and the original tool,
and the 8-second OMPL attempt timed out. After addressing equivalent windings
and local seeds, two additional native-only diagnostics each exceeded the
180-second transport/generation/check cap in PTP. They are retained as timeouts,
not evidence of path infeasibility. They did not execute any motion.

The most complete physical run is `physical04`: 1.0 s of actual settling physics,
new RGB capture, target formation and rejected transfer. Contact=false,
retreat=false, no non-target physical contact reported. Its two 1-second videos
show the observation stage only, not a completed look/touch cycle. Guarded
contact, contact-stop and retreat branches remain physically unverified.
The later source-binding and contact-metric additions compile but were not
exercised in a completed physical cycle. Debug runs predate per-file runtime
source bindings; their evidence preserves inputs, native requests and outputs,
but must not be represented as fully revision-qualified acceptance runs.

Directed checks: five camera/endpoint/winding tests; the existing 23 calibration
binding/acceptance tests; native clearance and process-policy executables.
Actual worker compilation, rootfs mesh verification, FCL/MuJoCo state queries,
Isaac rendering and limited articulation physics ran in the existing environment.
The synthetic native regressions do not substitute for the failed integration.

The remaining condition is a validated native transfer in this full scene,
followed by one uninterrupted fresh-image physical contact and retreat run.
The authorized server and assets are available; missing hardware credentials or
an old tool.json are not the blocker. This round stops here without unloading,
transit optimization or a wider search.
