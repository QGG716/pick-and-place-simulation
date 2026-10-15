# Full-face static layout review — 2026-10-15

This round creates a separate 27-carton geometry snapshot and eight actual Isaac
views. It does not change the native four-carton task, search HOME/IK/paths, play
the timeline, or qualify unloading. The reviewed starting commit was d0769cc.
All detailed USD, private geometry, images and reports remain under ignored
`outputs/eco65_native/full_face_20261015_run01/`.

## Layout and asset composition

`configs/workcells/eco65_full_face_layout_v1.yaml` defines 250 × 210 × 100 mm
cartons, one depth row, three columns and nine layers. Column centres are
−220/0/+220 mm. The wall envelope is 250 × 650 × 900 mm; the effective cargo
floor is the solid pedestal top at D-Z 180 mm, aligned with both belt surfaces.
The table remains at W-Z 800 mm. Roof underside D-Z 1080 mm and inner width
766.666667 mm follow the nominal 1:3 trailer cross section. Robot and tool stay
1:1; no dynamics similarity is asserted.

The old four-carton snapshot and actual accepted plan04 initial joint values
are provenance inputs. The new static pose explicitly does **not** inherit HOME
acceptance for the changed scene. Robot base, actual flange/tool mounting, TCP,
warehouse placement and original STEP/mesh bytes are unchanged.

The pinned NVIDIA A06 asset is version 5.0; the existing runtime is Isaac Sim
6.0.1.0. Thirty cached source/dependency files were hash checked. A byte-identical
local alias resolves the authored percent-encoded physics-material sublayer.
Both modules use a uniform scale of approximately 1/3, giving 300 mm effective
belt width. The derived layer clips lower supports at the desktop plane while
interpolating UVs/normals and preserving material subsets. Retained upper frame,
rollers, decals and belt are actual source geometry. No complete frame is
nonuniformly stretched. The exact retained non-decal triangle meshes are also
the single static collision representation; the old belt/body box proxies are
removed from this new scene. Original asset files are not edited.

The actual most protruding conveyor X is D=108.404593 mm, so the carton front
is D=208.404593 mm. Belt bearing-surface front X is 70 mm. Thus the structural
gap is 100 mm and the bearing-surface gap is approximately 138.405 mm. These are
different measurements. The rear inner face is D-X=478.404593 mm, providing
20 mm behind the carton/pedestal. The retained side/roof panels start 50 mm
ahead of the carton front (D-X=158.404593 mm); this explicit new opening keeps
the widened conveyors outside the trailer panels. The robot base is not moved.

The L layout has a measured 13.413 mm structural gap and 52.232 mm bearing-surface
gap at its junction. The two solid belts do not overlap. Receiving regions and
30 mm end exclusions are recorded, but no transfer mechanism is qualified.
Historical A43 was a ROLLER/Y_MERGE with a −45° branch and approximately
4.000 × 2.831 × 1.166 m envelope, incompatible with this 90° L junction.
It remains rejected and is not instantiated.

## Actual checks and blockers

Final USD readback: 384 checks match, maximum error 4.77e−8 m. The full warehouse
782-leaf representation is retained. All 27 IDs, individual proxies, visual
bounds, support chains, equipment dimensions, robot/tool transforms and
T_W_D applied once are checked. A focused support-clipping test passes in the
existing USD environment; the old four-carton static composition also passes.
Local Python lacks optional pxr, so that one unit test is skipped locally.

The actual tool/flange envelope above a top-contact TCP is 142.750434 mm.
With the existing 30 mm preapproach and 2.4 mm pair clearance, necessary
headroom is 175.150434 mm. Actual headroom is zero: **BLOCKED**. The height-only
bound is seven layers; it proves neither whole-chain reach nor a collision-free
path. The original 27-carton target remains displayed. Red/yellow envelope lines
in view 08 are hypothetical contact/approach diagnostics, not achieved poses.

The twelve seal contours have minimum overall width 131.113578 mm over all
in-plane rotations, exceeding either side face's 100 mm height even before
edge margin. Simply changing to front/side suction is not a verified solution.
Bottom and middle layers also have occupied top approach space while supported
by the layers above. No IK coverage is asserted.

The inherited HOME has actual tool surface vertices strictly inside
`carton_r00_l01_c00` and `carton_r00_l02_c00`. These witnesses are stronger than
AABB overlap. They are retained and displayed, not hidden by moving the robot.
Absence of a vertex witness elsewhere is not a complete clearance proof;
remaining AABB overlaps are labelled potential only.

Before further unloading work, the user must decide whether to change the
full-height requirement or the grasp/tool/operating arrangement, approve a new
initial-state search for the changed scene, and resolve the L-transfer gap and
mechanism. None of these changes is automatically applied in this round.

## Reproduction and evidence

Use the existing Isaac Python and `PYTHONPATH=src OPENBLAS_NUM_THREADS=1`:

```bash
python tools/build_eco65_full_face_layout.py \
  --snapshot outputs/eco65_native/full_20261015_build/snapshot.json \
  --candidate outputs/eco65_native/native_HOME_20261015/home_candidate.json \
  --failure outputs/eco65_native/full_20261015_plan04/attempt_01/failure.json \
  --conveyor-cache outputs/eco65_native/conveyor_assets --output <fresh-output>
python tools/export_eco65_warehouse_workstation.py \
  --snapshot <fresh-output>/snapshot.json --home <fresh-output>/static_pose.json \
  --config configs/workcells/eco65_full_face_layout_v1.yaml --output <fresh-output>/usd
python scripts/isaacsim_render_eco65_warehouse.py \
  --assets outputs/eco65_native/warehouse_assets \
  --workstation <fresh-output>/usd/workstation_D.usdc \
  --output <fresh-output>/isaac --layout-review
```

The delivered final images are in `isaac_final/`; the first camera pass remains
in `isaac/`. The final render took 64.82 s, compatibility composition 4.25 s,
and support-clip test 0.28 s. `commands_and_timings.json` records actual argv.
Final timeline time is 0 s with zero PLAY events. Eight raw and eight annotated
1920×1080 PNGs are retained, plus a revised top annotation using unchanged raw
pixels and the recorded camera. Views 02/07 share exactly the same camera.

`STATIC_LAYOUT_REVIEW` / `FULL_FACE_TARGET_NOT_MOTION_VALIDATED` /
`NO_MOTION_EXECUTED` / `Dynamics: NOT_EVALUATED` remain the only acceptance scope.
Stop after image delivery and wait for layout confirmation.
