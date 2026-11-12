# November 12 real-site continuous single-carton entry (current active override)

Work from the actual latest HEAD on `feat/v0.6-eco65-ubuntu-real-integration`;
preserve prior commits and LFS assets. The user authorizes development, commit
and push. The explicit single-carton UI automatically connects the configured
D435i, robot and vacuum transports asynchronously after the UI appears. Default
capture is 640x480@30fps, with no silent fallback or ambiguous camera selection.
Connection, capture and inference never authorize motion or valve commands.
Only the operator's single-carton execute button starts the physical cycle.
Before robot TCP connection, its background worker restores the explicitly
saved NetworkManager wired profile and checks the actual route. Reuse an active
profile without restarting it; do not create profiles, guess IPs, change the
default route, or interrupt an unrelated network. Failures stay in the device UI.

Continuous detection and same-frame SAM segmentation use one model owner and
bounded latest-frame scheduling. Independent result windows may be hidden
without affecting inference or execution. Task observations, q_capture and
q_start remain immutable; every session transition revokes old observations.
The default read-only and controlled-empty entries retain their separate scopes.

For real tasks, configurable carton priors (default 150x100x100 mm) may constrain
partially observed geometry. Do not reject image/occlusion boundaries alone or
present inferred thickness as measured. Keep credible observed contact patches,
finite pose/size ambiguity and the full conservative payload envelope separate.
All active seals must fit depth-supported CAD contours with edge margin; retain
passive common-valve semantics, every rigid solid and the measured pressure gates.

Use the separate base_link real-site scene: frozen RGB-D, measured fixed boxes,
and explicit measured blind clearance. Unknown space is occupied; no simulated
trailer/conveyors/lift/box stack/base placement enters physical collision scenes.
The source snapshot supplies robot/tool/camera hardware geometry only. q_start
receiving geometry is now checked against measured site support, superseding the
November 11 deferral. Unknown receiving data is not invented. Every uncertainty
envelope must pass the complete loaded return and release checks before motion.

Keep pure 5 ms CANFD, feedback-based transitions and independent stop/pressure
supervision. Closing a result window is display-only. Fault/stop/close never
releases a held load. Development tests use fake IO or inert smoke mode; only a
real operator-started complete cycle can satisfy physical acceptance. Preserve
original calibration bytes/status and bind field confirmation separately.
See `docs/eco65_real_site.md` for current startup, configuration and acceptance.

# November 11 physical single-carton cycle (historical where superseded above)

The latest user removes the 150x100x100 mm restriction for real observed cartons:
select by a credible visible face and actual CAD seal coverage, with dimensions
measured automatically from visible face and side boundaries. The user confirmed
that a side face can be shown. Never fabricate hidden thickness from one plane;
keep full payload/neighbor collision, loaded return, mass and pressure checks.
Deduplicate synonymous detections of the same carton. This supersedes the fixed
carton requirement only in the real single-carton entry; historical offline scenes
and existing assets stay unchanged. No automatic motion or valve writes in tests.

The current user authorizes incremental development, commit and push on
`feat/v0.6-eco65-ubuntu-real-integration`, including `f13cda3daa850fac97337f730f6717d697c86c7f`.
The explicit `unloading_sim.workbench.single_carton` entry may execute ONE
operator-started native-planned carton cycle and the verified vacuum protocol.
This supersedes older empty-only/no-valve restrictions for this entry and the
shared adapters needed by it. Default observation and replay remain read-only;
retain RobotObserver's whitelist and the separate controlled-empty entry.
Startup, import, connect, preparation, preview and development tests never move
hardware or send suction/release commands. No legacy enable_hardware activation.

User-confirmed field premises: connected functional devices and verified frames;
camera on link_1, world=base, tool 1.5 kg, carton 2 kg, robot rating 5 kg. All
12 cups share one valve and have passive self-closing check valves (explicitly
confirmed this round). Never treat them as separately commanded cups: retain all
geometry, require complete seal contours on the observed target for anticipated
sealing cups, and verify actual gripper pressure before extraction. Preserve the
original calibration file bytes/status; bind the user's field confirmation
separately. No historical acceptance-report gate and no training-example fallback.

For confirmed passive self-closing cups, distinguish sealing IDs from permitted
target-lip contact IDs. Nonsealing terminal lips may meet this same target face
only under the existing 0.4 mm plane-penetration and 0.01 rad normal guards;
they do not count as seals. Bind exact source CAD lip IDs, retain rigid inserts
and all other collision pairs, and recheck sealed coverage/gap in the fixed
loaded attachment. Approach stages have no target contact permission.
Actual measured contact FK must also check every permitted lip for penetration
before establishing vacuum and throughout the pressure-confirmation dwell.

Physical single-carton planning uses the existing bounded RRTConnect adapter,
with the same monotonic deadline inside search extensions and clearance loops.
Keep solver-budget exhaustion distinct from IPC timeout and physical infeasibility.
Persist each request before dispatch and retain each Pilz/OMPL attempt separately.
Conservative transformed local AABBs may prune far collision pairs; preserve
original FCL solids, pair rules, margins, search density and final sent-path audit.
This deadline adapter does not enable J1 direction constraints or simulation modes.

The user explicitly deferred receiving-region qualification at q_start for this
round. Still validate the complete loaded return and release collision envelope;
record RECEIVER_QUALIFICATION_DEFERRED_BY_USER, never support certification.
Freeze q_start once; never substitute HOME or redefine it during execution.
The user approved at least two fresh pressure heartbeats spanning >=1 second,
with 3-second grip and release timeouts. Reference thresholds are -60 kPa grip,
-30 kPa holding-loss and -10 kPa release, using only the gripper channel. Keep
pressure freshness explicit; do not interpolate fake high-rate measurements.

Pure 5 ms CANFD transmission and the existing stop/feedback supervisor remain
mandatory. Stop/fault/close must never automatically release a held load, return
home, resume, retry a task or claim a stop without feedback. A release already
explicitly initiated at the returned start is recorded separately. A stopped
held-load task requires local handling. Only real complete evidence can yield
PHYSICAL_CYCLE_COMPLETE; all fake/native-only/development evidence is NOT_RUN
for physical acceptance. No conveyors, lift, multi-box scheduling, N+1 plans,
visual servo, J1 direction or top-height constraints. Keep all Git LFS assets.
See docs/eco65_single_carton.md for the implemented entry and record format.

# November 11 ECO65-B controlled empty execution (active override)

The explicit current user request authorizes development on
`feat/v0.6-eco65-ubuntu-real-integration` from the actual latest HEAD. This scope
supersedes the old branch-only/no-SDK/stage-one prohibition below **only for**
`unloading_sim.workbench.controlled` and its explicit entry. Default
`eco65-workbench` remains read-only: preserve RobotObserver's three-call
whitelist, DisabledExecution, replay source isolation and passive shutdown.
Legacy enable_hardware and imported configuration never enable motion.

User-confirmed premises: devices connected; coordinates/calibration checked;
camera on link_1; world=robot base; ECO65-B rated payload 5 kg; suction tool
mass 1.5 kg. Preserve historical calibration statuses as evidence; missing old
acceptance reports are not a development gate. CAD-derived COM/inertia are
explicit estimates, never measured evidence. Keep full geometry and inertials.

The November 11 explicit user request sets the controller payload to 3 kg for
empty tests. This is controller_payload_kg, distinct from the 1.5 kg physical
tool model; do not relabel 3 kg as a measured tool mass. The authorized one-time
configuration created/selected Empty3kg after Arm_Tip modification was rejected,
preserving its pose and COM. Ordinary startup/connect/import/preview still never
write tool configuration. Submit checks the explicit controller setting again.

Only an explicit operator click may submit one newly generated, source-bound,
collision-checked, time-scaled native MoveIt empty trajectory from fresh real
joints. No carton grasp, vacuum/valve/conveyor action, online scheduler or
unplanned catch-up movement. No simulation flexible-contact, ideal-attachment,
robot/mast or robot/tool exemptions. Retain only exact fixed mating/assembly
interfaces and adjacent official links. Do not restore J1 direction or top-grasp
height constraints; keep the 150 x 100 x 100 mm cartons and layout.

Use rm_movej_canfd pure high-follow passthrough with SI internally, owned
connection/UDP feedback, independent software-stop channel and one active task.
Late sends, stale feedback, alarms, persistent errors or SDK errors stop sending.
Never infer arrival/stop from a successful API return or preview time. Keep
STOP_UNCONFIRMED latched without reconnect/resume. Constructors, imports,
planning, development checks and fake tests never move physical equipment.
User performs real low-speed arrival/stop trials; until then record NOT_RUN.
See docs/eco65_controlled_execution.md for the executable entry and thresholds.

# Active ECO65 native migration (2026-10-13)
The current user request supersedes branch-specific FANUC instructions below.
Work only on feat/v0.6-eco65-moveit2-native-single-carton, rooted at 3a84888.
Use ECO65-B assets and scene definitions pinned at c07fb73; no source branch writes.
Preserve resident native MoveIt/MTC/Pilz/OMPL generation and FANUC regressions.
Preserve all 12 physical cups, stable IDs and verified CAD contours. Current small-carton
tasks use SIMULATED_INDEPENDENT_CUPS: a nonempty selected subset must fully seal on
one target face at actual FK; unselected cups retain all collision geometry. Hardware
independent_control_verified remains evidence, never inferred from simulation authorization.
No FANUC wrist/neighbor exemptions for ECO65. The 2026-10-19 user authorizes 150x100x100 mm
cartons, narrower conveyors and finite robot/stack layout changes; regenerate all identities.
Keep private assets and generated snapshots/USD/images/videos ignored.
Use one combined compact/warehouse scene and a newly generated cold single-carton plan.
No hardware or SDK; offline replay stays PLANNING_ONLY_NOT_EXECUTABLE / KINEMATIC_REPLAY.
Run directed checks only. Reuse existing isolated remote environments, no upgrades.

# Historical FANUC instructions (apply only to retained FANUC entries)

# Codex development instructions

## Goal
Build a production-oriented first-layer geometric simulator for trailer unloading by a six-axis industrial robot. Keep it deterministic, testable, and independent from any heavyweight simulator.

For the current FANUC M-710iD/70 branch, the immediate goal is to close one physically executed, layout-bound pick-and-place cycle using the complete official FANUC model at `FANUC-CORPORATION/fanuc_description@fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`, the supplied Wantai tool geometry, fail-closed collision acceptance, and Isaac rigid-body dynamics.  The confirmed layout and its frozen scene snapshot remain the single source for CPU planning, collision geometry, replay, and evidence.  The official public link inertials, joint limits, velocities, and effort limits are accepted engineering simulation inputs for this work; machine certification remains a separate evidence field and must not gate simulation.  The old V3 104-task and 129-carton populations remain historical algorithm regressions and are not engineering acceptance denominators for this new layout.

The active user-approved simulation policy exempts the official J5_link/J6_link versus owned tool colliders, consistently in CPU planning and Isaac, plus the persistent flexible-cup/non-target-stack acceptance exception specified below.  During attached stack separation, target/stack contacts may use planner_relaxed_physics_checked: real carton collision, gravity, friction, progress and disturbance monitoring remain active in Isaac.  Free transit restores normal object-pair rules.  These explicit assumptions permit engineering execution and are not machine certification.  Resolve tool representation coverage from the existing CAD, rather than waiting for a user certificate.  Historical strict initial-clearance failures remain evidence of the previous policy, not blockers for the active one.

This round uses `ideal_independent_cups`: all 72 cups have stable independent IDs and commands, every commanded cup must have a geometrically valid full seal ring on the selected target, and at least one valid cup is required.  No minimum load-bearing cup count, vacuum-force, shear, peel, leakage, or break-envelope check may reject a grasp in this explicit mode.  This holding-capacity assumption never permits remote attachment, wrong-carton attachment, rigid-tool penetration, collision bypass, payload mass reduction, infinite joint effort, or pose teleportation.

The September 17 step-one update selects the explicit `proof_of_concept`
profile for the ordinary planning and continuation entries. It uses
`ideal_independent_cups`, bounded ideal reception after actual attachment and
constraint removal on the same carton in a checked receiving region, and ideal
outfeed via -Y transverse / -X longitudinal routing until the entire envelope
crosses X=-3.2 m. Record `RECEPTION_ASSUMED` separately from measured physical
reception; never synthesize top-contact observations. Other colliders and all
pre-handoff physics retain the existing rules. Explicit historical configurations
retain physical-reception/strict-physics semantics. This is not machine or
physical-downstream qualification.

Planning in this profile has no business wall-clock deadline. Finite solver
iterations and the existing finite candidate definitions remain, with fair seeded
retries and visible unsearched counts. Preserve input resource limits and all
physical failure monitors. Deliver a checked complete baseline and execution
bundle before optional optimization. This round runs one representative new-world
single-carton cycle only, retaining any failure; no full row or repeated physical
trials without further instruction.

The user-selected default recording profile is 640x360 at 5 fps, normal time.
Keep the existing live world and its recording profile until that run finishes.
Lower rendering cadence must not lower physics or receiver-monitor cadence.

September 20 persistent video preference: place runtime parameter panels at the
bottom left and keep only target, phase/simulation time, attachment/cup state,
and reception/outfeed counts. Clearly distinguish ideal reception from physical
reception. Omit verbose planner, collision-policy, belt and waiting diagnostics
from video panels; retain them in evidence. Apply this to all future recordings.
Historical burned-in overlays remain part of original evidence. The requested
16x 720P montage is a presentation derivative, not a new recording default.

## November 9 ECO65-B stage-one integration scope

The explicit current user request supersedes earlier source-branch and no-SDK restrictions for
`feat/v0.6-eco65-ubuntu-real-integration`, based on `269ab67f8932771b6f1ad1f43ed73beaf6f90602`.
Target Ubuntu 22.04 with existing Humble/MoveIt. Keep all physical execution unavailable:
no robot motion/stop, valve/threshold/conveyor commands or automatic connection, including
legacy enable_hardware configuration, alarms and shutdown. Keep observation adapters isolated
from the old GUI/executor. Preserve original calibration status, missing-session evidence,
70 mm probe versus 140 mm tool, explicit world/base transforms and original observation clocks.
Do not change grasp candidates, IK search or placement internals. The prototype cartons remain
150 x 100 x 100 mm with all placements on the transverse conveyor and retained trailer/layout;
do not restore forced 40-box counts, the 100 mm top-suction threshold or J1 direction constraints.
Never describe replay/fake-device acceptance as hardware validation. See docs/eco65_ubuntu_workbench.md.

## Architecture constraints
The November 10 explicit asset-publication request overrides the older local-only asset
ignore rules: publish the current project's local/GPU assets and available model weights
on `feat/v0.6-eco65-ubuntu-real-integration`, using Git LFS for large files and an exact
SHA-256 inventory. Preserve distinct versions and original bytes; do not rewrite Git
history. Keep credentials, runtime installations, incomplete downloads, transfer caches
and run logs excluded. Asset publication does not authorize hardware control, new
simulation runs, model-weight execution or changes to calibration validation status.

- September 15 persistent user update, overriding the earlier flexible-cup neighbor rules: this round and future rounds do not reject flexible-cup contacts with named non-target stack cartons (`compliant_cup_neighbor_contact_mode=ignore`), in CPU and Isaac contact acceptance. Retain physical collider ownership, target identity, full target seal-ring and target compression checks. This does not exempt rigid inserts, the rigid tool, robot, floor, walls, conveyors, or unknown objects. Physical response is retained; this acceptance assumption does not claim neighbor-contact qualification.
- Keep `geometry.py`, `robot.py`, `scene.py`, `ik.py`, `planner.py`, and `grasp.py` independently testable.
- Do not introduce Isaac Sim, ROS 2, or GPU dependencies into the core package.
- Add heavyweight backends behind adapters and optional dependencies.
- Preserve the world convention: +X into trailer, +Y left, +Z up.
- SI units only: metres, radians, seconds, kilograms.
- Never silently accept an invalid home configuration or colliding goal.
- Every new planner or robot backend must expose deterministic RNG seeding.
- September 17 POC collision step 2: the explicit POC pair policy uses total uninflated surface gaps of 5 mm for robot/tool, robot/environment, tool/environment and free payload/environment; robot self pairs retain intersection/contact checks with no extra engineering gap. Legacy explicit configs retain their prior margins. POC side walls use the finite frozen scene geometry. Preserve all checks and existing owned J5/J6, named flexible-cup neighbor, assembly and stage-scoped target/stack permissions. PhysX offsets, response, dynamics and control rates remain unchanged; proxy overlap and unknown evidence fail closed and must not be called proven original-CAD penetration.
- Verify tool solid sources, units, transforms and outward coverage automatically from existing CAD assets; preserve uncertain rigid parts conservatively and distinguish flexible cup lips from rigid inserts.  Compute readiness from these checks and the effective policy, never from permanent false flags.
- Apply wrist/tool exemptions by physical link and collider ownership, not broad path substrings or whole-body filtering that could hide environment contacts.
- Do not let payload qualification failures short-circuit geometric search, and do not modify FANUC load curves to manufacture a pass.
- Keep official-model simulation readiness independent from machine certification; record both without combining their booleans.
- Preserve all official per-link inertials including non-diagonal tensor terms, the independent J3 joint, `flange`/`fanuc_flange` transforms, and official finite velocity/effort limits.
- In `ideal_independent_cups`, retain three distinct 72-bit masks: geometrically eligible, commanded active, and actual contact.  Recompute actual contact from actual robot and carton poses before attachment.
- Preserve the original 104-task grid denominator and the 40/27/32/30-carton continuous scenes.
- Do not reinterpret those legacy denominators as acceptance evidence for `m710id70_unloading_layout_v1`.

## Preferred next milestones
1. Load and hash-verify the fixed official FANUC xacro, visual meshes, collision meshes, license, joint/frame semantics, and exact per-link inertials as one model; retain the supplied raw CAD only as historical assembly evidence.
2. Recompute the fixed mounting transform from the official base mesh so its front extent remains at world X=-1.1 m without changing the confirmed assembly envelope.
3. Unify the user-approved wrist/tool and stage-scoped depalletizing contact policy, verify actual tool coverage, and derive a legal initial state without moving the fixed assembly.
4. Execute one complete dynamic pick, then continue in the same physical world using highest-row-first, stable row-center expansion, height-dependent soft face preferences and adaptive placement on either conveyor.  Derive task counts, rows and occupancy from actual remaining states; do not parse carton IDs or hard-code five candidates.
5. Execute through the optional Isaac adapter with official inertials and finite limits, a 20 kg tool and the unchanged 42.5 kg carton. Attach only after actual contact. SUPPORTED_RELEASE requires actual support; SHORT_DROP_RELEASE requires a bounded qualified release region and available predicted landing region, followed by separately measured actual reception. Preserve the same body's velocity, mass, inertia and collision on release.
6. Optimize bounded offline approach, SE(3) extraction, release and departure with one-step next-carton lookahead in the existing same-world executor. Pregrasp, support lift, normal withdrawal and vertical residence are optional motions, never mandatory 100/300 mm stations. Keep base scans, lift/extension changes, general online control and ROS/vision out of scope.
7. Placement working normals point into the carton: TOP_DOWN=[0,0,-1], RIGHT_WALL_FACING=[0,-1,0], TRANSVERSE_SIDE=[1,0,0]. Preserve historical plans separately and invalidate stale motion semantics.
8. The October 1 integration request supersedes the source-branch restriction: integrate only the reviewed feasibility core `4ceb9487e8e650a82243fa10b2c4e91ff3ec48cd` and MoveIt backend `c7e4b486b3bf50fc607cf1c826c441fc688dbdfe` in `feat/v0.5-moveit2-native-single-carton`. Run planning, checks and Isaac execution on the authorized GPU server in the existing isolated environments. Never modify or push the source branches, merge unrelated branches, reset, or force-push. Native-cold forbids historical solution input and legacy motion generation. Every nonzero edge needs current-run native provenance; Isaac may run once only after every planning/export/execution gate passes. Preserve any failure and report its exact scope.
9. The September 17 proof-of-concept step stops after the directed CPU checks and one representative single-carton Isaac cycle. Retain failures. Do not default to another full row, fifth-carton search, or the historical populations. Use 640x360 at 5 fps and 1x physical time, preserving physics/control/contact-monitor rates. The previous physical world ended; new-world trials must be identified separately without cross-world completion counts.

The detailed contracts, failure taxonomy, metrics, and acceptance gates are in `docs/development_priorities_m710id70_v3_recovery.md`.

## Testing
- Add unit tests for every geometry primitive.
- Add regression fixtures for known reachable and unreachable grasps.
- Add focused regressions for allowed initial payload proximity, motion toward/away from a neighbor, and restoration of normal collision margins in free space.
- Keep strict final FK, joint-limit, official robot mesh collision, qualified rigid-tool clearance, and full-ring suction-coverage checks on every task-set IK candidate.
- Keep the default demo under five seconds on a laptop CPU.
- Run focused regressions for this round's changed policy, row ordering, support-region placement and physical execution.  Do not default to the old 104/129 populations or all historical backends.
- Add literal numeric tests for every confirmed layout dimension; do not prove a constructor with formulas copied from that same constructor.
- Verify snapshot and replay fingerprints, all 40 carton identities, legal assembly contacts, illegal penetration, and deterministic initial-state evidence.
- During focused Isaac development rounds, do not make full `pytest`, the 104/129 task populations, a 40-carton clear, repeated qualification audits, or performance sweeps delivery prerequisites.  Run only changed-module import/numeric checks and the requested physical cycle unless a concrete failure needs a direct regression; all runtime collision, joint-limit, attachment, release, and state-feedback gates remain mandatory.
