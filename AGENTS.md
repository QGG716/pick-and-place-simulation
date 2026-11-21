# November 20 seam-band ownership, stereo shadow, RGB colour boundary, own carton returns and front-face bound (current active override)

User decisions of 2026-11-20, given on the November 19 pending items:
(1) in a confirmed stack the upper carton's visible face decides the seam band;
(2) the D435 no-return band left of a carton is the background screen and the
depth difference separates them; (3) the 7-14 px continuation rule against
silhouette depth fattening is confirmed; (4) the detection-box-edge rule
follows the implementer's reading and may change; (5) the remaining right/back
returns at rest are the carton's own points; (6) where the envelope growth
in front of the contact face or the 2.4 mm requirement is unreasonable, fix it;
(7) RGB colour easily separates the carton from its surroundings. User
decision 2026-11-21: the field detection thresholds are box 0.25 / text 0.25
(were 0.35 / 0.30; field frame 7fc259a5 returned one whole-stack box), with
`detector.threshold_basis` recorded; the shared example is unchanged.

- Stack support plane: the upper measured bottom's direct bound (lowest
  observed face sample minus one pixel footprint plus the 3 mm depth-noise
  allowance), or the seam line minus its uncertainty where the seam bounded
  that edge, whichever is lower. The lower visible top is occlusion-limited; it
  only checks consistency (conflict above, >15 mm spread). This supersedes
  "lowest of the upper measured bottom, the lower visible top and the seam
  line". Returns inside the upper envelope between that plane and the upper
  measured bottom + 4 mm, within 15 px of the upper mask, labelled upper, lower
  or unlabelled, are the upper's; below the plane, the lower's. Other
  instances are never claimed. This supersedes "other instance masks remain"
  only inside this band of a confirmed stack.
- `geometry.stereo_shadow_boundary_policy={mode,basis}`: DISABLED (default;
  shared example) or INVALID_BAND_BEFORE_FARTHER_RETURN_IS_BACKGROUND with a
  basis. Only image-left in-plane edges of the measured front face, after the
  direct boundary failed: face, at most 6 px coplanar fattening, a no-return
  band, then a return farther by the 12 mm direct step. Same strip/coverage
  rule and pad as the direct boundary; used only when tighter than free rays.
  Recorded USER_POLICY_INVALID_BAND_IS_BACKGROUND_BEYOND_DEPTH_STEP; the band
  never becomes free space.
- Detection-box edge: a seam segment touching the box is excluded only when
  no other detected instance covers >= 50 % of the 1-6 px band beyond it.
- Own carton returns: own-mask returns, plus unlabelled returns within 15 px of
  the mask, 4-connected without a 12 mm depth jump to the accepted face
  samples, are the carton. The envelope grows (never shrinks) to contain them
  plus the pad, except in front of the measured front face (recorded only);
  inside the envelope they are claimed by the target.
- Stack support representation: besides lower-instance returns, upper-
  instance and unlabelled returns inside the support body, at most 15 mm
  under the plane and within 15 px of the upper mask, are represented by the
  support (below the upper visible face is the lower). Deeper unlabelled
  objects and other instances keep their own colliders.
- `geometry.rgb_color_boundary_policy={mode,uncertainty_m,basis}` (user
  decision: RGB colour easily separates the carton from its surroundings):
  DISABLED (default; shared example) or CARTON_COLOUR_DIFFERS_FROM_SURROUNDINGS,
  uncertainty 0.005-0.02 m. Only an in-plane front-face edge still MISSING
  after direct, free-ray, stereo-shadow and seam rules; face chroma (Lab a*b*)
  inside, none of it 3-14 px outside (<= 10 %), no foreground in that band, no
  coplanar continuation 7-14 px out, not at the image border. Bound = mask
  boundary + 3 px footprint + uncertainty; recorded
  USER_POLICY_RGB_COLOUR_DIFFERS_FROM_SURROUNDINGS_NOT_DEPTH_BOUNDARY.
- Stack support pieces (replacing a global front extension of the support
  body): stack returns directly under the upper envelope, <= 15 mm under the
  plane and not inside the body, become 10 mm support pieces aligned with the
  plane, top exactly on it, same category/plane/basis as the body. Observed
  boxes whose points are all under the plane keep their pad below it.
  Near-target refinement covers every run within NEAR_TARGET_M (60 mm).
- Holding capacity: tipping about the convex hull of the sealing CAD rings
  (suction at cup centres, load at the COM, sent-state FK acceleration)
  replaces the point-cup least-norm peel model; safety factor 2 and friction
  0.5 unchanged. Optional `holding_capacity.cup_suction_force_n` with a basis
  (bounded by CAD area x |grip threshold|); the field file uses the user's
  42 N per cup (2026-11-20).
- User decision 2026-11-21 ("try IK branch 4"), superseding the November
  16 3h limit of three branches: up to four distinct approach IK branches are
  checked in the unchanged order (Python screen and C++ composite stop at
  the first clear branch, equivalence kept); when that branch's approach path
  fails, the next unchecked branch is endpoint-checked and planned (PYTHON
  planning path).
  link_3 vs tool part_026 stays a checked pair (no exemption); OMPL validation
  density is unchanged (already 5e-05). A Cartesian LIN segment that still
  exceeds joint limits after the x2 uniform slowdown is planned again
  natively at 0.5/0.25/0.1/0.05 velocity/acceleration scale and fully
  rechecked; motion limits are unchanged. Position-identical native knots
  closer than 1 ms (Pilz LIN end duplicate) are merged before the send grid;
  the sent path is still densely checked.
- The stack lower is chosen (and corroborated) only from interface returns
  under the upper measured footprint plus the 3 mm measured-extent pad, not
  the 60 mm near-target neighbourhood (a representation constant).
- Another instance's pixels within 6 px of the target mask (stereo
  fattening) or of the target's own face chroma (not for neutral faces),
  depth-connected to its measured surface and inside its envelope, are the
  target's own; at depth jumps they are removed as flying.
- Optional `contact_search.precontact_m` (0.02-0.15 m, default 0.03); the
  field file uses 0.06 (2026-11-21) so the joint-space approach ends farther
  from the target and the remaining distance is a LIN.
- Front face (+Z) bound: fitted plane + 3 mm depth-noise allowance, never
  inside an accepted face sample. The pixel-footprint pad and extreme-sample
  double count no longer apply to the face along its normal. The 2.4 mm pair
  clearance is kept: it protects the rigid cup core against the real face and
  lip compression; only the envelope's double count was unreasonable.

All other November 19 items and the unchanged list below stay in force; the
true carton stays inside the envelope; development is offline and physical
acceptance stays NOT_RUN.

# November 19 carton geometry robustness and coplanar seam policy (in force where not superseded)

User decisions of 2026-11-19 20:41, given with the geometry-robustness task:
(1) a coplanar seam may be trusted where the SAM mask boundary coincides with a
strong straight RGB edge, only under the explicit policy
`geometry.coplanar_seam_policy={mode,uncertainty_m,basis}`; (2) the camera
mounting stays unchanged: do not implement or propose relocation, and views
that cannot measure an edge keep an explicit failure class; (3) all other
robustness fixes follow the implementer's proposal (tilted support clip,
corner-proof direct boundaries, absolute second-plane evidence, connected side
thickness, free rays on every visible face, +/-3 px RGB-edge band, nested masks
and stack-box NMS, interval-envelope stack support). The user then directed
that this session, not other sessions, carries out the work.

Seam policy: mode DISABLED (default; shared example) or MASK_RGB_EDGE_COINCIDENT
with uncertainty_m 0.005-0.02 m (field values may only enlarge it) and a
non-empty basis. It applies only to the four in-plane edges of the measured
front face that remain MISSING after direct and free-ray evidence; never to
thickness; a confirmed stack support decides the bottom. The interval records
USER_POLICY_RGB_SEAM_NOT_DEPTH_BOUNDARY; the measured box is unchanged. It is a
field premise, not depth evidence. No collision exemption is added: an at-rest
overlap lying entirely in the seam band is reported as
COPLANAR_SEAM_CONTACT_NOT_PERMITTED and any seam contact rule is the user's
decision. A stack support plane is the lowest of the upper measured bottom, the
lower visible top and the seam line minus its uncertainty.

Unchanged: 2.4 mm pair clearance, 5 mm tool/robot reserve, 1 mm rest depth,
0.4 mm support penetration, 0.40 m maximum extent, 6 mm free margin, 2 px guard,
3 mm depth noise, 2 mm plane residual, 715 solids, 12 cups, CAD seals and 3 mm
edge band, pressure gates, motion limits, 5 ms period, stop and lock
semantics, camera extrinsics and status, unknown_policy and the named
pointcloud list. The true carton must stay inside the envelope; changes only
replace unevidenced enlargement by evidenced bounds, and every new threshold
records its calibration basis. Development is offline (no device, no motion
or valve command); physical acceptance stays NOT_RUN.

# November 19 user-confirmed upper-carton pointcloud artefacts (in force where not superseded)

The user selects ONLY the current failed upper-carton pointcloud object list,
not a 60 mm exclusion neighbourhood. Apply USER_CONFIRMED_NAMED_LIST to
unclaimed RGB-D returns inside the original listed base_link volumes, with no
added margin or automatic list expansion. Bind the manifest SHA-256 and source
observation/failure record. Record per-object counts and the ignored-mask hash.
Other instance masks, returns below the measured bottom/support noise window,
observed supports, fixed geometry, neighbour hidden volumes, workspace bounds,
robot/tool solids and the full attached payload remain. This supersedes
observed-point retention only within that finite source-bound scope. It is a
field premise, not sensor-proven free space. KEEP remains selectable. A higher
unresolved instance or an upper measured centre outside the reference measured
box prevents applying the exception to another box. The original 60 mm scene
refinement neighbourhood keeps its previous representation-only purpose.

# November 19 user-requested 5 mm tool-to-robot clearance (in force where not superseded)

The latest user explicitly changes the existing 40 mm tool/robot collision
reserve to 5 mm. Set `tool_robot_clearance.clearance_m=0.005` in the current
field configuration and shared single-carton example, with the user-request
basis. This supersedes only the 40 mm requirement and the prohibition on
changing that field below. Preserve all solids, pair exemptions, the other
2.4 mm pair rules, complete carton uncertainty volumes and device/stop checks.
This change does not certify a path or authorize startup motion. Re-plan and
audit with the new source-bound configuration; historical 40 mm evidence stays
historical. Physical acceptance still needs an operator-started complete cycle.

# November 19 registered site layout and conservative payload safety (in force where not superseded)

Work from latest HEAD on `feat/v0.6-eco65-ubuntu-real-integration` (not before
ecca7896); preserve all commits and LFS resources. Development, commit and push
are authorized. Import only the user's actual site layout STEP as static CAD
design geometry, never measured geometry, after frozen RGB-D registration
passes RMS <= 5 mm and p95 <= 10 mm. This overrides the no-layout/conveyor/stack
import rule only for that registered layout. Explicit, justified part exclusions
remove duplicate hardware and movable objects; never guess exclusions.

Build the full scene once, then update it from confirmed pick/release results
and new frozen observations, with conservative retention and required rebuilds.
This supersedes only scene carry-over in November 16 section 6; plans, IK seeds
and paths never carry over. Each native task still resets with a fixed seed and
a new versioned scene handle. The continuous-job entry is not implemented here.

The user confirms execution-unlock prerequisites are fulfilled. Do not modify
execution_lock, wrist_branch_limits, posture_preference or tool_robot_clearance.
Add start tolerance and a non-waivable hard cap, exposure-bound plan validity,
measured-box versus conservative-envelope separation, holding-capacity checks,
and unresolved-neighbor extrusion along camera rays by the configured maximum
carton extent. Detections entirely outside the workspace do not block tasks.

User-approved values: start tolerance 0.02 deg (measured stationary encoder
jitter <= 0.006 deg; absent tolerance defaults to 0.01 deg, absolute schema cap
0.05 deg), hard cap 0.5 deg, single-carton plan validity 120 s from frozen-frame
receipt, maximum carton extent 0.40 m, scene maximum age 300 s. Holding capacity
uses vacuum.hold_below_kpa, friction 0.5, safety factor 2.0, effective area factor
0.8; basis: engineering estimate pending field pull-off calibration. Record each
basis in configuration; field values may only tighten the authorized bounds.

Keep all 715 collision solids, 12 cups, 2.4 mm pair clearance and pair rules,
40 mm tool/robot reserve, seals/contact/lip/attachment rules, 0.10 rad/s and
0.20 rad/s^2, 5 ms CANFD and monitor thresholds, -60/-30/-10 kPa pressure gates,
two fresh heartbeats spanning >= 1 s, 3 s timeouts, stop semantics and explicit
PYTHON/NATIVE equivalence rules. No other threshold changes. Development uses
fake IO or inert mode only: imports, startup, connection and preview never move
hardware or write valves. Physical acceptance stays NOT_RUN. Commit each step
separately, run directed checks and write dated evidence. If actual STEP is
absent, finish the converter and synthetic STEP tests, then stop at step 5 and
record "waiting for real STEP"; do not substitute simulation layouts.

# November 18 user-requested posture retry (in force where not superseded)

The user requests removal of the J5 wrist-flip restriction, a 180-degree
rotation about J6 relative to the displayed grasp orientation, and upward
contact shifts with partially sealing passive cups, followed by an offline
retry and 3D display. Disable the J5 same-sign/zero-band rule explicitly with
the user basis. The subsequent explicit reply also removes J4's relative
travel gate (max_j4_travel_deg=null); retain J6's travel limit, official joint bounds, 40 mm
tool/robot reserve, all collision geometry, seal contours, pressure gates and
the existing execution lock. No physical motion is authorized by this retry.

For this request only, the prior candidate and closest-q_start ordering may
include bounded rotated/raised contacts and a soft source-URDF link-direction
cost (upper arm toward base +Z, forearm toward the base XY plane, plus raw
joint continuity). These are preferences, not fixed joint commands or new
state rejection thresholds. Recheck actual CAD seals after each transform;
unsealed cups retain geometry and common-valve passive semantics. Keep TCP,
calibration, attachment transforms, frozen observations and q_start unchanged.
Python and native composite ordering must agree. Archived retries remain
NOT_EXECUTABLE / physical NOT_RUN, even if all four path audits pass.

# November 16 real-time refactor and continuous multi-carton job (in force where not superseded)

Work from the actual latest HEAD on `feat/v0.6-eco65-ubuntu-real-integration`;
preserve prior commits and LFS assets. The user authorizes development, commit
and push. Every rule in the sections below stays in force unless a numbered
item here names what it supersedes. Development tests use fake IO or inert
smoke mode. Physical acceptance of everything in this section is NOT_RUN until
a real operator-started run produces the evidence.

## 1. Scope and latency target

Refactor three modules of the ECO65 real-site chain for latency: vision
(frozen-frame carton geometry and site scene), motion planning, and online
scheduling (task, cycle and execution orchestration). Move only measured hot
paths from Python to C++. UI, configuration, device connection, network
restore, model inference, reports, offline tools and tests remain Python.
FANUC, M710, Isaac and offline-simulation entries are frozen: no refactor, and
their retained regressions keep passing.

Target: `prepare_plan` entry to `PLAN_READY`, p95 <= 2.0 s on the saved field
records, with a success rate no lower than the measured baseline. Vision and
scheduling latency are measured and reported separately. A missed target is
reported per stage; it is never met by weakening or skipping a check.

## 2. Invariants

Unchanged by this refactor: the 2.4 mm required pair clearance and every pair
rule; all 715 robot/tool collision solids and all 12 cups; contact,
seal-coverage, lip-penetration and attachment rules; full uncertainty
envelopes; unknown-space, target-envelope-point and receiver policies with
their recorded bases; contact-candidate generation and priority order (the
accepted candidate is still the highest-priority one whose plan and final
audit both pass); joint limits 0.10 rad/s and 0.20 rad/s^2; the 5 ms CANFD
period and every reviewed monitor threshold; the pressure gate (-60 / -30 /
-10 kPa, two fresh heartbeats spanning >= 1 s, 3 s timeouts); all stop
semantics.

Each migrated function keeps its Python implementation until an equivalence
report on saved records shows identical decisions. Selection is an explicit
`PYTHON | NATIVE` configuration value per module, default PYTHON until that
report exists. A NATIVE failure is reported; it never falls back silently.

## 3. Permitted performance mechanisms

a. Collision proxies. Conservative proxies (spheres or capsules that contain
   the source solids, conservatively rounded distance fields built from the
   current scene, boxes for payload envelopes) may prove a state clear when
   their distance lower bound exceeds the required clearance. Any state they
   cannot prove clear is decided by the existing exact FCL check on the
   original solids. Pairs with permitted contact never use proxies. Proxy
   generation is source-bound and hashed into the model identity.
b. Final audit. This supersedes "preserve ... search density and final
   sent-path audit" (November 11) only as follows: the final audit may advance
   along the geometric path of the exact quintic sent trajectory with steps
   bounded by a proven distance lower bound, provided the clearance guaranteed
   between checked states is never lower than under the present 1 mm
   pair-displacement rule. The audit stays mandatory for every segment before
   registration. Uniform time scaling of an audited segment needs no new
   audit; any change of geometry does. Planner search density is unchanged.
c. Resident native workers may serve consecutive tasks. No world object,
   attachment, collision permission or random-generator state of an earlier
   task may affect a later one: scenes are versioned handles, and each task
   starts from a reset with a fixed seed.
d. One absolute monotonic deadline per planning request, passed to every
   native call. Expiry is reported as budget exhaustion, distinct from IPC
   failure and from physical infeasibility, with searched and unsearched
   candidate counts. Unsearched candidates are never reported as infeasible.
e. Composite native requests. This supersedes the form, not the purpose, of
   "Persist each request before dispatch and retain each Pilz/OMPL attempt
   separately": the composite request is persisted before dispatch; the worker
   appends every internal IK, Pilz, OMPL and validation attempt to its own
   journal while it runs, so a timeout or crash still leaves them. Replies and
   human-readable evidence may be written asynchronously. `PLAN_READY`
   requires the plan evidence record to be written; execution still refuses to
   start when the record writer has failed.
f. File binding may reuse a stored content hash while the file's device,
   inode, size, mtime_ns and ctime_ns are all unchanged; any difference forces
   a full rehash. Evidence states REUSED or REHASHED per file.
g. Binary IPC and shared memory for scenes and trajectories. Digests are
   computed over a canonical binary form, identically at issue and at claim.
h. IK may be analytic or batched multi-seed inside the worker. It stays
   deterministic, uses seeds of the current task only, and every solution is
   verified by native FK within the existing 2e-5 m and 2e-4 rad tolerances.
   The branch order (closest to q_start first, at most three checked) is kept.

## 4. Execution process

The timed sender, feedback receiver and motion supervisor may move to one C++
process. It uses the same vendor SDK binary (1.1.7), the same model, controller
generation and firmware checks, the same `rm_movej_canfd` high-follow call, the
same independent stop channel and owned UDP feedback. No kernel or system
package change: use real-time scheduling where the existing system grants it
and report the scheduling class actually obtained. The Python path stays
selectable until a fake-IO equivalence report exists.

## 5. Pre-execution checks and motion-time behaviour

Once planning meets the target, field configurations return `start_joint_check`
to ENABLED (0.01 degree) and `pre_execute_visual_recheck` to TARGET_ONLY.
DISABLED_BY_USER stays expressible with its recorded basis. A threshold change
is proposed with measured evidence and decided by the user, never applied by
the implementer.

Still excluded: motion-time replanning, replacing a running trajectory,
starting a segment at nonzero velocity, and using moving-camera frames to
judge a running path.

## 6. Continuous multi-carton job

A new explicit entry, `unloading_sim.workbench.continuous`
(`eco65-workbench-continuous`), may run consecutive carton cycles. For this
entry only, this supersedes "ONE operator-started ... carton cycle", "Only the
operator's single-carton execute button starts the physical cycle" and "no
multi-box scheduling". The single-carton, controlled-empty and read-only
entries keep their scopes. Startup, import, connect, preview and tests never
move hardware or write valve commands.

- One operator click starts one job. Each cycle inside it is the unchanged
  single-carton cycle (approach, contact, detach, loaded return to that
  cycle's frozen q_start, release) with its own task record. A job record
  links the task ids in order and stores each round's target order and skip
  reasons.
- A cycle starts automatically only when all of these hold: the previous
  cycle reached COMPLETE with release confirmed by the pressure gate; no load
  is held; no STOP_UNCONFIRMED; the session generation is unchanged; device
  data is valid; a new stationary frozen RGB-D was taken after that release;
  a complete plan exists with every final audit PASS; the start-joint check is
  ENABLED and passes; and motion starts no later than `plan_validity_s` after
  the frozen exposure (default 5 s; a field file may only tighten it).
  Otherwise the job re-observes and replans; it never reuses.
- No observation, plan, IK seed or path is carried from one cycle to the
  next. No N+1 lookahead planning.
- Target order is deterministic and recorded. A target whose planning fails
  is recorded with its reason and searched/unsearched counts, and the next
  target is tried within the same deadline. A planning failure is not a fault.
- The job ends, with no automatic resume or retry, on: operator stop; any
  cycle state other than COMPLETE; pressure loss; a device or session
  transition; a record write failure; a frozen q_start further than the
  arrival tolerance from the job's first q_start; no target detected
  (recorded as NO_TARGET_DETECTED, never as proof of an empty site); no
  complete plan for any detected target; receiver not cleared in time; the
  configured carton count reached. A new job needs a new operator click. A
  stopped job that may hold a load requires local handling exactly as today.
- Released cartons are objects this system placed. USER_CONFIRMED_CLEAR for
  unobserved space never covers them. The job configuration must name
  `receiver_clearing` with a recorded field basis:
  SENSOR_OBSERVED_CLEAR (the release envelope is measured free in the new
  frozen frame; available only with a measured receiver whose region is in
  view), USER_CONFIRMED_SELF_CLEARING (the user confirms each released carton
  leaves within `clearing_time_s`; recorded as field confirmation, not sensor
  evidence), or OPERATOR_CONFIRM_EACH. A missing policy or basis is a
  configuration error. Until the policy is satisfied the next cycle does not
  start, and where the release pose is a measured support the released
  carton's full envelope stays a collision object.
- One carton type per job: carton mass and controller payload stay the
  configured values and are checked as today.
- Only real complete evidence yields PHYSICAL_CYCLE_COMPLETE per cycle, and
  PHYSICAL_JOB_COMPLETE only when every cycle of the job has it.
- Still excluded: conveyor or lift commands, valve commands outside the
  reviewed set, visual servo, J1 direction and top-height constraints, and
  simulated objects in real scenes.

## 7. Tests and evidence

Run directed checks only. `m710_clearance_test optimized` and `reference`,
`tools/check_eco65_single_carton_native.py` and the directed pytest set of
`docs/eco65_real_site.md` must pass after every step. Each step writes its
report under `docs/evidence/` and appends a dated section to
`docs/eco65_real_site.md`, stating what was not run or not verified.

# November 12 real-site continuous single-carton entry (in force where not superseded above)

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
and explicit blind-clearance evidence. The latest user explicitly confirms all
unobserved regions have been cleared and requests no collision checks for those
regions. Use USER_CONFIRMED_CLEAR with the source of that field confirmation;
omit unknown-only voxel colliders, record that this is not sensor-observed free
space, and retain observed obstacles, fixed boxes, neighbors, workspace bounds,
all robot/tool solids and the complete attached carton collision geometry.
Retain OCCUPIED as an explicit alternative, never silently infer clearance.
No simulated trailer/conveyors/lift/box stack/base placement enters real scenes.
The source snapshot supplies robot/tool/camera hardware geometry only. Honor the
user's explicit current AGENTS instruction deferring q_start receiving-region
qualification for this round: record RECEIVER_QUALIFICATION_DEFERRED_BY_USER,
never support certification. Keep the deferral explicit in configuration with
its user-instruction basis; do not invent receiver bounds or support IDs. Every
uncertainty envelope must pass the complete loaded return and release collision
checks before motion. Deferred qualification grants no environment contact
exemption; unknown-region clearance above is a separate user confirmation.
Retain the explicitly selected
measured-receiver policy for future use without silently falling back.

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
