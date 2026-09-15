# Codex development instructions

## Goal
Build a production-oriented first-layer geometric simulator for trailer unloading by a six-axis industrial robot. Keep it deterministic, testable, and independent from any heavyweight simulator.

For the current FANUC M-710iD/70 branch, the immediate goal is to close one physically executed, layout-bound pick-and-place cycle using the complete official FANUC model at `FANUC-CORPORATION/fanuc_description@fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`, the supplied Wantai tool geometry, fail-closed collision acceptance, and Isaac rigid-body dynamics.  The confirmed layout and its frozen scene snapshot remain the single source for CPU planning, collision geometry, replay, and evidence.  The official public link inertials, joint limits, velocities, and effort limits are accepted engineering simulation inputs for this work; machine certification remains a separate evidence field and must not gate simulation.  The old V3 104-task and 129-carton populations remain historical algorithm regressions and are not engineering acceptance denominators for this new layout.

The active user-approved simulation policy exempts only the official J5_link/J6_link versus owned tool colliders, consistently in CPU planning and Isaac.  During attached stack separation, target/stack contacts may use planner_relaxed_physics_checked: real carton collision, gravity, friction, progress and disturbance monitoring remain active in Isaac.  Free transit restores normal object-pair rules.  These explicit assumptions permit engineering execution and are not machine certification.  Resolve tool representation coverage from the existing CAD, rather than waiting for a user certificate.  Historical strict initial-clearance failures remain evidence of the previous policy, not blockers for the active one.

This round uses `ideal_independent_cups`: all 72 cups have stable independent IDs and commands, every commanded cup must have a geometrically valid full seal ring on the selected target, and at least one valid cup is required.  No minimum load-bearing cup count, vacuum-force, shear, peel, leakage, or break-envelope check may reject a grasp in this explicit mode.  This holding-capacity assumption never permits remote attachment, wrong-carton attachment, rigid-tool penetration, collision bypass, payload mass reduction, infinite joint effort, or pose teleportation.

The September 15 user update selects `post_landing_transport.mode=ideal_outfeed`.
Only after real constraint removal and the same body's first qualified actual
receiver-top contact, that carton becomes collision-disabled kinematic ideal
transport. It moves continuously in simulation time via -Y transverse / -X
longitudinal routing until its entire envelope crosses trailer opening X=-3.2 m.
It then records OUTFED_ASSUMED and may become inactive, retaining its identity.
Post-landing tipping, collisions, tail holds and capacity are not evaluated in
this explicit assumption. All pre-landing physics and other object pairs retain
their prior rules. Count actual reception and assumed outfeed separately; do not
claim downstream physical qualification. The current run stops after the initial
highest row and its ideal outfeed, leaving the lower 35 cartons physical.

The user-selected default recording profile is 640x360 at 5 fps, normal time.
Keep the existing live world and its recording profile until that run finishes.
Lower rendering cadence must not lower physics or receiver-monitor cadence.

## Architecture constraints
- Keep `geometry.py`, `robot.py`, `scene.py`, `ik.py`, `planner.py`, and `grasp.py` independently testable.
- Do not introduce Isaac Sim, ROS 2, or GPU dependencies into the core package.
- Add heavyweight backends behind adapters and optional dependencies.
- Preserve the world convention: +X into trailer, +Y left, +Z up.
- SI units only: metres, radians, seconds, kilograms.
- Never silently accept an invalid home configuration or colliding goal.
- Every new planner or robot backend must expose deterministic RNG seeding.
- Do not globally reduce collision margins.  Preserve all checks except explicit J5/J6 versus owned-tool simulation exemptions and stage-scoped target/stack planning contact relaxation.  Tool/environment, wrist/environment and other robot/tool pairs remain checked.
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
8. The user restored GPU-server access during this round. Run all further planning, checks and Isaac execution on that server. Commit and push only feat/v0.5-feasibility-core; record actual physical execution and continuous video separately from CPU plans.
9. The current motion-quality round stops after a same-actual-state unloaded-transition comparison, one bounded reproduction of the fifth-carton failure, and a real release-to-next-grasp fragment. Retain failures. Do not default to another full row or the historical populations. Use 640x360 at 5 fps and 1x physical time, preserving physics/control/contact-monitor rates. The previous physical world ended; new-world trials must be identified separately without cross-world completion counts.

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
