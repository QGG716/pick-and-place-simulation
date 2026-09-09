# Codex development instructions

## Goal
Build a production-oriented first-layer geometric simulator for trailer unloading by a six-axis industrial robot. Keep it deterministic, testable, and independent from any heavyweight simulator.

For the current FANUC M-710iD/70 branch, the immediate goal is to close one physically executed, layout-bound pick-and-place cycle using traceable real robot/tool CAD, the existing strict kinematic planner, and an explicitly sourced rigid-body dynamics configuration.  The confirmed layout and its frozen scene snapshot remain the single source for CPU planning, collision geometry, replay, and evidence.  Never present source-only CAD, proxy collision, an incomplete trajectory, or an engineering dynamics estimate as an execution-qualified result.  The old V3 104-task and 129-carton populations remain historical algorithm regressions and are not engineering acceptance denominators for this new layout.

## Architecture constraints
- Keep `geometry.py`, `robot.py`, `scene.py`, `ik.py`, `planner.py`, and `grasp.py` independently testable.
- Do not introduce Isaac Sim, ROS 2, or GPU dependencies into the core package.
- Add heavyweight backends behind adapters and optional dependencies.
- Preserve the world convention: +X into trailer, +Y left, +Z up.
- SI units only: metres, radians, seconds, kilograms.
- Never silently accept an invalid home configuration or colliding goal.
- Every new planner or robot backend must expose deterministic RNG seeding.
- Do not globally reduce collision margins or create contact exceptions for robot-trailer, robot-tool, robot-chassis, or other non-payload safety pairs.
- Do not let payload qualification failures short-circuit geometric search, and do not modify FANUC load curves to manufacture a pass.
- Preserve the original 104-task grid denominator and the 40/27/32/30-carton continuous scenes.
- Do not reinterpret those legacy denominators as acceptance evidence for `m710id70_unloading_layout_v1`.

## Preferred next milestones
1. Resolve the ROBCAD `J3 follows J2` interpretation, the 180-degree tool0 clocking discrepancy, and the real base mounting extent without changing the confirmed assembly envelope.
2. Convert the supplied M-710iD/70 sources into audited per-link render meshes and CAD-derived compound collision meshes with a converter that genuinely supports Parasolid/JT; never rename formats or substitute proxy geometry.
3. Restore a strict, collision-checked complete trajectory for at least one of the five exposed top-layer cartons while retaining all 40 cartons and the existing suction, FK, joint-limit, and contact criteria.
4. Execute that trajectory through the optional Isaac adapter with finite engineering masses, inertias, drives, contact parameters, and vacuum break limits; all 40 cartons must remain dynamic over a real physical floor and known side-wall constraints.
5. Save real-state execution logs and normal-time video only after the CAD, trajectory, and physics preflight gates pass.  Keep base scans, lift/extension changes, online replanning, ROS/vision, and cycle optimization out of this branch milestone.

The detailed contracts, failure taxonomy, metrics, and acceptance gates are in `docs/development_priorities_m710id70_v3_recovery.md`.

## Testing
- Add unit tests for every geometry primitive.
- Add regression fixtures for known reachable and unreachable grasps.
- Add focused regressions for allowed initial payload proximity, motion toward/away from a neighbor, and restoration of normal collision margins in free space.
- Keep strict final FK, joint-limit, collision, tool-clearance, and suction-coverage checks on every task-set IK candidate.
- Keep the default demo under five seconds on a laptop CPU.
- Run `pytest -q` before finalizing changes.
- Add literal numeric tests for every confirmed layout dimension; do not prove a constructor with formulas copied from that same constructor.
- Verify snapshot and replay fingerprints, all 40 carton identities, legal assembly contacts, illegal penetration, and deterministic initial-state evidence.
