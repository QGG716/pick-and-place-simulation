# Codex development instructions

## Goal
Build a production-oriented first-layer geometric simulator for trailer unloading by a six-axis industrial robot. Keep it deterministic, testable, and independent from any heavyweight simulator.

For the current FANUC M-710iD/70 branch, the immediate goal is to make the confirmed unloading workcell layout the single source used by CPU collision geometry, drawings, frozen scene snapshots, and replay adapters, then accept its static geometry and initial state. The old V3 104-task and 129-carton populations remain historical algorithm regressions and are not engineering acceptance denominators for this new layout. Do not resume search optimization until a new layout-bound task population has been explicitly defined.

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
1. Keep `configs/workcells/m710id70_unloading_layout_v1.yaml` as the only confirmed assembly definition and reject incompatible legacy parameters.
2. Keep planning/collision, figures, and replay bound to the content-addressed frozen scene snapshot; never reinterpret an old trajectory through the current mutable config.
3. Verify the known geometry, deterministic initial state, and actual replay scene dump without claiming unknown trailer length/height or missing mounting-plate CAD.
4. Define a new layout-bound task population before applying the existing contact-aware separation, task-set IK, escape-path extraction, and `SUPPORT_RELEASE` work to success-rate acceptance.
5. Keep dynamic-conveyor A/B, base installation scans, independent lift/extension, and cycle optimization paused until the layout-bound static and task definitions are accepted.

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
