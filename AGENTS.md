# Codex development instructions

## Goal
Build a production-oriented first-layer geometric simulator for trailer unloading by a six-axis industrial robot. Keep it deterministic, testable, and independent from any heavyweight simulator.

For the current FANUC M-710iD/70 V3 recovery branch, the immediate goal is to restore physically reasonable and explainable single-carton full-cycle geometric feasibility without weakening physical constraints, collision checking, safety margins, numerical correctness, the original scenes, or acceptance denominators. Frozen V2 success rates are historical evidence only and are not a valid optimization baseline.

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

## Preferred next milestones
1. P0-1: payload-neighbor pre-existing-contact/proximity-aware initial separation semantics.
2. P0-2: strict task-set grasp IK over face position, roll, bounded orientation freedom, deterministic seeds, and valid wrist-flip/mirrored branches.
3. P0-3: escape-path-based extraction with incremental local SE(3) escape checks and explicit extraction-distance metrics.
4. P0-4: a `SupportRelationGraph`-driven `SUPPORT_RELEASE` motion primitive between grasp and extraction/transit when required.
5. Establish reproducible nonzero full-cycle front, side, or top success cases in the unchanged V3 task populations.
6. Only after milestone 5, resume dynamic-conveyor A/B and base-Z/lift-range optimization.

The detailed contracts, failure taxonomy, metrics, and acceptance gates are in `docs/development_priorities_m710id70_v3_recovery.md`.

## Testing
- Add unit tests for every geometry primitive.
- Add regression fixtures for known reachable and unreachable grasps.
- Add focused regressions for allowed initial payload proximity, motion toward/away from a neighbor, and restoration of normal collision margins in free space.
- Keep strict final FK, joint-limit, collision, tool-clearance, and suction-coverage checks on every task-set IK candidate.
- Keep the default demo under five seconds on a laptop CPU.
- Run `pytest -q` before finalizing changes.
