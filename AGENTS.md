# Codex development instructions

## Goal
Build a production-oriented first-layer geometric simulator for trailer unloading by a six-axis industrial robot. Keep it deterministic, testable, and independent from any heavyweight simulator.

## Architecture constraints
- Keep `geometry.py`, `robot.py`, `scene.py`, `ik.py`, `planner.py`, and `grasp.py` independently testable.
- Do not introduce Isaac Sim, ROS 2, or GPU dependencies into the core package.
- Add heavyweight backends behind adapters and optional dependencies.
- Preserve the world convention: +X into trailer, +Y left, +Z up.
- SI units only: metres, radians, seconds, kilograms.
- Never silently accept an invalid home configuration or colliding goal.
- Every new planner or robot backend must expose deterministic RNG seeding.

## Preferred next milestones
1. Batch reachability and collision heatmap.
2. Base-pose optimization for trailer coverage.
3. Support-relation graph and box-removal ordering.
4. Pinocchio + hpp-fcl URDF adapter.
5. ROS 2 trajectory and scene export.
6. Isaac Sim USD scene export.

## Testing
- Add unit tests for every geometry primitive.
- Add regression fixtures for known reachable and unreachable grasps.
- Keep the default demo under five seconds on a laptop CPU.
- Run `pytest -q` before finalizing changes.
