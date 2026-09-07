# Online continuous planning control plane

## Acceptance scope

This branch validates the online control plane independently of bottom-layer
geometric planning success. The V3 baseline remains 0/104 full geometric tasks
and 0/129 continuous-scene tasks for the original M-710iD/70 workload. Neither
those task sets nor 900 boxes/h is an acceptance gate for this module.

The current conservative CPU timing model is stop-to-stop. Reports produced by
the control plane contain only planning latency and robot idle time while
waiting for a planner. `production_throughput` is deliberately `null`.

## Boundary

`unloading_sim.online_planning` owns:

- immutable `SceneRevision`, `RobotStateRevision`, `PlanningWorldSnapshot`,
  `PlanningRequest`, `PlanningResult`, and `PlanEnvelope` contracts;
- replaceable and deterministically seeded `PlannerBackend` instances;
- FAST, WARM, and COLD escalation;
- rolling-horizon and speculative-plan bookkeeping;
- scene-bound validation, invalidation, and explicit `ReplanReason` events;
- cooperative deterministic asynchronous and synchronous execution;
- `ContinuousPlanningSession`, `ExecutionMonitor`, and latency/idle metrics.

It does not own or modify grasp generation, IK, depalletizing, conveyor layout,
base optimization, collision algorithms, or trajectory time parameterization.
A stronger planner can replace `PlannerBackend` without changing the session.

## Fail-closed behavior

Every request is bound to an immutable planning-world snapshot containing the
actual scene, current joints, robot/tool/payload identity, base and conveyor
state, and configuration identity. Every executable envelope also records its
expected start/end state, predecessor, and planning generation.

Before execution the control plane checks trajectory DOF and finite values,
continuity from actual `current_q`, and scene/config/tool/payload/base/conveyor
identity. Backend result and validation contract violations enter `RECOVERY`.

If the scene changes during execution, the session enters `STOPPING`. It cannot
start a replan until it receives both a stop acknowledgement with the real
`stopped_q` and the latest scene snapshot. A scene change advances the planning
generation; pending or late results from older generations cannot enter
`READY`.

For each target/candidate the control plane tries FAST, then WARM, then COLD.
After the last path, it advances to the next candidate and then the next
target. `NO_IK`, `GRASP_CONSTRAINT_FAILED`, `INITIAL_CLEARANCE_FAILED`, and
`COLLISION` remain failures. If all evaluated candidates fail, the state is
`BLOCKED`; if every result is `NOT_EVALUATED`, it is `WAITING_FOR_SCENE`.
Backend exceptions enter `RECOVERY`. No branch manufactures a trajectory.

While plan k is in `EXECUTING`, a speculative request for k+1 may be advanced.
On completion of k, an exact scene match (or explicit backend revalidation)
promotes k+1 to `READY`; a mismatch invalidates it and queues a fresh request.
The speculative start must equal plan k's predicted end. If execution completes
without a new scene revision, the session remains `WAITING_FOR_SCENE`.

## Test fixtures

The tests use three layers:

1. synthetic feasible scenes for state transitions, revisions, invalidation,
   replan, blocked behavior, and speculative reuse;
2. deterministic unit scenes for FAST/WARM/COLD ordering and latency regression;
3. `tests/fixtures/m710id70_v3_bottom_alternative.json`, referencing the
   independently recorded V3 bottom-support-release evidence. This is one
   real-robot-model integration witness, not a claim about continuous unloading
   or payload qualification.
