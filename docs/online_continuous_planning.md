# Online continuous planning control plane (0.5.2.dev0)

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
  `MotionBoundaryState`, `PlanningRequest`, `PlanningResult`, and `PlanEnvelope`
  contracts;
- provider-neutral `BackendIdentity`, `BackendProvenance`,
  `BackendCapabilities`, `PlannerBackend`, and `PlanValidator` contracts;
- replaceable and deterministically seeded backend instances, including their
  initialize, prewarm, health, logical-cancel, and shutdown lifecycle;
- FAST, WARM, and COLD escalation;
- rolling-horizon and speculative-plan bookkeeping;
- scene-bound validation, invalidation, and explicit `ReplanReason` events;
- cooperative deterministic asynchronous and synchronous execution;
- `ContinuousPlanningSession`, `ExecutionMonitor`, and latency/idle metrics.

It does not own or modify grasp generation, IK, depalletizing, conveyor layout,
base optimization, collision algorithms, or trajectory time parameterization.
A stronger planner can replace `PlannerBackend` without changing the session.

FAST, WARM, and COLD are scheduling policies, not product or algorithm names.
The scheduler intersects each requested path and motion boundary with declared
backend capabilities. Unsupported routes are skipped, not counted as planning
failures. `PlanArtifactKind.GEOMETRIC_PATH` and
`TIME_PARAMETERIZED_TRAJECTORY` remain distinct;
only the latter can satisfy a `CONTINUOUS_BOUNDARY` contract.

`MotionBoundaryState` contains joint position, velocity, acceleration, time,
and boundary mode. Compatibility compares every value with explicit
tolerances. Equal position alone is never sufficient for a continuous
boundary. STOP boundaries require velocity and acceleration to be zero within
the contract tolerance. Existing geometric fake/default backends produce only
stop-to-stop geometric paths; this module adds no time-parameterization
algorithm.

## Fail-closed behavior

Every request is bound to an immutable planning-world snapshot containing the
actual scene, current joints, robot/tool/payload identity, base and conveyor
state, and configuration identity. Every executable envelope also records its
expected start/end boundary, predecessor, and planning generation. The legacy
read-only `expected_start_state` and `expected_end_state` properties expose the
boundary positions only.

Before execution the control plane checks trajectory DOF and finite values,
continuity from actual `current_q`, and scene/config/tool/payload/base/conveyor
identity. Backend result contract violations and authoritative validation
exceptions enter `RECOVERY`. An ordinary backend exception is represented as
a backend-local operational outcome and may safely fall back when its immutable
failure metadata permits it.

Backend `SUCCESS` is only a proposal. The envelope records proposal provenance,
artifact kind, model fingerprints, planned revision, predecessor, and
generation. It cannot enter `READY` until the independent authoritative
`PlanValidator` accepts the current world snapshot and complete motion
boundary. Validation identity, revision, timestamp, and generation are then
recorded. A backend's own collision-free claim cannot bypass this gate.
For a continuous boundary, execution also requires an actual measured
`MotionBoundaryState`; the control plane never substitutes the planned
velocity as observed robot state.

The default compatibility validator delegates the legacy backend validation
hook through the separate `PlanValidator` interface, after provider-neutral
checks. Production integrations can inject a stronger authoritative validator.
Validator results are rejected if their snapshot, revision, complete boundary,
or robot/world model fingerprints do not match the validation call.

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
No branch manufactures a trajectory.

Domain results (`NO_IK`, `GRASP_CONSTRAINT_FAILED`,
`INITIAL_CLEARANCE_FAILED`, `COLLISION`, and `NOT_EVALUATED`) are separate from
operational outcomes (`PATH_UNSUPPORTED`, `BACKEND_UNAVAILABLE`,
`DEADLINE_EXCEEDED`, `CANCELLED`, `BACKEND_ERROR`, and `CACHE_MISS`). Failure
details record retryability, candidate/target/scene/backend scope, immutable
native diagnostics, and whether another path or backend is allowed. A cache
miss is not evidence that a target is geometrically infeasible.

While plan k is in `EXECUTING`, a speculative request for k+1 may be advanced.
On completion of k, an exact scene match (or explicit backend revalidation)
promotes k+1 to `READY`; a mismatch invalidates it and queues a fresh request.
The speculative start must match plan k's complete predicted end boundary.
Each speculative successor records its predecessor. It can become executable
only after that exact predecessor completes successfully and is the last
successful plan. Failure, deviation, STOPPING, or invalidation cascades to live
descendant plans and requests; unknown and orphan predecessors fail closed.

Rolling-horizon occupancy counts plan slots, without double-counting the same
request as both active and submitted: executing, ready, speculative, active
planning, and queued planning work are all included. Thus horizon 2 means plan
k plus at most one k+1, and one predecessor can have only one live successor.
A rejected submission is checked before request IDs, queues, events, or metrics
are mutated. If execution completes without a new scene revision, the session
remains `WAITING_FOR_SCENE`.

## Test fixtures

The tests use three layers:

1. synthetic feasible scenes for state transitions, revisions, invalidation,
   replan, blocked behavior, and speculative reuse;
2. deterministic unit scenes for FAST/WARM/COLD ordering and latency regression;
3. `tests/fixtures/m710id70_v3_bottom_alternative.json`, referencing the
   independently recorded V3 bottom-support-release evidence. This is one
   real-robot-model integration witness, not a claim about continuous unloading
   or payload qualification.

## Metrics

Initialization and prewarm latency are recorded separately per backend. Each
attempt records queue, planning compute, authoritative validation, and
end-to-end planning latency. Reports also include robot idle time waiting for a
planner, fallback counts by scheduling path, and outcomes by backend and
domain/operational category.

These metrics do not represent production throughput. Startup costs are not
folded into per-request planning performance, and `production_throughput`
remains `null`.

## Future Backend Integration

VAMP and cuRobo may later connect through `PlannerBackend` and `PlanValidator`
adapters. CoAd-class methods may only act as FAST proposal sources whose output
is independently revalidated before it can become ready. The online control
plane contains no provider-name or provider-type dispatch.

Concrete adapters, robot collision geometry, FK/IK, environment collision
models, trajectory optimization, and time parameterization belong to
feasibility/integration work. This branch neither implements nor claims support
for those external planners. It proves only that the stable interfaces,
scheduling, lifecycle, validation gate, metrics, and state machine can safely
host such backends in the future.

The deterministic cooperative executor remains the dev0 execution abstraction.
A real threaded executor is deferred to dev1. Robot/controller execution
backends, runtime stop acknowledgement, and production integration are deferred
to dev2 and later integration work.
