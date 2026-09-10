# Online continuous planning control plane (0.5.2.dev7)

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
- synchronous, deterministic cooperative, and single-worker threaded planning
  execution;
- `ContinuousPlanningSession`, `ExecutionMonitor`, and latency/idle metrics.

`unloading_sim.online_execution` depends only on planning contracts and owns
execution-side identity, capabilities, commands, feedback, and the deterministic
execution fixture. `unloading_sim.online_runtime` depends on planning and
execution and owns their caller-thread orchestration. Planning does not depend
back on runtime, and an execution backend cannot modify a session or its queues.

It does not own or modify grasp generation, IK, depalletizing, conveyor layout,
base optimization, collision algorithms, or trajectory time parameterization.
A stronger planner can replace `PlannerBackend` without changing the session.

## External ingress and owner-thread boundary

`RuntimeIngressMailbox` is a bounded, thread-safe multi-producer/single-consumer
port for immutable `PlanningRequest` and authoritative `WorldObservation`
messages. External callbacks may call the compatibility entry points
`runtime.submit_initial(...)` and `runtime.observe_world(...)`; neither call
mutates session state. Only the runtime step owner drains the mailbox and
applies transitions. Capacity, backpressure, duplicate, stale, conflict,
coalescing, and producer/stream/epoch/sequence semantics remain explicit.
Shutdown closes ingress first and subsequent submissions return `REJECTED`.

Execution feedback continues to enter exclusively through
`ExecutionBackend.poll()`. There is no second callback path that can bypass
feedback identity, sequence, progress, terminal, boundary, or lineage gates.

## Deterministic watchdogs and time domains

`RuntimeWatchdogPolicy` uses the injected runtime monotonic clock. Its
conservative defaults are 30 s scene freshness, 5 s execution-feedback
silence, 5 s stop acknowledgement, 10 s stopped-observation reconciliation,
and 30 s planning completion. Every deadline can be disabled with `None`.
Scene staleness defaults to `WAITING_FOR_SCENE`; stopped-observation timeout
defaults to `RECOVERY` and may instead be configured as `BLOCKED`.

At or beyond a deadline, a stale scene cannot start execution; feedback
silence initiates one safe-stop request; missing stop acknowledgement enters
recovery without inventing a STOP boundary; missing post-stop world evidence
remains non-executing and enters the configured terminal state; planning
timeout logically cancels the task and isolates its late result by generation;
and an unhealthy execution backend blocks a start or faults an active
execution. Python planning threads are never forcibly killed.

Producer observation timestamps and producer feedback timestamps are
diagnostic values within their producer domains. Watchdogs use the local
receive time captured by the runtime mailbox or feedback poll. Monotonic values
from different processes are never compared, and wall-clock calendar time is
not used. A future ROS adapter should map its header timestamp to diagnostic
metadata, maintain explicit producer epoch and sequence, and let runtime record
its own receive timestamp; no ROS dependency or adapter is implemented here.

### dev5 compound-failure rules

A planning timeout is classified by ownership. An initial/current planning
timeout enters recovery. When k is still executing and only its speculative
k+1 request times out, the runtime cooperatively cancels and generation-isolates
k+1 without changing k's execution monitor to FAILED or asking the execution
backend to stop. The late k+1 result cannot become READY. If k later completes
without a successor, the existing actual-boundary and authoritative-scene gates
remain mandatory before further planning.

By contrast, an execution protocol error, backend-health loss, or other fault
that makes the active motion uncertain enters the real STOPPING path and issues
one `request_stop`. The returned plan and execution identities must match.
Rejected, exceptional, mismatched, or timed-out stop acknowledgement records
`stop_unconfirmed`; it does not clear the active execution monitor, invent a
STOP boundary, or permit another start. A later valid STOPPED boundary can
still be reconciled with an authoritative world observation.

Feedback has two local runtime times: receive time is captured exactly when
`ExecutionBackend.poll()` first yields the immutable feedback; processing time
is recorded when the owner thread consumes it. Only valid feedback receive
time renews the silence watchdog. Duplicate, stale, conflicting, illegal, or
queued-old feedback cannot renew it. This covers the runtime's poll boundary
only; it does not measure or guarantee upstream adapter/network transport
latency.

Ingress capacity is explicitly layered. The configured request and observation
capacity applies independently to the mailbox and runtime staging, and the
snapshot reports each layer plus the maximum combined bound. Draining respects
remaining staging capacity. Recovery continues consuming authoritative world
observations for audit/reconciliation but does not clear the recovery fault or
start motion. Accepted requests discarded by recovery or shutdown are counted
and journaled rather than silently evicted.

### dev6 safe-stop lifecycle

Execution-task outcome and physical stop evidence are separate facts. A normal
`FAILED`, `DEVIATED`, or `FAULTED` report records the task result and triggers or
continues STOPPING; it does not clear the active execution identity and does not
prove zero velocity. STOPPING/STOPPED feedback used as stop evidence carries
`stop_command_id` and must match the current stop attempt's execution ID, plan
ID, command ID, feedback stream, and producer epoch. Its sequence is checked in
an independent stop-feedback domain, so a valid STOPPED may follow a previously
recorded FAULTED task outcome without rewriting that outcome. An adapter that
cannot correlate stop evidence must fail closed and cannot use an uncorrelated
STOPPED report to authorize motion.

The runtime snapshot exposes the immutable view of the current/last stop
lifecycle: trigger and later failure reasons, command state and identity,
execution outcome, actual stopped boundary, confirmation stream/epoch, world
reconciliation, recovery requirement, and whether explicit resume is permitted.
Original trigger reason is not overwritten by a later command rejection or
watchdog timeout. Each new stop attempt has a new attempt identity and isolated
sequence/watchdog state.

| Condition | Command/evidence | Runtime/session effect | May resume? |
| --- | --- | --- | --- |
| Stop required | No command yet | STOPPING; mandatory step epilogue dispatches once | No |
| Stop accepted | Correlated command result | Continue supervision; `stop_unconfirmed=true` until STOPPED | No |
| Stop rejected/error/timeout | No valid stop evidence | RECOVERY with execution monitor and identities retained | No |
| STOPPED first | Correlated actual STOP boundary | WAITING_FOR_OBSERVATION | No |
| Observation first | Authoritative compatible world retained | Continue STOPPING until STOPPED | No |
| Normal scene-change stop reconciled | STOPPED plus compatible world | Automatic replan from actual stopped boundary | Yes, automatically |
| Severe task/protocol/backend fault reconciled | STOPPED plus compatible world | RECOVERY; original failure remains recorded | Only through public explicit resume |

`resume_after_recovery()` rejects an unconfirmed or unreconciled stop and an
unhealthy execution backend. A normal observation alone never clears a serious
fault. Stop rejection, exception, or timeout cannot fabricate a boundary or
claim hardware emergency-stop capability. The step loop services the mandatory
stop epilogue even when ordinary ingress/error handling returns early, preventing
continuous conflicting input from starving the one-shot stop command.

### dev7 strict execution-adapter contract

`ExecutionFeedback.stop_command_id` remains optional at object-construction
level for source compatibility with ordinary execution feedback. It is not
optional evidence: STOPPING or STOPPED received while a stop lifecycle is
active must carry the exact command identity returned for that attempt. The
runtime never infers, supplies, or substitutes a missing identity. Missing,
wrong, or old identities cannot enter `acknowledge_stop`, reconcile a world,
queue a replan, or authorize a subsequent execution start.

The public `ExecutionStartState` separates `ACCEPTED`, `REJECTED`, and
`UNKNOWN`. A backend exception, malformed result, ambiguous already-running
status, or accepted result with invalid identity means the command may have
reached the controller. The plan and local start-attempt identity are retained,
start is never retried, and the runtime makes at most one bounded plan-scoped
stop request. A correlated stop result or later feedback may fill an unknown
execution identity; it does not rewrite the original start outcome. Snapshot
fields record both the returned identity and the source of any later identity.

The terminal-confirmation key includes feedback stream, producer epoch,
execution, plan, stop command, and local attempt. An identical repeat is
idempotent. A different STOPPED boundary for the active authorization attempt
sets `STOP_EVIDENCE_CONFLICT`, preserves both boundaries, clears automatic
resume eligibility, and ignores ordinary observations as recovery authority.
`retry_stop_after_evidence_conflict()` creates one new stop attempt with fresh
sequence/watchdog state; its new correlated STOPPED plus a compatible
authoritative observation must reconcile before `resume_after_recovery()` can
authorize work. Reusing a stop command identity within the same execution and
plan is rejected because an old confirmation would be indistinguishable.

| Condition | Trusted execution fact | Stop status | Start/resume rule |
| --- | --- | --- | --- |
| Start explicitly rejected | Not started | No stop required | RECOVERY; no automatic retry |
| Start response unknown | Plan known; execution ID may be unknown | Stop required and unconfirmed | Block every new start |
| Correlated STOPPED, world absent/mismatched | Start/execution identity retained | Confirmed, awaiting world | No start |
| Active STOPPED evidence conflicts | Both boundaries retained | Untrusted/conflicting | New stop attempt plus explicit recovery required |
| Fresh correlated proof and compatible world | Original failure still retained | Confirmed and reconciled | Explicit resume may authorize planning |
| Historical execution/attempt feedback | Historical only | Cannot affect current stop | Ignore duplicate or reject conflict |

The execution-start final gate independently checks unresolved stop evidence,
recovery authorization, and ambiguous-start state. This is defense in depth;
it does not depend on a particular upstream session state happening to remain
correct.

## Bounded audit journals

Runtime and session events use configurable `BoundedEventJournal` instances.
Sequence numbers remain globally monotonic after eviction. Consumers call
`events_since(sequence, limit)` and receive retained events plus oldest/latest
sequence, overflow, and explicit history-gap indicators. Snapshots contain
bounded summaries rather than unbounded copies. Planning latency samples,
backend lifecycle samples, invalidated-plan history, request IDs, lineage
tombstones, and terminal histories also have configurable retention bounds.
Eviction means an audit consumer must resynchronize; it is never presented as
a complete history.

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
start a replan until it receives both a stop acknowledgement with the complete
actual STOP boundary and the latest scene snapshot. The world snapshot joint
position must match that boundary. A scene change advances the planning
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

## Planning executors

`SynchronousPlanningExecutor` runs the operation during `submit`.
`CooperativePlanningExecutor` only queues during `submit`; `advance` runs FIFO
work in its caller. The historical `DeterministicAsyncPlanningExecutor` remains
as a deprecated compatibility subclass, but it is not asynchronous in the
threading sense and never creates a background thread.

`ThreadedPlanningExecutor` owns exactly one worker thread. FIFO operations run
on that worker and produce immutable `PlanningTaskCompletion` values. The
worker never mutates `ContinuousPlanningSession`; only an explicit, nonblocking
session `advance`/poll consumes a completion and changes READY/speculative
queues, events, generation, lineage, state, or statistics. `run_until_stable`
uses a finite completion timeout instead of busy-spinning.

Queued work can be cancelled before it starts and never calls
`PlannerBackend.plan`. Once an operation is running, Python cannot safely stop
its thread. The executor records a logical cancellation request; when the
backend declares logical-cancel support, the session also calls
`PlannerBackend.logical_cancel(request_id)`. Without that capability, the
session emits an explicit unsupported event. In both cases generation and
lineage invalidation reject the late completion, so it cannot enter `READY` or
be counted as a successful attempt.

`shutdown(wait, cancel_pending)` is explicit and idempotent. It rejects later
submissions and can cancel queued work. `wait=False` only initiates shutdown;
it does not claim that a running worker has stopped. `wait=True` has a finite
timeout and is appropriate only when the caller knows the backend operation can
return. A threaded executor passed into a session remains caller-owned; the
session does not unexpectedly close a potentially shared executor. Tests
release every synchronization primitive before shutdown and do not rely on
object destruction for cleanup.

Python threads provide control-plane concurrency, but do not guarantee parallel
execution for a pure-Python CPU-bound planner because of the interpreter lock.
They also do not provide hard deadlines or force termination. A hard deadline
requires cooperation from the backend or future process isolation.

## Typed world observations and ingress backpressure

External world updates use a frozen `WorldObservation` envelope. Safety meaning
comes only from `ObservationAuthority.AUTHORITATIVE` or `PREDICTED`; diagnostic
`source` text never selects a branch. Therefore an authoritative producer whose
name contains “predict” is accepted, while a PREDICTED envelope with an ordinary
camera name cannot update the actual session world. Predictions continue to
enter only through speculative `PlanningRequest` creation.

Observation ordering is scoped by producer, stream, producer epoch, and sequence.
An identical same-sequence snapshot is idempotent, conflicting content at the
same identity fails closed, lower sequence/epoch is stale, and a newer producer
epoch may restart at sequence zero. Only accepted authoritative observations can
trigger validation, invalidation, STOPPING, or replan. Scene and robot revision
monotonicity remains independently enforced by `ContinuousPlanningSession`.

Runtime ingress is bounded independently for initial requests, world observations,
and execution feedback. Inputs report or audit ACCEPTED, DUPLICATE, STALE,
COALESCED, BACKPRESSURED, REJECTED, or CONFLICT. A newer pending observation from
the same stream may replace an older one, but the latest revision still passes
session invalidation gates. Different streams are not silently dropped when the
queue is full. Feedback polling stops at the runtime capacity, leaving further
items—including terminal feedback—in the backend until a later bounded step.
Every step also has explicit per-port processing limits.

## Execution control plane

`ExecutionBackend` is a provider-neutral command and feedback port. Its frozen
identity records backend, backend version, and adapter version. Its independent
capabilities declare supported artifact/boundary kinds, stop and command
acknowledgement, reported position/velocity/acceleration/progress, deterministic
stepping, and continuous handoff. A continuous-boundary command fails closed if
the backend cannot report every required q/qd/qdd component; absent velocity is
never filled with zero. Start and stop return an `ExecutionCommandResult`, not a
boolean, so rejection, unsupported operation, unavailability, and error cannot
leave the session falsely executing or stopped.

Frozen `ExecutionFeedback` distinguishes command acceptance, running, braking,
safe stop, normal completion, start rejection, failure, deviation, and backend
fault. Ordering is scoped by feedback stream, producer epoch, and execution ID,
so each new execution may restart at sequence zero. Within one domain, sequence
must increase and RUNNING progress cannot decrease. A late prior-execution report
is stale and cannot advance the current execution. A byte-for-byte equivalent
terminal repeat is idempotently ignored; conflicting content at one identity or
an illegal transition fails closed. Terminal history is capacity-bounded.

Legal transitions are explicit: acknowledgement may lead to RUNNING, STOPPING or
a terminal result; RUNNING may repeat or advance; STOPPING may repeat or become
STOPPED/failure; terminal states cannot transition again. RUNNING to ACCEPTED,
STOPPING back to ordinary RUNNING, and conflicting terminal outcomes are rejected.
When `supports_command_acknowledgement` is true, the first normal feedback must be
ACCEPTED. When false, the accepted start command is the acknowledgement and an
extra ACCEPTED feedback is invalid. Runtime still requires matching execution/plan
identity and a finite complete boundary with the expected DOF.
`stop_command_id` is an optional compatibility extension on the immutable
envelope, but is mandatory when STOPPING/STOPPED is offered as authoritative
evidence for a runtime stop request. It is invalid on ordinary task feedback.
`MotionBoundaryState.time_seconds` is trajectory/execution logical time, while
`observed_at_monotonic_seconds` is the feedback clock; they are not directly
compared and calendar time never participates in boundary matching.

Successful completion passes the reported actual end boundary into the session.
It never substitutes the envelope's expected end. A mismatch in q, qd, qdd,
logical time, or mode is an execution deviation and invalidates descendants.
Stop acknowledgement likewise requires the reported actual STOP boundary and a
latest world snapshot whose `current_q` matches it. The legacy `stopped_q`
compatibility path is stop-only and explicitly constructs zero qd/qdd; it cannot
be used for a continuous boundary.

`DeterministicSimExecutionBackend` is a caller-stepped state-machine fixture. It
uses no background thread or sleep and can deterministically emit success,
failure, deviation, rejection, STOPPING, and STOPPED feedback. Its optional
joint interpolation is only test data: it is not a dynamics, velocity,
acceleration, controller, tracking, or physical execution-time model.

## Continuous planning runtime

`ContinuousPlanningRuntime` composes one session, one planning executor, one
execution backend, and an optional `SuccessorRequestFactory`. Every mutation is
owned by the thread that first operates the runtime. Each `step` has this stable
order, which is part of the tested contract:

1. fail closed any deferred ingress conflict;
2. accept queued initial submissions;
3. consume a bounded number of execution feedback items;
4. apply a bounded number of authoritative world observations;
5. poll planning completion;
6. attempt one READY execution start;
7. attempt at most one speculative successor for the executing plan;
8. issue at most one stop request for the current STOPPING process;
9. advance a deterministic execution backend, when supported;
10. synchronize runtime state and append audit events.

Every early-return path also runs a mandatory safety epilogue that dispatches a
required stop once. This epilogue does not reorder ordinary work; it prevents
error handling from starving safety-command delivery.

Planning workers and execution backends only produce completions or feedback;
they never mutate runtime/session state. A caller-supplied planning executor is
not closed unless `owns_executor=True`. Execution-backend ownership is also
explicit, and shutdown is idempotent.

The successor factory generates only a `PlanningRequest`; it performs no target
selection, grasping, IK, collision, depalletizing, or unloading order logic.
`None` means there is currently no speculative request. Factory output must pass
`submit_speculative`, so horizon, predecessor, predicted revision, generation,
boundary, and authoritative validation checks remain in force. An exception is
an explicit runtime recovery, not a stalled state.

Execution feedback and world observation are separate facts. Feedback never
carries or invents a `PlanningWorldSnapshot`, predicted scenes cannot be observed
as actual scenes, and successful execution without an independent post-execution
observation leaves the session `WAITING_FOR_SCENE`. If a safety-relevant world
revision arrives while running, the session enters `STOPPING`; runtime requests
stop once and cannot replan until STOPPED supplies an actual STOP boundary and
an accepted authoritative world is available. The two facts may arrive in either
order. If STOPPED arrives without a matching world position, runtime enters the
explicit `WAITING_FOR_OBSERVATION` state, retains the boundary, blocks successors,
and reconciles only after a compatible observation. DOF mismatch, observation
identity conflict, or rejected stop fails closed. Rejected/unsupported stop,
controller fault, invalid feedback, execution failure, or deviation preserves
stop supervision and cascades lineage invalidation without pretending that the
robot stopped. Serious faults remain in recovery after STOPPED/world
reconciliation until the public explicit-resume gate is called.

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
attempt records executor-measured queue and compute latency, optional
backend-reported latency, main-thread authoritative validation latency, and
submit-to-validation end-to-end latency. Executor compute is the authoritative
control-plane planning-compute value. A backend-reported value is diagnostic
only and cannot overwrite it. End-to-end latency is at least queue + executor
compute + validation. Reports also include robot idle time waiting for a
planner, cancellation/stale-result counts, fallback counts by scheduling path,
and outcomes by backend and domain/operational category.

Runtime metrics add command/feedback control-plane counts: execution start and
stop requests/rejections, feedback by status, successes, failures, deviations,
duplicate feedback ignored, invalid feedback, and speculative requests created
or skipped. Dev3 additionally counts ingress accepted/stale/duplicate/conflict/
coalesced/backpressured/rejected results, stale execution feedback, and deterministic
steps/time spent waiting to reconcile STOPPED with a world observation. They are
audit counters only, not controller tracking or robot execution performance
measurements.

Dev4 additionally records mailbox results, watchdog activations, journal
evictions/overflows, history-gap reads, bounded sample retention, and whether a
timed-out planning worker is still exiting. Producer timestamps are never
folded into planning or watchdog latency.

For compatibility, the legacy `planning_latency` and
`planning_latency_by_path` report fields retain the backend-reported value when
one exists (otherwise executor compute). New consumers should use
`planning_compute_latency`/`executor_compute_latency` and
`backend_reported_latency` explicitly.

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

Dev2 provides the execution port, feedback contract, deterministic fixture, and
caller-thread runtime, but no real robot adapter. It proves that k+1 can compute
while k remains `EXECUTING`, that actual boundaries and stop acknowledgement are
handled safely, and that failures cannot bypass lineage and validation gates. It
does not provide controller lookahead, trajectory streaming, PLC handshake,
nonzero-velocity seamless blending, real controller feedback, or real unloading
throughput. Dev3 hardens the neutral event envelopes and ports that future
ROS/perception adapters may implement while preserving separate execution-feedback
and world-observation facts; it does not implement those adapters. Production
integration remains future work.
