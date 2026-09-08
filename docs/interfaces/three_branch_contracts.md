# Three-branch contract, schema 1.0.0

All domain objects live in the dependency-free `unloading_contracts` package. ROS messages are transport mappings, not a second business model. Every producer rejects non-finite values and preserves absent optional evidence as absent.

## Perception to world state

`SensorFrame -> PerceptionObservation -> PerceptionSceneUpdate -> PlanningWorldSnapshot`

- Vision owns pixels, source instance identity, hypotheses, evidence, capture time, model/config provenance, and coverage. It does not own robot or attachment state.
- Execution or explicit simulation feedback owns actual joints, tool/payload attachment, base, and conveyor state.
- `SnapshotAssembler` refuses to create a snapshot until all mechanism/config inputs exist. A complete snapshot may still be `COMPLETE_BUT_NOT_PLANNABLE` when an unknown region, missing transform, stale observation, or unverified metric scale exists.
- `SceneRevision` changes on geometry, admission, attachment, calibration, or unknown-region changes. Observation sequence and log/receive times do not by themselves alter the canonical scene fingerprint.
- Source `instance_id` is frame-local. `ObservationTracker` provides deterministic IoU association, reports ambiguity, rejects duplicate/late frames, resets on epoch changes, and retains missed objects as stale conservative obstacles.

## World state to feasibility

`CandidateProvider.candidates(snapshot)` is the only candidate-generation seam. It receives a complete immutable snapshot and may emit candidates only for `candidate_eligible` objects. Actual grasp candidates, FK/IK, collision, extraction, payload/dynamics qualification, and time parameterization remain in `feat/v0.5-feasibility-core`.

## Online control to feasibility and validation

The online branch calls `PlannerBackend.plan(PlanningRequest, PlanningCandidate, PlanningPath) -> PlanningResult`. The request binds the full world fingerprint and `MotionBoundaryState`. It does not create a request when there are no real candidates.

`PlanValidator.validate(PlanEnvelope, PlanningWorldSnapshot, MotionBoundaryState)` is the independent gate. Contract checks may report identity/version completeness but never claim collision, payload, or dynamics success. Physical gates separately report pass, explicit failure, or `NOT_EVALUATED`.

## Online control to ROS execution

`ExecutionCommand` binds command/plan/request/session/epoch, planning generation and predecessor, exact world/model/config identities, validation reference/generation, and a complete `TimedJointTrajectory`.

The ROS bridge rejects geometric-only paths, joint-order changes, non-increasing time, missing authorization, duplicate command IDs, stale epoch/generation, a changed world, and unverified hardware mode. It forwards the unchanged trajectory to Humble `FollowJointTrajectory`; it never inserts a fixed `dt`, interpolates, smooths, or rescales.

Cancellation acceptance emits `CANCEL_ACCEPTED`, not `StopAcknowledgement`. Stop confirmation requires matching plan/generation/epoch plus measured positions, measured near-zero velocity, a timestamp, criterion, and evidence reference. Recovery remains an online-control decision and additionally requires a current valid world snapshot.

## Compatibility and failure codes

Schema major versions must match. Minor additions are optional only when their absence retains fail-closed behavior. Unknown enum values, missing identity fields, and malformed finite/dimension checks reject the message.

Stable failure codes include `BACKEND_UNAVAILABLE`, `WORKER_TIMEOUT`, `WORKER_CRASH`, `WORKER_SCHEMA_ERROR`, `LATE_WORKER_RESULT`, `GEOMETRY_3D_MISSING`, `GEOMETRY_3D_REJECTED`, `MONOCULAR_SCALE_UNVERIFIED`, `CAMERA_EXTRINSICS_MISSING`, `WORLD_TRANSFORM_MISSING`, `UNKNOWN_OR_UNTRANSFORMED_REGIONS`, `DUPLICATE_COMMAND`, `STALE_EPOCH_OR_GENERATION`, `WORLD_CHANGED_BEFORE_SEND`, and `HARDWARE_ADAPTER_NOT_VERIFIED`.

Apply the contracts-only commit first, then `online-continuous-contract-reexport.patch`, then `feasibility-core-contract-adapter.patch`. Patch verification uses detached worktrees and isolated processes; different `unloading_sim` branches must never share one `PYTHONPATH`.
