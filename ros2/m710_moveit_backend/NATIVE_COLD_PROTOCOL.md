# Native-cold worker protocol

This protocol is implementation support, not a successful physical-run claim.
JSONL remains the only IPC, with diagnostics on stderr. All units are SI.

`init` binds the official robot/tool model, exact flexible collider ownership,
72 cup geometries, physical contact transform, TCP transform, and collision
policy. `compliant_tool_links` and `process_geometry` are mandatory for process
planning. The fixed seed belongs to the resident context. Static context reuse
never imports motion results from another invocation.

`ik` receives a current seed `q_start` and geometric `goal_pose`. It invokes
MoveIt's KDL `getPositionIK` once, without random restart or collision acceptance.
The returned state passes finite-value, joint-bound and FK residual checks;
the caller must perform the independent endpoint checks. This is distinct from
Pilz's internal IK work, whose call count is not exposed by its pipeline API.

For strict `plan`, `require_native_motion=true` requires a `task_id`, unique
`stage_id`, and `parent_stage_id` (empty string for the frozen root). A task owns
one persistent MTC `Task` object. Each `NativeProcessStage::computeForward`
invokes the native PipelinePlanner on its first expansion. Resetting MTC for a
new extension retains only native-owned results computed in this task. Selecting
an earlier parent removes the active suffix and restores that parent's native
stage caches. The input state and full private scene/policy remain bound to each
record. There is no JSON path import. Each context permits eight task sessions;
each task has a finite 4096-record limit, including failed candidates.

The parent field must be present and a string: omitted/null parents are not
interpreted as a frozen root. Every empty-parent branch must still use the
original unattached state and world. A session binds its selected target ID;
changing targets within that session, including at audit, is rejected. A child
can reference only a successful native stage stored under that same task ID.
Selecting another branch requires its explicit native stage identity; matching
joint values alone does not select a parent.

All strict stages carry `m710_native_process_v1` and
`m710_native_process_clearance_v1`. The process policy binds the target's current
pose/size, 72 eligible/commanded/contact mask bits, full-ring geometry, named
stack objects, named supports, and any initial-proximity state. The target
exists exactly once in world or attachment. Adjacent stages preserve all other
world objects and preserve target geometry/pose through attach/release.
World shape poses are read from MoveIt's `global_shape_poses_`; the similarly
named `shape_poses_` are object-relative and cannot establish this continuity.

The candidate-private ACM exempts only exact pairs also covered by process
validation: approved flexible lips against named non-target cartons; target
lip compression during contact, withdrawal, or attachment; qualified named
support faces; and stage-scoped attached stack separation. Rigid inserts and
unknown objects keep the ordinary checks. Clearance uses uninflated FCL
geometry and the existing pair gaps. Ordered native output checks enforce cup
compression, full seal rings, support-face separation, gross stack penetration,
and free-space restoration after separation. The legacy non-worsening tracker
also carries its last distances and released state across native stages.
Predicted cup contact masks are planning geometry, not physical observations.

Only free pregrasp/transit/residence requests can use OMPL; geometric LIN goals
and constrained process stages reject OMPL. Pilz LIN uses the bound task TCP
link including rotation and offset. A zero-motion support/attachment/release
event does not manufacture a solver call.

`task_audit` receives the task ID and ordered native stage IDs. It verifies
the native parent chain, real motion records, and one attach/release transition
on a complete contact/extraction cycle. The caller independently audits all
final nonzero edges against these outputs, including any resampling or timing
transformation. This audit cannot replace execution qualification. `compose`
is rejected when strict native motion is requested.

A final zero-motion withdrawal may supply `terminal_state_request` to the
audit. It must name the last native stage as parent, retain its exact joint
state and task/model/target identity, and describe an unattached withdrawal or
residence scene. Goal/path/planner inputs are forbidden. The worker validates
that single state with its existing clearance and process gates, then checks
the same-target ownership conversion and unchanged non-target scene against
the native endpoint. Only this checked transition can count a terminal RELEASE;
the event text alone cannot. It creates no trajectory or solver invocation.

Checks are deterministic finite sampling with conservative geometry; evidence
explicitly records `continuous_swept_proof=false`. Native planning success does
not establish Isaac execution, measured reception, or machine qualification.

Directed C++ checks built in the same Humble environment:

```sh
./build/m710_process_geometry_test
./build/m710_process_policy_test
```

These synthetic counterexamples are never part of the single-carton success
denominator. Retain the actual compiler/test output and native request logs.
