# Shared motion validation and bounded recovery

Branch: `feat/v0.5-feasibility-core`. Start: `4f6e3037aceb47a525711e58360d98c8751fec6b`.

## Production chain and original defect

`LayoutTrajectoryConnector._transit` used RRT's endpoint-excluding coarse grid,
then `_path_failure` used an endpoint-inclusive, denser grid (especially POC).
A successful direct/RRT candidate failing that final check ended the connection.
The deterministic thin-obstacle regression reproduces this on the baseline:
coarse direct success, strict rejection, although a detour exists.

Direct connection, directed RRT tree edges, candidate assembly, bounded shortcuts,
Cartesian/history path rechecks and final local geometric verification now use
`MotionValidator`. The goal-rooted tree checks child-to-parent travel, matching
the direction eventually output. Timing, jerk, loads, execution preflight and
runtime physics remain separate; geometric validity never claims execution readiness.

## Contract and evidence

`ValidationContext` owns immutable canonical JSON and a semantic SHA-256 identity.
It includes audited model/tool identity, model revision, transforms, joint limits,
scene poses/shapes/names, attached payload identity/extents/transform, contact stage,
target/mask/named-stack permissions, margins, reserves, numerical tolerances,
interpolation and strategy version. Request IDs are not geometric semantics.
New requests clear request-owned evidence; completed-task proofs retain the existing
request-generation binding. Stateful separation additionally binds its initial
tracker checkpoint, uses ordered scalar observation, and disables verdict caching.

Four outcomes: `VALID`, `INVALID`, `INDETERMINATE`, `CANCELLED`. Unknown distance,
changed context, incomplete continuous proof or exhausted budget cannot be valid
or an infeasibility proof. Results retain failure pair/reason/type, motion/edge
location, checked range, context, guarantee, sampling/subdivision/cache counters.

The default guarantee is **DISCRETE_LEGACY_STRICT**: exactly the previous final
grid, including endpoints. POC retains `2*max(ceil(max|dq|/resolution),
ceil(4*sum|dq|/.0025))` intervals. This is not continuous qualification.
An explicit continuous request requires complete contract evidence; otherwise it
returns indeterminate even when all discrete samples pass.

## Recovery and budget

Rejected direct edges enter RRT. Final candidate failures support bounded tree
restart under the original iteration/work/deadline/cancellation budget. Restart
removes both dependent trees. Failed edges bind exact IEEE joint bytes, direction,
context and motion parameters; no rounded endpoints or neighborhood blacklist.
Indeterminate results are not negative-cached. A contradictory completed verdict
for the same contract/motion raises `VALIDATION_CONSISTENCY_DEFECT`.

The first verified baseline remains available when optional optimization fails,
times out or is cancelled. Existing contact/placement orchestration is retained.

## Actual native work and fallback

`ValidationKernel` traverses the fixed URDF chain once per bounded batch. NumPy's
compiled matmul/ufunc/einsum operations process the state dimension: FK, Jacobians,
official collision transforms, all tool centers/extents and outward AABBs.
`KinematicState` shares these with Pinocchio mesh placement, tool, plane and stage
checks. Coal narrow phase and stage predicates remain scalar. This is not a claim
of native end-to-end collision checking or GPU planning.

Profiling the baseline identified repeated `_world_aabb` tool work as a major
hotspot. The production backend now consumes the batched outward bounds using
owned object identity. Tests compare official FK/Jacobian/mesh transforms and all
tool corners against scalar reference and verify containment by the new bounds.

Default mode is `NUMPY_NATIVE_BATCH_FK_AND_TRANSFORMS_SCALAR_COAL`.
`M710_VALIDATION_KERNEL=scalar_reference` explicitly selects the reference mode;
no unavailable extension is silently replaced while claiming native performance.
No dependency or server environment was installed/replaced.

## Adaptive pair certificates

For linear joint interpolation, each moving body receives a conservative swept
point bound from all upstream joint angles, chain offsets, collision mesh extent,
tool mounting/solid extent and attached payload corners. Both bodies contribute
for self collision and robot/tool pairs. A pair is skipped only if an outward
AABB Euclidean distance lower bound minus both motion bounds exceeds the existing
pair clearance plus 1 nm. The POC external 5 mm and self-contact policies are
unchanged. Proofs never grant contact permission.

Long edges subdivide on indices of the original strict grid. Proven pair checks
are omitted inside their bound interval; unproven pairs and all stage constraints
retain the strict samples. Small discrete edges skip expensive certificate
preparation, preserving every sample. Unsupported/stateful/legacy collision
semantics retain their scalar checks. The production certificate deliberately
does **not** claim the complete stage contract continuously safe.

Caches bind context/exact state or exact directed motion. Bounded incremental
eviction replaces size-triggered full flushes; geometry changes still invalidate
the corresponding model cache. Unknown query results are not retained as invalid.

## Validation and limitations

All CPU checks and measurements run on the existing GPU server's CPU venv.
Directed command:

```sh
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest -q \
 tests/test_motion_validation.py tests/test_validation_kernel_official.py \
 tests/test_planner.py tests/test_layout_trajectory.py tests/test_feasible_result_budget.py \
 tests/test_pinocchio_broadphase.py tests/test_lookahead_contact.py
```

`tools/benchmark_validation_contract.py` runs unchanged in the start-commit and
current trees, against the same official assets/environment/policy. It records
cold and warm results, actual native/Coal/AABB calls, sample counts, preparation
time, isolated batch FK and thin-obstacle detour behavior. Invalid baseline
detour output is not compared as a faster successful solution.

Performance is workload-dependent: batching can do extra work on an early-invalid
edge; context hashing can exceed the old warm-cache overhead. These costs must be
reported alongside improvements. Stateful extraction remains an ordered scalar
path. No full continuous stage proof, new backend, row stability or real-machine
qualification is claimed. One new-world single-carton Isaac outcome is retained
separately from CPU geometry and historical completed cartons.

Final test, benchmark and single-world evidence are recorded below after execution.
