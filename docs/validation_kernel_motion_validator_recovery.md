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

## Measured delivery

Execution-source candidate: `9b90c6bf6b4528041df2bfd13ef07079a23d1d81`.
Directed checks: **93 passed** (2.77 s). Thin-obstacle detour: 2 RRT iterations,
strictly valid; the baseline returns the invalid direct edge. Repeated runs reuse
the exact rejected edge and keep the detour available.

After locking that source snapshot, reporting-only corrections separate cached
edge proof sample coverage from newly performed work and retain the first direct
edge verdict in planner evidence. The 25 motion-validation/planner regressions
passed in a separate CPU source directory (0.14 s). The locked
planning/Isaac tree remains byte-identical to `9b90c6b`; final-branch code includes
these reporting corrections. They change neither a validation verdict nor physics.

Same-server profiled times (seconds; these small cases are not a throughput claim):

| Case | Baseline cold | Current cold | Baseline warm | Current warm |
| --- | ---: | ---: | ---: | ---: |
| Open direct, 35 strict samples | .44225 | .29599 | .00113 | .00319 |
| Layout direct, 5 strict samples | .07915 | .06969 | .00128 | .00455 |
| Loaded near stack, rejected | .01796 | .07735 | .00034 | .00477 |

Open-edge Python AABB calls decrease **8365 → 385**; layout-edge calls decrease
**1313 → 303**. The open edge previously made 35 Pin FK, 35 Jacobian,
35 geometry-update and 70 Python-chain FK calls; these per-state FK calls become
zero in native-batch mode. Exact Coal queries remain one collide and one distance
in each cold case, zero warm: no speedup is falsely attributed to fewer exact
queries. The current measurement reports 9090 native tool-AABB hits and 455
interval pair skips across the cases.

Pinocchio/Coal/Python-AABB function counts and native batch counts are instrumented
counts. The kernel's `native_array_operations` field is a structural operation
estimate, not a complete measurement of every NumPy C entry or allocation.

The 512-state official FK submeasurement is .04051 s scalar versus .001263 s batch
on the current implementation; it is **not** an end-to-end 32× speedup. Warm paths
regress because of context/evidence overhead. The loaded failing edge regresses
because batch preparation/checking evaluates five samples before returning the
first rejection, whereas the scalar reference stopped after one. Neither result
is presented as a successful loaded transit.

Artifacts live under server `/root/autodl-tmp/m710-validation-kernel-20260922/`:
`baseline_benchmark.json`, `current_benchmark.json`, their profiles, tests,
source manifest and the separately identified physical outcome.

The representative existing `carton_l07_c04` hint was processed by the normal
current planning entry with explicitly paired `m710id70_handoff_continuation`
motion/execution configurations and archived actual-state binding. It performed
one history adaptation, zero ordinary candidate attempts, 84674 actual state
validations and 25 local IK calls; no new candidate search was expanded. Full
geometric revalidation took **1656.17 s**, first exportable result **1659.45 s**.
This substantial remaining cost is not hidden by the microbenchmark improvements.
Preflight was READY with no blockers, fresh export/readback PASS, 8302 commands,
161.6044 s reference duration. Old motion/source fingerprints were not rewritten.

Representative revalidation command (executed once on the configured server):

```sh
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python \
 tools/run_m710_contact_unloading.py \
 --config configs/validation/m710id70_handoff_continuation.yaml \
 --execution-config configs/simulation/m710id70_handoff_continuation.yaml \
 --reuse-motion /root/autodl-tmp/m710-auto-resume-20260920/repo/outputs/plan_002/first_feasible/motion.json \
 --actual-state /root/autodl-tmp/m710-auto-resume-20260920/delivery/segments/001_carton_l07_c03/actual_remaining_state.json \
 --target-id carton_l07_c04 --output outputs/kernel_single \
 --execution-bundle outputs/kernel_single/replay_bundle.json --diagnostics
```

The fresh bundle SHA-256 is
`06c2bd3714a3f0d1471a6ab440c7abf0a2c1b8d047a47f94577a9961736a19279`.
The physical launcher, archived inputs and complete planning artifacts are retained
in the delivery directory. The launcher checks the locked source manifest and
readback PASS before starting a single segment. No old fingerprint is patched.

## Single Isaac outcome

Exactly one trial ran in the existing Isaac environment, exit code **0**.
World **`1790070472.2478814`**, target **`carton_l07_c04`**, executed source
**`9b90c6bf6b4528041df2bfd13ef07079a23d1d81`**. Source hashes were checked again
after exit and matched. No environment installation, additional physical trial,
automatic recovery or next-carton request occurred.

The full schedule completed: **38940 steps at 240 Hz**, **162.25 s** physics,
**1888.76 s** replay wall time. Runtime first failure and stop reason are null.
Actual grasp, transport, production constraint removal and departure completed.
New-target counts are grasp **1**, release **1**, ideal reception **1**, ideal
outfeed **1**, workflow completion **1**, measured physical reception **0**.
Historical three cartons are separately identified and are not new-world results.

`simulation_execution_qualified=true`, `workflow_cycle_completed=true`,
`physical_cycle_completed=false`, `actual_reception_succeeded=false`,
`machine_qualified=false`. Reception is explicitly `RECEPTION_ASSUMED` under the
unchanged proof-of-concept profile. The legacy `qualification_passed=false`
records the unassessed actual-drive-effort channel (`NOT_EVALUATED`), not a newly
measured effort failure. No commanded effort/model estimate substitutes for it.

Unexpected contacts: **0**. Peak tracking error **0.00151929 rad** against the
existing **0.05 rad** limit. Attachment peak displacement **0.0000169704 m**
against **0.02 m**, peak rotation **0.0000396667 rad** against **0.1 rad**.

Video readback: **640x360**, **5 fps**, **811 frames**, **162.2 s**, normal physical
time, bottom-left minimal overlay. The overlay retains the archive's cumulative
time and historical counts; `new_target_execution_counts` is authoritative for
this world's single target. Saved-frame inspection does not advance the world.

Raw planning/physics records and video are in
`outputs/delivery/20260922_validation_kernel/delivery/` locally and
`/root/autodl-tmp/m710-validation-kernel-20260922/delivery/` on the server.
The [tracked evidence index](evidence/validation_kernel_20260922/README.md)
contains the compact result, source/file hashes, tests and performance records.
This single POC result establishes neither row stability nor real-machine or
physical-downstream qualification.
