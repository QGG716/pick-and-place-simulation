# Saved TRANSIT validation and complete-task handoff (2026-09-28)

Baseline: `570955d80009d0d1f0ad05d744a83edbac6db005`, branch
`feat/v0.5-backend-curobo-v2`. Previous evidence remains unchanged.
`frozen_inputs.json` binds the original successful result, request, bundle,
GPU model, provenance, actual state and source motion by SHA256.

## Isolated saved-path comparison

No planner, RRT, or GPU search was invoked for this comparison. Both versions
read the SAME saved successful GPU result. The old authority is the unmodified
baseline checkout; its layout_trajectory.py and pinocchio_backend.py source
hashes in old/validation.json match `git show 570955d:<file>`.

CPU: Intel Xeon Platinum 8470Q, 208 logical CPUs (affinity all 208).
OMP_NUM_THREADS, OPENBLAS_NUM_THREADS and MKL_NUM_THREADS were each 1.
Pinocchio/Coal CPU virtualenv: /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv.
The GPU server has an RTX PRO 6000 Blackwell Server Edition, driver 595.71.05.
No concurrent heavyweight audit, test or simulation ran during either complete
measurement. See old/isolation.txt and new/isolation.txt. Cold means a new
process/validator and empty state/edge caches; the operating-system file cache
was NOT purged. Historical parallel-audit timing is NOT the denominator.

| Run | Accepted | Wall s | CPU s | Fresh samples | Unique fresh states | State hit/miss | Edge hit/miss |
|---|---|---:|---:|---:|---:|---|---|
| Old cold | yes | 1007.792759 | 1007.786288 | 24464 | 24198 | 266 / 24198 | absent |
| Old same-context warm | yes | 1022.024606 | 1021.915583 | 24464 | 24198 | 266 / 24198 | absent |
| New cold | yes | 270.915964 | 270.787732 | 24464 | 24198 | 266 / 24198 | 0 / 260 |
| New same-context warm | yes | 0.067662 | 0.067649 | 0 | 0 | 0 / 0 | 260 / 0 |

Input: 261 points, 260 directed edges. Both cold runs visit the exact same
ordered floating-point states, SHA256
`a084a3b87e0289fcb3a7ce9627e1ff7988480b35dd734f1526788f3487f1cd0a`.
Cold speedup is 3.72x for equal measured work. Warm new validation reuses
certified edges covering 24464 logical samples; it is evidence reuse, not
same-work compute speedup. The old 4096-state LRU thrashes on this long path.

Each measurement calls _path_failure once, _state_failure 24464 times on cold
runs, and the exact validator 24198 times after state-cache hits. There are no
stage_backend or adapter authority calls in this isolated replay. Warm new
validation invokes neither state nor exact collision checks.

## Measured breakdown (inclusive/nested: never add these rows)

| Counter | Old cold s | New cold s |
|---|---:|---:|
| OBB pair failure queries | 670.847919 (3500056 calls) | 6.638751 (1064842 calls) |
| Robot/tool checks | 80.236165 | 78.274810 |
| Robot/fixed including self | 24.550238 | 23.937911 |
| Robot/dynamic environment | 10.595846 | 10.009433 |
| Official mesh self collision (nested) | 15.510961 | 15.274841 |
| Official mesh environment (nested) | 98.186194 | 95.332427 |
| Plane bounds | 43.787529 | 39.341130 |
| Tool/fixed broad phase | 13.346271 | 12.618159 |
| Tool/dynamic broad phase | 21.967363 | 21.347341 |
| FK, Jacobian, joint checks together | 7.076797 | 6.653477 |
| Scene/model context construction (outside path wall time) | 0.467814 | 0.462039 |

New context/key construction: 1 build, 0.000765 s; 1056 mutation guards,
0.286874 s; small state-key assembly, 0.032116 s. Old per-state context/key
assembly, FK vs Jacobian vs joint checks separately, and tool/payload pose
construction separately were NOT isolated. These are not inferred by subtraction.
Full counters and raw measurements are in old/validation.json and new/validation.json.

## Implemented changes and retained safety

The dominant repeated work was exact OBB distance calculation between the held
carton and its own rigidly co-moving tool. The validator computes 202 pairs in
their common verified TCP frame once (0.044640 s), and reuses accepted relative
geometry under a leased context (4084391 hits, including already-permitted cup
checks). This is not a collision exemption. Colliding, uncertain and
roundoff-sensitive boundary pairs retain the original world-frame query.
The optimization is TRANSIT-only, for the two FK/frame-verified robot objects,
and applies only when the queried object IS the current attached payload.
Environment objects with an aliased name cannot receive the result.

One context binds the model/geometry version, joints/base/tool frames, scene,
attachment identity/size/pose, phase, masks, support/contact names, all effective
thresholds, interpolation and authority source version. Context mutation fails
closed; a different context invalidates dependent FK/static caches. Edges are
cached only after successful complete validation. Path-dependent initial
proximity checks cannot use this cache. Ordered legacy sampling remains intact
in blocks of 64; first failing edge/fraction/q/pair/reason remains intact.

The small ContextLease/LRU snapshot primitives were reviewed from feasibility-core
at aa525e873c6a6c8618355923d01f05c9162e87cb. No branch merge, second planning
framework, adaptive sampler, or claimed native batch acceleration is involved.
`_legacy_path_failure` retains the sampling reference; the full OLD performance
entry remains the frozen baseline checkout, not a method using the new backend.

Official robot mesh self/environment checks, actual robot/tool/payload geometry,
all existing permissions, 5 mm pair gaps and the POC edge sampling rule remain
unchanged. No implicit CPU planner fallback was added.

## Fit/planning RNG separation

`geometry_fit_seed` is explicit, defaults to 716, and belongs to geometric fit
configuration. `planning_seed` is explicit in new requests (with the legacy
`seed` alias checked for equality). Changing only planning seed preserves the
original successful geometry key:
`497d64641b11956d0e11a1863a05f3d4cfbd09688360e2a42fc7af7700603da7`.
Fit seed, fitting parameters, schema or mesh content changes invalidate geometry.
Policy changes reassemble runtime masks without fitting again.

cuRobo remains v0.8.0 / 4ea77366ca48ee453e7df139e39fa6532af49f3b.
The pinned MotionPlanner.reset_seed has no replacement-seed argument; it resets
existing samplers. MotionPlannerCfg.create(random_seed=...) initializes the
planner/graph seeds. Therefore a planning-seed change rebuilds the worker while
reusing its geometry file. No unsupported RNG reset or third-party patch is used.

## Directed validation

Saved direct GPU positive: 21 points, 20 edges, 60 expanded / 37 unique states,
accepted in 0.460544 s cold. This is authority replay of the original GPU result,
not a replacement CPU trajectory or the business-path speed denominator.

Initial directed server suite: 106 CPU tests passed; 7 GPU pair tests passed.
After correcting complete-entry backend provenance, 9 runner tests passed,
including the new foreign-build-evidence rejection. See the individual logs.
The regressions cover held-box corner collisions, legal endpoints with a blocked
edge interior and unchanged first failure, J5/J6 versus environment, rigid tool
versus flexible cups, state/edge context invalidation, attachment size/identity,
policy withdrawal, geometry/planning seeds, and changed interpolation rejection.

## Reproduction commands

Run from the appropriate checkout with OMP_NUM_THREADS=1,
OPENBLAS_NUM_THREADS=1, MKL_NUM_THREADS=1, PYTHONPATH=src.
`CPU` below is the CPU virtualenv interpreter above; `GPU` is
/root/autodl-tmp/curobo-v2-20260922/venv/bin/python.

```sh
# Old: baseline checkout at 570955d, with only this measurement tool added.
$CPU tools/validate_saved_curobo_transit.py   --result outputs/curobo26/business/result.json --output evidence/old
# New: candidate source, same result file. No planning command in either run.
$CPU tools/validate_saved_curobo_transit.py   --result outputs/curobo26/business/result.json --output evidence/new
$CPU tools/validate_saved_curobo_transit.py --fixture direct_unit   --result outputs/curobo26/direct_final/result.json --output evidence/direct --warm-runs 0
$CPU -m pytest tests/test_validation_context_reuse.py tests/test_layout_trajectory.py   tests/test_stage_backend.py tests/test_curobo_geometry_cache.py   tests/test_contact_unloading_runner.py tests/test_poc_pair_clearance.py   tests/test_mesh_plane_clearance.py tests/test_m710_drive_reference.py -q
$GPU -m pytest tests/test_curobo_gpu_pairs.py -q
$CPU -m pytest tests/test_contact_unloading_runner.py -q
$CPU tools/run_m710_contact_unloading.py   --config configs/validation/m710id70_handoff_continuation.yaml   --output evidence/full_task --actual-state fixtures/curobo_v2/actual_remaining_state.json   --reuse-motion fixtures/curobo_v2/source_motion.json --target-id carton_l07_c04   --transit-backend curobo_v2 --gpu-python "$GPU" --diagnostics
```

## Preserved diagnostic attempts

new_initial_cache_not_engaged: first new replay was stopped because eligibility
recognized only the verified mesh object while this fixed fixture uses the
also-verified lightweight URDF attachment object. No incomplete-run acceleration
claim is made. Both exact verified identities are now eligible; arbitrary robot
objects remain ineligible. The final isolated comparison was then completed.

full_task_initial_missing_backend_provenance: stopped our own first integration
run after discovering that the injected connector lost its official BuildResult.
The history compatibility checker correctly refused missing source_identity;
ordinary pregrasp search had started. The runner now forwards the actual build
object, checks connector ownership, and preserves all official source evidence.
The corrected complete task uses the existing history adaptation workflow.
No Isaac process was launched for either diagnostic attempt.


## Real complete-task entry

The actual command above starts from the archived actual initial q and carton
registry, validates the unloaded initial state, approaches the carton, reconstructs
contact/attachment and extraction, invokes cuRobo only between extraction and
preplace, then checks placement, release prediction and withdrawal. It does not
insert a historical TRANSIT into a different request. The history file is a
checked candidate hint, not inherited geometric acceptance.

The corrected provenance run completed its first history candidate geometrically:
PASS, 358 points, 911.704282 s complete-task planning/validation. Stage ranges are
home [0,0], pregrasp [0,35], contact [35,38], extraction [38,79], transit [79,341],
place [341,342], withdrawal [342,357]. No ordinary RRT iteration was consumed in
this history-adaptation run. Its actual TRANSIT request differs from the frozen
benchmark in q_goal and planning seed (3263559582), and was planned anew.

Actual TRANSIT: first native attempt CANDIDATE_GENERATED, 261 valid points;
AUTHORITY_ACCEPTED after 268.429595 s authority validation. Native solve including
GPU wait was 11.516086 s, output conversion 0.000833938 s, initialization 1.650202 s
(includes preparation), warmup 0.036672 s, runtime configuration assembly
0.017974 s. Geometry cache hit in 0.001137 s, fit seed 716 and the same successful
geometry key. Request entry to result was 285.156390 s; one-time initialization
is not added again per attempt. These are actual integration timings, not the
isolated performance comparison.

Every native q is retained in the authoritative delivery. Two tiny explicit
endpoint bridge edges produce 263 delivered points; this list exactly equals the
full task TRANSIT slice. No native endpoint was forcibly replaced. Raw native
B-spline data and q/dq/ddq, the checked linear delivery path, and final controller
reference are separate artifacts. See full_task/trajectory_layers.json and
full_task/curobo_transit/request_0001/.

## Execution precondition failure and offline correction

The full-task invocation initially selected the default proof_of_concept execution
configuration while using the continuation motion configuration. The existing
preflight correctly rejected `ValueError: actual motion input policy mismatch`.
The differing search settings were NOT silently ignored. This is the first
execution-precondition failure, not a geometric collision or infeasibility proof.
See full_task/first_execution_failure.json and the unchanged original run.log.
Per the user's stop rule, zero Isaac trials were launched this round.

The final CLI now chooses the repository's matching simulation configuration
for a repository motion configuration. It preserves the verified connector build
provenance and checks its connector ownership. The final CLI also calls the
new execution handoff checker before accepting an immutable first-feasible export.
The complete run above began before these terminal integration fixes; its saved
motion is finalized separately below without a second planning search. Do not
claim that the final edited CLI was rerun end-to-end or that a physical box completed.

The final handoff checker consumes metadata.joint_names and the actual exporter
schema, validates the same C2 reference consumed by runtime, retains attachment,
release and phase boundaries, and rechecks the exported retained geometric edges
through the existing stage validator and current release-envelope helper. Therefore
collinear-node removal cannot inherit acceptance solely from old endpoints.
Original motion/effort limits, official inertias and runtime feedback/contact gates
remain required. Native B-spline derivatives do not certify the C2 controller path.

First offline finalization (final_execution/) passed the matching preflight and
bundle readback, but the new handoff adapter incorrectly looked for joint_names at
the top level. It failed closed before geometric checks. The field mapping was
fixed to metadata.joint_names and the test fixture now uses IsaacReplayBundle's
real serializer. The failed artifacts remain intact. The next isolated output is
final_execution_corrected_schema/.

```sh
$CPU tools/validate_saved_curobo_execution.py   --motion evidence/full_task/motion.json   --state fixtures/curobo_v2/actual_remaining_state.json   --execution-config configs/simulation/m710id70_handoff_continuation.yaml   --output evidence/final_execution_corrected_schema
$CPU -m pytest tests/test_validation_context_reuse.py -q
$CPU -m pytest tests/test_contact_unloading_runner.py tests/test_curobo_execution.py -q
$CPU -m pytest tests/test_m710_replay_contract.py::test_workspace_verifier_checks_sources_manifests_and_optional_asset_audit -q
$CPU -m pytest tests/test_curobo_execution.py -q
```

Final directed inventory is 118 distinct CPU cases plus 7 CUDA cases (125 total),
including focused reruns after fixes. test_inventory_final.txt is collection only;
passing execution evidence is in the corresponding test logs. The additional
preflight-exception recording regression is in preflight_failure_record_tests.log. No full pytest,
row/40-box population, parameter sweep or physical trial was run.


## Final offline execution result

final_execution_corrected_schema/delivery.json is PASS. Matching execution
preflight: 0.491349 s; export: 0.336328 s; bundle readback: PASS. Final controller
geometry handoff: accepted, 920.888355 s cold; complete offline finalization wall
time: 922.409361 s. This is a different full-task workload, not the isolated
TRANSIT benchmark and not physical execution time.

358 source points became 349 retained geometric points and 10186 controller
commands over 196.690047 s. The TRANSIT has 263 delivered source points including
two checked endpoint bridges, and 261 final retained points. The actual retained
edges were rechecked after reduction. All final stages passed: home, pregrasp
(433.661123 s), contact (3.992700 s), extraction (169.942007 s), TRANSIT
(277.738224 s), place (0.176360 s), withdrawal against the placed carton
(17.487999 s) and against the current flight envelope (17.693569 s), followed by
residence checks. These are sequential stage timings; their internal collision
subcounters are not added again.

The exporter uses analytic C2 quintic extrema, not native cuRobo spline derivatives,
for final velocity/acceleration/jerk limits. The pre-release maximum velocity ratio
is 0.406921; acceleration and jerk ratios are 1.0 to floating-point roundoff,
within the existing audit tolerance. Finite official effort limits and inertial
properties remain bound in the replay bundle, and runtime effort, tracking,
collision, attachment, release and receiver monitors remain mandatory.
This is no torque-constrained-planning or physical-reception qualification claim.

The first execution-precondition failure remains the configuration mismatch from
the actual complete entry. It was corrected offline; it was not a collision, an
endpoint model failure, or proof that the task is infeasible. In accordance with
the first-failure stop instruction, Isaac was never launched and there is no new
video or physically completed carton. Ideal reception remains an explicit
assumption in the ready export, not certified physical reception.

Remaining costs are state-wise robot/tool and official mesh environment checks,
pregrasp/extraction grids, and fresh-context full-task validation. Accepted edge
reuse is in-process and context-bound; no old authority_accepted boolean is
accepted across processes. The final edited CLI wiring has focused regression
coverage; the actual full task and its corrected final export were exercised in
separate processes, without repeating GPU search. A new physical cycle remains
unverified.

final_sources.json records the changed source/test bytes. source_match.json
confirms all 17 Python files match the tested server and the staged Git bytes.
.gitattributes preserves this round's evidence bytes across platform checkouts.
