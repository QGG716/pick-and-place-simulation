# Stage-aware generation evidence

Baseline: `6ec356a9c917e0dd31d3103123dbdd9ec7782a51` (also the clean initial
workspace/remote HEAD). Production changes: `7ba5b37`, `b4c3357`, `c0b9c28`. Final tested
tree including the loaded-route regression/harness: `c0b9c28`. The delivery
commit after it changes documentation/evidence only.

The user authorized local CPU execution after SSH access was unavailable.
The existing `.conda-cpu/python.exe` was used, without installation/replacement.
Python 3.14.7, NumPy 2.5.2, Pinocchio 4.1.0, Coal 3.0.4, Windows x64.

* `source_identity.json`: exact final source/test/harness file SHA-256 values and
  Git commit. These are actual workspace bytes; Git's LF normalization can make
  blob hashes different without altering source semantics.
* `cpu_tests.txt`: **306 passed, 22 skipped in 129.54 s**, one combined invocation,
  not a sum of repeated/overlapping test runs. The 31 new policy cases exercise production entries, with actual
  payload geometry in the real-RRT case.
* `baseline_seam.txt`, `current_seam.txt`: the same five pre-existing seam-test
  failures against both implementations, recorded separately from the directed
  passing set. Three assertions disagree with existing seam support/pair
  reporting; two tool-provider doubles lack `tool_all_physical_obbs`.
* `baseline.json`, `current.json`, `comparison.json`: four short cases, three
  unprofiled cold/warm repetitions, separately profiled hot guards and source
  identities. The baseline source was extracted from the exact Git commit;
  the same current harness was used against each source tree.
* `single_carton_outcome.json`: manually cancelled offline development probe,
  not a geometric failure or a complete task result.

The benchmark cases are:

1. Official-model empty direct connection, through `_connect_pose`.
2. Official-model loaded short connection between unchanged recorded edge-120
   endpoints. Actual remaining state is rebound using the existing handoff
   motion configuration. This reconstructs the old local-first vs new
   joint-direct-first connection order; it does not replay the historical path
   or perform new grasp candidate search.
3. Analytic robot carrying an actual attached OBB, a blocked direct edge and
   real `RRTConnectPlanner` expansion under the same MotionValidator. This is
   explicitly not an official FANUC loaded-obstacle benchmark.
4. Official-model free/contact approach entry, including the controlled terminal
   arc. The legacy explicit motion policy and optional quality behavior are
   retained identically on both sides; these are not substituted for POC rules.

Cold is a new request with the model already loaded, not process/model startup.
Warm repeats the same binding. Optional non-POC comparison windows are included
in total connection latency. `first_checked_free_connection_seconds` separately
records the first retained free connection where the existing evidence exposes
its completion timestamp; it is not full-task completion or the completion of
the subsequent contact arc. Initial measurement trials are kept in ignored local
outputs; only the final serial measurements are presented here.

Work counters distinguish endpoint IK streams/seeds from along-path Cartesian
IK samples. Creating a helper planner during optional simplification is not
counted as a random-tree expansion. The measured `rrt_interface_calls`,
`rrt_extensions` and `rrt_iterations` wrap actual existing planner calls/results.
Profiling the entire connection can legitimately see context construction for
different endpoint stages. The isolated 100-lookup/guard profile checks that hot
guards themselves do not rebuild full JSON/SHA identities.

The source/history/wrist fixtures behind 22 skips were not configured for this
local run. The available archived actual state used by the short benchmark
contains ideal-reception evidence; it cannot be substituted into older physical
reception fixtures without changing their contract. Those skips are unverified,
not passes. No full pytest, historical population, full row or backend sweep ran.

The single-carton development probe supplied one historical contact seed but no
historical path, rebound the actual state and called normal `plan()` in the POC
configuration. It did not finish within the intended short-validation scope and
was externally stopped after about 12 minutes; no production timeout was added,
no request budget was reset, and no second probe was launched. It began before
the final commits while development continued, so it is explicitly **not** a
source-bound final-commit full-cycle validation. No complete path, execution
export or preflight result was obtained. Internal progress was not checkpointed
by that initial probe, so no precise final planning stage or collision cause is
claimed. Future runs of the committed harness capture source identity at entry
and verify it at exit. This round did not run Isaac or connect a real robot.

Reproduction (PowerShell, repository root):

```powershell
$env:PYTHONPATH='src'
$env:PYTHONUTF8='1'
& .conda-cpu/python.exe -m pytest -q `
 tests/test_stage_motion_policy.py tests/test_motion_validation.py `
 tests/test_validation_kernel_official.py tests/test_validation_binding.py `
 tests/test_validation_prefix.py tests/test_pinocchio_broadphase.py `
 tests/test_planner.py tests/test_layout_trajectory.py `
 tests/test_feasible_result_budget.py tests/test_lookahead_contact.py `
 tests/test_contact_candidate_scheduler.py tests/test_motion_quality.py `
 tests/test_search_diagnostics.py tests/test_wrist_transfer_quality.py `
 tests/test_layout_place_fallback.py tests/test_extraction_boundary_guard.py `
 tests/test_conveyor_placement.py tests/test_poc_release_reserves.py `
 tests/test_adaptive_release_motion.py tests/test_default_history_adaptation.py `
 tests/test_ideal_release_handoff.py --basetemp outputs/stage_cpu_unique --tb=short -r s

& .conda-cpu/python.exe tools/benchmark_stage_motion.py `
 --history outputs/delivery/20260922_validation_kernel/delivery `
 --output outputs/current_stage_benchmark.json
```

For baseline, set PYTHONPATH to the extracted baseline `src`, using the same
interpreter/harness, working directory, unchanged configs and assets. The source
paths/hashes in its report identify which implementation was actually imported.
