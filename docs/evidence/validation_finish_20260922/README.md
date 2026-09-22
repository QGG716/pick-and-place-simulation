# Validation performance finishing evidence

Baseline: `720fafc6a65765dab849cf75ac873969c2f3aafc`.
Tested source: `20a6c6d0fcf2f0b490ee2f4081d63e36c8f6e4ae`.
Only documentation/evidence commits follow that source snapshot.

- `comparison.json`: same-input/verdict assertions, cold/warm medians, measured
  work, memory and explicit **Isaac not executed** status.
- `baseline.json`, `current.json`: three raw repetitions, exact historical
  fragment endpoints/fractions, source/input hashes, counters and profile rows.
- `*.profile.txt`: independent cold-call profiles for six representative inputs.
- `source_identity.json`: tested commit, exact source/test hashes, archive hash,
  matching benchmark-script hash and existing CPU environment versions.
- `tests.log`: **117 passed in 3.32 s**, one combined suite; overlapping tests
  from earlier partial runs are not added to that count.
- `artifact_manifest.json`: SHA-256 of original server artifact bytes. Git's
  platform newline conversion may change a checkout's text bytes; the downloaded
  archive under `outputs/perf_finish_20260922/` retains the original evidence.

Final directed command, run in the server's current source directory:

```sh
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest -q \
 tests/test_motion_validation.py tests/test_validation_prefix.py \
 tests/test_validation_kernel_official.py tests/test_planner.py \
 tests/test_layout_trajectory.py tests/test_feasible_result_budget.py \
 tests/test_pinocchio_broadphase.py tests/test_lookahead_contact.py
```

Both measured trees run the identical `tools/benchmark_validation_finish.py`.
Cold timings reset dynamic caches and the prepared tool template; model loading
is separately recorded (0.512 s baseline, 0.499 s current). Stateful warm tests
repeat observations from a fresh copy of the original tracker checkpoint.
Non-stateful warm tests exercise edge-verdict cache hits. Profiler and traced
memory runs are excluded from timing medians. Native exact-query counts are
instrumented directly; per-binding query time is not independently resolved.

Server artifacts:
`/root/autodl-tmp/m710-validation-finish-20260922/release_evidence/`.
The original history remains at
`/root/autodl-tmp/m710-validation-kernel-20260922/delivery/` and is never rewritten.
No full task, candidate search, Isaac world, population run or physical
qualification was performed in this finishing round.

See [the technical report](../../validation_kernel_motion_validator_recovery.md)
for interpretations and remaining bottlenecks.
