# Validator input-binding patch evidence

Baseline production source: `f5ad9bf73334ba430d67c74253d96b8847434480`.
Tested production source, tests and harness: `cc724ad32d510ae4641f5c94cfcc4bbd6e23f997`.
The delivery commit after this source commit changes documentation/evidence only.

Both remote trees came from Git archives. [source_identity.json](source_identity.json)
records archive Git IDs, SHA-256 values and actual file hashes: all 277 current
archive files match exactly. Baseline production files also match; only the
benchmark harness was deliberately overlaid from the tested commit, plus the
new test file added for baseline reproduction. Historical motion/state inputs
were read unchanged; their hashes are in both measurement files.

* [baseline_paths.txt](baseline_paths.txt): final production-fixture path-A/B
  tests on baseline source, **4 failed / 7 deselected**, all four return stale
  VALID after changing B. This is an intentional reproduction, not a passing run.
* [final_cpu_tests.txt](final_cpu_tests.txt): **128 passed in 3.82 s**, exit 0,
  from the single combined invocation below. Includes all 11 new binding cases
  and the prior preparation/batch invalidation, ordered stopping and geometry tests.
* [baseline.json](baseline.json), [current.json](current.json): three unprofiled
  cold/warm timings, separately profiled work counts/hotspots, input/source hashes,
  binding-switch measurements and isolated hot lookup/guard counts.
* [comparison.json](comparison.json): paired medians, percentage changes, work
  counts and binding-preparation breakdown. Input hashes, verdicts, failure details,
  sample grids and FK/expensive-check counts were compared for equality.

Server evidence root: `/root/autodl-tmp/m710-validation-binding-20260926/evidence`.
Trees: adjacent `baseline/` and `current/`. Existing environment was reused;
no installation or replacement. Full textual profiles also remain in the server
evidence directory and local ignored `outputs/validation_binding_20260926/evidence`.

Reproduce the final CPU invocation from `current/`:

```sh
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest -q \
 tests/test_motion_validation.py tests/test_validation_prefix.py \
 tests/test_validation_kernel_official.py tests/test_planner.py \
 tests/test_layout_trajectory.py tests/test_feasible_result_budget.py \
 tests/test_pinocchio_broadphase.py tests/test_lookahead_contact.py \
 tests/test_validation_binding.py
```

From `baseline/`, with the new test file copied unchanged:

```sh
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest -q \
 tests/test_validation_binding.py \
 -k 'equal_scene_new_binding or same_elements_different_container'
```

Run the same harness from either tree, selecting the corresponding output name:

```sh
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python \
 tools/benchmark_validation_finish.py \
 --history-dir /root/autodl-tmp/m710-validation-kernel-20260922/delivery \
 --output ../evidence/current.json \
 --cases open_direct very_short_valid loaded_early_invalid history_loaded_transit \
 --baseline-commit f5ad9bf73334ba430d67c74253d96b8847434480 --binding-probe
```

Cold timings exclude model construction; binding timings exclude copying inputs
and checking edges. The separate profiled guard timing includes instrumentation
overhead. Small regressions and the necessary new-container preparation cost are
reported in the [technical note](../../validation_kernel_motion_validator_recovery.md#september-26-bind-cached-validators-to-the-actual-request-inputs).

**No Isaac, candidate search, complete historical task, 104/129 population or
machine connection was run.** These are CPU correctness and short geometry
revalidation results, not new physical or machine qualification evidence.
