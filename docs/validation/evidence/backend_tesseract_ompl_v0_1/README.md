# Tesseract + OMPL v0.1 evidence

See [the experiment report](../../../backend_tesseract_ompl_v0_1.md) for interpretation, commands, scope and limitations. These are CPU planning results; no file in this directory constitutes an Isaac execution pass.

| Files | Meaning |
|---|---|
| `comparison.json`, `comparison.log` | Original three fixed FANUC cases, both backends, including failures. Preserve unchanged. |
| `benchmark-build.sha256`, `worker-benchmark-source.cpp` | Native binary/source provenance for the original comparison and long task call. |
| `task-result.json`, `task.log`, `task-native.log` | The single real `LayoutTrajectoryConnector.plan` call and its raw logs. |
| `task-summary.json` | Explicitly derived summary of `task-result.json`; native iterations are unavailable. Legacy counters must not be read as native counters. |
| `task-candidate-definition.json` | Read-only expansion of existing approach candidate definitions, without another search. |
| `final-native-tests.json`, `final-native-regression.log` | Latest six real native regressions, including exact success followed by approximate rejection under a smaller finite budget in the same synthetic unit scene. No cached path is returned. |
| `final-tests.log`, `before-approximate-budget-test-native.json` | Earlier 55-test related-module batch and its native responses. |
| `final-contract-tests.log` | Last 14-test contract-only check, including one additional legacy identity regression. Final unique test coverage is 56. |
| `environment.json`, `platform-check.log`, `final-build.log` | Final native package builds, CPU authority environment, actual platform, linkage, binary hash and successful build. |
| `before-approximate-flag-*` | Earlier build/environment/test snapshots retained separately before approximate-solution reporting was clarified. |
| `final-functional-smoke.json`, `.log` | Real FANUC native adapter plus common authority check with conversion timers. This predates the approximate-reporting clarification; not an extra performance-comparison sample. |
| `native-tests.json`, `initial-native-tests.log`, `baseline-existing-failure.log` | Initial native checks and the independently reproduced pre-existing scripted-fixture failure. Only that fixture's missing IK tolerance was repaired. |
| `directed-tests.log` | Earlier 37-test focused batch. |
| `test-command-path-error.log` | Incorrect test filename invocation; it ran zero tests and is not counted as validation. |
| `install.log`, `install-pypi.log`, `source-fetch.log`, `conda-build-env.log`, `configure.log`, `build.log` | Dependency selection, retained installation/source-fetch failures and successful isolated C++ route. |
| `probe.json` | Initial real native geometry/FK/straight-line probe; this alone is not common-authority validation. |
| `evidence-index.json` | Content hashes and source provenance for this delivered evidence set. |

Frozen inputs are in `tests/fixtures/tesseract_ompl/`. `historical_segment.json` is the selected segment extracted from the original motion result; `historical_state.json` is the unchanged historical remaining-state file. The report distinguishes their hashes from the source motion file hash.

Raw results retain the implementation's original field semantics. In the early worker, approximate solutions appear in `ompl_status` but `candidate_found` remains false because no exact candidate was returned. The final worker records approximate candidate existence while retaining `exact_solution=false`, no deliverable path and a non-success termination. No historical result was rewritten to fit the newer schema.
