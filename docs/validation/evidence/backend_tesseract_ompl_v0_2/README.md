# v0.2 evidence index

This directory records one frozen FANUC empty 78→208 request, seed 71070,
range .18 rad, and at most 100000 actual state computations. No Isaac, full task,
legacy rerun, seed search or parameter sweep was performed. The final engineering
outcome is BUDGET_EXHAUSTED with an approximate solution only; no path delivered.

## Read first

- `baseline.json`: clean local/remote Git baseline and historical scene-identity projection.
- `baseline-audit/audit.json`: current authority full-path check, original native full-grid audit,
  and 31639 same-actual-q comparisons, all accepted with zero disagreements.
- `final-audit-validated/audit.json`: final worker accepts the identical required grid;
  exact q bytes and unchanged authority sources bind the earlier authority evidence.
- `engineering-request/request.json`: exclusive-start marker, full endpoint/config/source identity.
- `engineering-request/native-result.json`: the one real final native search result.
- `engineering-request/result.json`, `engineering-summary.json`: fail-closed adapter result;
  no exact candidate, so expensive authority candidate validation was not invoked.
- `profile-overhead.json`: 100 fixed distinct accepted states, identical warmup and two off/on pairs.
- `fast-rejection-witness.json`: endpoints plus midpoint reject in three samples;
  native and authority reject the same tool/carton pair. Native signed penetration
  and authority nonnegative surface distance use different conventions.
- `test-summary.json`: 79 distinct directed cases passed across recorded batches.

`native-samples.json.gz` stores actual q values, edge/index/subdivision identities;
this opt-in audit trace is separate from aggregate profiling and is not enabled
in the engineering search. Each audit has its full exported `scene.json.gz`.

## Build provenance

- Earlier v0.1 comparison: worker `8cb22acfb19898d8fbdd271cc1aa5e5270993b3ba5f405cd307ba120f1dd7c9e`.
  Its historical failed search was not rerun or overwritten.
- This round's pre-optimization audit: worker
  `06ca6576066cf36222154c007b147e726bc19af7257ff9ef7aab796fdb141d81`;
  `worker-audit-source.cpp` and `baseline-audit-driver.py` preserve exact experimental sources.
  Other authority modules match the Git review baseline and their hashes appear in audit.json.
- Rejected development build: `before-contact-fix-build.json`,
  `before-contact-release-worker.cpp`, `before-contact-release-*tests*`.
  Two native regressions found access to a retained empty contact-pair vector.
  The fix finds a nonempty result entry and releases the container on scene load.
- Intermediate optimized worker `b8f2577e…`: `final-audit/` and
  `before-refinement-*`. These are explicitly intermediate, not final-binary results.
- Final worker: `8196a41723e4baa94aec3bd164cf8665e4f2857a819c087b3e9b1e61139c20fe`;
  worker.cpp `03fc4729ae4b37cd9b233a210f9dac3fc6e34a08abca291dbd9491e76f4ca737`.
  See `final-build.sha256`, `build-final.log`, `environment.json`, and
  `final-audit-validated/`. The final input-guard change validates the integer range
  before deriving the dyadic rule. No engineering request ran on earlier v0.2 builds.

`initial-update-source.tar.gz` is the exact first uploaded patch payload.
`validated-source.tar.gz` preserves the source bytes used for final validation and
engineering execution. `source-config-sha256.json` binds sources and frozen inputs.
The engineering driver subsequently had only its BOM/final CRLF normalized for
Git; both identities and the executed archive are recorded in
`runner-normalization.json`. No executed C++ or authority algorithm changed.
The extended same-q midpoint regression source is `final-test-source.py`.

All raw failures remain. The early `directed-tests.log` failure is superseded only
for the final outcome by the focused native reruns, not deleted or rewritten.
The initially skipped evidence test was run after the full audit completed and
passed in `known-path-evidence-test.log`. `fast-rejection-test.log` strengthens an
existing test and is not counted twice.

## Clock and budget interpretation

State and edge inclusive clocks contain their child clocks. Budget polling is
nested in state checks and also called by OMPL. Edge scheduling excludes nested
state time. Native total excludes input parsing, response serialization and IPC;
worker roundtrip includes them. Audit trace allocation is included in edge scheduling.
The matched profile pairs are noisy overhead observations, not calibrated clock
costs or an uninstrumented engineering benchmark. Earlier audit-attached profile
pairs had unmatched cache-cleanup/warmup state and are not used for overhead claims.

Old state_checks counted every executed validation (no native cache). New
state_requests includes hits and interrupted requests; state_checks/actual_state_computations
counts uncached validation. The final request had 100223 requests, 100000 computations,
222 hits and one interrupted request. This is not an equal-work search comparison.
OMPL iterations remain null. PlannerData reports 89 tree vertices / 87 tree edges;
the approximate goal residual is 1.0383452429 rad in joint-space L2 distance.
Budget exhaustion is not collision or proof of physical unreachability.


Large scene and native-test JSON outputs are losslessly stored as `.json.gz` to
keep review diffs manageable. `compressed-evidence.json` records their original
byte lengths and SHA256; decompression was checked byte-for-byte. Their raw JSON
files remain on the server. Expand with `gzip -dk FILE.json.gz` before using a tool
that expects the original `.json` filename. Log files and result summaries remain
plain text. Whitespace checks are disabled only for this raw evidence directory,
so original log formatting and source snapshots are not rewritten.
