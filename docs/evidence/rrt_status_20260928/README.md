# RRT outcome and single-carton validation evidence

- Review/start commit: `aa525e873c6a6c8618355923d01f05c9162e87cb`.
- Final tested source: `8d059dd0931fe6cd9690ec948438f9cd16ab5549`.
- Exact source archive SHA-256: `13ba09df0d58ccebc2f8cd960d54865c389633724a948e4cc8d676088f1f463c`.
- Existing server Python environment; no reinstall, Isaac or machine connection.

`source_8d059dd.json` hashes all 284 Git-archive files. `cpu_run_8d059dd.json`
records the exact final 21-file invocation and verifies unchanged source at exit:
323 passed, 22 unavailable-history skips, zero failures in that invocation.
The five seam failures are separate baseline/final logs and remain unresolved.
Development runs are not included in final test totals.

`baseline_reproduction.json` and `final_reproduction.json` record the same
analytic production connector with real RRT: one-iteration exhaustion and
budget-entry cancellation. `baseline_short.json` and `final_short.json` retain
all three cold/hot measurements and profiles; `short_comparison.json` summarizes
them. The current short-probe source hashes match `source_8d059dd.json`.
`progress_overhead.py` is the small evidence-only flushed-JSONL measurement,
not a new planner path or production fault-injection facility.

`single_carton_run.json` is the one normal new-planning command. It used the
archived target identity and actual scene, not the old trajectory as an answer.
The external 300-second offline envelope ended at 300.008 seconds while still
in free-approach RRT. `single_carton.json` distinguishes that external stop from
manual cancellation and geometric failure, with entry/exit source/input identity.
`single_carton_summary.json` and `single_carton_delivery/planning_progress.jsonl`
retain the last known position and actual work checkpoints. Zero completed
planning/export/preflight/readback/Isaac results are claimed. Logs contain
completed-call counters as well as in-flight RRT checkpoints; their differing
update cadence is described in the main technical note.

Server root: `/root/autodl-tmp/m710-rrt-status-20260928`.
The source snapshot and original archived input files remain there. Documentation
commits after `8d059dd` do not change the tested Python/configuration files.
