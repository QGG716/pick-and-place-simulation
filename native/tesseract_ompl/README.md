# Native worker protocol and v0.2 audit

Build with the existing optional native prefix using `tools/build_tesseract_ompl.sh`.
The core Python interpreter does not import native bindings. The default application
planning path remains legacy; there is no fallback from a failed native request.

The JSON-lines protocol retains handshake `{"ready":true,"protocol":1}`. A normal
request includes the exported immutable `scene`, `q_start`, `q_goal`, `seed`,
`max_state_checks`, optional `wall_time_s`, and the adapter-owned cancellation file.
`operation` defaults to `plan`. Each call starts a fresh tree and exact-state cache;
only immutable parsed geometry can persist between calls.

For an audit, add `"operation":"audit"` and either `"audit_path":[q0,q1,...]`
or `"states":[q0,q1,...]` to the same envelope. No OMPL solve is invoked:

- `audit_path` invokes the exact `DenseMotion` used by planning; success requires
  every mandatory integer-grid sample of every edge.
- `states` invokes the same `Context::valid` used by planning. Verdicts are boolean;
  an interrupted sample is null. Budget/cancellation are not collision verdicts.
- Only `AUDIT_VALID` with `audit_complete=true` certifies the requested check set.
  Audit acceptance is not an exact planning solution or a deliverable task.
- `trace_samples=true` exports actual edge/index/subdivision/q samples for a bounded
  audit. It is off during planning. Aggregate profiles do not print state logs.

The only supported point-motion contract is 4 m / 1.25 mm. Its L1 resolution is
`.0003125 / 2**refinement`, with integer refinement 0..7. The optional legacy
`l1_resolution_rad` field must equal that derived value. The angular rule supports
finite `edge_resolution_rad` in `(0,.055]`; changing it changes the executed rule.
Unsupported inputs are rejected rather than silently ignored.

`state_requests` counts every validity request; `state_checks` and
`actual_state_computations` count cache misses that execute validation. The bounded
8192-entry FIFO cache uses exact double bytes, never quantized neighbors. It clears
on every request, including scene, attachment, stage, policy or refinement changes.
An additional `32 * max_state_checks` request guard prevents unbounded cached work.
`valid_edges`, `invalid_edges`, and `incomplete_edges` are disjoint. OMPL iterations
remain null; `search_progress` contains actual PlannerData counts and the library's
approximate solution distance when available.

Set `profile=true` to aggregate FK/scene state, Jacobian/SVD, collision transforms,
FCL contactTest, budget/cancel polling and exclusive edge scheduling. Inclusive
state and edge clocks contain child clocks and must not be added to them. Total
native solve time excludes input JSON parsing, output serialization and transport;
the worker roundtrip clock includes those costs. Optional trace allocation is part
of audit edge scheduling. Instrumented and uninstrumented timings are distinct.

Reproduce the bounded audit with `tools/run_tesseract_ompl_audit.py`. The final
engineering runner `tools/run_tesseract_ompl_v2_request.py` requires matching worker
and successful audit evidence, fixes endpoints 78→208, seed 71070, range .18, one
attempt and 100000 actual computations, and refuses to overwrite a started request.
See `docs/backend_tesseract_ompl_v0_2.md` for measured results and evidence identities.
