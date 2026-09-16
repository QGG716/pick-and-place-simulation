# Lazy IK deadline follow-up

This is a narrow follow-up to `934da38ae62b60714eb9af1c014437f3f5633878`.
The original [third-fix report](m710_feasible_result_budget_fix.md) and its
historical results remain unchanged.

## Cause and change

`_connect_pose()` constructs its lazy stream before entering the optional
comparison scope. Previously `_ik_stream()` copied the numeric parent deadline
into that stream's kwargs. A later `_budget_scope()` shortened the connector's
deadline but did not change the deadline received by `solve_ik()`.

`_ik_stream()` now supplies an explicit live deadline provider. The stream caps
that value by its immutable original parent deadline; the provider also includes
the request hard deadline. Before every seed the stream checks this effective
deadline and passes it, plus the live provider, into `solve_ik()`. Fixed numeric
deadline callers remain supported. `None` means no cap at that layer; zero is an
expired cap. Restoring the connector scope restores the visible parent window,
without resetting the stream, replaying seeds, clearing deduplication state or
resetting the absolute clock.

`solve_ik()` reads the effective deadline at numerical checkpoints, including
before and after FK, Jacobian, linear algebra and endpoint validation. A single
native call cannot be preempted; an overrun is rejected immediately on return and
records the actual observation time. Both ordinary convergence and final-update
convergence retain this check.

The stream counts only started seeds and distinguishes deadline termination from
seed exhaustion. `_connect_pose()` reports a started optional comparison that
timed out, retains the first strictly verified connection, and returns to the
existing necessary contact phase. A successful better comparison remains eligible.

## Verification

All checks ran in the isolated GPU-server directory
`/root/autodl-tmp/m710-lazy-ik-deadline-fix-20260916`, using the existing CPU venv.
The [compact machine evidence](evidence/m710_lazy_ik_deadline_fix.json) records
the exact commands, observed calls, termination, environment and source hashes.

| Controlled-clock observation | Before | After |
| --- | --- | --- |
| First strict free connection completes | 100.2 | 100.2 |
| Outer optional deadline | 103 | 103 |
| Original stream/parent deadline | 106 | 106 |
| Deadlines actually passed to pregrasp IK calls | 106, 106, 106, 106 | 106, 103 |
| Second IK stops | 106 | 103 |
| Started seeds / available seeds | 4 / 4 | 2 / 4 |
| Stream termination | SEED_STREAM_EXHAUSTED | PLANNING_WALL_CLOCK_DEADLINE |
| First verified connection retained | Yes | Yes |
| Necessary contact validator calls | 0 | 23 |
| Complete approach | No: parent deadline exhausted | Yes: last contact check at about 103.2 |

These numbers are injected test-clock values, not simulation time or performance
measurements. The reproduction uses real `_approach()`, `_connect_pose()`,
`_ik_stream()`, `IKCandidateStream` and production edge checks, with a synthetic
translation robot, injected state validator and timed numerical solver boundary.
It neither replaces the stream nor claims physical task completion.

Before patching, the original skip-second-IK regression and official near-contact
fixture passed (2 passed). The new reproduction plus six late-call cases failed
on the unchanged baseline (7 failed). After patching, the focused selection passed
**44 tests in 1.94 s**, with no skips. It includes:

- Real `solve_ik()` arithmetic with late FK, Jacobian, linear solve and ordinary /
  final-update endpoint returns; each deadline of 101 is overrun to 101.25 and
  rejected, with the actual stop time recorded. Another real iteration observes
  a deadline tightened inside its Jacobian call.
- Multiple seeds within one `next()`, duplicate/rejected solutions, timeout
  counting, resumption without replay, and genuine seed exhaustion.
- Normal, StopIteration, failure and exception scope exits; nested caps,
  zero/expired/None and fixed deadlines; original lookahead caps.
- First-connection retention, actual terminal checks, contact failure rejection,
  context restoration, and selection of a completed better second candidate.
- Existing official near-contact regressions, using real official FK/IK, Coal
  collision checks, full seal rings and free/contact edges. The retained fixture
  still reports `C_COMPLETE_APPROACH`, 40 active cups, no endpoint failure and
  `execution_ready=false`.

The existing implementation fingerprint already includes both changed production
modules; its fresh hashes and the LF-normalized tested source hashes are saved in
the evidence. No fingerprint file list or historical evidence was rewritten.

These checks do not establish complete physical task execution or execution
preflight readiness. No budget, collision policy, model, layout, mass, contact
permission or candidate ordering is changed.
