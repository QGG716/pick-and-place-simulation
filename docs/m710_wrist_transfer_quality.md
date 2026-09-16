# Bounded wrist and unloaded-transfer quality

Baseline: `c02dd35df25727f837a021d1e9e0469f744d2170`. Local and remote heads
matched and the worktree was clean. All numerical checks ran in
`/root/autodl-tmp/m710-wrist-transfer-quality-20260916` using the existing CPU venv.
Original inputs, fourth-round outputs, and earlier reports remain unchanged.

**Implementation and focused verification are complete. The single real-input
comparison retained the complete baseline: no geometric or planned-time
improvement was measured in that request.** Planning, preflight, export and
bundle readback succeeded. No Isaac execution was performed.

## Diagnosis from preserved raw paths

Joint names come from the official model and are checked against bundle names.
Metrics use raw bounded-joint values, without modulo or unwrapping. Each joint
reports `abs(end-start)`, summed absolute edge motion, and their difference.
The latter is excess over an endpoint lower bound, not necessarily removable
motion. SO(3) and TCP lengths sample each joint edge at at most 0.04 rad.

| Existing unloaded free segment | Fourth-round c01 | Earlier c03, separate state |
| --- | ---: | ---: |
| J6 net / rad | 0.735442 | 7.988410 |
| J6 total / rad | 1.058328 | 8.728903 |
| J6 extra / rad | 0.322886 | 0.740493 |
| J4+J5+J6 total / rad | 4.597314 | 13.301277 |
| Tool SO(3) travel / rad | 3.220480 | 7.214287 |
| TCP length / m | 2.137108 | 2.756225 |
| Nodes | 20 | 11 |
| Planned free-segment time / s | 15.318273 | 17.488727 |

The c03 path is the preserved `runtime-repo/outputs/same_state_003/motion.json`
from the September 15 motion-quality round. Its large J6 endpoint difference
cannot be removed by fixed-endpoint shortcutting. It is a read-only diagnostic,
not a same-input comparison with c01 and not a newly improved official task.
Full per-joint metrics, endpoints and source hashes are in the
[compact evidence](evidence/m710_wrist_transfer_quality.json).

## Production changes and bounds

`_improve_free_path()` is shared by ordinary `_transit()` and historical
`adapt_branch()`. Both first validate the original free segment. Optimization
works on copies, preserves its endpoints, and checks the replacement on both the
shortcut and caller edge grids. No shortcut crosses the free/contact boundary,
attachment, release or payload/permission transitions. Final assembly regenerates
stage ranges/events and the existing request-bound D completion record.

The common optimizer keeps the original 3-second postprocessing window and
necessary-continuation reserve. It limits proposal work to 8 shortcut attempts and
300 states, below the existing shortcut maxima, leaving work available for strict
recheck and scoring. An incomplete recheck or score retains the original path.
The unchanged soft score uses total joint motion (weight 1/rad), wrist motion
(0.5/rad), reversal count (0.1), sampled orientation (0.25/rad), velocity-only time
lower bound (0.1/s), inverse joint margin (0.02 rad), and log Jacobian condition
(0.02). There is no additional overlapping J6 penalty. Replacement also requires
no increase above `1e-6 rad` in J6, wrist-total or all-joint-total travel and a score
decrease greater than `1e-9`. These preferences never invalidate a legal baseline.

After the first complete task, the default audit calls `improve_complete_task()`
before returning. It uses at most 12 seconds, capped by the existing parent and,
for historical results, the original history window. It tries at most three seed
solves, each capped at one second, and two independent complete-task connections,
each capped at eight seconds and the remaining window. Counts consume the original
task allowance and are reported separately from the unchanged ordinary queue.

Current q and the existing contact solution seed same-pose IK first. Named J6
periodic endpoints are generated only for a scalar revolute model, inside its
finite limits, with full tool FK equivalence checked. They remain distinct raw
physical joint states; the actual start is never changed. Candidates are cheaply
ordered by maximum raw joint delta / official velocity, then total raw delta.
At most one different already configured roll is considered after same-pose work.
Each surviving endpoint gets new cup selection and strict state checks, then
ordinary planning of every stage. History's small-joint-adaptation limit is not
changed or used to excuse a large branch change. Only a complete D-level candidate
with better whole-task quality can replace the independently retained baseline.
This prevents moving the wrist penalty into the loaded suffix.

Historical withdrawal first passes its current flight-envelope and residence
checks, then enters the ordinary `_departure()` / `_next_contact_cost()` flow.
The historical baseline is previewed first and counts toward the three-departure
limit. At most two alternatives follow, only when the baseline next-contact cost
is known. Comparisons require the same next target/row, completed scores, and a
lower departure-plus-next-approach cost. Unknown remains `None`; it cannot win.
The original 4-second per-call, 12-second cumulative request cap, contact context
restoration and parent deadline remain. Next-task execution still needs fresh
actual-state planning. `wrist_transfer.py` is included in implementation identity.

## The one default-entry comparison

From the isolated GPU workspace:

```bash
export PYTHONPATH="$PWD/src"
CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
"$CPU" tools/run_m710_contact_unloading.py \
  --actual-state inputs/actual_remaining_state.json \
  --config configs/validation/m710id70_layout_v1_single_carton.yaml \
  --execution-config configs/simulation/m710id70_first_row_recording_v1.yaml \
  --history-source inputs/history --planning-wall-time-s 120 \
  --output outputs/wrist_quality_default
```

The state and raw historical motion hashes match the fourth-round inputs exactly.
The same current c01 target is selected. Nominal contact is rejected for rigid
collision, and the automatically generated 0.5 mm normal variant completes.

- Free optimization actually ran: 8 proposals, 251 states, 5 intermediate
  shortcuts accepted. The caller-grid recheck then reached its 3-second deadline;
  no candidate score or shortened path was published. The original 20-node free
  prefix was retained. Optional duration: 3.003174 seconds.
- Both same-pose seeds converged to the existing raw solution. The equivalent
  J6+2pi endpoint had a worse endpoint heuristic. One configured roll 270 candidate
  passed contact IK/geometry and entered ordinary planning. Its free connection
  succeeded, but extraction reached the candidate deadline before the loaded
  suffix completed. It was rejected. Endpoint comparison: 8.608523 seconds.
- Historical withdrawal previewed c03 through the real contact checker (40 cups),
  then failed to finish its approach before the parent deadline: 2.456170 seconds,
  `STAGE_IK_DEADLINE/pregrasp`, cost UNKNOWN. Alternatives were not launched.

The exported raw 155-node path, all endpoints, stage ranges and events are exactly
equal to the fourth-round path. Its current proof and preflight are newly computed.
Free-segment metrics above are therefore unchanged, including reversals
`[3,3,1,1,3,1]`. Full-task J6 total is 1.807062 rad; wrist total 7.984197 rad;
all-joint total 12.693684 rad; SO(3) 4.791334 rad; TCP length 4.104174 m;
reversals `[5,5,4,3,5,1]`. Planned full duration remains 93.304895 seconds under the
same timing model and process holds. None of these times is an observed cycle time.

Planning took 26.037831 seconds versus the preserved baseline's 12.268464 seconds.
The optional comparisons added work without producing an adopted improvement.
Independent serialization/preflight/export/readback took 0.009138 / 0.280505 /
0.292276 / 0.103217 seconds. Preflight was READY, export produced 4,818 commands,
and readback passed. Full artifacts remain in `outputs/wrist_quality_default/`.

## Verification and limits

The focused selection in evidence passed **154 tests in 47.92 seconds**, including
18 new cases and the prior 136 regressions. Tests cover raw-angle/whole-turn
metrics, named joints, both production optimizer callers, guarded replacements,
blocked edges, deadlines and immutable fallback, real IK and connections in a
controlled periodic-endpoint task, full-task failure rejection, official FK/limits,
real near-contact historical lookahead, baseline-first departure comparison,
unknown cost, previous mask/context isolation and replay binding. Synthetic task
geometry and injected fault/cost boundaries are explicit; they do not establish
an official robot task improvement. Earlier failed test logs remain on the server.

No configuration, robot/tool geometry, joint limits, collision/contact permissions,
motion-limit scale, interpolation, process holds, drives or recording settings
changed. No Isaac, full suite, population sweep, fifth-carton recovery or row run
was attempted. No physical completion count or live continuation request changed.
The high-J6 official case's alternative complete task remains unverified, and this
one bounded request does not demonstrate that large wrist rotations are resolved.
