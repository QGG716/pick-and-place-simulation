# Default historical motion adaptation

This fourth improvement starts from
`c5afff9a9776b2c26045d6510c5781f667e00cbd`, with matching local/remote heads and a
clean worktree. The first three fixes and lazy-IK deadline follow-up remain the
baseline. Historical paths provide hints only; no old validation, completion,
preflight or replay bundle is inherited.

## Inputs located on the server

The archived first-carton end state is
`/root/autodl-tmp/m710-motion-quality-20260915/trial03/isaac/actual_remaining_state.json`:
SHA-256 `f1d54533724cedd39991964fba73f8dd338a38126740b98e9e03157ece34e129`.
Its world session is `1789467788.5108013`, physical time `90.49583333333334`,
with one explicitly completed carton and no current attachment. The archive
does not provide `qd_rad_s`; no velocity observation is invented.

The unmodified historical candidate identified by the retained adaptation script
is `/root/autodl-tmp/m710-ideal-outfeed-20260915-retry02/repo/outputs/ideal_plan_002/motion.json`:
SHA-256 `0f09a05c29171c79b1a752560a6b476e787584f56de8b262886ac6226355fcb4`.
It contains 154 path nodes and the original contact, before trial03's normal
correction. The old scripts and corrected results were inspected as evidence,
not executed or substituted for this input. Both inputs were copied into the
isolated directory `/root/autodl-tmp/m710-default-history-adaptation-20260916/inputs`.

## Implementation

`history_candidates.py` reads only a configured nonrecursive directory (or an
explicit single JSON file). It bounds directory entries, file count/size and
screening time, rejects duplicate/nonfinite JSON and inconsistent content hashes,
and checks the known schema, stage boundaries, model/tool/layout, joint/FK
interpretation, target identity and dimensions. Current pose, policy and source
implementation differences invalidate old conclusions without discarding every
geometric hint. Unknown historical semantics fail closed.

The existing row selector chooses the current target first. At most two compatible
sources provide up to four deterministic contact variants each. The nominal
box-local contact intent is transformed onto the current carton first; optional
offsets move outward along the current physical contact frame's negative Z axis.
Defaults are a 0.5 mm step and 1.5 mm maximum. These are candidate generation
bounds, never attachment-distance or compression permissions; every new commanded
cup still passes the original full-ring checks.

History gets at most 45 seconds and half the remaining request time, including
screening (at most 2 seconds). Each variant gets at most 20 seconds, chosen against
the historical approximately 16.87-second complete recheck. Candidate and final
validation deadlines are capped by the history window. Its actual attempts also
consume the existing per-task attempt/connection allowances; ordinary queue order,
retry identities/seeds and existing caps remain unchanged. History attempts and
combined counts are reported separately. Exact repeated source/pose identities
are deduplicated; a failed historical path does not suppress ordinary search at
the same contact pose, since that search can produce a different route.

`history_adaptation.py` recomputes IK, FK, cup selection and the attachment using
the current target. The current actual start gets a checked edge to the old free
prefix; every prefix node remains checked. The terminal contact arc is rebuilt.
Loaded stage edges are checked with the new attachment and current neighbors.
Receiver endpoints are solved from that attachment and the retained receiver pose
intent, then feed the existing `_finish_place_branch()` checks for support,
occupancy, process relation, release flight, edge reserve, withdrawal and residence.
Events, stage ranges, contact/release evidence and the request-bound complete-task
record are assembled anew by the production connector.

The ordinary audit assembles current task, policy, scene and search metadata.
Both run and continuation CLIs use it; the old reuse CLI is now a compatibility
wrapper around the same request. The continuation publisher retains state/world
identity checks and atomic no-overwrite publication. Archived-input planning never
publishes a live continuation request. Ready results proceed to the original
preflight and exporter using the same motion result; serialization, preflight and
export times are recorded separately. Optional registration appends a hashed JSON
hint only after matching plan/preflight success, with physical execution explicitly
false and no changes to completed carton IDs.

## Reproducible bounded check

From the isolated server workspace, the executed one-shot check is:

```bash
export PYTHONPATH="$PWD/src"
CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
"$CPU" tools/run_m710_contact_unloading.py \
  --actual-state inputs/actual_remaining_state.json \
  --config configs/validation/m710id70_layout_v1_single_carton.yaml \
  --execution-config configs/simulation/m710id70_first_row_recording_v1.yaml \
  --history-source inputs/history --planning-wall-time-s 120 \
  --output outputs/history_default_entry
```

For regular use, `search_strategy.history.source` can be configured once; neither
`--reuse-motion` nor a selected correction value is needed. A ready result exports
`replay_bundle.json` by default. Production's existing default request budget is
unchanged.

## Verified results

The single 120-second request returned `PASS` and a new `D_COMPLETE_TASK` record.
The current target order was retained. On `carton_l07_c01`, directory screening
selected the raw candidate above; nominal adaptation failed with
`RIGID_TOOL_COLLISION`. The next generated variant, 0.5 mm outward in the current
contact frame, passed. No ordinary search was needed after that complete result;
fallback behavior is covered separately by the focused regressions.

The successful attempt took 11.829 seconds, including current collision validation.
Planning internal wall time was 12.268 seconds (CLI elapsed through serialization
12.741 seconds). Nested collision validation consumed 11.716 seconds, IK 0.0022
seconds, and final completion recheck 0.0086 seconds; these overlap the adaptation
total and must not be added to it. Independent preflight took 0.294 seconds,
export 0.321 seconds, and bundle readback 0.106 seconds. Preflight was `READY`,
with no blockers; readback was `PASS`. The exporter produced 4,818 commands for a
93.305-second planned replay. This duration is not physical execution evidence.

The new path has 155 nodes, starts exactly at the archived actual q, and retains
every old free-prefix node behind a newly checked join. Contact/extraction/transit/
place/withdrawal boundaries and the attach/release events were regenerated.
The new attachment differs by up to 0.0005000000001 m; the release endpoint was
re-solved (maximum joint change 0.000456137 rad), preserving the intended release
box pose to a maximum matrix-element difference of 1.41e-11. Both this archive's
old and new commanded masks contain 40 cups. A separate regression uses a valid
one-cup historical command and verifies freshly generated, different current masks.
Full old/new poses, masks, attachment transforms, stage ranges, deadlines and
content bindings are in the [compact evidence](evidence/m710_default_history_adaptation.json).

On the GPU server, 131 focused tests passed in 10.36 seconds, followed by five
additional context/mask/coverage regressions in 18.61 seconds: 136 distinct passing
tests. These include the previous three fixes, lazy deadline propagation, ordinary
fallback, bounded history time, rotated contact frames, current official mesh/IK
checks, rejection after only partial approach, changed rigid obstacles, missing
cup coverage, failed joins, same-world state-change protection, and replay binding.
Fault-boundary injections are identified in the tests; they do not establish a
geometric planning success. That success comes from the one unmocked CLI request.
The evidence records the exact test selections and retains earlier failed logs.

An initial compatibility test exposed CRLF versus LF in otherwise identical SRDF
files. Compatibility now compares the audited group, disabled pairs and prohibition
on nonadjacent exclusions. The original LF SHA-256 is
`5991658df1a6035019b2f9bf3e0e8eccef3c4da201d472b25783301b13a5812a`;
the current CRLF file is
`dd859deb310764c70d77c5a5bbe52d2033c959c738171344f17ac903deb29a16`,
and normalizing its line endings gives the original hash. Model, tool and collision
policy gates were not relaxed.

Artifacts remain under the isolated server workspace's
`outputs/history_default_entry/`. Input hashes still match the protected originals.
Only source, tests, configuration, this report and compact evidence enter Git.
The production sources used for the successful request were unchanged by the
subsequent five test additions; their normalized hashes are recorded in evidence.

This was offline planning from an archived measured state. No live continuation
request was published, no history registration was requested, and no completed
carton IDs were added. No Isaac, video, full suite, old population, fifth-carton
search or row execution ran. History departure checks current flight, withdrawal
and residence; its next-carton contact lookahead is explicitly unevaluated.
Physical grasp/reception/outfeed and downstream qualification remain unverified.
