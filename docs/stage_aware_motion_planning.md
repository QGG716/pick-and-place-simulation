# Stage-aware motion generation

Review/start baseline: `6ec356a9c917e0dd31d3103123dbdd9ec7782a51`.
Production implementation: `7ba5b37`, `b4c3357`, `c0b9c28`.
Final source/test/harness validation commit: `c0b9c28` (loaded probe in `864a538`).
This round changes generation order, not geometric acceptance or the validation
kernel. `MotionValidator`, input-bound `ContextLease`, prefix stopping, batch FK,
directed edge caches, failure feedback and retained complete results remain.

## Production wiring

Previously `_approach` already separated a free connection from a terminal
Cartesian contact arc. Direct-mode export merged both into `contact`.
`_finish_place_branch` tried `_local_cartesian_transit` before joint connection;
historical reconstruction also preferred its old prefix and Cartesian suffix.
The existing RRT already tested direct edges and could resume after rejection.

Now `_connect_pose` checks the initial state, lazily obtains the existing finite
IK candidates and gives each candidate a cheap direct connection opportunity.
`_transit` calls `free_connection_prefix` with the actual input-bound validator.
A VALID direct edge returns immediately in POC mode. Rejected direct candidates
are deferred until the other bounded IK endpoints get their direct opportunity.
The fallback pass tries available hints/local candidates, then `_rrt_transit`,
which retains the existing `RRTConnectPlanner` and request budget. Its duplicate
direct query hits the same directed edge cache. No second RRT was implemented.
Once any checked connection exists, optional comparison cannot enter the
deferred candidate/RRT pass, including legacy non-POC configurations. A short
measurement exposed this remaining optional-search branch and it was closed
with a production-entry regression. Cheap alternate direct comparison remains.

| Motion purpose | Actual entry | Allowed generation |
| --- | --- | --- |
| FREE_APPROACH | `_approach`, historical free prefix, next-contact preview after safe departure | Joint direct, current-validated hints/templates, bounded candidates, RRT |
| CONTACT_PROCESS | `_approach` terminal arc; historical rebuilt terminal | Existing Cartesian continuation |
| SUPPORT_RELEASE_PROCESS | `_support_release` | Existing support-release process |
| EXTRACTION_PROCESS | `_extraction_options` | Existing straight, outward/turn, lift/turn candidates |
| FREE_LOADED_TRANSFER | `_finish_place_branch` to preplace | Joint direct, hints/templates, bounded local Cartesian candidate, RRT |
| PLACEMENT_PROCESS | `_finish_place_branch` preplace to release | Existing receiving candidate and Cartesian process |
| DEPARTURE_PROCESS | `_departure` / `_departure_search` | Existing swept-occupancy candidates and legal wait |

Every Cartesian/free connection call specifies its purpose. Unknown purposes
raise; controlled purposes cannot enter the free dispatcher. Free connection
rejects local target/support permissions and attachment/purpose disagreement.
Before loaded dispatch, the production finish entry requires both the branch
tracker's `fully_released` and the actual attached box's existing extraction
reserve. A renamed stage or a requested extraction distance is insufficient.

Uncertain state/edge validation, cancellation, expired context and exhausted
validation budgets do not become geometric rejection followed by RRT. Existing
RRT now preserves incomplete validation as a latched interruption and returns
its original evidence. Controlled Cartesian generation checks interruption
before the next IK sample; extraction stops without advancing another tracker.
An unknown IK endpoint retains its reason instead of being reported as ordinary
IK unreachability. Invalid numerical/geometry endpoints never trigger RRT alone.

## Templates and request bindings

`VerifiedTemplate` is a request-local immutable tuple of path nodes plus a strong
reference to the validator that validated it. Only the same current validator
can label the receipt VERIFIED_TEMPLATE. A different binding, including equal
new scene objects, demotes it to HISTORY_HINT. Bare `transit_hint` nodes and
archived paths are always HISTORY_HINT. Both endpoint bridges and the candidate
path are validated under the current contract; the label alone grants nothing.

`LOCAL_CARTESIAN_CANDIDATE` is newly generated work, not a historical template.
The loaded fallback's local allocation comes out of the existing sample pool,
with a bounded share per endpoint and actual consumption deducted. No new
unbounded candidate family or search budget was added. Absent candidates are
recorded UNAVAILABLE. The obsolete, unused Cartesian-first historical loaded
suffix helper was removed; adaptation reaches the normal finish dispatcher.

No callbacks/leases are rebound in place. Semantic context IDs remain independent
of Python object IDs. Context guards remain lightweight; the previous binding,
preparation-change, mid-batch mutation and unobserved-suffix tests remain included.

## Process and result contracts

Extraction clones trackers for each existing route, observes them in order and
checks actual free clearance and reserve before transition. No route, geometry,
mask, permission, margin, joint limit or sample resolution was relaxed.
Conveyor placement generation, `placement_working_normal`, longitudinal priority
in overlaps, transverse top/side choices and longitudinal top/right-facing
choices remain owned by the existing receiving policy. Actual FK, attachment,
support union, release prediction and departure sweep still decide acceptance.
Attachment and occupancy are retained through the existing release boundary.
Ideal cups, ideal reception and ideal outfeed remain explicit POC assumptions.

Exported stage names/ranges and ATTACH, RELEASE, RELEASE_RETREAT_COMPLETE events
are unchanged. `motion_subsegments` adds seven contiguous internal-purpose
intervals. Direct-mode contact export records a separate free prefix whose
validation stage is `pregrasp`. The stage-contract checker checks these intervals
against original process ranges and attachment/release indices. Historical
records lacking this new field remain compatible only through existing explicit
adaptation boundaries; no missing free/contact boundary is invented.

Evidence reuses context IDs and counters: chosen method, attempts/reasons,
endpoint IK seeds, along-path IK/Cartesian samples, actual state work/cache reuse,
RRT interface calls and actual extension/iteration counts. `rrt_constructed`
describes the generation search object; optional simplification may independently
construct its existing helper planner and is reported separately. Per-state
evidence hashes or scene serialization were not added. Controlled-process
evidence can be nested under selected route/parts; the summary preserves that
structure instead of claiming an unmeasured aggregate.

Segment checks are discrete with existing conservative pair proofs, not a full
continuous guarantee. Complete assembly checks, time parameterization and
independent execution preflight remain separate. Optional quality/preview cannot
replace an already checked result with an incomplete candidate. The new policy
module is included in both motion and execution source identity lists.

## Validation and limitations

Local CPU execution was explicitly authorized after the previous SSH endpoint
refused connections. The existing `.conda-cpu` environment was reused: Python
3.14.7, NumPy 2.5.2, Pinocchio 4.1.0, Coal 3.0.4. No environment was installed or
replaced. See [recorded evidence](evidence/stage_motion_20260926/README.md) for the
final source manifest, combined test result, measurement data and single-carton
outcome.

The final 21-file CPU invocation passed **306 tests, with 22 archived-fixture
skips, in 129.54 s**. Earlier overlapping invocations are not added. A new
single-carton development probe was attempted with one recorded grasp seed and
current actual-state binding, without historical path input. It remained
unfinished after about 12 minutes and was manually stopped to contain this
round's offline scope. There was no automatic retry, no production budget
change, no complete export and no preflight result. Because that development
probe started before the final commits, it is not final-source full-cycle
evidence; the final-source evidence here is CPU regression and short connections.

The new tests exercise production dispatch with real IK/geometry and real RRT,
official-model merged contact, all three extraction routes and independent
trackers, template binding/bridge validation, invalid and unknown endpoints,
cancellation/budget/context failure, controlled-process failure, loaded-boundary
guards and event ranges. Scheduling-only tests explicitly identify their IK or
connection doubles and do not claim physical planning success.

The five existing `test_conveyor_seam_contact.py` failures reproduce on baseline
and current source: three old support/pair assertions and two incomplete tool
provider fixtures. They are recorded separately; collision/support rules were
not changed to accommodate them. Archive-dependent older history/wrist fixtures
are not counted as passing when their required input fixture is unavailable.

Performance probes use the same input endpoints, seeds, geometry and rules for
both versions. The loaded short probe reconstructs the old local-first versus
new direct-first scheduling at the connection boundary; it is not a full task.
The blocked case uses an analytic Cartesian robot and actual RRT, not the
official FANUC model. Official contact and free cases use production entries.
Cold means a new planning request with a loaded model; warm repeats the same
binding. Three unprofiled repetitions are separate from profiling. No result
from rechecking historical endpoints is claimed as new full-cycle planning.

Remaining costs include endpoint IK/collision checks, scalar collision/policy
work, optional non-POC quality windows and process Cartesian continuation. The
new dispatch can add overhead on very short paths that already had cheap local
solutions. No throughput multiplier, full-row stability, machine qualification
or physical reception success follows from this work. Isaac was not run.

## Same-input short measurements

Final serial measurements, three unprofiled repetitions; times are medians in
milliseconds. Baseline and current use the same harness, environment and inputs.

| Case | Baseline cold | Current cold | Baseline warm | Current warm |
| --- | ---: | ---: | ---: | ---: |
| Official empty free connection | 3116.27 | 3046.52 | 3073.80 | 2990.24 |
| Official loaded short connection | 3289.49 | 3394.87 | 5.828 | 19.473 |
| Analytic loaded blocked connection / real RRT | 58.496 | 72.242 | 7.164 | 10.627 |
| Official approach including contact process | 3327.31 | 1181.39 | 3201.38 | 828.920 |

The old explicit empty/contact cases retain their bounded optional quality
windows. Actual work therefore varies with cache warmth and window expiry,
even with identical random seeds; the raw three observations are retained.
First retained free-connection medians are 75.96 -> 25.17 ms for the empty case
and 130.97 -> 95.64 ms for the free prefix of contact. Neither number includes
completion of the subsequent contact process or the full task.

| Case | Cold expensive states: baseline -> current | RRT expansion attempts: baseline -> current | Cartesian samples: baseline -> current |
| --- | --- | --- | --- |
| Empty | [132,306,473] -> [133,310,397] | [0,14,36] -> [0,0,0] | 0 -> 0 |
| Loaded short | [15,15,15] -> [16,16,16] | 0 -> 0 | 1 -> 0 |
| Loaded blocked | [50,50,50] -> [50,50,50] | [6,6,6] -> [6,6,6] | 0 -> 0 |
| Contact entry | [68,156,219] -> [27,27,27] | [17,45,63] -> [0,0,0] | 3 -> 3 |

Loaded short work changes from one along-path Cartesian IK sample to one
endpoint IK seed, with one additional distinct endpoint state check. Both sides
perform zero expensive checks in the warm repeat. Cold latency regresses 3.20%;
warm latency adds 13.65 ms (234%). The blocked case retains one RRT call, three
iterations, six extension attempts and 50 expensive states; latency regresses
23.50% cold and 48.34% warm. Extra dispatch/endpoint/guard work and short-run
timing variation are included; there is no measured general speedup claim.

Empty endpoint seeds are [4,4,4] -> [4,5,5]. Contact endpoint seeds are 4 -> 8,
plus the unchanged three along-path IK samples (total seeds 7 -> 11). Eliminating
optional RRT after a retained direct reduces contact total latency 64.49% cold,
74.11% warm, without changing its terminal Cartesian samples. The blocked
case already used appropriate RRT before this change, so its work does not fall.

Both implementations' isolated 100 same-binding prepared lookups produce 100
hits and 200 lightweight guard calls, with **0 full context builds, 0 JSON
calls and 0 SHA calls**. Profiled total guard time is .632 -> .571 seconds;
this includes profiler overhead, not an unprofiled per-edge latency claim.
Warm loaded and blocked paths still reuse their checked states/edges. Existing
early-stop and stale-binding regressions pass on the final source.

## September 28 follow-up: RRT outcomes and observable single-carton delivery

Review/start baseline: `aa525e873c6a6c8618355923d01f05c9162e87cb`.
The baseline was the clean local and fetched remote HEAD. Code and regression
snapshot: `8d059dd0931fe6cd9690ec948438f9cd16ab5549`. Validation runs use a separate
Git archive on the existing GPU server CPU environment; source files are not
edited during the runs. This patch leaves generation policy, geometry,
permissions, sampling and validation-kernel behavior unchanged.

### Structured outcomes across the real call chain

The baseline reproduction uses the production analytic-carriage connector,
real MotionValidator and real RRT. With valid endpoints and a blocked direct
edge, one actual extension exhausting a one-iteration quota reported
`MAXIMUM_ITERATIONS_REACHED` in the planner but `INVALID` in `_transit`.
Cancellation injected at the real extension's budget-check entry likewise
became `INVALID / PATH_SEARCH_EXHAUSTED`, without a validation-failure object.

`planner._result` now publishes `validation.status`, `reason`,
`termination_scope` and `can_continue_candidates`, independently of an optional
state-validation failure. Budget-entry stops latch their own outcome. Publication
also checks the existing lease; incomplete results cannot become successful
paths or geometric blacklist entries. Completed validation may use the last work
unit; cancellation and deadline checks still bind.

`_rrt_transit`, `_transit`, `_connect_pose`, the grasp-branch loop and the ordinary
single-carton audit preserve these outcomes. The direct-edge rejection remains
separate evidence. A candidate's iteration quota is INDETERMINATE/CANDIDATE and
leaves other candidates their original remaining allocation. Cancellation is
CANCELLED; input invalidation, request work exhaustion and deadlines stop further
use of the request. No budget is restarted. An exhausted finite candidate set
is not a global unreachable proof. Legacy local producers keep their reasons;
missing status is not inferred to be geometric INVALID. Known local sample
quota failures can still advance to another bounded candidate. Explicit
geometric rejections remain INVALID in their checked scope.

### Observable normal entry

The validation tool now invokes `run_m710_contact_unloading.main` once, with
explicit POC motion/execution policies and the archived actual state. The old
motion supplies only the target name: no old path, no history adaptation and no
archived contact seed is supplied. A successful new result follows the existing
motion serialization, independent preflight, replay exporter and bundle readback.
The optional source guard prevents publication/export if the captured source or
input identity changes.

Entry identity is written atomically before scene construction. Connector stage
entry/exit events precede/follow expensive calls; work and real RRT checkpoints
are limited to one per five seconds per producer. They reuse counters and active
entry labels, with no scene JSON or hash per state. JSONL records flush at each
write. Normal completion, manual cancellation and an explicitly external offline
resource stop have separate summary states, with source/input checks at exit.
A native call between observation points can only be localized to its last
observed stage; no collision cause is inferred from elapsed time.

### Validation records

Final measurements, single-carton outcome and remaining limits are recorded below.

The final 21-file CPU invocation on `8d059dd` reports **323 passed, 0 failed,
22 skipped in 24.23 s** (24.48 s subprocess wall time). No overlapping development
runs are added. The skips still require the specified older private history
fixtures; the newer available POC archive is not substituted for them. New real
RRT interruption tests and production dispatch regressions do not need these
files. The separate seam invocation remains **5 failed** on both `aa525e8` and
`8d059dd`; those five acceptance/fixture expectations are not changed here.

New regressions cover actual RRT extension-entry cancellation with no separate
validation-failure object, work/deadline/context stops after RRT starts,
publication-time context changes, candidate quotas, downstream candidate
opportunity, legacy producer status formats, geometric INVALID, actual detour
VALID, final connector/audit propagation, progress durability and stale-source
export prevention. The ordinary bounded-retry regression also caught and fixed
a development error that had attached a request-stop meaning to an old local
candidate failure. Its final result retains all 195 existing finite dispatch
opportunities; cancellation, stale input and request work exhaustion stop after
the first observed interrupted candidate. The runner's six pre-existing test
failures were separately reproduced on the baseline: their incomplete mock
policy now uses the real POC configuration, and separate outputs respect the
existing immutable-output rule. These are wiring tests, not physical evidence.

Same-input server short probes, three unprofiled repetitions (median ms):

| Case | Baseline cold | Final cold | Baseline hot | Final hot |
| --- | ---: | ---: | ---: | ---: |
| Official empty direct | 136.838 | 137.802 | 78.344 | 78.740 |
| Official loaded short endpoints | 451.640 | 468.648 | 5.068 | 5.267 |
| Analytic loaded blocked / real RRT | 12.083 | 12.448 | 2.363 | 2.442 |
| Official contact process | 181.625 | 243.613 | 131.685 | 140.124 |

Recorded work is unchanged in these probes: cold expensive state counts are
397 / 16 / 50 / 27 respectively, and hot counts are zero. Direct connections and
the contact probe perform zero RRT expansions. The blocked case still performs
one real RRT call, three iterations and six extensions; contact retains three
Cartesian samples. The loaded probe generates a new connection between archived
short endpoints; it is not a new full pick/place cycle. Latency regresses in all
four measurements, especially contact cold (+34.1%, hot +6.4%). Counts do not
identify the cause of that timing variation; no general speedup is claimed.
Nested legacy timers are not additive wall time.

Both 100-lookup hot-binding probes retain 100 prepared hits, 200 lightweight
guards, **zero context rebuilds, zero JSON calls and zero SHA calls**. Existing
prefix regressions still check exactly 1 / 5 / 10 / 18 states for failures at
samples 0 / 4 / 9 / 17 (65 for an all-valid edge), with no suffix cache entries.
Five alternating batches of 20 analytic hot direct calls measure real flushed
JSONL telemetry: 0.161 ms/call without telemetry versus 0.197 ms/call with it
(+0.036 ms, 200 boundary records, no RRT). This is a small explicit logging cost,
not a claim of zero overhead or a full-task performance profile.

### The single new planning attempt

Exactly one ordinary new planning attempt ran from the fixed `8d059dd` archive:
`/root/autodl-tmp/m710-rrt-status-20260928/source-8d059dd`.
The unchanged historical actual scene and target `carton_l07_c04` were used;
no obstacles were deleted, no margins changed and no full historical path was
supplied. Motion/execution policies were explicitly paired as
`configs/validation/m710id70_proof_of_concept.yaml` and
`configs/simulation/m710id70_proof_of_concept.yaml`.

The explicitly external 300 s experiment alarm stopped the run at **300.008 s**:
`OFFLINE_RESOURCE_LIMIT / INDETERMINATE`, not geometric infeasibility and not a
planner iteration-budget verdict. Source and input hashes match at exit; all
284 files from the source archive also match after the run. There was no retry,
seed change or reset. No complete planning result, motion export, independent
preflight, bundle readback or Isaac execution was obtained. The complete chain
therefore remains **unaccepted** in this round.

The last observed active chain was `_plan_branch_search -> _approach ->
_connect_pose -> _transit`, purpose `FREE_APPROACH`, stage `pregrasp`, strategy
`RRT_CONNECT`. At 298.525 s the RRT checkpoint recorded 96 extension attempts,
96 edge calls and 36,727 edge samples. The 296.929 s connector checkpoint
recorded 40,285 state-validation calls. These are separate counters and must not
be added. Some connector totals (IK stream calls, completed RRT iterations) are
updated only when the enclosing call returns; their unfinished zero values do
not mean no IK or RRT work occurred. The RRT-specific checkpoints are the
available in-flight evidence. No contact, extraction, placement or release
cycle completed.

The completed direct checks include a rigid-tool/carton clearance rejection:
`tool_rigid_0 / carton_l05_c02`, measured gap 0.004984748077 m versus the unchanged
0.005 m requirement. It is proxy-OBB clearance evidence, not a claim of
original-CAD penetration, and does not prove that all routes fail. A rejected
edge recorded 744 additional state samples and 41,574 unobserved suffix states;
its suffix was not continued after failure. The later external stop provides
no additional collision diagnosis.

Evidence is under [evidence/rrt_status_20260928](evidence/rrt_status_20260928/):
`source_8d059dd.json`, `cpu_run_8d059dd.json`, `cpu_8d059dd.txt`, the two
`*_reproduction.json` files, same-input short measurements and separate seam
logs. `single_carton_run.json` records the exact command; `single_carton.json`
and `single_carton_summary.json` bind inputs/source and termination.
`single_carton_delivery/planning_progress.jsonl` contains 108 flushed records
(123,058 bytes), with 5-second work-checkpoint cadence and explicit phase
boundaries. Full server evidence remains at
`/root/autodl-tmp/m710-rrt-status-20260928/evidence`.

Remaining limits: this particular free-approach RRT did not finish within the
offline resource envelope; scalar narrow-phase/clearance and dense edge checks
remain substantial work. Their optimization is outside this patch. Progress
localizes native calls to the last observation, not an unobserved internal cause.
The five seam failures and 22 unavailable-history skips remain separate from
passing regressions. No full-row stability, continuous-motion safety proof,
physical reception success or machine qualification is claimed.

## September 28: original-scene free approach

This follow-up starts at `d74b53fa0901720b79426970ac351615b7b3da39`;
the worktree and fetched branch were clean and equal. The implementation/test
archive is `0bfd708489219b8635b836ab307c186faa7f03e6` (267 archived files,
SHA256 `705a5d92a2a3fe7a576d82b2ff44a441a4d585e9c7e1a57aea6f8c6ac933a56e`).
All measurements use the existing server CPU environment, with no dependency,
collision, sampling, joint-limit, contact-policy or receiver-policy changes.

### Frozen input and diagnosis

`tools/probe_m710_free_approach.py --mode freeze` stops the ordinary new-plan
entry at its first `_connect_pose`. It reconstructs the original candidate;
it does not supply a historical path. The frozen JSON contains the complete
actual start, four grasp candidates, target/contact transforms, first gate,
IK seeds, solver budgets, official joint limits, all obstacle poses, 72-bit
mask, named stack membership and full semantic validation binding. Rebuild
checks the original actual-state hash, geometry, mask, solver configuration
and validation binding before any planning.

The actual-state SHA256 is
`c2478fbd880806d8499cdc26a4f5613548709e7e76aab4af7731c67d430b2b1a`;
the frozen case SHA256 is
`391796d1af3f8f94f4211cdfd7e2ed2f69a2630481a2c7d0973db45a15d3d345`.
It retains all **37 actual remaining cartons**. Target `carton_l07_c04` uses
the first front/90-degree grasp branch
`a2fe9288df95bda9a1da120b70c6eb203043dd13273754facbb7580daf4ef9cb`;
path seed is 2106872174 and approach seed is 2106872184. Start radians:

```text
[-0.5284209847450256, 0.09451229125261307, -0.21063973009586334,
 -3.1416585445404053, 1.2654187679290771, 0.5286964774131775]
```

Requested physical contact position is approximately
`[-0.000745664929, 0.840006481564, 2.250029521781] m`; the full rotation and
virtual-task transform are in the frozen input. The original near gate is
35.0008889 mm outward, derived from the same projected tool depth, runtime
reserve and IK/contact tolerances. No new mandatory process station is added.

All three first-gate IK endpoints match the previous progress log exactly.
Their ordinary (not angle-wrapped) joint L2 changes are 2.84835, 11.21182 and
7.80330 rad. All endpoints are valid, but all direct edges fail. The shorter
first solution **already received a chance**; simply avoiding the large wrist
winding is insufficient. The third reproduces the exact reported pair and
gap: `tool_rigid_0 / carton_l05_c02`, 0.004984748077057233 m < 0.005 m.
This remains a clearance rejection, not a claim of original-CAD penetration.

Previously the first near gate entered RRT before any adaptive entry. There
was no applicable template or local candidate at that entry. A bounded probe
of the unchanged existing entries (35.0009 mm, its existing adaptive duplicate,
100 mm and 140.0036 mm) checks twelve finite IK/direct attempts: all reject,
including one exact cache reuse. Thus entry fairness alone does not solve this
scene. Moving only the start away before a joint connection also still sweeps
into the stack; that diagnostic was stopped after its observed rejection,
with its partial results retained as incomplete, not globally infeasible.

### Two local changes

1. `_connect_pose(allow_rrt=False)` exposes its existing finite cheap pass.
   `_approach` gives every existing entry that pass before local alternatives
   and RRT. Existing seeds stay attached to their entry, all original IK and
   adaptive possibilities remain, and later RRT reuses exact edge evidence.
   Candidate-local exhaustion remains resumable; request invalidation stops.
2. After these direct rejections, `_approach_clearance_candidate` generates
   a bounded free-space transition from the current stack support plane and
   complete tool radius about the physical contact frame. It moves outward,
   translates/turns outside that plane, then returns to the **original gate**.
   The existing Cartesian solver and MotionValidator check every edge with
   `pregrasp` permissions; only the original terminal arc uses CONTACT_PROCESS.
   Long waypoint legs are chunked with the existing per-stage sample cap; all
   local attempts share the existing 240-sample local-candidate allowance.
   A failed candidate leaves the original bounded RRT available.

The candidate is `LOCAL_CARTESIAN_CANDIDATE`, never VERIFIED_TEMPLATE or a
history hint. The plane is a waypoint heuristic, **not a clearance certificate**
for the robot or a continuous-motion proof. Here the full tool radius is
0.33208594 m and the transition plane coordinate along the outward normal is
0.33767070 m. Geometry and actual joint/IK checks still decide validity.
The request retains its original input lease across entry/candidate changes;
stale input cannot publish a completed approach. Original merged `contact`
output and free/terminal indices retain their meanings.

### Directed checks and short measurements

The final-source combined CPU invocation is **329 passed, 22 skipped, zero
failed**, 24.43 s. This is one non-overlapping total including six new tests;
the skips still require unavailable legacy history files. Tests cover actual
adaptive-entry opportunity with real IK/MotionValidator, request cancellation,
work exhaustion/context change, deferred-search uncertainty, complete-tool
candidate geometry/free permissions and bounded Cartesian work. Existing real
RRT, binding, early-stop, process, reception and baseline-protection regressions
remain included. The five earlier seam failures are not changed or reclassified.

Same-server, same-input short medians (three cold/hot repetitions, milliseconds):

| Case | d74 cold / hot | 0bfd cold / hot |
| --- | ---: | ---: |
| Official empty direct | 133.703 / 79.057 | 140.087 / 79.076 |
| Official loaded historical endpoints, new connection | 469.072 / 4.924 | 466.908 / 5.210 |
| Analytic loaded obstruction, real RRT | 12.838 / 2.408 | 12.137 / 2.397 |
| Official contact process | 185.084 / 133.259 | 191.920 / 138.191 |

Cold expensive-state counts remain 397 / 16 / 50 / 27; hot counts are zero.
Only the real detour expands RRT (six extensions, three iterations). Both
100-lookup binding probes retain 100 prepared hits, 200 lightweight guards,
zero full context rebuilds and zero JSON/SHA calls. Timing includes small
regressions as well as improvements; no general speedup is claimed.

A separately profiled original first direct edge requires 18,143 strict grid
states but stops at sample 1,734, leaving 16,408 suffix states unobserved.
It prepares 1,736 FK states and executes 1,735 expensive checks (including the
start); hot edge reuse takes 0.289 ms with zero new checks. Profiled wall time
is 19.585 s. Inclusive component times are: lightweight context guards 2.030 s,
FK/geometry preparation 0.483 s, environment collision 6.327 s, interval-proof
preparation 1.335 s, and the entire scalar state predicate 15.525 s. These
**overlap and must not be added**. Batch kinematics accounts for 0.449 s inside
preparation. Full remaining profile/counters are retained in the evidence.
Scalar collision organization, tool-object materialization and dense strict
sampling remain hotspots; this patch does not reopen validation-kernel work.

### Production approach result

The fixed-source production `_approach` invocation returns **VALID** in
403.819 s. The four existing entry cheap passes take 25.225, 21.836, 111.499
and 50.750 s; then the first clearance candidate plus original terminal arc
succeeds in 194.464 s. There are **zero RRT iterations/expansions**. This is a
newly generated original-scene FANUC path, not an analytic-robot substitute or
historical path replay. Source, frozen case and actual-state hashes still match.

The merged direct-mode output contains 126 nodes, with free connection end and
terminal contact start both at index **122**, and final contact at 125. The
free candidate's four Cartesian chunks use 9 / 46 / 46 / 21 samples and
1,088 / 7,326 / 14,654 / 3,318 expensive checks. The terminal arc adds three
samples. All free chunks use the original `pregrasp` validation contract
`dff1fcd583753ffafa387b58588351f4232a67db55c463c0b8ec14035c80b542`;
contact uses its existing target-scoped contract. The final actual FK error
is 0.0000821882 m / 0.0000201098 rad, within the unchanged 0.0001 m / 0.0002 rad
tolerances. Final full-ring selection retains the same **48/72 commanded cups**,
target and mask, and the actual endpoint collision check passes.

Total connector work is 56,902 state checks, four endpoint IK streams plus
125 Cartesian IK calls (154 seeds and 2,571 IK iterations in all). The 26,743
`edge_state_samples` counter covers the Cartesian path checks; it is not the
same counter as all 56,902 state checks and must not be added to it. Local
candidate work is bounded at 122 of the existing 240 samples. Original direct
failures still stop early and remain invalid cached edges; none becomes valid
merely because another route succeeds.

The previous 300.008 s result was externally censored inside first-entry RRT
(96 extensions, 36,727 edge samples), so it provides **no baseline time to
success**. The new 403.819 s successful connection is not a speedup ratio.
Existing adaptive direct checks add substantial work before the new candidate;
this is an explicit scheduling cost. It preserves their opportunities and
avoids attributing the old unfinished run to one algorithmic cause. Neither
global RRT reachability nor all possible IK alternatives have been decided.

Reproduce the scoped check on the server's fixed archive:

```sh
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python \
  tools/probe_m710_free_approach.py --mode approach \
  --case ../evidence/frozen/frozen_case.json --output /fresh/path/approach.json
```

Repository evidence is under
[evidence/free_approach_20260928](evidence/free_approach_20260928/). The frozen
case and actual scene are included unchanged; `production_approach.json` retains
path, endpoint checks, attempts, failures and counters. Its compact copy omits
only repetitive `checked` interval lists; the full originals and diagnostic
scripts remain in `/root/autodl-tmp/m710-free-approach-20260928/evidence`.
`source_0bfd708.json` binds the archived files, and `cpu_run_0bfd708.json` records
the exact combined test invocation and post-test source check.

### One subsequent normal single-carton attempt

After the production approach passed, exactly one ordinary single-carton run
used the same immutable `source-0bfd708` archive, original actual scene, target
and paired POC motion/execution policies. It supplied neither the successful
approach path nor a historical path/contact seed. The external experimental
allowance was 900 s, justified by the measured 404 s approach; this did not
change production deadlines or reset any search budget.

The ordinary run completed `_approach` at **429.052 s** with the same work
counts, accepted its actual contact endpoint, and passed `_support_release`
at 429.076 s. Under the existing POC policy no separate support lift is required.
The first controlled extraction completed at **582.529 s** and passed the
existing release/clearance boundary into FREE_LOADED_TRANSFER. It then stopped
naturally at **600.894 s**, exit code 2, with **INDETERMINATE**:
`SHARED_LOCAL_TRANSIT_SAMPLE_BUDGET`, stage `transit`. The external 900 s alarm
did **not** trigger, and no second full attempt was started.

The new stop is precise: the existing loaded local candidate needs 121 samples
but its per-candidate allocation is 80 from the configured shared 240-sample
pool. Its trace still reports all 80 allocated samples remaining. This is
**not evidence that all 240 samples were consumed**, and the new approach's
122-sample allowance is local and separate; the original loaded pool is
initialized after approach/extraction entry as before. The existing propagation
classifies this candidate rejection as a request stop, so loaded RRT is not
expanded. This downstream quota/stop behavior is retained for a later scoped
fix, not changed or repeatedly retried in this free-approach round.

The failure records 64,914 total state checks, 166 Cartesian samples and zero
RRT iterations. Only one of 108 generated candidates was attempted; 107 remain
unsearched. No global geometric infeasibility is claimed. The normal failure
`motion.json`, scene snapshot, delivery timings and flushed progress are saved,
but there is **no complete trajectory, execution bundle export, independent
preflight or bundle readback**. `execution_ready` stays false. **Isaac was not
run.** The full single-carton chain remains unaccepted beyond the new stop.

See `single_carton_summary.json`, `single_carton_0bfd708.json`,
`single_carton_run_0bfd708.json` and `single_carton_delivery/` in the evidence
directory. Original failure JSON is preserved byte-for-byte with its original
new-run evidence fingerprint. `server_raw_manifest.json` locates and hashes
the unabridged diagnostic records. All 267 archived source/config/test files
and the inputs still match after this final attempt. The later documentation
commit does not change the tested implementation.

Remaining limits are the loaded candidate quota/stop issue above, dense
scalar clearance work, and the unverified downstream placement/export/physics
chain. This closes the measured original free-approach plus terminal-contact
gap; it does not establish whole-row stability, continuous collision proof or
machine qualification.

### Local Cartesian quota scope correction (2026-09-28, baseline 20c7a375)

The loaded Cartesian producer now reports `INDETERMINATE` with an explicit
`LOCAL_CARTESIAN_SAMPLES` budget domain and candidate scope. The small
`_bounded_local_transit` allocation owner, used by the existing receiver branch,
adds the shared account and marks an exhausted local pool as method scope.
Neither condition exhausts the independent RRT or request validation budget.
The original 80-candidate / 240-shared configuration is unchanged. A predicted
121-sample edge consumes zero samples; no speculative reservation is charged.
Cartesian counters count attempted IK samples, separately from collision checks
and RRT iterations. Unlimited request checks are recorded as null, never as 240.

Actual completed sample work is settled in `finally`, including partial failure,
cancellation and exceptions. The outer allocation is settled once. Request
work counters are neither refunded nor restarted. `ValidationResult.evidence`
preserves producer termination metadata through wrapping; the dispatch reads
candidate/method scope and retains explicit compatibility with the old named
local producer format. Unknown request budget stops remain conservative.
An inexpensive post-producer validator guard catches cancellation, stale input
and exhausted request work/deadline before another candidate or RRT can run.

`test_local_transit_budget_scope.py` exercises the real producer, actual attached
OBB, production dispatch and real RRT on the existing analytic robot fixture.
It covers zero/partial consumption, zero shared pool, later candidate success,
RRT success/exhaustion, request stops, exception settlement and nested/outer
plan wrappers. These analytic routes are not FANUC original-scene evidence.
`probe_m710_loaded_transit.py` reconstructs missing extraction nodes from the
saved successful contact endpoint, then rechecks any saved extraction with a
fresh tracker and the unchanged reserve. It runs only the first production
receiver connection and records an external resource interruption separately
from geometric failure. Prefix reuse is explicitly offline evidence, not a new
full-cycle plan or measured physical state.

The final implementation tested is **01a92826b005a8590599cfa4cdefa557dba8e4ba**.
The immutable server archive SHA-256 is
`260a12b542cd5650f5a239c22c1ff80a3258945197adca0cfb1582a0f64268c1`;
all 269 archived source/config/test/tool files matched after the tests and
fragment run. The later evidence/documentation commit changes no implementation.
The final combined 23-file CPU run passed **346**, skipped **22**, failed **0**
in **24.55 s**. This includes the 17 new regressions; earlier subset runs are
not added to that count. Skips still require unavailable private historical
inputs. The five previously identified seam failures were not included or fixed.

Original-scene evidence is under
[evidence/local_transit_budget_20260928](evidence/local_transit_budget_20260928/).
The baseline executable archive is `0bfd708`, whose `src/tests/tools/configs`
and `pyproject.toml` are identical to review baseline `20c7a375`. Its production
branch reconstructs only the missing 42-node controlled extraction from the
saved successful contact endpoint, retaining actual contact selection and
attachment capture. The patched run rechecks those extraction edges with a
fresh production tracker and the original reserve before `_finish_place_branch`.
Both recover the exact original start (radians):
`[0.9862454563859637, -0.6632776457504109, -0.4577065817616128,
-1.7051491620249044, 0.9999112930290731, -1.3257242474822586]`.
The original first longitudinal receiver, 0.025 m ideal release-height
candidate, preplace transform, three IK endpoint attempts, mask and remaining
scene are preserved in the frozen input. The transit context remains
`f1510ea7db557b10af16da59f42e17d90840def9227e280c83f918507feb0bb6`.
No `fully_released` flag was manually assigned. Missing original request work
counter data is explicitly unknown; the current POC request work limit is
unlimited (null). These are planned states, not physically measured extraction.

| Original local producer account | Before | After |
| --- | ---: | ---: |
| Required / allocated samples | 121 / 80 | 121 / 80 |
| Actually consumed / candidate remaining | 0 / 80 | 0 / 80 |
| Shared remaining | 240 | 240 |
| Status | INDETERMINATE | INDETERMINATE |
| Interrupt request | yes | no (CANDIDATE, continuation allowed) |

The same-input producer comparison invokes the real method with the recovered
attachment and checks the exact transit context ID. The baseline connection
stopped after **16.517 s / 1,065 state checks**, with no RRT. The patched
production receiver connection reached real `RRT_CONNECT`, seed **2106872244**,
candidate allowance **200** within the unchanged 600 receiver / 1,800 placement
iteration accounts. Its last flushed checkpoint recorded **29 extensions
started**, **29 edge-validation calls**, and **9,690 edge state samples**.
These are lower bounds at that checkpoint; completed RRT iterations are unknown
because interruption precedes normal planner return. In particular, the
connector's zero *completed-return* iteration counter is not zero search work.

The patched connection used **299.988 s / 10,948 actual state checks** before
the explicit **300 s offline connection resource limit** interrupted it. Total
probe time was **442.279 s**, including extraction revalidation; total state
checks were 17,896. It returned **INDETERMINATE /
OFFLINE_CONNECTION_RESOURCE_LIMIT**. This longer duration represents newly
allowed search, not a matched successful-path speed comparison. The shared
Cartesian pool remains 240. Cancellation, stale bindings and request exhaustion
still stop subsequent generation in the final CPU regressions. Analytic RRT
regressions obtained valid detours; the original FANUC fragment has **not** yet
obtained a valid loaded connection and is not proven unreachable.

Accordingly, no new normal full single-carton run, execution export, independent
preflight, bundle readback or Isaac run was started this round. The remaining
blocker is the expensive unresolved original loaded connection; placement and
departure were not reworked. One premature archive extraction failed at import
before planning; its setup log is retained separately. The actual patched run
started only after complete archive/file verification, and its source and all
inputs still matched at exit. Raw progress, frozen data, comparison script,
source manifest and final CPU invocation are included; no prior fingerprint was
rewritten and no whole-cycle success is inferred from prefix reuse.

### Demand-aware local allocation (following 09dbd951)

The shared Cartesian pool stays at 240. The original 80 is the default fair
share, not an absolute candidate cap. `_local_transit_work` estimates the same
production chunks, allowing for the existing strict IK position/orientation
residual at later chunk starts. `_bounded_local_transit` also budgets the existing
finite short outward retries, and lends up to `pool - reserved`, reserving one
default share for later candidates when the pool permits. At 240 this caps the
loan at 160; smaller pools still permit the default share before retaining the
remainder. No allocation is charged in advance. Existing actual-work settlement,
method/candidate stop scope and independent RRT/request budgets are unchanged.
Every chunk still recalculates demand using actual FK; the estimate is neither
an exact future sample count nor geometric acceptance.

The previous scope regressions now use an unallocatable 201-sample demand where
they require zero-work rejection; the explicit real 121/80 producer rejection
is still tested separately. Partial residual-induced exhaustion is retained
with a genuinely limited 80-sample pool. New regressions cover executable loans,
residual headroom, partial geometry rejection followed by real RRT, insufficient
pools and request cancellation/context/work stops during borrowed interpolation.
Valid direct edges still bypass the local producer entirely.
