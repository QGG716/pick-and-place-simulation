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
