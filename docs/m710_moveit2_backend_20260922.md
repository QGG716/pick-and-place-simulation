# MoveIt 2 / MTC / Pilz experiment — 2026-09-22

This branch implements and actually runs the optional native backend. It does
**not** claim that a complete new-backend task has passed execution acceptance.
The authoritative check is still the existing official-mesh/compound-tool,
pair-clearance/contact validator. Native intersection-free candidates can fail
the project's 5 mm gap rule. See the recorded per-attempt results below.

## Baseline and environment

- Branch: `feat/v0.5-backend-moveit2-mtc-pilz`.
- Inspected local/remote baseline: `4f6e3037aceb47a525711e58360d98c8751fec6b`.
- No merge/cherry-pick from other experimental branches.
- Server: Ubuntu 22.04.5; NVIDIA RTX PRO 6000 Blackwell Server Edition.
- Experiment root: `/root/autodl-tmp/m710-moveit2-20260922`.
- Existing CPU environment: `/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv`.
- MoveIt/Pilz **2.5.10**, MTC **0.1.3**, OMPL **1.7.0**, ROS 2 Humble.
  Full Debian build versions are in `rootfs-packages.lock` in the evidence folder.
- The host apt dry run proposed ten upgrades. Instead, an Ubuntu base rootfs was
  bootstrapped under the experiment directory. No host packages or Isaac
  environment were upgraded. `chroot` is used because Docker and mount capability
  were unavailable; ROS and the worker run inside this rootfs, CPU authority
  remains in the existing environment outside it.

The scene remains `m710id70_unloading_layout_v1`, with all **40 cartons**, the
20 kg tool and 42.5 kg target. Scene fingerprint:
`e2af5af3746cd1ce311f4ad93473469d6b377377f7509def59a6d894971d0302`.
The official FANUC assets remain at upstream
`fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`. The adapter changes the generated
world mounting joint to the existing frozen mounting transform; it does not
change axes, independent J3, flange clocking, mesh scale or joint limits.

## Integration and semantic coverage

The ordinary `run_m710id70_layout_single_carton.py --backend moveit2` entry calls
`run_layout_single_carton_audit`, constructs the existing exact connector, and
wraps it with `MoveItLayoutConnector`. Explicit selection never silently invokes
the old RRT and labels it MoveIt. Missing worker or invalid identity is an error.
ROS remains absent from the Python import dependency graph.

One serialized **JSONL/stdin/stdout** protocol is used. A resident C++ worker
keeps RobotModel, PlanningScene and both pipelines alive. Scene imports are
content-versioned; each candidate uses a detached scene diff. IK, native state
validity and native edge planning stay in C++. Python does business adaptation
and the final existing full-edge authority check, not a per-state RPC callback.

| Operation | Implementation and boundary |
| --- | --- |
| Free joint segment | MTC FixedState + MoveTo, explicit Pilz `PTP` |
| Rejected/free blocked segment | Explicit retry through OMPL `RRTConnectkConfigDefault`; up to three candidates |
| Fixed-orientation Cartesian segment without special contact history | MTC MoveTo + explicit Pilz `LIN`, per-request flange-to-task-TCP transform |
| Rotating offset task TCP LIN | `UNSUPPORTED_ROTATING_TASK_TCP_LIN`; no unconstrained OMPL substitution |
| Contact, support release, extraction, receiver contact | Existing process stages, contact selection, physical attachment and authority |
| Full candidate organization | Custom MTC forward stages import current checked process paths, perform planning-state attach/release, and preserve stage ranges/events |
| Execution | Existing preflight, replay export, Isaac adapter and physical monitors; never `task.execute()` |

All 202 existing tool collision bodies are represented as separate fixed links,
including 72 flexible cups. Tool-assembly self exclusions and owned J5/J6
exceptions are explicit. A flexible-cup/neighbor allowance cannot waive a rigid
solid on the same original flange. The base/chassis assembly pair is retained.
The target moves between world and attached state with the same ID; narrow cup
touch links are used. Release reinstates its planned world pose. Native scene
isolation checks verify there is no duplicate body or cross-candidate ACM leak.

**Coverage is candidate generation plus authority, not a fully unified native
policy.** Native collision search uses uninflated intersection/contact geometry.
The existing authority retains total pair gaps of 5 mm, self intersection checks,
bounded compression, stage/history-dependent separation, receiver and swept-body
rules. Unsupported process semantics remain with existing stages or fail closed.
No whole-arm touch links, global collision bypass, missing stack bodies, reduced
clearance, payload changes or relaxed IK tolerances were introduced.

The worker rejects nonzero start velocity, unsupported path constraints,
out-of-bounds starts, inconsistent scene contents under the same fingerprint,
and wrong model/policy identity. Request IDs are checked; cancellation or timeout
terminates that client so a late reply cannot become a later request's success.
OMPL's seed initializes a resident RNG stream once (default `71070`). The stream
seed and call sequence are recorded separately from the upstream business seed;
per-request reseeding of an already-running OMPL instance is **not** claimed.
Wall-clock-limited OMPL runs are not promised bitwise deterministic.

PTP is joint-space motion, not a TCP line. LIN's task TCP line/orientation and
endpoint are checked against existing IK tolerances. The experimental LIN fixture
is a 2 mm fixed-orientation translation at the real home state; it is a constructed
local regression, not a demonstrated production contact action.

## Timing and execution boundary

Original Pilz timestamps, velocities and accelerations are retained in the
result. OMPL uses IPTP, preserving joint waypoints. Fixed-joint goal tolerance is
explicitly `1e-12 rad`; an early run exposed MTC's default `1e-4 rad` goal sampling
and was rejected by the fixed-endpoint gate. The tolerance was tightened in the
solver, not loosened at acceptance. No start-fixing adapters are enabled.

The existing Isaac command generator consumes stopped C2 quintic joint edges,
not ROS spline derivatives. Therefore native durations become per-edge lower
bounds in that existing exporter; original samples/timestamps remain in metadata,
durations are never shortened, and analytic velocity/acceleration/jerk limits are
audited again. The joint-edge geometry is unchanged. This is an explicit time-law
conversion with stops, not blended or online trajectory replacement.

The experiment uses 0.5 rad/s² acceleration limits and 0.2 speed/acceleration
scaling; original URDF velocity/position/effort limits remain. These conservative
planning settings are not vendor acceleration qualification. Humble Pilz derives
deceleration as negative acceleration. Predeclaring its extension parameters
caused a duplicate-declaration warning in early runs; the final adapter leaves
that documented derivation in place, with the same effective deceleration.

## Evidence and interpretation

Evidence is in `docs/validation/evidence/m710_moveit2_20260922/`.
The historical candidate fixture includes its original path, source SHA256 and
scene identity. It is a candidate only: no historical acceptance is inherited.
Endpoint-fixed motion comparisons and the fixed complete-task comparison are
separate. No large sweep, historical population suite or full row was run.

Three FK configurations, including nonzero J3/wrist joints, match the existing
model and TCP to maximum matrix error **2.22e-16**. Native dependency, scene,
attachment, permission, endpoint and cancellation checks are separately recorded.

Initial fixed motion observations:

| Request | Core | Native candidate / authority | Wall time |
| --- | --- | --- | --- |
| Constructed small empty joint move at real home | PASS | PTP PASS / PASS | about 0.4–0.5 s each |
| Historical complete empty approach endpoints | PASS | PTP PASS / PASS | core 78.87 s; native 75.92 s |
| Historical loaded transit endpoints | 600 iterations exhausted | PTP collision; 3 OMPL candidates / all rejected | core 62.04 s; native 458.88 s |
| Constructed fixed-orientation LIN probe | PASS | Pilz LIN PASS / PASS | about 0.4 s each; different IK makes this a pose test, not a pure path-speed comparison |

The loaded case is also the real blocked-direct-path regression. Native PTP
reported collision between the attached target `carton_l07_c02` and neighbor
`carton_l07_c01`. Three subsequent OMPL candidates were generated in about
3.14, 1.60 and 2.06 s. Existing authority rejected them after 218.18, 82.26 and
151.43 s, respectively, for surface gaps of 4.98446, 4.99419 and 4.99893 mm.
The threshold remains 5 mm with the existing numerical tolerance. Rejection
therefore demonstrably continues search rather than stopping at the first
candidate. Search exhaustion is not physical unreachability.

The main measured cost is Python-side authoritative edge checking. Native
planning speed alone is not validated end-to-end success or a production
throughput result. Cold model startup, scene import, worker roundtrip, MTC solve,
authority and whole-request times are recorded; nested times must not be summed
with their inclusive parents. Small samples do not justify a P95 or speedup claim.

## Directed checks and evidence map

The final directed command passed **32 tests in 0.42 s**. It covers native result
joint order, exact endpoints, finite derivatives, increasing time, stopped
boundaries, rejection continuation, duplicate rejection, unavailable dependency,
cancellation/obsolete reply handling, measured nonzero start rejection, native
duration floors and the actual existing replay exporter. Existing trajectory
checks and a small timing selection also pass. No full pytest or population suite
was run. The pre-existing scripted connector test fixture lacked the required IK
position tolerance; `baseline-test-failure.log` records the same failure against
the baseline source. Only that fixture was corrected.

`final-native-contracts.json` is an actual resident-worker run: three FK poses,
48 world obstacles, a detached diff with 47 world objects plus one attached
object, no inherited permission on the next candidate, and rejection of an
out-of-bounds start without modification. Its **synthetic, zero-motion** custom
MTC task successfully attaches, releases and ends with 48 world objects and no
attached body. That tests bookkeeping only; it is not a physically reachable
grasp or an accepted complete motion.

The actual ordinary CLI with explicit `--backend moveit2` and no worker returns
`BLOCKED / MOVEIT2_UNAVAILABLE`, saved in
`ordinary-entry-missing-dependency.json`. Core-stage and core-task runs work in
the CPU environment without ROS Python modules.

| Evidence | Meaning |
| --- | --- |
| `native-stages.json` | Initial empty PTP passes, then an explicit early endpoint-tolerance error; the whole file is not a successful suite |
| `native-loaded-lin.json` | Early loaded endpoint rejection and actual LIN call |
| `native-loaded-tight-goal.json` | Tight-goal loaded PTP rejection and three distinct authority-rejected OMPL candidates |
| `core-stages.json` | Four corresponding core stage observations |
| `final-native-small.json` | PTP and LIN passes with real counted pipeline calls and discrete TCP-line audit |
| `final-native-contracts.json` | Final actual model/scene/MTC bookkeeping checks |
| `core-task.json`, `native-task.json` | Separate fixed complete-task observations |
| `worker-*.log` | Actual ROS/Pilz/OMPL/MTC diagnostics, including early warnings |
| `build-entry.log`, `build-final.log` | Successful build; the delivered shell build and launcher were also run directly |
| `actual-bootstrap.sh`, `bootstrap.log`, `rootfs-packages.lock` | Actual isolated environment setup and installed versions |
| `development-archives.sha256`, `final-source-binary.sha256` | Development upload archive and final binary provenance |

Development evidence is retained chronologically, including failed attempts.
`native-task-running-binary.sha256` identifies the already-running worker
separately from the later final build. The final request now also records the
existing core candidate identity (or an explicit directed-case identity).
Early runs preceded final counted-planner metadata and the duplicate-declaration
cleanup; they must not be represented as a rerun of the final tree. Their planner
selection and actual invocations are visible in ROS logs and per-attempt JSON.
The final short probes explicitly count one PTP and one LIN invocation in the
same resident process. No real loaded-path failure is overwritten by a later
short-probe success.

Representative nested timing from `final-native-small.json` (seconds):

| Stage | Scene import/diff | MTC planning | Worker inclusive | IPC + worker inclusive | Authority | Whole stage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| PTP | .01216 | .00711 | .02488 | .02824 | .45086 | .48186 |
| LIN | .00445 | .00658 | .01708 | .02071 | .42542 | .44871 |

The final contract run measured cold model/pipeline initialization at .16891 s
inside the worker and .38499 s including launch/IPC. Scene import is cold on a
new world hash; a warm request reuses the parent scene but still constructs an
isolated diff. Native IK, MTC organization and planner time are currently bundled
in `mtc_plan_s`; IPTP/result conversion is inside worker total. Separate native
IK and serialization/export timers are **not instrumented** and are not inferred
by subtracting overlapping numbers. Existing complete-task statistics retain
core IK, candidate, edge and nested collision profiles. LIN line checking samples
knots and joint-edge midpoints; this is a discrete tolerance check, not a proof
of exact continuous Cartesian geometry.

## Fixed complete task and execution status

The request uses `carton_l07_c02`, the same frozen 40-carton world, the same one
historical candidate, current cup selection/physical contact, the current POC
release heights and receiver/departure policy. Historical free nodes are checked
again and contact/release are reconstructed; no old acceptance is inherited.

Core completed with planning **PASS**, 820.904 s task time / 821.376 s including
setup. It performed 29,730 state validations and 29,841 edge state samples;
nested collision validation took 815.606 s. `D_COMPLETE_TASK` is retained, with
`execution_ready=false` and `INDEPENDENT_PREFLIGHT_NOT_RUN`. This is a checked
planning result, not physical execution.

Native completed with planning **FAIL**: `UNSUPPORTED_ROTATING_TASK_TCP_LIN`
at `transit`, 1790.224 s task time / 1791.237 s including setup. The fixed
historical contact, support-release and extraction were rechecked, then the
current loaded suffix requested a changing orientation with an offset task TCP.
The adapter returned unsupported rather than substituting an unconstrained
path. Existing finite POC release alternatives were exhausted. It performed
56,524 state validations / 56,815 edge samples, with 1779.163 s nested collision
validation. Repeated checking before discovering this unsupported suffix is a
real integration cost, not native solver time.

This complete-task run has **zero native planning calls**: it reused and
rechecked the fixed historical prefix, then stopped at the adapter's LIN
capability gate. Full real-cycle MTC composition, native complete-task export,
independent native-plan preflight and Isaac are all **NOT_RUN**. Real MTC/Pilz/OMPL
motion calls are demonstrated by the separate directed stage runs; the synthetic
MTC state-transition check does not close this missing real full-cycle result.
No native complete accepted trajectory exists, so no Isaac world, recording or
physical success is claimed. The core accepted plan is retained separately and
is not relabeled as a new-backend execution.

The two complete-task processes overlapped on the server; the exact overlap is
recorded in `task-concurrency.txt`. Their end-to-end durations include current
process reconstruction and authority and are reported individually. They do not
support a controlled speedup claim. The fixed stage comparison was run separately.

## Remaining boundary and smallest next step

The native search/authority distance mismatch is the first demonstrated limit:
intersection-free OMPL solutions can repeatedly miss the required 5 mm gap.
The next implementation step is an equivalent native pair-distance validity
check with the same ownership and stage policy, followed by these same directed
cases. Increasing seeds alone is not a demonstrated solution. A second concrete next step is to support the exact rotating offset TCP
semantics (or detect unsupported requests before repeating long prefix checks).
Rotating offset TCP LIN and continuous blending remain unsupported; contact/history semantics
stay with the current core. Dynamics, load qualification and physical reception
remain the existing execution gates, not implications of MoveIt SUCCESS.

## Reproduction

The focused Python command (run with the existing server CPU venv) was:

```bash
PYTHONPATH=src python -m pytest tests/test_moveit2_backend.py \
  tests/test_layout_trajectory.py \
  tests/test_isaac_bridge.py::test_native_duration_floor_reaches_existing_replay_exporter \
  tests/test_isaac_bridge.py::test_build_fanuc_replay_is_deterministic_and_limit_audited \
  tests/test_isaac_bridge.py::test_replay_retimes_empty_tool_phase_after_stopped_release \
  tests/test_timing.py -q --tb=short
```

On a compatible Humble machine, install the packages named by
`ros2/m710_moveit_backend/package.xml`, using the recorded package versions, then:

```bash
bash ros2/m710_moveit_backend/build.sh
export M710_MOVEIT_COMMAND="$PWD/ros2/m710_moveit_backend/worker.sh"
export M710_MOVEIT_LOG="$PWD/outputs/moveit-worker.log"
export M710_MOVEIT_SEED=71070
export M710_MOVEIT_STAGE_SECONDS=12
PYTHONPATH=src python tools/check_m710_moveit_contracts.py outputs/native-contracts.json
PYTHONPATH=src python tools/run_m710_moveit_integration.py \
  --suite stages --backend moveit2 --output outputs/native-stages.json
PYTHONPATH=src python tools/run_m710_moveit_integration.py \
  --suite stages --backend core --output outputs/core-stages.json
PYTHONPATH=src python tools/run_m710_moveit_integration.py \
  --suite task --backend moveit2 --output outputs/native-task.json
PYTHONPATH=src python tools/run_m710_moveit_integration.py \
  --suite task --backend core --output outputs/core-task.json
# Ordinary production search entry, not just the fixed-candidate experiment:
PYTHONPATH=src python tools/run_m710id70_layout_single_carton.py \
  --backend moveit2 --target carton_l07_c02 --output outputs/moveit-motion.json
```

For the isolated server installation, the host launcher is
`/root/autodl-tmp/m710-moveit2-20260922/worker.sh`, the asset root inside chroot is
`/work` (`M710_MOVEIT_ASSET_ROOT=/work`), and Python is the existing CPU venv above.
To reproduce isolation from an empty directory, use
`ros2/m710_moveit_backend/bootstrap_rootfs.sh`; copy the repo to `<rootfs>/work`,
then build and launch inside chroot. The original bootstrap transcript and full
installed package lock preserve the actual experiment installation.

Optional `M710_MOVEIT_REQUEST_LOG` writes submitted stage JSONL for inspection.
The init payload is regenerated from frozen audited assets, not hand-authored
Panda examples. `--case` selects a named directed stage when diagnosing a failure.

Primary API references:
[Humble Pilz](https://moveit.picknik.ai/humble/doc/examples/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html),
[Humble MTC PipelinePlanner](https://github.com/moveit/moveit_task_constructor/blob/humble/core/src/solvers/pipeline_planner.cpp),
[Humble Pilz limits](https://github.com/moveit/moveit2/blob/humble/moveit_planners/pilz_industrial_motion_planner/src/joint_limits_aggregator.cpp).
