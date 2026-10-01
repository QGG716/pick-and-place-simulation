# Native-cold integration, 2026-10-01

This branch integrates only the two reviewed commits. Integration and directed
tests do not constitute a complete native single-carton or Isaac success.

| Role | Commit |
| --- | --- |
| Feasibility core baseline | `4ceb9487e8e650a82243fa10b2c4e91ff3ec48cd` |
| MoveIt backend | `c7e4b486b3bf50fc607cf1c826c441fc688dbdfe` |
| Verified common ancestor | `4f6e3037aceb47a525711e58360d98c8751fec6b` |
| Actual two-parent merge | `37c23db28f663c4cdbab36e0c4658e926d2a2bfb` |

The source branches matched these SHAs when inspected. The target remote branch
initially pointed to the core baseline. Work uses an independent worktree and
only `feat/v0.5-moveit2-native-single-carton`; no reset, force-push, unrelated
branch merge, or change to either source branch is part of this work.

## Semantic integration

- `history_adaptation.py`: resolve the conflict at function level, preserving
  core `free_loaded_hint` and MoveIt `loaded_prefix_geometry`. Retain explicit
  historical regression behavior outside strict mode. Every historical adapter
  entry rejects strict requests before reading hint contents.
- `tests/test_layout_trajectory.py`: retain both position and orientation
  tolerances required by the core's unified validation context.
- `layout_single_carton.py`: preserve immutable scene, exact validation,
  candidate scheduling and shared deadline. Strict mode uses native grasp IK
  at the outer ordinary entry, avoids constructing `HistorySource`, and binds
  its actual initial scene to a retained new world.
- `isaac_bridge.py`, `m710_execution.py`, `m710_replay_contract.py` and the
  standard exporter: retain TCP LIN contracts, final reference index mapping,
  implementation/asset identity and final execution loading checks. Strict
  request markers survive preflight and export; removing native metadata does
  not downgrade a strict request into a legacy executable bundle.
- Native geometric path generation and existing C2 execution time-law
  conversion remain separate provenance fields. Native Pilz samples and times
  are retained; final command timestamps are not claimed as unmodified Pilz
  output.

## Native generation and cold input boundary

The resident worker owns a persistent MTC Task. Its custom stages invoke the
native pipeline during `computeForward()`. Only native-owned subsolutions from
the same Task session can be reused. Parent-stage links support bounded suffix
backtracking; final audit consumes stage identities, never an external path to
be wrapped by `compose`. Every nonzero final edge is matched against returned
native points and stage request identities. Zero-motion semantic events have
separate records.

Native process scene diffs bind the target, attachment, 72 cup geometries and
three masks, support objects, stage and policy. Analytic full seal rings,
compression, support geometry, bounded stack penetration and ordered
separation/re-entry checks supplement narrowly scoped ACM permissions. Python
retains the independent existing exact path authority. The explicit
`ideal_independent_cups`, named flexible-cup neighbor acceptance, and
`planner_relaxed_physics_checked` assumptions are not physical holding-capacity
or real-machine qualification.

`--native-cold --backend moveit2` rejects history fixtures, configured history
sources/registration, legacy motion entry points, and missing current measured
initial state. Default configurable budgets are 3600 s shared task, 300 s
native stage and 360 s IPC watchdog. A shared remaining deadline bounds nested
requests; failed candidates do not reset it. Actual solver, IK and native check
records remain distinct from inclusive timings.

`--bootstrap-contract` is initialization only: all 40 cartons, official robot,
20 kg tool and 42.5 kg cartons; zero cup commands and no executable motion.
After the existing settling gates it exports the measured state and pauses
that same world. A strict native bundle must pass every existing execution
gate and bind the initial state hash/world identity before motion is permitted.
Only one physical segment is allowed. Recording remains 640x360, 5 fps, normal
physical time; physics and monitoring rates are unchanged.

## Environment and evidence scope

Server development root: `/root/autodl-tmp/m710-native-cold-20261001`.
The existing isolated Ubuntu 22.04 / ROS 2 Humble rootfs is reused without
package upgrades. This branch builds in its own `/work-native-cold-20261001`
directory. MoveIt/Pilz 2.5.10, MTC 0.1.3 and OMPL 1.7.0 were observed; exact
installed package locks are retained with this run's evidence. Isaac and other
branch environments are not upgraded.

Development probes explicitly identify configured-home or synthetic process
inputs. They do not read historical solutions, count toward carton success, or
replace the formal measured-state native-cold request. Actual final run status,
commands, source/binary hashes and any blocking gate are recorded in the
accompanying evidence and delivery record.

## Formal outcome: BLOCKED at native stage connection

The one formal run used source commit
`3588954fdae9c1829d27e018c8816a1f3299e2ea`, run ID
`native-cold-449ad1c9fc0d4dc1bc18b929eb55a311`, world session
`1790859213.4127235`, and seed `71070`. The worker SHA-256 was
`5ebd7af181ccfb899d7937c358e24c7a228105d995e4515a36cfa796a9ebe44f`.
The 333 transferred source/config/test files matched the local source manifest.
The implementation bytes committed in that formal source snapshot matched its
tested working-tree bytes; this statement does not cover the later fix.

The new 40-carton world passed the existing settling gate after 0.904167 s
physical time, including 0.5 s stable duration. Measured maximum carton linear
and angular speeds were zero; current maximum penetration was 0.000006751 m,
below the unchanged 0.001 m limit. The additional measured robot rest-start
gate passed at 0.104167 s. Raw joint velocities remain in the evidence; the
native zero-velocity boundary uses the explicit multi-frame gate contract.
The world then stayed paused during offline planning.

The ordinary entry generated new grasp candidates (front face, roll 90 degrees,
pose index 0; three valid grasps). A native MTC/Pilz PTP pregrasp produced 78
points and passed both native and independent geometric checks. The subsequent
contact request failed with `NATIVE_TASK_INITIAL_STATE_CHANGED` before invoking
LIN. The Python scheduler returned its requested IK endpoint instead of the
PTP's actual final sample. Their J3/J6 differences were
`2.7755575615628914e-17` / `8.881784197001252e-16` rad. Exact parent matching
therefore left `parent_stage_id` empty, and the worker correctly refused to
treat that noninitial state as a new root stage. This is a candidate connection
interface defect, not evidence of geometric impossibility or a budget timeout.

| Acceptance item | Actual result |
| --- | --- |
| Reviewed-commit integration | Complete |
| Installed native components and directed probes | Connected; 10 native development checks passed |
| New-world initialization and actual-state binding | Passed |
| Formal native IK / PTP calls | 7 IK calls, 1 PTP call |
| Formal LIN / OMPL calls | 0 / 0; contact rejected before solver invocation |
| History reads / legacy motion generation | 0 / 0 in recorded strict run |
| Complete native geometry and whole-path source coverage | Failed / not reached; no coverage percentage claimed |
| Execution preflight, export, final bundle loading | Not reached |
| Isaac carton execution | Not started; one world, one planning request, zero execution requests |
| Grasp, release, ideal reception and outfeed | Not reached |
| Physical and torque qualification | Not established |

The supervisor sent an identity-bound stop to the same paused world. Isaac
reported `planning_stopped_before_motion`, zero telemetry samples, and shut
down. No second world, target substitution, complete-plan retry, historical
path completion, or physical execution was attempted. There is no first
feasible complete path, execution bundle, or execution video. The preserved
[failure frame](validation/evidence/m710_native_cold_20261001/formal/physics/failure.png)
shows the initialized world; it is not a successful execution frame.

Timings are separate measurements, with nested intervals deliberately not
added together: initialization wall time 32.006 s; ordinary planning through
failure 61.364 s; model warmup 0.292 s; summed native IK computation 0.001432 s;
IK including transport/endpoint validation 0.203 s; MTC planning 0.095 s;
native output checking 3.382 s; independent prefix checking 49.978 s. The
supervisor recorded BLOCKED at 94.226 s. Complete-path final checks, export,
and motion execution never began.

## Directed verification and retained evidence

The final C++ build passed 15 process-geometry cases, 7 process-policy cases,
and the clearance test. Native probe 2 passed all ten checks in 19.399 s:
configured-home validity, native IK, PTP, same-Task LIN append, same-Task
backtracking, OMPL connectivity, actual PTP-rejection-to-OMPL fallback, contact
seal/compression, transport cancellation, and deadline expiry. These are
development inputs, not formal carton successes. Native probe 1's earlier
contact failure is retained; its cause was the undeformed bellows geometry
in the native model versus nominal compressed cup geometry in the core.
The fix changed only the same 72 compliant cup shapes to the existing core's
nominal geometry; rigid geometry and compression allowances stayed unchanged.

The broad directed Python run recorded 131 passes and one outdated synthetic
source-audit fixture failure. After correcting the fixture, the affected
replay-gate tests, bootstrap tests and new supervisor/startup regressions all
passed: 29 tests in 0.51 s. Earlier build/test failures are retained with their
logs; they are not relabeled as passing runs. Full historical populations,
all-backend comparisons and repeated Isaac trials were not run.

Start with the machine-readable
[summary](validation/evidence/m710_native_cold_20261001/summary.json),
[formal input manifest](validation/evidence/m710_native_cold_20261001/formal/input-manifest.json),
[actual native failure records](validation/evidence/m710_native_cold_20261001/formal/plan/motion.json),
and [exact subprocess commands](validation/evidence/m710_native_cold_20261001/formal/commands.json).
The `formal/` folder contains original run, scene, measured state, source
manifest, requests, diagnostics, failure frame and logs. The formal archive
additionally retains the freshly generated USD files. The development archive
contains complete probe responses and requests, build/test history, version
locks and the isolated worker launcher. Archive SHA-256 values are in the
summary. These records are output evidence and are forbidden as motion inputs
to a future native-cold request.

The actual supervisor command was:

```bash
D=/root/autodl-tmp/m710-native-cold-20261001
cd "$D/repo"
export OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1
export XDG_RUNTIME_DIR=/root/autodl-tmp/runtime-root
export XDG_CACHE_HOME=/root/autodl-tmp/cache/xdg
export XDG_CONFIG_HOME=/root/autodl-tmp/config/xdg
export XDG_DATA_HOME=/root/autodl-tmp/data/xdg
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -u \
  tools/run_m710_native_cold_once.py \
  --output "$D/repo/outputs/native-cold-20261001-once" \
  --isaac-python /root/autodl-tmp/envs/isaacsim-clean/bin/python \
  --worker-command "$D/worker.sh" \
  --worker-binary /root/autodl-tmp/m710-moveit2-20260922/rootfs/work-native-cold-20261001/build/m710_moveit_worker \
  --native-asset-root /work-native-cold-20261001 \
  --source-commit 3588954fdae9c1829d27e018c8816a1f3299e2ea
```

This records the completed attempt; its existing output directory deliberately
rejects reuse. Effective budgets were 3600 s shared task, 300 s native stage,
360 s IPC watchdog, and 0.25 s per deterministic native IK call. The supervisor
also imposed 1200 s initialization, task-budget-plus-600 s planner process,
task-budget-plus-900 s paused-world handshake, and 3600 s execution watchdogs.
Those process watchdogs do not reset the planner's shared deadline.

## Post-run correction; formal result remains BLOCKED

Commit `7701fa64f48a881ad9df49403519724090baaac9` makes strict
`_connect_pose()` carry the actual native path's final sample to the next
stage, after the existing candidate selection and validation. It leaves the
native path, receipt, exact parent lookup and coverage thresholds unchanged;
it adds no connection edge. This covers both pregrasp-to-contact and
loaded-transit-to-place callers. The regression reproduces the observed
machine-precision differences with independent synthetic joint values and
checks unchanged native records and complete coverage of its test edge.

In a new server directory, `/root/autodl-tmp/m710-native-cold-20261001/postrun-regression`,
the two changed backend test modules passed 43 tests in 0.45 s
([original log](validation/evidence/m710_native_cold_20261001/development/postrun-regression.log)):

```bash
PYTHONPATH="$PWD/src:$PWD" \
  /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python \
  -m pytest tests/test_moveit2_native_cold.py tests/test_moveit2_backend.py -q
```

This did not rerun native full-task planning or Isaac. The original formal
source directory remained unchanged: all 209 implementation/config hashes in
its run manifest still matched after those tests. No simulator, worker or
supervisor process remained, and GPU memory usage returned to zero. The fix
has directed regression evidence only; full native geometry, downstream
process stages and execution still require a future authorized formal run.
