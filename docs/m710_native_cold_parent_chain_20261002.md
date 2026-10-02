# Native-cold explicit parent identity, 2026-10-02

Final status: **ISAAC_WORKFLOW_COMPLETED_UNDER_DECLARED_ASSUMPTIONS**. The one
new-world native-cold attempt completed its workflow after every planning and
delivery gate passed. Actual grasp and release counts are one each; ideal
reception and ideal outfeed counts are one each. `physical_cycle_completed`
and `qualification_passed` are both false, and actual reception count is zero.
All 17 targeted development checks also passed.

## Baseline and authorized scope

The approved baseline is `22291f0`, following the endpoint propagation fix
`7701fa6`. It contains the reviewed feasibility core
`4ceb9487e8e650a82243fa10b2c4e91ff3ec48cd` and MoveIt backend
`c7e4b486b3bf50fc607cf1c826c441fc688dbdfe`. The
[October 1 formal failure](m710_native_cold_integration_20261001.md) remains
BLOCKED; this round does not relabel or continue that stopped world.

At the start of this round, the current integration branch, local HEAD and
remote branch all matched `22291f06fd626fe1bd0beeb0d2bdce38155c06ff`, with a
clean working tree. No source branch was changed or merged again. Before
delivery, the remote still matched that baseline; this round adds the frozen
implementation commit and a separate evidence/documentation commit.

The newly authorized scope is the minimal parent-identity correction, directed
GPU verification, and then one new-world single-carton native-cold attempt on
`feat/v0.5-moveit2-native-single-carton`. Formal planning must start from this
new world's measured, stable 40-carton scene; the same world stays paused until
all planning, source-coverage, export and execution gates pass. Failure stops
the attempt and is retained. No second formal world or physical retry is
included. The original recording is 640x360 at 5 fps and normal
physical time without reducing physics or monitoring rates.

Historical paths, prior contact states, successful masks and saved native
subsolutions are forbidden planning inputs. Prior and current probe outputs
are read only as audit evidence. The official model, current collision policy,
20 kg tool, 42.5 kg carton, finite limits and independent geometric authority
remain in force.

## Minimal implementation

- `NativeStageState` carries an issued immutable receipt binding Task, stage,
  exact native endpoint and scene/context identity. Ordinary `_connect_pose`
  preserves the selected native final sample and receipt, including copying.
  Parent selection reads that receipt; it does not search earlier records for
  equal joint values. Selecting an earlier sibling restores that explicit
  prefix, and final source records follow its parent chain.
- `native_root_state` explicitly binds the current frozen initial state and
  declared world. Worker gates reject missing explicit parent, changed target
  identity and an initially attached target. Task, scene, process and ownership
  transition checks remain fail-closed. Contact selection context is restored
  between scoped candidate operations.
- A selected zero-motion release supplies a complete `terminal_state_request`
  to the native Task audit. The worker revalidates its exact final q, parent,
  identity, process and real scene transition, with zero solver calls. No
  fabricated motion or external path is accepted as a native subsolution.
  The real-worker terminal-release audit and changed-terminal-q rejection now
  pass in the declared synthetic ownership Task.
- World object shapes use MoveIt's `global_shape_poses_` for transition
  continuity, inspection and non-target-world comparisons. The previous use
  of object-local `shape_poses_` incorrectly treated local coordinates as
  world coordinates. The original `1e-7` continuity threshold is unchanged.

These implementation and directed-test statements are separate from the
formal planning evidence below and do not establish physical reception or
qualification.

## Directed GPU results

The isolated GPU development environment recorded these successive focused
Python runs: **95 passed**, then **88 passed / 1 skipped**, then **49 passed**.
The final source check, `pytest-source-final`, recorded **101 passed in
0.54 s**. These are overlapping directed runs, not distinct-test totals. The bootstrap
check skipped in the later run had already passed; that skip is not counted as
a new pass. No full historical population or repeated physical trial was used.
The native process-policy executable passed **11 cases**, including four new
literal world-frame regression cases.

The frozen implementation is commit
`f94a1ca4c48b4b4bf5805466549c941ed07dd830`; the built worker SHA-256 is
`e933e868ac5c50879847dc1161046b6b9ad081adf23fe4f6359338bc9caf723c`.
Implementation source bytes match the committed bytes. All 335 transferred
manifest entries match the remote source; the retained raw manifest also
records ten unmodified files whose working-tree CRLF bytes differ from Git's
raw blob bytes.

The compiled-worker results preserve earlier failures alongside the final pass:

| Evidence | Actual result and limit |
| --- | --- |
| `parent-probe-1.json` | Overall `ERROR`; ordinary main parent-chain checks passed in the unchanged 40-carton scene. |
| Ordinary `_connect_pose` PTP | A newly generated native endpoint naturally differed from its IK goal at machine precision; the selected state matched the returned endpoint and retained its receipt. |
| Equal-q siblings and following LIN | Two fresh native PTP stages had equal numeric endpoints but distinct identities. LIN explicitly continued selected A, although B was newer; native backtracking, current source coverage and independent authority passed. |
| Probe 1 synthetic ownership addendum | Contact passed. Extraction start authority rejected `PAYLOAD_COLLISION(carton_l07_c02, carton_l06_c02)` before an extraction solver call. Moving the synthetic target to the current home TCP overlapped a retained neighbor; this is a diagnostic-scene construction error, not a parent-chain failure or a collision exemption. |
| `ownership-probe-2.json` | Overall `ERROR`; original-scene home and sparse synthetic contact passed, then attachment transition raised `NATIVE_TASK_ATTACHMENT_TELEPORT`. This failed attempt remains retained. |
| Ownership Probe 3 diagnosis | World `shape_poses_` was object-local, but the worker compared it as world coordinates. Correcting all three affected reads to `global_shape_poses_` fixed the coordinate semantics without changing attachment transforms or the continuity threshold. |
| `parent-probe-4.json` | Overall `PASS`: all 17 checks in 16.2276313 s, including the original 40-carton parent chain, synthetic world-to-attached-to-world readback, independent checks, zero-motion final release audit and required rejection cases. These are 17 checks, not 17 solver calls or physical cycles. |

Probe 2's ownership Task explicitly declares one current-size target and the
unchanged static scene, with the other 39 carton IDs listed as excluded.
Snapshot contents, fingerprint, support graph and stack IDs agree with that
single-carton diagnostic. The declared mass remains 42.5 kg; dynamics are not
evaluated. No ACM permission, gap threshold or geometry is relaxed. Its native
model identity identifies the initialized official model/tool/policy baseline;
the separately recorded derived scene fingerprint identifies this synthetic
Task. Probe 4 retains this declared sparse ownership scope. This is **not
40-carton acceptance** and cannot enter the formal request.

`--ownership-only` is self-contained: it creates its negative-case geometric
request from the current configured q and a short new J1 goal, without reading
Probe 1. Probe 4 used the full targeted suite and freshly generated native
solutions. It passed the changed zero-terminal-q, changed target identity,
missing explicit parent and attached-root rejections. Production ERROR handling
closes the worker stream; isolated negative-case workers preserve this behavior.
The development result records zero history inputs, Isaac `NOT_RUN`, and zero
physical successes. The earlier failed attempts remain failures.

## Formal native planning and delivery

The one formal run is `native-cold-c66166b61f2548bfabe896a5010fa444`, world
session `1790909283.6163392`, target `carton_l07_c02`, seed `71070`. It uses
the frozen source commit and worker hash stated above. Recorded counts are
one world launch, one ordinary native-cold planning request and one execution
request. The new world's measured 40-carton state stayed paused during
planning; execution used that retained world. History reads and legacy motion
generation calls are both zero.

Native Task `native-cold-1c240bb2901b431a91c624133df99eca` selected the explicit
chain `:1 → :2 → :3 → :10 → :11 → :12` after six suffix backtracks. Its 440
source waypoints contain 436 nonzero edges; all 436 have current native source
coverage, with no uncovered edge or external-path import. Native Task audit
confirms one scene attachment and one scene release transition; these are
planning scene facts, not measured physical grasp/release completion.

| Selected stage | Native solver | Returned samples | Nonzero edges |
| --- | --- | ---: | ---: |
| `:1` pregrasp | PTP | 78 | 77 |
| `:2` contact | LIN | 15 | 14 |
| `:3` extraction | LIN | 159 | 157 |
| `:10` transit | OMPL | 84 | 83 |
| `:11` place | LIN | 2 | 0 |
| `:12` withdrawal | LIN | 107 | 105 |

Every selected stage passed native output checks and independent authority.
Stage `:11` made an actual LIN call and returned two equal joint-state samples;
its zero nonzero-edge count does not erase that call or manufacture motion.
Across all returned attempts, actual counters are **17 native IK, 5 PTP,
5 LIN and 2 OMPL calls**. The 12 stage request records are separate evidence,
not the basis for inferring invocation counts. The first transit OMPL output
was rejected for payload/neighbor clearance of 0.004986852 m against the
unchanged 0.005 m requirement; the subsequent newly generated OMPL stage
passed. Failed alternatives remain in the raw evidence.

## Nine acceptance items and final execution scope

| # | Acceptance item | Observed result |
| ---: | --- | --- |
| 1 | Current measured initial scene and cold inputs | Passed: all 40 cartons retained; history reads and legacy motion generation are zero. |
| 2 | Complete native geometry | `PASS`; selected stages are generated by the current MTC Task, including the real OMPL transit fallback. |
| 3 | Parent identity and all-edge provenance | `PASS`; 436/436 nonzero edges covered, zero external-path imports; Task audit `SUCCESS` with the explicit selected chain. |
| 4 | Existing independent authority and final path checks | Passed for every selected stage and the complete trajectory. |
| 5 | Preflight, export and final bundle loading checks | Preflight `READY`, `simulation_execution_ready=true`; export, bundle readback and final reference TCP audit `PASS`. |
| 6 | One execution in the retained world | `workflow_cycle_completed=true`, `runtime_stop_reason=null`; actual grasp 1, actual release 1. At initial bundle binding, all 40 bodies were retained and `scene_restored_or_teleported=false`; the same world was then executed once. |
| 7 | Reception and outfeed under the declared profile | Ideal reception 1, ideal outfeed 1, workflow processed 1; actual reception 0. `downstream_physical_qualification_claimed=false`. |
| 8 | Physical and joint-effort qualification | `physical_cycle_completed=false`, `qualification_passed=false`; `joint_efforts_within_limit.status=NOT_EVALUATED` because joint effort ratios are unavailable. No physical qualification or measured torque-limit pass is claimed. |
| 9 | Original recording and decode validation | [Original video](validation/evidence/m710_native_cold_20261002/formal/physics/replay.mp4): 640x360, 5 fps, 615 frames, 123.0 s; full ffmpeg decode exited 0 with an empty error log, without re-encoding. |

The final bundle payload SHA-256 is
`c88118f5104389982ae96382366051d7b86bfdb6d83ac7952b98a289f321ceed`.
The same carton was actually attached and released. Reception and downstream
transport use the declared ideal assumptions; no physical top-contact
reception is synthesized. The bounded ideal handoff records a vertical
correction of -0.022816649 m after actual constraint removal; the initial
same-world binding check does not assert unassisted downstream physics.
The joint-effort qualification detail is exactly
`{"status":"NOT_EVALUATED","reason":"joint effort ratios are unavailable"}`.
Input effort commands are not measured drive output, so this result establishes
neither actual torque exceedance nor a measured torque-limit pass.

The first complete checked plan took 807.414 s; ordinary planning through
delivery took 828.508 s. No subsequent quality optimization was run. Timings
below are separate measurements with nested intervals; they must not be added
as independent costs.

| Measurement | Seconds |
| --- | ---: |
| New-world initialization wall time | 23.003927 |
| Native model warmup | 0.281929 |
| Native IK computation, 17 returned records | 0.005574 |
| Native IK including transport/endpoint checking | 0.489818 |
| Native MTC planning, 12 returned stage records | 12.808556 |
| Native output checking, 7 measured records | 19.205378 |
| Independent stage checking, 6 measured records | 762.965000 |
| Complete-path final recheck | 0.203118 |
| Preflight | 2.350541 |
| Export | 11.370458 |
| Bundle readback | 5.192790 |
| Simulated physics duration | 123.133333 |
| Replay wall time | 1211.344859 |
| Supervisor execution wall time | 1225.200904 |
| Total supervisor wall time | 2077.769818 |

Independent checking dominates this run: transit is approximately 533.02 s
and extraction 158.41 s. Physics ran at 240 Hz for 29,552 steps. The 5 fps
recording did not reduce the physics cadence; its 123.0 s duration describes
the encoded frames. Replay and supervisor wall intervals include different
execution overheads and must not be substituted for simulated physical time.

## Actual formal command and evidence delivery

The command below records the single submitted attempt. Its existing output
directory cannot be reused to launch another attempt.

```bash
D=/root/autodl-tmp/m710-native-cold-20261002
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs
cd "$D/repo"
export OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1
export XDG_RUNTIME_DIR=/root/autodl-tmp/runtime-root
export XDG_CACHE_HOME=/root/autodl-tmp/cache/xdg
export XDG_CONFIG_HOME=/root/autodl-tmp/config/xdg
export XDG_DATA_HOME=/root/autodl-tmp/data/xdg
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -u \
  tools/run_m710_native_cold_once.py \
  --output "$D/repo/outputs/native-cold-20261002-once" \
  --isaac-python /root/autodl-tmp/envs/isaacsim-clean/bin/python \
  --worker-command "$D/worker.sh" \
  --worker-binary "$R/work-native-cold-20261002/build/m710_moveit_worker" \
  --native-asset-root /work-native-cold-20261002 \
  --source-commit f94a1ca4c48b4b4bf5805466549c941ed07dd830 \
  --task-budget-s 3600 --stage-budget-s 300 --ipc-timeout-s 360
```

The [machine-readable summary](validation/evidence/m710_native_cold_20261002/summary.json),
[final physics result](validation/evidence/m710_native_cold_20261002/formal/physics/result.json)
and [complete formal archive](validation/evidence/m710_native_cold_20261002/formal-evidence.tar.gz)
retain the final outcome. Large raw JSON outputs are compressed in the
archive, accompanied by readable summaries, logs, execution events and run
controls. Earlier planning snapshots remain checkpoint evidence; the final
physics result supplies the completed workflow and qualification fields.
These outputs are audit evidence and cannot become solution inputs to a
future native-cold request.

The existing supervisor permits a single retained world and one formal request,
with a shared planning deadline inside stage/IPC and outer process watchdogs.
Those limits do not authorize another attempt. The record preserves exact
commands, source/binary identities and the failed development and planning
alternatives alongside the completed formal workflow.
