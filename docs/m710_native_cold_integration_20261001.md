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
