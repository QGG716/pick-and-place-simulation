# Joint effort semantics and qualification evidence

Baseline: `63160332d765a718fd228ec2500deb7a0b30eefe`, branch `feat/v0.5-backend-curobo-v2`.
This round changes telemetry/qualification only. No business planning, controller tuning,
geometry, mass, inertia, collision permission, effort limit, or complete business replay.

## Preserved business result

The original trial remains `7f0149ae-8d76-49ac-ad61-7f30bf012214`, world
`1790577723.9653654`, target `carton_l07_c04`. Its workflow completed under the approved
ideal reception/outfeed assumptions. `workflow_cycle_completed=true`,
`physical_cycle_completed=false`, `joint_efforts_within_limit=NOT_EVALUATED`,
`qualification_passed=false`. There is no measured-overload conclusion.
All prior files, including motion, reference, events, video and CreateJoint warning,
remain untouched. `historical_files_sha256.json` inventories the previous evidence.
No historical runtime monitoring or qualification fields have been backfilled.

## Installed implementation and official basis

`installed_api.json` records installed method names/line locations, forwarding calls and SHA256, extension
manifests and the native tensor plugin hash. Server Python is 3.12.3, Isaac Sim/kernel
6.0.1.0, experimental prims 1.8.9, omni.physics.tensors / omni.physx.tensors / omni.physx
110.1.13 (build suffix 110.1.2.lx64.r.cp312.u7f4), torch 2.11.0+cu128, numpy 2.3.1,
Warp 1.13.0. The native implementation is a compiled binary; its exact internal
contribution decomposition is not available as installed Python source. The PhysX
extension version is recorded, not an inferred SDK version from another release.
CPU PhysX, MBP, 240 Hz, existing position drive mode; RTX PRO 6000 Blackwell renderer,
Xeon Platinum 8470Q. OMP/OPENBLAS/MKL threads=1. No dependency upgrade.

Version-matched NVIDIA reference:
[Isaac 6.0.1 articulation joint sensors](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_physics_articulation_force.html).
It describes the projected channel as the active motion-axis component, and the
incoming wrench as parent-to-child joint transmission. The installed tensor API
specifies the child joint frame for the 6D wrench. No latest-only API was introduced.

| Quantity | Actual source and semantics | Comparison allowed |
| --- | --- | --- |
| Explicit effort input | `Articulation.get_dof_efforts()` forwards to tensor `get_dof_actuation_forces()`, the input set by `set_dof_efforts`; revolute Nm, DOF order | Does not observe implicit position drive output. All-zero historical values cannot qualify it. |
| Implicit drive output | No separately identified output channel was saved. The configured PD law alone is not a measurement of the implicit solver output. | Still unmeasured over the business cycle. |
| Projected DOF effort | `get_dof_projected_joint_forces()` forwards to the same tensor method: net active incoming joint effort projected along the signed DOF motion axis, revolute Nm | Can represent implicit drive output when other active joint contributions are absent and applicability is established. It is not passive-only reaction. |
| Incoming wrench | `get_link_incoming_joint_force()` forwards to tensor API, splits last dimension into force N and torque Nm, in child joint frame | Link-indexed, not DOF-indexed; use returned link names, never an assumed joint index + 1. Structural load / decomposition evidence, not six actuator efforts. Absent historically, now collected. |
| Inverse dynamics | `M(q)*finite_difference(qd)+c(q,qd)+g(q)` | Model estimate; excludes external carton/attachment load. Not renamed measured drive torque. |
| Drive maximum | `get_dof_max_efforts()` -> tensor `get_dof_max_forces()` | Configuration readback only, separately recorded from measured comparison. |

External forces/contact/attachment loads change the forces transmitted through the
articulation. Joint friction, active limits and explicit efforts can also contribute
along a DOF; the projected sum does not separate these from the position drive. A
6D wrench includes both active and constrained components. The probe establishes
sign and inclusion of explicit/body torque for its simple tree, not a general
closed-constraint decomposition for contact and attached-carton phases.

The implemented projected-source applicability contract requires the exact recorded
Isaac/tensor/backend profile, correct axis/order, zero explicit effort and joint
friction, inactive position/velocity limits, no contact/external joint constraints,
and verified revolute tree axis mapping. Raw units/sign are retained (identity
conversion). Missing applicability remains NOT_EVALUATED. The current business
adapter deliberately does not assert these unproved conditions. This is a bounded
usable scope, not a permanent rejection of projected measurements.

## Existing telemetry: offline reanalysis

`history_analysis.json` was produced by `tools/analyze_saved_joint_effort.py` from the
original gzip CSV, summary, runtime feedback and bundle; each source is fingerprinted.
There are 47,361 ordered post-step samples, J1..J6, time 1/240 through 197.3375 s;
maximum departure from the 240 Hz grid is 2.8422e-14 s. Physics step is derived from
row order, not falsely claimed as an original CSV column. Source code reads after
`world.step`, assigning `(step+1)*physics_dt`. Initialization/settling is outside this
business-action interval.

Explicit input and projected channels each have 47,361 finite values per joint.
Explicit input is zero throughout. Peak absolute projected Nm:
`[86.8591,1557.9622,1298.9636,372.9795,320.4324,65.8072]`.
The diagnostic ratios to `[8000,10000,5000,2000,1000,900]` Nm peak at 0.3204324.
These are projected-load diagnostic comparisons, NOT an implicit-drive qualification.
The first inverse-dynamics sample is unavailable because there is no preceding
velocity; all 47,360 later samples remain usable as model estimates. External load
residual is deliberately unavailable at every step, not zero. Full-cycle separated
drive output and historical applicability/constraint-wrench data are missing.

`attachment_anchor_analysis.json` independently reconstructs the saved attachment
anchors using the logged same-step contact/box frames, body0 local pose, body1
identity anchor and bundle contact offset. Position residual is 1.11e-16 m,
rotation matrix residual 4.44e-16. The external FixedJoint joins J6 to the original
carton and is excluded from the articulation. This does not establish solver-time
creation equality or quantify its impulse. The CreateJoint warning at original log
line 408 remains; no transform was changed to hide it and no warning is called an
overload measurement. Historical external-constraint impulse/wrench was not recorded.

## Independent physical channel probes

One pre-step setup attempt failed because `/Probe/base` did not contain the root API
on sibling `/Probe/fixed`; `probe_01.log` is retained. Two completed 8-second probes
followed: `probe_02` checks channels, `probe_03` verifies the shared production
collector/context/contract. These are channel experiments, not business cycles.
Both use the installed CPU PhysX position drive, a single +Y revolute hinge, 2 kg,
COM=(0.5,0,0), Iyy at COM=0.2, pivot inertia=0.7 kg m2, gravity -Z 9.81, no contact,
friction or joint limits. The authored finite drive limit is 100 Nm and is read back.

Independent expectation is `tau_net=0.7*qdd - 9.81*cos(q) - tau_external`.
No fitted signs, scaling factors or altered loads. Each phase is 480 steps:

| Phase | Last 100 mean measured / expected Nm | Max analytic discrepancy over finite samples |
| --- | --- | --- |
| Gravity hold | -9.810018 / -9.810000 | 0.0000214 Nm |
| +3 Nm external body torque | -12.809399 / -12.810000 | 0.000608 Nm |
| +2 Nm explicit joint input | -9.810021 / -9.810000 | 0.0000274 Nm |
| Small position-driven motion | -9.488366 / -9.488090 | 0.001073 Nm |

The explicit-input phase shows that projected effort is a sum; it does not identify
position-drive output alone. Shared collector: 1,920 finite samples, 1,440 comparable;
the 480 explicit-input samples correctly remain outside the comparison scope.
Gravity-only first 480 steps pass the typed evidence check. Mixed full probe remains
NOT_EVALUATED for isolated implicit-drive output. Probe PASS is not applied to history.
Both pre-step and post-step projected readings and 6D wrenches are retained. The
first acceleration is explicitly null. `probe_summary.json` has exact statistics;
compressed artifacts preserve original decompressed bytes and hashes.

## Implementation and verification

`joint_effort.py` provides the CPU-importable typed source/scope/coverage monitor and
optional duck-typed Isaac collection functions. Actual replay now writes
`joint_effort_context.json`, `joint_effort_observations.jsonl` and
`joint_effort_evidence.json`; CSV column names remain compatible. Context includes
source file hashes, joint/link names, friction properties and joint USD attributes.
Each row has step/time, post-step timing, raw projected and explicit values, 6D wrench,
validity/reasons, attachment/release flags and conditional comparison. Missing reads
remain null/NaN with reasons. Only verified comparable data can cause measured FAIL;
missing primary/input observations cause JOINT_EFFORT_OBSERVATION_LOST, separately
from JOINT_EFFORT_LIMIT_EXCEEDED, through the existing stop/persist mechanism.
Existing collision/release first-failure priority is retained. There is no threshold
relaxation or ignored warmup window.

Qualification reports retain compatibility booleans and `qualification_failures`
(all required non-passing checks), while separately exposing
`qualification_measured_failures`, `qualification_not_evaluated`, and blocked checks.
Untyped legacy effort ratios alone now yield NOT_EVALUATED, including zeros, model
ratios, NaNs or apparent exceedances with unidentified provenance. Typed valid
exceedance yields FAIL even if other steps are missing; PASS requires every required
step to be comparable. New module is included in execution source fingerprints.
Old execution bundles are not refingerprinted; any future business execution needs
the existing fresh handoff checks against the unchanged motion.

Actual server commands (working directory `/root/autodl-tmp/curobo-v2-20260928-effort/repo`):

```bash
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest -q tests/test_joint_effort.py tests/test_qualification.py tests/test_m710_runtime_feedback.py tests/test_m710_replay_contract.py::test_workspace_verifier_checks_sources_manifests_and_optional_asset_audit
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python tools/analyze_saved_joint_effort.py --history docs/evidence/curobo_v2_20260928_isaac --output ../history_analysis.json
PYTHONPATH=src OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1 XDG_RUNTIME_DIR=/root/autodl-tmp/runtime-root XDG_CACHE_HOME=/root/autodl-tmp/cache/xdg XDG_CONFIG_HOME=/root/autodl-tmp/config/xdg XDG_DATA_HOME=/root/autodl-tmp/data/xdg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 /root/autodl-tmp/envs/isaacsim-clean/bin/python tools/probe_isaac_joint_effort.py --output ../probe_03
```

Final directed test log records 92 passing tests; compile check passed. Earlier 84/88
runs preceded additional source/compatibility tests. No full pytest or complete
business rerun. `server_source_sha256.json` binds final server code to this worktree.
The final explicit-input-channel exception propagation change was unit tested after
the shared collector probe; it does not change the probe's successful read path.

## Remaining gap / next bounded collection

The full historical workflow still lacks an identified implicit-drive output across
contact, attached transport and release. The now-verified collector can acquire raw
post-step projected/input/6D channels and configuration evidence in a future separately
authorized business run. Before using them for full-cycle drive qualification,
validate the multi-DOF/contact/external-fixed-joint contribution decomposition (or
identify a separately exposed solver drive output) without changing control mode.
No new business run was performed here, and collecting more projected values alone
would not close that semantic gap. Ideal downstream reception remains unverified
physical reception and is independent of this torque-observation work.
