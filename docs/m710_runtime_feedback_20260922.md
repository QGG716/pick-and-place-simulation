# Actual-feedback stop, first repair round — 2026-09-22

Start: `899305be42d266c5c5e319dcba5228072ed75961` (local and fetched origin;
clean working tree). Execution source: `f44b564df58fe4e8172d4014f3d4524e89494025`.
Only `feat/v0.5-feasibility-core` is modified.

## Runtime contract

The production Isaac loop previously read actual joints late in the step and
primarily rejected tracking, position and attachment errors during final
qualification. Release and ideal takeover could precede those decisions.
`RuntimeFeedbackMonitor` now guards the actual loop before its actions and
immediately after `world.step`, before release confirmation, stage progress or
ideal takeover. Initialization settling and legacy warmup use the same gates.

| Feedback | Shared acceptance source |
| --- | --- |
| Actual joint position | Bound official per-joint limits; existing `1e-9 rad` numerical allowance |
| Actual joint velocity | Bound official per-joint velocity limits; existing `1e-5 rad/s` allowance |
| Actual/reference position error | `ReplayQualificationPolicy.tracking_error_limit_rad = 0.05` |
| Attached translation error | `ReplayQualificationPolicy.attachment_position_tolerance_m = 0.02` |
| Attached rotation error | `ReplayQualificationPolicy.attachment_rotation_tolerance_rad = 0.10` |

No debounce or new capacity/effort qualification gate is added. Required vectors
must have the correct dimension and finite values. Attachment poses additionally
require a nondegenerate finite quaternion. Joint velocity is measured feedback,
separate from the existing reference-velocity check.

The reference is the unbiased, zero-order-held trajectory command that produced
the observed step. Its recorded command-clock time stays fixed during a hold;
neither the next trajectory sample nor the gravity-biased drive position target
is substituted. The last issued position/velocity reference and biased drive
target are retained separately.

Fixed-joint attachment error uses the relative transform captured at actual
contact and constraint creation. Surface-gripper attachment begins at first
observed closure on the target. Missing established attachment fails closed.
Attachment error stops applying after a requested release is observed to have
removed the constraint; joint feedback checks continue. Planned attachment times
do not activate this gate.

The first failure is latched. Within one observation the stable order is invalid
required state, position, velocity, tracking, then attachment; every detected
violation in that observation is retained. Contact and cleanup failures cannot
replace that primary cause. Failure JSON is atomically persisted before finite
drive hold commands and again after their outcome. Stop does not advance physics,
remove constraints, reset joints, change payload state or claim settled stability.
An already completed physical step is recorded, not retroactively undone.

`runtime_feedback.json` records source hashes/commit, world/target/stage, step and
both clocks, actual/reference joints, named violations and limits, actual
attachment, release/removal/independence flags, last command, last valid feedback
and hold outcome. Existing final statistics remain; the first stopped sample is
available in this file even when it precedes the late legacy telemetry append.
Failure disables full-schedule/workflow completion and the next-task request.
The existing zero-time snapshot capture verifies unchanged world time and poses.

The classifier AST fixture now supplies the real `paired_contact_key` and
`declared_receiver_top_contact` helpers plus their required configuration.
The directed tests execute extracted production checkpoints, hold handling and
the actual loop body, including due release and post-physics takeover barriers,
first-cause persistence through a failed hold, workflow gating and per-task reset.

## Validation and reproduction

On the existing GPU server, using the existing CPU virtual environment:

```text
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest \
  tests/test_m710_runtime_feedback.py tests/test_m710_drive_reference.py \
  tests/test_qualification.py tests/test_m710_replay_physics.py -q --tb=short
99 passed in 1.68s
```

Changed runtime modules also passed `py_compile`. No full suite, population
regression, candidate search or environment replacement was performed.

The representative historical `carton_l07_c04` motion already matches the current
motion implementation identity. The paired configurations are
`configs/validation/m710id70_handoff_continuation.yaml` and
`configs/simulation/m710id70_handoff_continuation.yaml`. Its original actual state
was applied through `build_verified_motion_input` / `apply_actual_motion_state`;
scene equality, current preflight, normal export, asset audit and bundle readback
passed. Historical motion, execution packages and fingerprints were not edited.

Server work root: `/root/autodl-tmp/m710-runtime-feedback-20260922`.
`prepare_existing.py`, `source_manifest.json`, `run_once.sh`, directed-test log,
preflight, readback and immutable input provenance accompany the physical evidence.
All 278 source archive members were byte-verified against the execution commit
archive (SHA256 `ffaa3915c84d7360477aada2c403a2bb4c8d96f1249d849aafe95792c452739a`).

One independent world is launched with maximum segments 1 and no continuation
directory, at 640×360 / 5 fps / 1×. Physics and feedback remain 240 Hz. Historical
processed cartons c01/c02/c03 are initialization provenance; the new-world
completion count starts at zero. The physical outcome and artifact hashes are
recorded in `docs/evidence/m710_runtime_feedback_20260922/`.

## Single physical outcome: failure retained

World `1790058027.8202078`, target `carton_l07_c04`: failed at physical step
14864, task simulation time **61.9333333333 s**, command time
**60.8250000000 s** (world physics time 62.8375032772 s).
The existing stack-contact gate reported
`STACK_CONTACT_ORIGIN_OR_CONTINUITY_UNRESOLVED` for
`carton_l06_c04` / `carton_l07_c04`, during extraction after the free-space
transition. This is the retained primary reason, not a newly invented joint or
attachment threshold failure. No second Isaac execution was performed.

The target remained attached, with 48 actual contacting cups at attachment.
Release had not been requested or confirmed; workflow and physical-cycle
completion were both false. The hold command was issued through the existing
finite drive, with zero further physics steps and no constraint release attempt.
The failure screenshot advanced physics by exactly 0 s and changed neither
joints nor cartons.

Last actual joints (J1…J6), radians:
`[0.9820306897, -0.6613820195, -0.4571959972, -1.7057110071, 0.9956713319, -1.3262544870]`.
Last tracking error maximum was 0.0003144443 rad; attachment errors were
0.0000095456 m and 0.0000018265 rad. The existing telemetry over the completed
prefix reports maximum tracking error 0.0015192851 rad. These values were within
the unchanged feedback gates. The injected directed tests, rather than this
stack-contact failure, establish abnormal-feedback release/action blocking.

New-world counts: one actual grasp, zero releases, zero receptions/outfeeds and
zero completed cycles. Raw scene execution counts retain three historical ideal
receptions/outfeeds as required by the unchanged continuation protocol; those
three are explicitly excluded from this round's counts in `round_summary.json`.

The preserved video is 640×360, 5 fps, 309 frames / 61.8 s, 1× physical time;
first and last frames decode successfully. Full evidence includes the original
motion/state inputs, new preflight/bundle, actual frames and joints, contact and
stack lifecycle evidence, failure snapshot, runtime latch and hold outcome, both
logs and source/artifact hashes. The remaining stack-contact continuity issue
requires a separately authorized follow-up; it was not weakened or retried here.

Official model, masses, layout, collision policy, finite drives, ideal independent
cups and ideal downstream assumptions are unchanged. Actual drive output torque
remains unevaluated in the existing evidence convention. This round does not
establish full-row reliability or machine qualification.
