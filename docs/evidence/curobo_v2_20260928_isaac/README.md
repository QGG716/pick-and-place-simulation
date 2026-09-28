# Saved cuRobo full task: one new-world Isaac trial

Baseline: dfa5ffb9607485d8f4579958f88870e19c33370b, branch
feat/v0.5-backend-curobo-v2. This directory preserves the new trial;
September 26/28 planning and offline evidence remains unchanged.

## Inputs and scope

See input_sources.json for raw file SHA-256 values and baseline_sources.json
for the Git baseline runtime sources. The remote baseline comparison is in
source_check.json. Server deployment is an archive, not a Git checkout.
The deployed source snapshot is identified by the baseline plus changed_sources.json
and trial/trial.json's complete runtime source hashes.

The input is the existing full_task/motion.json, actual_remaining_state.json,
and final_execution_corrected_schema/replay_bundle.json, with the paired
handoff_continuation validation/simulation configurations. The target is
carton_l07_c04. No planner is invoked. The previous native cuRobo TRANSIT,
geometrically checked delivery, and final C2 controller reference remain
separate artifacts; actual physical measurements are a fourth layer.

## Minimal integration changes

- tools/run_saved_curobo_isaac.py performs one fresh execution finalization,
  checks exact equality of old/new controller samples, C2 knots, joint order,
  target and initial attachment context, then invokes the existing
  scripts/isaacsim_fanuc_replay.py once. Output directories cannot be reused.
- The actual post-settling state records current World physics time. Archive
  world/time remain explicit provenance. A new world's task clock starts at zero.
- HUD and execution counts exclude historically processed cartons. Historical
  transport records remain intact to reconstruct the scene.
- No geometry, policy, margins, effort, mass/inertia, trajectory, controller,
  physics/control/contact monitoring frequency or failure gate was changed.

The runtime source has pre-existing mixed line endings and is byte-preserved;
runtime_changes.diff gives its focused textual changes for review.

## Execution

run.sh is the actual command/environment. It uses the existing CPU validator,
Isaac 6.0.1.0 environment and reusable audited official USD, without upgrades.
The saved bundle is regenerated because runtime source identity changed.
A fresh cold full-controller geometry check is performed once; no prior PASS
boolean or old process cache replaces this check. The runtime additionally
verifies current source/assets, PhysX backend, settling and actual state binding.

Recording is 640x360, 5 fps, 1x. PhysX remains CPU/CCD, 240 Hz. The native
TRANSIT derivatives do not certify the final C2 reference or measured dynamics.
Ideal independent cups and ideal reception/outfeed remain explicit POC assumptions.

## Pre-motion wiring failure (preserved)

Fresh finalization passed: preflight 0.341268 s, export 0.320295 s,
full-controller handoff 915.506596 s, total 916.841167 s.
The regenerated bundle canonical payload is
37f38787e81b16013e57b8d457422f5a30594d55c5f75fe1e5e579ba6c6ab346;
preflight fingerprint is
50ac40659f50ca3b5858f8446aef235e5895b0b649a77ffdb3547bbf46503cfc.

The first launch failed at `from isaacsim import SimulationApp`: the new
launcher used Path.resolve() on the venv Python symlink and selected the base
Conda interpreter. No SimulationApp/World or physical action existed in this
failed attempt. trial/isaac.log, trial/isaac/run_status.json and flow_exit_code.txt
preserve this error; that early `started` timestamp is not a physical world ID.

The launcher now preserves the absolute venv interpreter path. A symlink behavior
regression and the additional new-world timestamp counterexamples pass. The
final launcher also distinguishes an early Python startup timestamp from a
world identity backed by actual-state capture; the original failed-launch
manifest remains unmodified and is superseded by physical_trial_summary.json.
`resume_after_import_fix.sh` invokes only the intended existing Isaac command,
using the same freshly validated bundle in a distinct output directory. The
runtime source/config/geometry/reference did not change for this correction.
pre_isaac_restart_integrity.json records its current source binding recheck;
this is not a replacement for the cold handoff already performed. No second
planning search or duplicate cold full-path check was run.

## New world initialization

The actual Isaac world is 1790577723.9653654. Its 37 active cartons stabilized
in 0.904167 simulated seconds. Post-settling measurements are unattached and
passed existing binding tolerances: max joint delta 0.0000426024 rad and max
carton-center delta 0.0000484820 m. Source archive world 1789875150.9359868 and
its 366.883333 s timestamp remain provenance, not current feedback or completion.
See initial_state_comparison.json and runtime initialized_actual_state.json.

## Actual result

**One physical single-carton workflow completed under the approved ideal-cup /
ideal-reception / ideal-outfeed assumptions. Overall dynamics qualification did
not pass: actual position-drive output effort remains unmeasured.** There was
no runtime stop, collision rejection, attachment loss or second physical trial.
The process exit code was 0; this alone is not the completion criterion.

Trial: 7f0149ae-8d76-49ac-ad61-7f30bf012214.
World: 1790577723.9653654.
Authoritative result: trial/isaac_after_import_fix/result.json, SHA-256
c7933ed2e8f5e9bfde019af4fbb83270923470a198ceb327ee316107664aab8b2.
physical_trial_summary.json distinguishes the failed pre-world Python import
from the one actual Isaac world and one physical-action trial.

- Actual attach/contact accepted at 35.554167 s, with 48 actual contact cups.
- Actual stack clearance entered at 60.916667 s: 5.239005 mm to nearest
  carton_l06_c04. The ordinary free-TRANSIT gate passed at 61.158333 s,
  with zero wait and no waiver.
- Physical TRANSIT reached placement. Constraint removal / release was accepted
  at 187.262500 s, with actual lowest-corner release height 24.886901 mm.
- Constraint independence and bounded ideal reception were recorded at
  187.283333 s, on the same carton/body. No physical support contact was claimed.
- Withdrawal reached the final reference. Ideal outfeed completed at
  197.337500 s; measured full-envelope max X was -3.200880851 m, beyond -3.2 m.
- New-world counts: actual grasp 1, actual release 1, ideal reception 1,
  ideal outfeed 1, actual physical reception 0, workflow completion 1.
  The three archived cartons are not included in these counts.

Observed stage samples (5 fps; these are observation intervals, not exact
transition timestamps):

| Stage | First / last observed task time (s) | Frames |
| --- | --- | --- |
| pregrasp | 0.2 / 33.6 | 168 |
| contact | 33.8 / 35.4 | 9 |
| extraction | 35.6 / 61.0 | 128 |
| TRANSIT | 61.2 / 187.0 | 630 |
| place | 187.2 / 187.2 | 1 |
| withdrawal | 187.4 / 197.2 | 50 |

The reference remained 196.690047 s. Actual task physics lasted 197.337500 s
(47361 steps at 240 Hz), including start gating and final ideal-outfeed completion.
The world clock also includes 0.904167 s of initialization settling. Execution
host wall time was 2214.512895 s; it is not physical motion duration or an online
performance claim. No timing/path limits were changed to shorten the trial.

Peak joint tracking error was 0.001519285 rad (J2), below the existing 0.05 rad
limit. Official joint positions and velocities passed. Attachment remained intact:
peak relative position error 0.0000169704 m and rotation error 0.0000396667 rad.
No unexpected robot/rigid-tool runtime contact was recorded. Final feedback
confirms released, independent, unattached, and the complete reference clock.

## Qualification gap and retained diagnostics

`workflow_cycle_completed=true` and `physical_cycle_completed=false` are both
retained. Physical downstream reception is not qualified under ideal reception.
`qualification_passed=false`, with sole entry `joint_efforts_within_limit`.
The existing qualification evaluator classifies its unavailable ratio as
**NOT_EVALUATED**, not a measured effort exceedance.

Isaac's get_dof_efforts channel is the explicit effort-control input; its zeros
are not realized position-drive output torque. Projected DOF reactions and
model inverse dynamics remain separate channels, not calibrated actuator effort.
Official finite limits [8000, 10000, 5000, 2000, 1000, 900] Nm were retained;
no check was deleted or changed to obtain PASS. The next issue directly supported
by this trial is binding actual position-drive output-effort evidence to the
existing limit check. This round does not solve it by declaring commanded zeros
safe, enabling a new planner torque feature, or repeating the physical trial.

The full log also retains PhysX's CreateJoint disjointed-body-transform warning
at attachment. Existing actual-contact and bounded attachment-error checks passed;
the warning was not a runtime stop and has not been suppressed or erased.

## Video, logs and telemetry

- trial/isaac_after_import_fix/replay.mp4: original continuous 640x360, 5 fps,
  1x stream, 986 frames, 197.2 s; ffprobe and full ffmpeg decode succeeded.
- Video samples cover task time 0.2 through 197.2 s. The final outfeed event
  occurs 0.1375 s after the last periodic frame; final_actual.png and its JSON
  capture the terminal state at 197.3375 s with zero physics advancement and
  unchanged joint/carton state. No frame was appended or retimed to fabricate it.
- trial/isaac_after_import_fix.log: complete actual Isaac process log.
- trial/isaac_after_import_fix/{execution_events,ideal_transport_events,
  runtime_feedback,carton_states,actual_remaining_state}.json: event/state evidence.
- joint_tracking.csv.gz: all 47361 command/feedback/dynamics rows, losslessly
  compressed. actual_frame_states.json.gz: all 986 measured render samples.
- trial/execution/: fresh preflight, actual replay bundle, controller handoff
  and delivery result; native TRANSIT evidence remains in the previous full_task
  directory and was not replaced with controller derivatives.
- artifacts.json binds all 72 delivered original artifacts to their stored and
  uncompressed SHA-256 values. Original uncompressed files also remain on the
  server under /root/autodl-tmp/curobo-v2-20260928-isaac/trial/.

## Directed validation and source binding

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=src \
 /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python \
 -m pytest tests/test_saved_isaac_entry.py tests/test_m710_archive_initialization.py -q
ffprobe -v error -show_entries stream=codec_name,width,height,r_frame_rate,nb_frames,duration \
 -show_entries format=duration,size -of json trial/isaac_after_import_fix/replay.mp4
ffmpeg -v error -i trial/isaac_after_import_fix/replay.mp4 -f null -
```

27 tests passed (directed_tests_delivery.log), covering unchanged-reference
requirements, single existing physics entry, interpreter symlink preservation,
archive provenance/identity failures, new-world clock validation and exclusion
of historical counts. Earlier 23/26-case logs are retained. No full test suite,
new planning search, second physical trial or parameter scan was run.

source_match_final.json confirms the six changed source/test files match the
server. Baseline 149 Python source files matched before the focused deployment.
The final launcher refinements have directed coverage; this actual execution
used the fresh-check entry followed by the explicitly recorded import-fix resume
command. The entire final launcher was not rerun to manufacture a second trial.
Historical input hashes were rechecked after collection and remain unchanged.

