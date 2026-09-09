# Isaac Sim server validation

Isaac Sim is an optional high-fidelity backend. It must stay outside the core
geometry and planning package so the planner remains deterministic and usable
on a CPU-only control computer.

## Validated server stack

- Ubuntu 22.04.5 LTS
- NVIDIA driver 595.71.05
- NVIDIA RTX PRO 6000 Blackwell Server Edition
- Vulkan 1.4
- Python 3.12
- PyTorch 2.11.0 with CUDA 12.8 wheels
- Isaac Sim 6.0.1.0 with the `all` and complete `extscache` extras

The Isaac environment and all Omniverse caches should live on the data disk.
The planner should use its own Python environment; do not install Isaac Sim
into the planner environment.

## Headless validation

NVIDIA's EULA must be accepted explicitly. A managed cloud container may only
provide a root account, in which case Kit also requires the explicit root
override.

```bash
export OMNI_KIT_ACCEPT_EULA=YES
export OMNI_KIT_ALLOW_ROOT=1
export XDG_RUNTIME_DIR=/tmp/runtime-root
export XDG_CACHE_HOME=/data/cache/xdg
export XDG_CONFIG_HOME=/data/config/xdg
export XDG_DATA_HOME=/data/data/xdg
export ISAACSIM_SMOKE_OUTPUT=/data/outputs/isaacsim-smoke

python scripts/isaacsim_smoke.py
```

The smoke test starts Kit with Vulkan/RTX, simulates a rigid cube falling onto
a ground plane, reads RGB and metric depth through Replicator annotators, and
writes `result.json`, `rgb.png`, and `depth_m.npy`. It exits non-zero if the
rigid-body or camera checks fail. Startup wall time and steady rendered-step
rate are reported separately.

On the validated server, the warm result at 320 x 240 was:

- Kit startup: 9.63 s
- 180 rendered physics steps: 2.20 s
- rendered step rate: 81.65 Hz
- simulation real-time factor: 1.36
- cube final centre height: 0.249995 m
- finite metric-depth pixels: 100%

This proves a one-camera low-resolution baseline, not final unloading-cell
performance. FANUC articulation, trailer meshes, carton contacts, multiple
cameras, segmentation, streaming, and Monte Carlo workloads require separate
benchmarks.

For interactive remote viewing, expose the TCP/UDP ports required by the
selected Isaac Sim livestream client through the cloud provider. SSH port
forwarding alone cannot replace a required UDP mapping.

## FANUC M-710iD/70 layout-bound replay status (2026-09-09)

This round was intentionally CPU-only and did not upload to a server or start
Isaac Sim.  The frozen-layout audit retained all 40 cartons but found no strict
complete trajectory among the five removable top cartons.  Consequently
`dynamic_execution_preflight.json` is `BLOCKED`, with
`trajectory_segment=null` and `trajectory_segment_status=NOT_AVAILABLE`; no
M-710 video or backend state log was produced.  Earlier smoke tests and the
static layout recording are not M-710 physical-execution evidence for this
round.  See the
[round report](validation/m710id70_real_cad_dynamics_round.md) and
[portable run summary](validation/evidence/m710id70_dynamic_execution_v1/run_summary.json).

M-710 export is fail-closed and accepts only a complete READY preflight:

```bash
python scripts/export_isaac_fanuc_replay.py \
  --preflight docs/validation/evidence/m710id70_dynamic_execution_v1/dynamic_execution_preflight.json \
  --output outputs/m710id70_layout_v1/isaacsim/qualified_replay_bundle.json
```

With the currently archived BLOCKED input this command must fail without
creating an output file.  A future qualified bundle is accepted only after the
runner, before importing or instantiating `SimulationApp`, verifies the bundle
payload hash, embedded READY preflight, exact plan/config/scene/unique
trajectory, current implementation files, manifests and asset audit.  These
SHA-256 checks provide content-integrity and staleness detection, not a digital
signature or authorization boundary.

The runtime contact gate uses the nominally compressed physical cup plane, all
active-cup rays, a 2 mm maximum gap, 0.2 mm penetration tolerance and 5 degree
normal tolerance before asking SurfaceGripper to close.  Conveyor transfer is
exclusive and break-before-make: at most one physical surface drive is active,
and transport metrics follow only that actual active direction.  The M-20
`--plan --config --segment` compatibility path below must not be used for
M-710.

## FANUC M-20iD/35 payload replay status (2026-08-25)

The FANUC M-20iD/35 replay now imports the real URDF collision meshes, creates
the trailer, locked AMR, conveyors and cartons, and records RGB, metric depth,
joint tracking, projected joint forces, contacts and payload pose. Controller
commands include a 0.35 s stationary vacuum-establishment hold and a 0.15 s
stationary pre-release hold. Planning time and simulated cycle time remain
separate metrics.

For the first certified segment (7 kg carton, 60 Hz physics), the diagnostic
run with non-breaking attachment produced:

- command schedule: 65.175 s;
- replay wall time: 17.1--18.6 s (3.55--3.87 times real time);
- peak joint tracking error: 0.02043 rad (limit 0.05 rad);
- peak payload attachment position error: 4.23 mm;
- release-centre error: 4.09 mm;
- settled placement error after 1 s: 11.86 mm (limit 80 mm);
- unexpected robot/scene contacts: zero;
- finite RGB-D depth fraction: 100%.

The confirmed suction head is `TVGL300×400-B40-CVM-BL2`: a 0.30 x 0.40 m
assembly containing 53 physical `TB40` cups. The force values below have different
meanings and must not be substituted for one another:

- 44 N per cup, multiplied by 53 cups, gives the active **whole-head** axial
  maximum of 2332 N;
- 43 N per cup, multiplied by 53 cups, gives the active **whole-head** shear
  maximum of 2279 N;
- 1997 N is the catalogue's theoretical **whole-head** attraction at -60 kPa;
- the 1997 N catalogue reference does not overwrite the user-directed 2332 N
  aggregate used by this replay.

The 44 N pull-off value and 43 N shear value are forces, not peel moments. Per
the confirmed engineering input, both whole-head limits are simple per-cup
aggregates. Neither can be converted to a peel/bending moment without a
validated load direction, lever arm, load-sharing model, and edge-seal failure
criterion. Peel/bending moment, effective seal area, and allowed eccentricity
therefore remain null in the configuration.

The 2332 N and 2279 N aggregate values are **not** continuous working limits or
safety-rated capacities. Effective vacuum level, leakage, surface friction,
load sharing and a safety factor still need physical calibration. Production
qualification consequently fails closed on both
`gripper_wrench_envelope_complete` and `gripper_limits_calibrated`.

The former 500 N / 45 N m FixedJoint experiments remain useful only as solver
diagnostics. The production-path adapter now constructs a 0.30 x 0.40 m
four-point NVIDIA Surface Gripper, commands close/open on the original payload
rigid body, checks both attachment translation and rotation, and preserves the
payload's physical release velocity. The 1800 N total hardware maximum is split
equally across the four diagnostic attachment points (450 N each) so the model
does not silently claim 7200 N. This equal split is itself an uncalibrated
assumption.

The generic FixedJoint plus free-body state handoff remains available only as
`--gripper-model fixed_joint_diagnostic`; it can never satisfy
`production_release_adapter`. USD imports are now invalidated by the URDF hash
and collision-import settings, self-collision reporting is enabled, and missing
or partial evidence is represented as `FAIL`, `NOT_EVALUATED`, or `BLOCKED_BY`
rather than being treated as a pass.

### Surface Gripper replay follow-up (2026-08-25)

The fresh server replay applied all seven SRDF allowed-collision pairs while
leaving every other robot self-collision enabled. This removed the persistent
J4/J6 housing overlap from the dynamics audit: peak tracking error returned to
0.02065 rad, maximum joint effort ratio to 0.775, and unexpected contacts to
zero.

In the historical four-point experiment, 1800 N total coaxial maximum (450 N
per numerical point) and an uncalibrated 500 N total shear assumption (125 N
per point) caused the payload to detach 0.567 s after closure. A matching
four-point isolation replay kept the same 1800 N total coaxial threshold and
relaxed only the diagnostic shear threshold. It completed transport and
same-body release with 17.08 mm
peak attachment translation error, 0.0685 rad peak attachment rotation error,
17.79 mm release-centre error, and 20.80 mm settled placement error. This
isolates the simulated failure to the shear threshold/reaction-force semantics;
it does not establish a real suction-cup shear rating.

The current configuration no longer contains the speculative 500 N / 45 N m
limits. It applies 2279 N total shear directly from 43 N x 53 cups. Since the
peel/bending moment remains unknown, the run still reports
`gripper_wrench_envelope_complete=FAIL`, marks `payload_attachment_intact` as
`NOT_EVALUATED`, and blocks `placement_within_tolerance`; it cannot be presented
as a production qualification. The four numerical attachment points
approximate the footprint and are not the 53 physical cups.

Run the current replay with the server environment described above:

```bash
python scripts/export_isaac_fanuc_replay.py \
  --plan outputs/fanuc_m20id35/isaacsim/source_plan.json \
  --config config/fanuc_m20id35.yaml \
  --segment 0 \
  --output outputs/fanuc_m20id35/isaacsim/replay_segment_0_payload.json

OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH="$PWD/src" \
python scripts/isaacsim_fanuc_replay.py \
  --bundle outputs/fanuc_m20id35/isaacsim/replay_segment_0_payload.json \
  --project-root "$PWD" \
  --usd-directory outputs/fanuc_m20id35/isaacsim/usd \
  --output outputs/fanuc_m20id35/isaacsim/qualification_segment_0 \
  --physics-hz 60 --render-every 30 --post-release-seconds 1.0 \
  --record-replay
```

Set `OMNI_KIT_ACCEPT_EULA=YES` only after the operator has explicitly accepted
the NVIDIA Omniverse EULA. Do not use the process exit code alone: a valid run
must have `run_status.json.status == "complete"`, a matching `result_sha256`,
and `result.json.format == "isaacsim_fanuc_replay_result_v2"`.
