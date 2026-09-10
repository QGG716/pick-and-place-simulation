# M-710iD/70 official-model dynamics evidence (2026-09-10)

This directory is a lightweight, content-addressed evidence set for the fixed
`m710id70_unloading_layout_v1` scene.  The evidence was generated from a source
copy of branch `feat/v0.5-feasibility-core` at base commit
`a372c7f117e61509229b2c15eef344cd7908d499`, with all participating changed
sources recorded by content hash.  `run_summary.json` therefore declares
`metadata_source=declared_source_copy` and `dirty=true`; it does **not** claim
that the server workspace was a clean checkout.

## Result and strict claim boundary

The CPU planner stopped fail-closed before task search because the configured
initial state is invalid.  Its concrete geometry failure is the official
`J5_link` against Wantai rigid-tool solid `tool_rigid_13`.  With the unchanged
10 mm per-body engineering margin, the required pairwise clearance is 20 mm.
The full hash-bound tool STL probe measured
19.126613227165--19.126613227165 mm across 49 deterministic J6 samples, a
maximum shortfall of 0.873386772835 mm.  A separate 256-draw deterministic
state probe (seed `71070`) found no valid witness; it is diagnostic sampling,
not a continuous configuration-space proof.

That J5/tool clearance failure is not the only execution blocker.  The final
CPU dynamic preflight records three independent fail-closed gates:
`INITIAL_STATE_INVALID`, `ROBOT_TOOL_MOUNT_CONTACT_SCOPE_NOT_QUALIFIED`, and
`TOOL_RIGID_COLLISION_COVERAGE_NOT_PROVEN`.  The latter two state that the
J6/tool mounting-contact exception has not been qualified and that the
CAD-derived compound-OBB tool representation has not proven conservative
coverage of the complete rigid tool.  No one blocker should be presented as a
complete explanation of execution readiness.

Consequently:

- complete trajectories: **0**;
- exposed top-layer tasks listed: **5**, tasks searched: **0**;
- selected target, face, cup IDs, or cup masks: **none**;
- attachment, loaded carry, support release, receiver placement, and conveyor
  execution: **not run**;
- physical pick-and-place success: **false**;
- collision margins, SRDF exclusions, layout, 40-carton population, masses,
  finite joint limits, and acceptance denominators were not relaxed or changed.

The canonical Isaac initialization diagnostic ran to completion at the backend
level with `backend_initialization_completed=true` and classification
`RETAINED_AND_SETTLED_DIAGNOSTIC_ONLY`.  It recorded 180 actual-state samples
and 180 render-synchronized frames over 6.0 physical seconds at normal time
scale, 1920x1080, with all 40 carton identities and no attachment.  USD/PhysX
state was authored before reset: maximum reset joint error was exactly 0 rad
and maximum reset carton-position error was
`5.5710418180491744e-08` m.  Formal image capture changed neither state
(`max_capture_q_delta_rad=0` and
`max_capture_carton_position_delta_m=0`) and produced three keyframes.

The initial carton configuration remained present throughout this diagnostic.
The maximum displacement from an authored carton pose was
`9.037468460292764e-05` m; full-run peak carton linear and angular speeds were
0.0184131 m/s and 0.0353579 rad/s.  Full-run and final maximum joint tracking
errors were 0.02712375 rad and 0.02455294 rad.  These are initialization and
settling observations only.  `penetration_gate=NOT_EVALUATED_DIAGNOSTIC_ONLY`,
so they do not validate a planned trajectory, suction contact, rigid
attachment, loaded motion, receiver support, or completed physical pick.

`initialization_contract.json` is explicitly scoped as
`INITIALIZATION_ONLY_NOT_PICK_SUCCESS`.  It sets
`motion_execution_permitted=false`, `attachment_permitted=false`, and labels
the robot pose `diagnostic_pose_not_motion_qualified_home`.  Any Isaac output
created from that contract is only an official-model gravity/initialization
diagnostic with a constant finite-effort target.  It cannot be cited as a
planned trajectory, carton contact, rigid attachment, loaded motion, placement,
or completed physical pick.

The historical 104-task and 129-carton populations were not run and are not an
acceptance denominator for this layout-bound round.  Machine certification is
also separate: the official public inertials and limits are accepted simulation
inputs, while `machine_qualified=false` remains recorded independently.

## Archived artifacts

The SHA-256 values below cover the exact files presently archived here.  JSON
embedded fingerprints identify normalized calculation inputs and outputs;
ordinary file SHA-256 values identify the serialized artifact bytes.  They are
different identity layers and are not interchangeable.

| Artifact | Bytes | File SHA-256 | Meaning |
|---|---:|---|---|
| `single_carton_motion_audit.json` | 29,432 | `75a5fa6870fcdc3b05900f7137f6d3e5e1a55e77eb7bb1a17cde0759261b4b4c` | Strict CPU motion audit; `BLOCKED`, `FAIL_CLOSED`, reason `INITIAL_STATE_INVALID`; evidence fingerprint `26001900fdc637943b276566e069e639ee0183bad60a818b45c2b69b99fb947c`. |
| `dynamic_execution_preflight.json` | 39,067 | `b202e72529211d7e3a2960cc075dd1d1dd8a09bca825dbb71619abe9b289b4b8` | Final dynamics/replay preflight; execution not ready because of the three blockers listed above; preflight fingerprint `f5071f67ca9895acbf2b965ce4e77850bbf7acbfe741e84c1595c21f2f996810`. |
| `run_summary.json` | 8,157 | `55904f0d251f02ddbc2e1dc83a30039564ec57247e0fe36782b93da454f1d59b` | Reconciles final motion/preflight asset identities; `mutual_identity_status=PASS`; summary fingerprint `fb4bbb7126f944c1d541f9cc5bab10f1b3b6a032130dce00b79db90e461f086d`. |
| `initial_clearance_probe.json` | 39,753 | `f8b19c0aea1f559e528aecada5781dc9d82a7a82e2e080cc37ad322f08e19059` | Read-only home, J6 sweep, complete-tool-mesh clearance, and 256-draw diagnostics. |
| `initialization_contract.json` | 79,395 | `0d8ada6b63c757a3544e9fa743e32d30eb9c175f3106f0de3d4e5e6e0b6b72a9` | Hash-bound initialization-only Isaac contract; contract fingerprint `ab0465e967ffee0d2b2a648e7fe077c8f37285a7be65f0bc1202b00906695933`. |
| `server_environment.txt` | 837 | `2fde35f558d9c1c8ee629e0657ef3b29d3766768b8c53946e19dde7fb75e0308` | Captured CPU, Python/package, Isaac, GPU, driver, isolated-root, branch, and base-commit metadata. |
| `pytest_full.txt` | 606 | `2dbf7ba74ad102a8f1b049e7a4295ea5f036334c2a26b7e1a47060e516a958f8` | Full server regression log: 468 passed, 1 skipped, 4 deselected in 37.62 s. |
| `pytest_targeted.txt` | 272 | `5b4e3cf9ed59af5f2a81a4a4f92a6f35e4720af2b8e3d7cbb7f06d5ea18b9884` | Targeted server regression log: 160 passed, 1 skipped in 10.90 s. |
| `evidence_sha256.txt` | 1,448 | `69b6d1ddc331e4d8cb347e84e2b0ed3f7f745633165ffdbc82bc1f971f7e6c90` | Unified canonical manifest for final CPU and canonical Isaac artifacts; it intentionally does not hash itself or superseded directories. |
| `isaac_initialization/run_status.json` | 14,515 | `8b2618e5a10505c8799855b3f9ccd80202a7298b862bf3ffa5f59a6a2c04a296` | Canonical render-synchronized backend record; scope `INITIALIZATION_ONLY_NOT_PICK_SUCCESS`, classification `RETAINED_AND_SETTLED_DIAGNOSTIC_ONLY`, 180 samples/frames, zero attachments, `physical_pick_success=false`, and adapter SHA-256 `5e1cac6c39eccc94b88ff93ab61f6f1d84dcb1df1e5cc868308291635cb89206`. |
| `isaac_initialization/actual_states.json` | 1,562,955 | `041dfe9f7f8581e8772fc1361f9a728c77364fe904152c13941de3d124662501` | Canonical 180 timestamped actual joint states and poses/velocities for all 40 cartons; source for the settling and tracking metrics above. |
| `isaac_initialization/initial.png` | 705,312 | `d28856018c4d8dbcbfcae24b42989443cae0cd82c49680df8debfae42e36b58c` | Canonical first render-synchronized initialization keyframe. |
| `isaac_initialization/second.png` | 676,165 | `efc0ee7f55d2b936f7e855940972379cecd995840ae7769dd37fa1561d1bf6d8` | Canonical second render-synchronized keyframe. |
| `isaac_initialization/final.png` | 864,174 | `4623dc4b88bf6ff50fcb4c682d026998fe32bbd0c61cb7ea7414b567ade701cc` | Canonical final render-synchronized initialization keyframe; not a placement result. |
| `isaac_initialization/initialization.mp4` | 2,228,965 | `4b3b74431d298179ca272de39a4d1b77957890472286a413929ae3c1f6c99cca` | Canonical continuous 180-frame, normal-time, render-synchronized initialization diagnostic video; its hash also appears in `run_status.json`. |
| `isaac_initialization/isaac_initialization.log` | 40,477 | `9d90465f533c8317f6c39c22a391900487eb34c0e2447439c9ee7dff394fab7c` | Isaac 6.0.1.0 console log for the canonical diagnostic run. |

The canonical `run_status.json` also embeds the complete generated Isaac USD
tree identity: nine relative files, entry point
`m710id_70_official_8/m710id_70_official.usda`, and aggregate SHA-256
`c7001b60ad6e2208a57a2c4d1244c52003702fe531263636542c95984ded3a7f`.
The USD bytes need not be inferred from the video or console log.

Superseded evidence is retained, never deleted or silently overwritten:

- `isaac_initialization_superseded_bad_wall/` used the same normal orientation
  for both trailer wall colliders and produced nonphysical dispersion;
- `isaac_initialization_superseded_postreset_joint_pose/` applied the intended
  joint pose after reset and did not prove the final pre-reset initialization
  contract;
- `isaac_initialization_superseded_stale_rgb_buffer/` did not establish that
  logged state and RGB capture were synchronized;
- `cpu_evidence_superseded_before_tool_coverage_gate/` predates the explicit
  rigid-tool coverage and J6 mounting-contact qualification gates.

Every directory above is audit history and **not canonical evidence**.  No
same-named file there should be substituted for a top-level file or a file
under `isaac_initialization/`.  `evidence_sha256.txt` deliberately identifies
the canonical set only.

The official robot identity inside the evidence is
`FANUC-CORPORATION/fanuc_description@fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`.
The verified manifest SHA-256 is
`adf419aed06c1b1ed8874dea98246ca25795e268fdd71515ff441d400dc72af6`;
the expanded official URDF SHA-256 is
`2a813af47694819c44c888fe54e0041b045355ffb5273b012e3ff995b4999bcf`;
the official SRDF SHA-256 is
`5991658df1a6035019b2f9bf3e0e8eccef3c4da201d472b25783301b13a5812a`.
The supplied Wantai tool collision STL SHA-256 is
`4113746138501efde8748f8c77d82612590fba1b6baca7c7e24127c66b2c5ce8`.

## Runtime

All computation was performed in the isolated server root
`/root/autodl-tmp/m710-official-dynamics-20260910`; the source copy was in its
`repo` subdirectory and the CPU virtual environment in `cpu-venv`.  The capture
records an Intel Xeon Platinum 8470Q host (208 logical CPUs; execution quota was
25 CPUs), CPython 3.12.3, NumPy 2.3.2, Pinocchio 4.1.0, Coal 3.0.3, pytest
9.1.1, Isaac Sim 6.0.1.0, an NVIDIA RTX PRO 6000 Blackwell Server Edition, and
driver 595.71.05.  `server_environment.txt` is authoritative for the captured
environment; no credential is part of this evidence set.

## Reproduction

Run these commands from the isolated server source copy.  They intentionally
retain the fixed layout and strict collision policy.

```bash
RUN_ROOT=/root/autodl-tmp/m710-official-dynamics-20260910
cd "$RUN_ROOT/repo"
export PYTHONPATH=src
RUN_OUTPUT=outputs/m710_official_dynamics_20260910_final_v2

"$RUN_ROOT/cpu-venv/bin/python" tools/run_m710id70_layout_single_carton.py \
  --config configs/validation/m710id70_layout_v1_single_carton.yaml \
  --output "$RUN_OUTPUT/single_carton_motion_audit.json"

"$RUN_ROOT/cpu-venv/bin/python" tools/prepare_m710id70_dynamic_execution.py \
  --config configs/simulation/m710id70_dynamic_execution_v1.yaml \
  --motion-result "$RUN_OUTPUT/single_carton_motion_audit.json" \
  --output "$RUN_OUTPUT/dynamic_execution_preflight.json"

"$RUN_ROOT/cpu-venv/bin/python" tools/probe_m710_layout_initial_state.py \
  --config configs/validation/m710id70_layout_v1_single_carton.yaml \
  --output "$RUN_OUTPUT/initial_clearance_probe.json" \
  --wrist-samples 49 --random-draws 256 --full-tool-mesh

"$RUN_ROOT/cpu-venv/bin/python" tools/prepare_m710_initialization_smoke.py \
  --validation-config configs/validation/m710id70_layout_v1.yaml \
  --dynamics-config configs/simulation/m710id70_official_dynamics_v2.yaml \
  --output "$RUN_OUTPUT/initialization_contract.json"

"$RUN_ROOT/cpu-venv/bin/python" tools/archive_m710id70_dynamic_execution_evidence.py \
  --motion-result "$RUN_OUTPUT/single_carton_motion_audit.json" \
  --preflight "$RUN_OUTPUT/dynamic_execution_preflight.json" \
  --output "$RUN_OUTPUT/run_summary.json" \
  --source-base-head a372c7f117e61509229b2c15eef344cd7908d499 \
  --source-branch feat/v0.5-feasibility-core
```

The optional Isaac initialization diagnostic requires an already installed
Isaac Sim 6.0.1.0 environment and explicit acceptance of the applicable NVIDIA
license terms.  It consumes only the initialization contract and must preserve
the scope label shown above:

```bash
RUN_ROOT=/root/autodl-tmp/m710-official-dynamics-20260910
cd "$RUN_ROOT/repo"
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libgcc_s.so.1 \
OMNI_KIT_ACCEPT_EULA=YES \
XDG_CACHE_HOME="$RUN_ROOT/runtime/cache" \
XDG_CONFIG_HOME="$RUN_ROOT/runtime/config" \
XDG_DATA_HOME="$RUN_ROOT/runtime/data" \
/root/autodl-tmp/envs/isaacsim-clean/bin/python \
  scripts/isaacsim_m710_initialization_smoke.py \
  --contract outputs/m710_official_dynamics_20260910_final_v2/initialization_contract.json \
  --project-root "$RUN_ROOT/repo" \
  --usd-directory "$RUN_ROOT/isaac_usd" \
  --output "$RUN_ROOT/isaac_outputs/initialization_render_sync_logged_final" \
  --seconds 6
```

`OMNI_KIT_ACCEPT_EULA=YES` must only be used when the operator has explicitly
accepted those terms.  It is shown to make the executed process contract
reproducible, not to grant or infer acceptance.

## Evidence intentionally not claimed

`evidence_sha256.txt` is the unified canonical file manifest, and generated USD
tree identity is embedded in canonical `run_status.json`.  The generated USD
byte tree itself is not copied into this lightweight archive.  More
importantly, no complete trajectory, suction-contact set, carton attachment,
loaded transit, receiver-support release, or physical pick video exists because
the three preflight gates stopped execution.  Machine certification evidence
is also outside this engineering-simulation evidence set.  The successful
initialization diagnostic must not replace or overwrite that fail-closed
planning result.
