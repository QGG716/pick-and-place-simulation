# Fifth-carton recovery evidence

The raw actual state, full search logs and video remain on the GPU server. The CPU full task passed. The single archive-reconstructed Isaac trial grasped c04 and stopped before release on a measured clearance shortfall; no physical cycle completion is claimed.

- `input_provenance.json`: immutable raw-state, old snapshot and failed-motion correspondence.
- `source_manifest.json` / `source_verification.json`: exact Git bytes for the V3 CPU and execution implementation.
- `focused_tests.txt`: the 70-test changed-area run on the existing server CPU environment.
- `geometry_regression_before.txt` / `context_mutation_reproduction.txt`: failing minimal tests and the production connection's observed rotation mutation before the fix. Trailing whitespace in the copied pytest text is trimmed; raw logs remain on the server.
- `fixed_rejections.json` / `fixed_replay.json`: eight representative V3 rejection groups and production replays, with optional source-CAD checks. These are rejected sampled configurations, not proof that the complete task is impossible.
- `old_post_rrt_recheck.json`: a V2 successful RRT path rejected by mandatory full-edge checking; its original implementation remains frozen on the server.
- `superseded_search_coverage.json`: actual coverage of the four searches cancelled after the input-mutation defect was confirmed. Counts include repeated/cached queries. Candidate generation is not candidate search.
- `fixed_transit_rejections.json` / `fixed_transit_replay.json`: the corrected implementation's first direct transit clearance rejection, replayed identically; it does not establish the minimum clearance of the whole edge.
- `runtime_source_manifest.json` / `runtime_source_verification.json`: V4 execution-source identity, plus the exact asset-bound analysis-script bytes. Its CPU source identity remains equal to V3.
- `archive_runtime_tests.txt` / `offline_archive_initialization.json`: 53 focused tests and the real archive's offline initialization check. No SimulationApp was started.
- `final_cpu_search_summary.json` / `cpu_delivery_status.json`: complete task coverage and independent preflight/export/readback. The delivery status is the immutable pre-Isaac record.
- `physical_trial_summary.json` / `physical_execution_events.json`: the single trial, distinct old/new world identities, new-target counts, first rejection with q/body pose/pair/distance, source hashes, and offline checks. Historical c00 outfeed is not c04 completion.
- `initial_archive_binding.json`: measured post-settling state accepted by the existing binding gate.
- `video_readback.json`: all 824 frames decoded, 640x360 at 5 fps, file hash and server path; no new SimulationApp.
- `replay_physical_rejection.py`: production pair-classifier replay plus saved-pose OBB distance check. This is an offline fixed-state check, not another PhysX trial or a full-state acceptance claim. Run on the existing server CPU environment with `PYTHONPATH=src python docs/evidence/m710_poc_fifth_carton_recovery_step3/replay_physical_rejection.py`.
- `physical_rejection_replay.json`: successful output of that command on the existing server CPU environment.

Replay the V3 examples from the frozen server directory:

```sh
cd /root/autodl-tmp/m710-poc-fifth-step3-v3-20260917
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python \
  tools/replay_m710_search_rejections.py \
  --config configs/validation/m710id70_fifth_step3_deep.yaml \
  --actual-state inputs/original_actual_remaining_state.json \
  --diagnostics outputs/review_evidence/fixed_rejections.json \
  --output outputs/replayed_review_examples.json --limit 8
```

The isolated search config retains the POC base rules and overrides only finite numerical quotas/seeding: `continuation_stage_connection_iterations=54000`, `local_transit_cartesian_sample_budget=720`, `ik.seed=175799`. It has no business wall-clock deadline. Default branch budgets remain unchanged.

The transit sample can also be replayed in the V4 directory with the same config and raw state, using `outputs/fixed_transit_checkpoint.json` as diagnostics and `--limit 1`. Execution-only initialization changes do not change the CPU validator's identity.
