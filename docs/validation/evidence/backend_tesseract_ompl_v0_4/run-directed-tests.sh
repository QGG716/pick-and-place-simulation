set -eu
EXP=/root/autodl-tmp/tesseract-ompl-20260922
cd "$EXP/repo"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=src
export UNLOADING_TESSERACT_WORKER="$EXP/v0_3/build/unloading_tesseract_ompl"
export UNLOADING_LAZY_TEST_EVIDENCE="$EXP/v0_4/evidence/native-configuration-contract.json"
PY=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
"$PY" -m pytest -q tests/test_tesseract_ompl_contract.py tests/test_tesseract_ompl_task.py > "$EXP/v0_4/evidence/final-contract-tests.txt" 2>&1
cat "$EXP/v0_4/evidence/final-contract-tests.txt"
"$PY" -m pytest -q tests/test_tesseract_ompl_lazy.py::test_actual_default_and_explicit_planners > "$EXP/v0_4/evidence/native-configuration-tests.txt" 2>&1
cat "$EXP/v0_4/evidence/native-configuration-tests.txt"
"$PY" -m pytest -q tests/test_layout_trajectory.py::test_complete_segment_requires_all_contiguous_stages_and_exact_events tests/test_layout_trajectory.py::test_complete_segment_rejects_empty_cup_attachment_and_wrong_actual_q tests/test_m710_replay_contract.py::test_ready_preflight_contract_and_bundle_round_trip tests/test_m710_replay_contract.py::test_blocked_preflight_has_no_exportable_trajectory tests/test_m710_replay_contract.py::test_preflight_rejects_plan_config_scene_or_trajectory_tampering_even_if_refingerprinted tests/test_m710_replay_contract.py::test_random_hash_and_self_reported_ready_cannot_forge_preflight tests/test_isaac_bridge.py::test_build_fanuc_replay_is_deterministic_and_limit_audited tests/test_isaac_bridge.py::test_m710_release_retreat_time_includes_every_inserted_hold tests/test_motion_quality.py::test_collinear_samples_pass_with_nonzero_velocity_and_corners_remain > "$EXP/v0_4/evidence/stage-export-execution-contract-tests.txt" 2>&1
cat "$EXP/v0_4/evidence/stage-export-execution-contract-tests.txt"
sha256sum src/unloading_sim/tesseract_ompl_backend.py tools/run_tesseract_ompl_task.py tests/test_tesseract_ompl_contract.py tests/test_tesseract_ompl_task.py "$UNLOADING_TESSERACT_WORKER" > "$EXP/v0_4/evidence/final-tested-sources.txt"
