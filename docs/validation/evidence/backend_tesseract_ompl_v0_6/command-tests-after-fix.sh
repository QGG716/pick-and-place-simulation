set -eu
EXP=/root/autodl-tmp/tesseract-ompl-20260922
cd "$EXP/repo"
export PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest -q tests/test_tesseract_ompl_v6.py tests/test_tesseract_ompl_task.py tests/test_tesseract_ompl_contract.py tests/test_timing.py tests/test_motion_quality.py::test_collinear_samples_pass_with_nonzero_velocity_and_corners_remain tests/test_motion_quality.py::test_runtime_analytic_reference_and_command_samples_are_bound tests/test_layout_trajectory.py::test_complete_segment_requires_all_contiguous_stages_and_exact_events tests/test_layout_trajectory.py::test_complete_segment_rejects_empty_cup_attachment_and_wrong_actual_q tests/test_layout_trajectory.py::test_complete_segment_rejects_cup_mask_or_nested_actual_pose_tampering tests/test_m710_replay_contract.py tests/test_tesseract_ompl_v5.py -k 'not real_native' > "$EXP/v0_6/tests-corrected.log" 2>&1
cat "$EXP/v0_6/tests-corrected.log"

