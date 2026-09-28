set -u
EXP=/root/autodl-tmp/tesseract-ompl-20260922
PY=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
unset LD_LIBRARY_PATH
tar xf "$EXP/v0_7/review-parent.tar" -C "$EXP/v0_7/review-parent"
cd "$EXP/v0_7/review-parent"
PYTHONPATH=src "$PY" -m pytest -q tests/test_motion_quality.py::test_reuse_tool_rejects_changed_non_target_obstacle_and_current_release_policy > "$EXP/v0_7/fixture-parent.log" 2>&1
printf '%s\n' "$?" > "$EXP/v0_7/fixture-parent.exit-code.txt"
cd "$EXP/repo"
PYTHONPATH=src "$PY" -m pytest -q tests/test_motion_quality.py::test_reuse_tool_rejects_changed_non_target_obstacle_and_current_release_policy > "$EXP/v0_7/fixture-review-base.log" 2>&1
printf '%s\n' "$?" > "$EXP/v0_7/fixture-review-base.exit-code.txt"
tail -n 8 "$EXP/v0_7/fixture-parent.log"
tail -n 8 "$EXP/v0_7/fixture-review-base.log"
