set -eu
EXP=/root/autodl-tmp/tesseract-ompl-20260922
cd "$EXP/repo"
unset LD_LIBRARY_PATH
export PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest -q tests/test_tesseract_ompl_v7.py tests/test_tesseract_ompl_v6.py tests/test_tesseract_ompl_task.py tests/test_tesseract_ompl_contract.py tests/test_motion_quality.py tests/test_timing.py tests/test_m710_replay_contract.py tests/test_tesseract_ompl_v5.py > "$EXP/v0_7/tests-directed.log" 2>&1
cat "$EXP/v0_7/tests-directed.log"
