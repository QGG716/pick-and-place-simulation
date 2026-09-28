#!/bin/bash
set -u
EXP=/root/autodl-tmp/tesseract-ompl-20260922
cd "$EXP/repo" || exit 90
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=src
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python tools/run_tesseract_ompl_task.py \
  --state tests/fixtures/tesseract_ompl/historical_state.json \
  --segment tests/fixtures/tesseract_ompl/historical_segment.json \
  --worker "$EXP/v0_3/build/unloading_tesseract_ompl" \
  --ompl-planner lazy_prm --native-max-attempts 1 --stop-on-native-block --profile \
  --output "$EXP/v0_4/evidence/task-result.json"
code=$?
printf '%s\n' "$code" > "$EXP/v0_4/evidence/task.exit"
exit "$code"
