set -eu
EXP=/root/autodl-tmp/tesseract-ompl-20260922
cd "$EXP/repo"
unset LD_LIBRARY_PATH
export PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
PY=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
"$PY" -c 'import json; from pathlib import Path; assert json.loads(Path("../v0_7/startup-01/startup.json").read_text())["status"] == "PASS"'
OUT="$EXP/v0_7/task-01"
mkdir "$OUT"
set +e
"$PY" tools/run_tesseract_ompl_task.py \
 --state tests/fixtures/tesseract_ompl/historical_state.json \
 --segment tests/fixtures/tesseract_ompl/historical_segment.json \
 --worker "$EXP/v0_5/build-improved/unloading_tesseract_ompl" \
 --planner-config docs/validation/evidence/backend_tesseract_ompl_v0_6/planner-config.json \
 --expected-first-request "$EXP/v0_5/evidence/improved/native-01.request.json" \
 --execution-config configs/simulation/m710id70_proof_of_concept.yaml \
 --native-max-attempts 1 --stop-on-native-block --profile \
 --output "$OUT/task-result.json" > "$OUT/run.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$OUT/exit-code.txt"
exit "$rc"
