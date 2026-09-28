set -eu
EXP=/root/autodl-tmp/tesseract-ompl-20260922
cd "$EXP/repo"
mkdir -p docs/validation/evidence/backend_tesseract_ompl_v0_4/task-result-native
cp -n "$EXP/v0_4/evidence/task-result-native/native-01.request.json" docs/validation/evidence/backend_tesseract_ompl_v0_4/task-result-native/native-01.request.json
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=src
"$EXP/v0_5/build-diagnostic/roadmap_diagnostics_test" > "$EXP/v0_5/evidence/graph-tests-diagnostic.txt" 2>&1
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest -q tests/test_tesseract_ompl_contract.py tests/test_tesseract_ompl_task.py > "$EXP/v0_5/evidence/contract-tests-diagnostic.txt" 2>&1
cat "$EXP/v0_5/evidence/contract-tests-diagnostic.txt"
tar -czf "$EXP/v0_5/evidence/diagnostic-sources.tar.gz" native/tesseract_ompl src/unloading_sim/tesseract_ompl_backend.py tools/run_tesseract_ompl_v5_request.py
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python tools/run_tesseract_ompl_v5_request.py --worker "$EXP/v0_5/build-diagnostic/unloading_tesseract_ompl" --output "$EXP/v0_5/evidence/diagnostic" > "$EXP/v0_5/evidence/diagnostic.log" 2>&1
cat "$EXP/v0_5/evidence/diagnostic.log"
