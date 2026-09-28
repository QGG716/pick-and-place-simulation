#!/bin/bash
set -eu
EXP=/root/autodl-tmp/tesseract-ompl-20260922
cd "$EXP/repo"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=src
tar -czf "$EXP/v0_5/evidence/improved-sources.tar.gz" native/tesseract_ompl src/unloading_sim/tesseract_ompl_backend.py src/unloading_sim/tesseract_ompl_config.py tools/run_tesseract_ompl_v5_request.py tests/test_tesseract_ompl_v5.py
sha256sum "$EXP/v0_5/build-diagnostic/unloading_tesseract_ompl" "$EXP/v0_5/build-improved/unloading_tesseract_ompl" > "$EXP/v0_5/evidence/worker-sha256.txt"
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python tools/run_tesseract_ompl_v5_request.py --worker "$EXP/v0_5/build-improved/unloading_tesseract_ompl" --planner-config "$EXP/v0_5/evidence/improvement-config.json" --output "$EXP/v0_5/evidence/improved" > "$EXP/v0_5/evidence/improved.log" 2>&1
cat "$EXP/v0_5/evidence/improved.log"
