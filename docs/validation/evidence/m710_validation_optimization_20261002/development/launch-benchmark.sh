#!/bin/bash
set -eu
D=/root/autodl-tmp/m710-validation-opt-20261002
cd "$D/repo"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
PY=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
BASE=/root/autodl-tmp/m710-native-cold-20261002/repo/outputs/native-cold-20261002-once
timeout 2400 "$PY" -u tools/benchmark_m710_stage_validation.py --formal-directory "$BASE" --output "$D/benchmark/final-reference.json" --mode reference --counterexample
timeout 2400 "$PY" -u tools/benchmark_m710_stage_validation.py --formal-directory "$BASE" --output "$D/benchmark/final-optimized.json" --mode optimized --counterexample
date --iso-8601=seconds > "$D/benchmark/completed-at.txt"
