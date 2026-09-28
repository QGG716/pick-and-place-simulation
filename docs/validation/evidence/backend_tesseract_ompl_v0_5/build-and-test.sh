set -eu
EXP=/root/autodl-tmp/tesseract-ompl-20260922
cd "$EXP/repo"
tar -xzf "$EXP/v05-improved-code.tar.gz"
cmake -S native/tesseract_ompl -B "$EXP/v0_5/build-improved" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$EXP/native" -DCMAKE_EXE_LINKER_FLAGS="-L$EXP/native/lib -Wl,-rpath,$EXP/native/lib -Wl,-rpath-link,$EXP/native/lib" -DUNLOADING_NATIVE_DIAGNOSTIC_TESTS=ON > "$EXP/v0_5/evidence/build-improved.log" 2>&1
cmake --build "$EXP/v0_5/build-improved" -j2 >> "$EXP/v0_5/evidence/build-improved.log" 2>&1
"$EXP/v0_5/build-improved/roadmap_diagnostics_test" > "$EXP/v0_5/evidence/graph-sampler-tests.txt" 2>&1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=src
export UNLOADING_TESSERACT_WORKER="$EXP/v0_5/build-improved/unloading_tesseract_ompl"
export UNLOADING_V5_TEST_EVIDENCE="$EXP/v0_5/evidence/v5-native-tests.json"
/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest -q tests/test_tesseract_ompl_contract.py tests/test_tesseract_ompl_task.py tests/test_tesseract_ompl_v5.py > "$EXP/v0_5/evidence/directed-tests.txt" 2>&1
cat "$EXP/v0_5/evidence/directed-tests.txt"
