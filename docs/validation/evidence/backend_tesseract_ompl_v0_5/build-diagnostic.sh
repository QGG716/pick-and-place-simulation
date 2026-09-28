set -eu
EXP=/root/autodl-tmp/tesseract-ompl-20260922
mkdir "$EXP/v0_5"
mkdir "$EXP/v0_5/evidence"
cd "$EXP/repo"
tar -czf "$EXP/v0_5/baseline-sources.tar.gz" native/tesseract_ompl src/unloading_sim/tesseract_ompl_backend.py
sha256sum native/tesseract_ompl/worker.cpp src/unloading_sim/tesseract_ompl_backend.py > "$EXP/v0_5/evidence/server-baseline.txt"
tar -xzf "$EXP/v05-diag-code.tar.gz"
cmake -S native/tesseract_ompl -B "$EXP/v0_5/build-diagnostic" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$EXP/native" -DCMAKE_EXE_LINKER_FLAGS="-L$EXP/native/lib -Wl,-rpath,$EXP/native/lib -Wl,-rpath-link,$EXP/native/lib" -DUNLOADING_NATIVE_DIAGNOSTIC_TESTS=ON > "$EXP/v0_5/evidence/build-diagnostic.log" 2>&1
cmake --build "$EXP/v0_5/build-diagnostic" -j2 >> "$EXP/v0_5/evidence/build-diagnostic.log" 2>&1
"$EXP/v0_5/build-diagnostic/roadmap_diagnostics_test"
