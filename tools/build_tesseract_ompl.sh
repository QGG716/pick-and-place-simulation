#!/usr/bin/env bash
# Usage: bash tools/build_tesseract_ompl.sh /absolute/native-prefix /absolute/build-dir
set -euo pipefail
prefix=$(realpath "${1:?supply an isolated Tesseract 0.35.0 conda prefix}")
build_dir=${2:?supply a build directory}
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cmake -S "$repo/native/tesseract_ompl" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$prefix" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$prefix/lib -Wl,-rpath,$prefix/lib -Wl,-rpath-link,$prefix/lib"
cmake --build "$build_dir" -j2
"$build_dir/unloading_tesseract_ompl" </dev/null
