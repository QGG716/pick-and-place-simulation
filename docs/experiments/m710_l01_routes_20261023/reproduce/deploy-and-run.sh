#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-l01-routes-6558d70"
test ! -e "$rootfs/l01-routes-20261023"
test "$(sha256sum "$base/l01-routes-source.tar.gz" | cut -d' ' -f1)" = cd3c96bb1a52ab117897818f8e4e1b33b166afc2f279a0f12a132042dfa50f28
test "$(sha256sum "$rootfs/l01-tool-v2-build/m710_moveit_worker" | cut -d' ' -f1)" = 17d9fcf691fe631ede46a6a92adcca20c90a05cf8b7bccee641080d79b604d38
test "$(sha256sum "$rootfs/l01-tool-v2-install/lib/libm710_search_trace.so" | cut -d' ' -f1)" = c095085d3d0fe806d6e8e1c3d35b11f881867d3f4e1d0a1f6ba9dfae44d1646b
tar xzf "$base/l01-routes-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/l01-routes-20261023"
cp "$base/l01-routes-formal.sh" "$rootfs/l01-routes-20261023/formal-v1.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-l01-routes-6558d70/src:/work-l01-routes-6558d70/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-l01-routes-6558d70
cd /work-l01-routes-6558d70
/opt/planning-venv/bin/python - <<'PY'
import json
from pathlib import Path
a=json.loads(Path('/work-l01-tool-e9dfef3/source-manifest.json').read_text())
b=json.loads(Path('/work-l01-routes-6558d70/source-manifest.json').read_text())
native=lambda m:{k:v for k,v in m['sha256'].items() if k.startswith('ros2/')}
assert native(a)==native(b)
Path('/l01-routes-20261023/native-build-reuse.json').write_text(json.dumps(dict(status='PASS',
 original_compile_source=a['source_commit'],measured_source=b['source_commit'],all_native_source_files_unchanged=True,
 native_source_files=len(native(b)),native_source_hashes=native(b)),indent=2)+'\n')
PY
/opt/planning-venv/bin/python -m pytest -q tests/test_l01_selected_routes.py tests/test_l01_tool_chassis.py tests/test_selected_layer_top.py tests/test_low_top_receiver.py > /l01-routes-20261023/focused-tests.log 2>&1
bash /l01-routes-20261023/formal-v1.sh /l01-routes-20261023/production-check --check-only > /l01-routes-20261023/production-check.log 2>&1
cat /l01-routes-20261023/focused-tests.log
INSIDE
set +e
chroot "$rootfs" /bin/bash /l01-routes-20261023/formal-v1.sh > "$rootfs/l01-routes-20261023/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/l01-routes-20261023/formal-v1.exit"
exit "$code"
