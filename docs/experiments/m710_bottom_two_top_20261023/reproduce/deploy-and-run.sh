#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-bottom-top-4337f91"
test ! -e "$rootfs/bottom-two-top-20261023"
test "$(sha256sum "$base/bottom-top-source.tar.gz" | cut -d' ' -f1)" = 07f7657706b12abc8d4ba29a9a65827b244c9b0e29a3eb878b9a3b3bc442120a
test "$(sha256sum "$rootfs/lowprofile-build/m710_moveit_worker" | cut -d' ' -f1)" = 5968bbfcd3509b514efb2e9ba1d6c1042589c84b80e85f8c79949e71bcd59aa5
test "$(sha256sum "$rootfs/lowprofile-install/lib/libm710_search_trace.so" | cut -d' ' -f1)" = 0b2c2bb91ab9b158b0a64a8a71082d0d1991c149f418d94b63220ba6f6979973
tar xzf "$base/bottom-top-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/bottom-two-top-20261023"
cp "$base/bottom-top-formal.sh" "$rootfs/bottom-two-top-20261023/formal-v1.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-bottom-top-4337f91/src:/work-bottom-top-4337f91/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-bottom-top-4337f91
cd /work-bottom-top-4337f91
/opt/planning-venv/bin/python - <<'PY'
import json
from pathlib import Path
a=json.loads(Path('/work-lowprofile-ab491d2/source-manifest.json').read_text())
b=json.loads(Path('/work-bottom-top-4337f91/source-manifest.json').read_text())
native=lambda m:{k:v for k,v in m['sha256'].items() if k.startswith('ros2/')}
assert native(a)==native(b)
Path('/bottom-two-top-20261023/native-build-reuse.json').write_text(json.dumps(dict(status='PASS',
 original_compile_source=a['source_commit'],measured_source=b['source_commit'],all_native_source_files_unchanged=True,
 native_source_files=len(native(b)),native_source_hashes=native(b)),indent=2)+'\n')
PY
/opt/planning-venv/bin/python -m pytest -q tests/test_bottom_two_top.py tests/test_selected_layer_top.py tests/test_lowprofile250.py tests/test_low_top_receiver.py > /bottom-two-top-20261023/focused-tests.log 2>&1
bash /bottom-two-top-20261023/formal-v1.sh /bottom-two-top-20261023/production-check --check-only > /bottom-two-top-20261023/production-check.log 2>&1
cat /bottom-two-top-20261023/focused-tests.log
INSIDE
set +e
chroot "$rootfs" /bin/bash /bottom-two-top-20261023/formal-v1.sh > "$rootfs/bottom-two-top-20261023/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/bottom-two-top-20261023/formal-v1.exit"
exit "$code"
