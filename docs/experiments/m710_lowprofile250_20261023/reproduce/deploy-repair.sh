#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-lowprofile-f17d636"
test ! -e "$rootfs/lowprofile250-20261023/formal-v1"
test "$(sha256sum "$base/lowprofile250-source-v2.tar.gz" | cut -d' ' -f1)" = 9d90763ec98a0a74d1e0deb12ec18d7f2ba4984f34257dbab14f3ab9b7b6ac07
tar xzf "$base/lowprofile250-source-v2.tar.gz" -C "$rootfs"
cp "$base/lowprofile250-formal-v2.sh" "$rootfs/lowprofile250-20261023/formal-after-audit-repair.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-lowprofile-f17d636/src:/work-lowprofile-f17d636/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-lowprofile-f17d636
cd /work-lowprofile-f17d636
/opt/planning-venv/bin/python - <<'PY'
import json
from pathlib import Path
a=json.loads(Path('/work-lowprofile-ab491d2/source-manifest.json').read_text())
b=json.loads(Path('/work-lowprofile-f17d636/source-manifest.json').read_text())
native=lambda m:{k:v for k,v in m['sha256'].items() if k.startswith('ros2/')}
assert native(a)==native(b)
Path('/lowprofile250-20261023/native-build-reuse.json').write_text(json.dumps(dict(status='PASS',
 original_compile_source=a['source_commit'],measured_source=b['source_commit'],all_native_source_files_unchanged=True,
 native_source_files=len(native(b)),native_source_hashes=native(b)),indent=2)+'\n')
PY
/opt/planning-venv/bin/python -m pytest -q tests/test_lowprofile250.py tests/test_contact_simulation_policy.py tests/test_selected_layer_top.py tests/test_low_top_receiver.py tests/test_min250_continuation.py > /lowprofile250-20261023/focused-tests-v2.log 2>&1
bash /lowprofile250-20261023/formal-after-audit-repair.sh /lowprofile250-20261023/production-check-v2 --check-only > /lowprofile250-20261023/production-check-v2.log 2>&1
/opt/planning-venv/bin/python - <<'PY'
from pathlib import Path
import json
r=json.loads(Path('/lowprofile250-20261023/production-check-v2/initial-state-check.json').read_text())
print(json.dumps(dict(event='PRODUCTION_INITIAL_CHECK',passed=r['passed'],backend=r['backend'],cartons=len(r['root']['world_cartons']),height=r['root']['height_m'],history=len(r['root']['completed']))),flush=True)
PY
INSIDE
set +e
chroot "$rootfs" /bin/bash /lowprofile250-20261023/formal-after-audit-repair.sh > "$rootfs/lowprofile250-20261023/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/lowprofile250-20261023/formal-v1.exit"
exit "$code"
