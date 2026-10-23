#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-layer-top-8770a4a"
test ! -e "$rootfs/layer-top-20261023"
test "$(sha256sum "$base/layer-top-source.tar.gz" | cut -d' ' -f1)" = f50a56c06a0340e6380a41dc36f327d67de4affd511979cb5b964b44285dcc0c
test "$(sha256sum "$rootfs/mount-build/m710_moveit_worker" | cut -d' ' -f1)" = 0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a
test "$(sha256sum "$rootfs/mount-install/lib/libm710_search_trace.so" | cut -d' ' -f1)" = 1e612f9e38df55bf55ecaee3a7efd0151235e53db553360a1a4d9b00fee95892
tar xzf "$base/layer-top-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/layer-top-20261023"
cp "$base/layer-top-formal.sh" "$rootfs/layer-top-20261023/formal-v1.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-layer-top-8770a4a/src:/work-layer-top-8770a4a/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-layer-top-8770a4a
cd /work-layer-top-8770a4a
/opt/planning-venv/bin/python -m pytest -q tests/test_selected_layer_top.py tests/test_low_top_receiver.py tests/test_min250_continuation.py tests/test_junction_reception.py > /layer-top-20261023/focused-tests.log 2>&1
bash /layer-top-20261023/formal-v1.sh /layer-top-20261023/production-check --check-only > /layer-top-20261023/production-check.log 2>&1
/opt/planning-venv/bin/python - <<'PY'
from pathlib import Path
import json
r=json.loads(Path('/layer-top-20261023/production-check/initial-state-check.json').read_text())
print(json.dumps(dict(event='PRODUCTION_INITIAL_CHECK',passed=r['passed'],backend=r['backend'],cartons=len(r['root']['world_cartons']),height=r['root']['height_m'],history=len(r['root']['completed']))),flush=True)
PY
INSIDE
set +e
chroot "$rootfs" /bin/bash /layer-top-20261023/formal-v1.sh > "$rootfs/layer-top-20261023/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/layer-top-20261023/formal-v1.exit"
exit "$code"
