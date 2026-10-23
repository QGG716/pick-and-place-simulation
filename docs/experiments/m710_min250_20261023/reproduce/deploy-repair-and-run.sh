#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-min250-3f449e8"
test ! -e "$rootfs/min250-20261023/formal-v2"
test "$(sha256sum "$base/min250-prepared-source-v3.tar.gz" | cut -d' ' -f1)" = 2c0d399a1c266cde5297674c6e072a6fd99cd540aa639481935bd58817395547
tar xzf "$base/min250-prepared-source-v3.tar.gz" -C "$rootfs"
cp "$base/min250-formal-v2.sh" "$rootfs/min250-20261023/formal-v2.sh"
cp "$base/min250-source-receipt-v3.json" "$rootfs/min250-20261023/source-receipt-v3.json"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-min250-3f449e8/src:/work-min250-3f449e8/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-min250-3f449e8
cd /work-min250-3f449e8
/opt/planning-venv/bin/python -m pytest tests/test_min250_continuation.py tests/test_junction_reception.py tests/test_coupled_batch.py -q > /min250-20261023/focused-tests-after-repair.log 2>&1
INSIDE
set +e
chroot "$rootfs" /bin/bash /min250-20261023/formal-v2.sh > "$rootfs/min250-20261023/formal-v2.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/min250-20261023/formal-v2.exit"
exit "$code"
