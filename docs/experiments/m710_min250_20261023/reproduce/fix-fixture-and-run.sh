#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test "$(sha256sum "$base/min250-paired-inputs.json" | cut -d' ' -f1)" = ff3a742af9fe5835a9321822b8a0f831bfeb903ddb04b921bf34625410212144
mkdir -p "$rootfs/work-min250-a300d23/docs/experiments/m710_mount_comparison_20261021"
test ! -e "$rootfs/work-min250-a300d23/docs/experiments/m710_mount_comparison_20261021/paired-inputs.json"
cp "$base/min250-paired-inputs.json" "$rootfs/work-min250-a300d23/docs/experiments/m710_mount_comparison_20261021/paired-inputs.json"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-min250-a300d23/src:/work-min250-a300d23/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-min250-a300d23
cd /work-min250-a300d23
/opt/planning-venv/bin/python -m pytest tests/test_min250_continuation.py tests/test_low_top_receiver.py tests/test_mount_comparison.py tests/test_seam_coupling.py -k 'not full_discrete_stroke' -q > /min250-20261023/focused-tests-v2.log 2>&1
/opt/planning-venv/bin/python tools/plan_m710_xy_top_population.py --output /min250-20261023/production-check --worker-command '/bin/bash /worker.sh' --worker-binary /mount-build/m710_moveit_worker --source-commit a300d234633e122c6d8328dddb259f3c9c38ad91 --source-manifest /work-min250-a300d23/source-manifest.json --mounting-variant x_minus_500_y_minus_100 --continue-min250-from /min250-20261023/previous-formal-v1 --check-only > /min250-20261023/production-check.log 2>&1
/opt/planning-venv/bin/python - <<'PY'
from pathlib import Path
import json
r=json.loads(Path('/min250-20261023/production-check/initial-state-check.json').read_text())
print(json.dumps(dict(event='PRODUCTION_INITIAL_CHECK',passed=r['passed'],backend=r['backend'],cartons=len(r['root']['world_cartons']),height=r['root']['height_m'],history=len(r['root']['completed']))),flush=True)
PY
INSIDE
set +e
chroot "$rootfs" /bin/bash /min250-20261023/formal-v1.sh > "$rootfs/min250-20261023/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/min250-20261023/formal-v1.exit"
exit "$code"
