#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-current-c04-c3b815d/src:/work-current-c04-c3b815d/tools PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-current-c04-c3b815d OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cd /current-c04-comparison-v2-20261024
for face in side front; do
test "$(cat "$face.exit")" = 0
/opt/planning-venv/bin/python /work-current-c04-c3b815d/tools/verify_m710_population.py "$face" --prepare-replay "replay-$face" > "$face-verification.log" 2>&1
/opt/planning-venv/bin/python "analyze-$face.py"
done
/opt/planning-venv/bin/python - <<'PY'
import json
from pathlib import Path
from export_m710_top_layer_replay import export
for face in ('side','front'):
    if json.loads(Path(face+'/summary.json').read_text())['completed_count']:
        export(Path('replay-'+face),71070,Path('analysis-'+face+'/accepted-cycle.html'))
PY
INSIDE
tar czf /root/autodl-tmp/m710-l05-x500-20261022/current-c04-v2-analysis.tar.gz -C "$rootfs/current-c04-comparison-v2-20261024" analysis-side analysis-front side/summary.json side/protocol.json side/verification.json side/initial-state-check.json side/predecessor.json front/summary.json front/protocol.json front/verification.json front/initial-state-check.json front/predecessor.json focused-tests.log production-check.log side.log front.log side.exit front.exit side-raw-receipt.json front-raw-receipt.json build.log native-policy-tests.log native-binary-sha256.txt side-verification.log front-verification.log
tar czf /root/autodl-tmp/m710-l05-x500-20261022/current-c04-v1-preflight-error.tar.gz -C "$rootfs/current-c04-comparison-20261024" .
