#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
test "$(cat "$rootfs/min250-20261023/formal-v2.exit")" = 0
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-min250-3f449e8/src PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-min250-3f449e8
cd /min250-20261023
/opt/planning-venv/bin/python /work-min250-3f449e8/tools/verify_m710_population.py formal-v2 --prepare-replay replay-v2
/opt/planning-venv/bin/python summarize-cycles-v2.py
/opt/planning-venv/bin/python failure-diagnostics-v2.py
/opt/planning-venv/bin/python /work-min250-3f449e8/tools/export_m710_top_layer_replay.py replay-v2 --seed 71070 --output analysis-v2/accepted-cycles.html
/opt/planning-venv/bin/python - <<'PY'
from pathlib import Path
import json,hashlib,subprocess
work=Path('/work-min250-3f449e8')
manifest=json.loads((work/'source-manifest.json').read_text())
assert manifest['source_commit']=='3f449e89da518ac89b3c112b4a83eb9a83000b11'
for p,h in manifest['sha256'].items():assert hashlib.sha256((work/p).read_bytes()).hexdigest()==h,p
receipt=json.loads(Path('/min250-20261023/source-receipt-v3.json').read_text())
for p,h in receipt['assets'].items():assert hashlib.sha256((work/p).read_bytes()).hexdigest()==h,p
expected={'/mount-build/m710_moveit_worker':'0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a','/mount-install/lib/libm710_search_trace.so':'1e612f9e38df55bf55ecaee3a7efd0151235e53db553360a1a4d9b00fee95892'}
for p,h in expected.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
versions={}
for line in Path('/ros-packages.lock').read_text().splitlines():
    name,want=line.split('=');got=subprocess.check_output(['dpkg-query','-W','-f=${Version}',name],text=True)
    assert got==want,(name,want,got)
    versions[name]=got
import pinocchio,hppfcl,numpy
report=dict(status='PASS',source_commit=manifest['source_commit'],source_files=len(manifest['sha256']),assets=len(receipt['assets']),native_binaries=expected,ros_versions=versions,pinocchio=pinocchio.__version__,hppfcl=hppfcl.__version__,numpy=numpy.__version__,motion_limit_exception=False)
Path('analysis-v2/source-check.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
PY
INSIDE
python3 "$rootfs/min250-20261023/pack-evidence-v2.py"
tar czf /root/autodl-tmp/m710-l05-x500-20261022/min250-v2-analysis.tar.gz -C "$rootfs/min250-20261023" analysis-v2 formal-v2/summary.json formal-v2/protocol.json formal-v2/verification.json formal-v2/initial-state-check.json focused-tests.log focused-tests-after-repair.log production-check.log formal-v2.log formal-v2.exit
