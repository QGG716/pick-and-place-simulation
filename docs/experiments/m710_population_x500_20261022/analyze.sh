#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
test "$(cat "$rootfs/population-x500-20261022/formal-v1.exit")" = 0
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-population-x500-66604f4/src PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-population-x500-66604f4
cd /population-x500-20261022
/opt/planning-venv/bin/python /work-population-x500-66604f4/tools/verify_m710_population.py formal-v1 --prepare-replay replay
/opt/planning-venv/bin/python summarize-cycles.py
/opt/planning-venv/bin/python failure-diagnostics.py
/opt/planning-venv/bin/python /work-population-x500-66604f4/tools/export_m710_top_layer_replay.py replay --seed 71070 --output analysis/accepted-cycles.html
/opt/planning-venv/bin/python - <<'PY'
from pathlib import Path
import json,hashlib,subprocess
work=Path('/work-population-x500-66604f4')
manifest=json.loads((work/'source-manifest.json').read_text())
assert manifest['source_commit']=='66604f4fd4a346be8d1947f3d3d44c544a5ad407'
for p,h in manifest['sha256'].items():assert hashlib.sha256((work/p).read_bytes()).hexdigest()==h,p
receipt=json.loads(Path('/population-x500-20261022/source-receipt.json').read_text())
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
Path('analysis/source-check.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
PY
INSIDE
python3 "$rootfs/population-x500-20261022/pack-evidence.py"
tar czf /root/autodl-tmp/m710-l05-x500-20261022/population-x500-analysis.tar.gz -C "$rootfs/population-x500-20261022" analysis formal-v1/summary.json formal-v1/protocol.json formal-v1/verification.json formal-v1/initial-state-check.json focused-tests.log formal-v1.log formal-v1.exit
