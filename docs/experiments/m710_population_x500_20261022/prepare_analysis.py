from pathlib import Path
import json

root=Path.cwd();art=root/'artifacts/m710_population_x500_20261022'
sha=json.loads((art/'source-manifest.json').read_text())['source_commit']
work='work-population-x500-'+sha[:7]
previous=root/'docs/experiments/m710_population_x400_20261021'
for name in ['summarize-cycles.py','failure-diagnostics.py','pack-evidence.py']:
    text=(previous/name).read_text(encoding='utf-8')
    text=text.replace('m710-mount-20261021','m710-l05-x500-20261022')
    text=text.replace('population-x400-20261021','population-x500-20261022')
    text=text.replace('work-population-x400-90c8304',work)
    text=text.replace('population-x400-formal-raw','population-x500-formal-raw')
    (art/name).write_text(text,encoding='utf-8',newline='\n')
script=f'''#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
test "$(cat "$rootfs/population-x500-20261022/formal-v1.exit")" = 0
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/{work}/src PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/{work}
cd /population-x500-20261022
/opt/planning-venv/bin/python /{work}/tools/verify_m710_population.py formal-v1 --prepare-replay replay
/opt/planning-venv/bin/python summarize-cycles.py
/opt/planning-venv/bin/python failure-diagnostics.py
/opt/planning-venv/bin/python /{work}/tools/export_m710_top_layer_replay.py replay --seed 71070 --output analysis/accepted-cycles.html
/opt/planning-venv/bin/python - <<'PY'
from pathlib import Path
import json,hashlib,subprocess
work=Path('/{work}')
manifest=json.loads((work/'source-manifest.json').read_text())
assert manifest['source_commit']=='{sha}'
for p,h in manifest['sha256'].items():assert hashlib.sha256((work/p).read_bytes()).hexdigest()==h,p
receipt=json.loads(Path('/population-x500-20261022/source-receipt.json').read_text())
for p,h in receipt['assets'].items():assert hashlib.sha256((work/p).read_bytes()).hexdigest()==h,p
expected={{'/mount-build/m710_moveit_worker':'0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a','/mount-install/lib/libm710_search_trace.so':'1e612f9e38df55bf55ecaee3a7efd0151235e53db553360a1a4d9b00fee95892'}}
for p,h in expected.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
versions={{}}
for line in Path('/ros-packages.lock').read_text().splitlines():
    name,want=line.split('=');got=subprocess.check_output(['dpkg-query','-W','-f=${{Version}}',name],text=True)
    assert got==want,(name,want,got)
    versions[name]=got
import pinocchio,hppfcl,numpy
report=dict(status='PASS',source_commit=manifest['source_commit'],source_files=len(manifest['sha256']),assets=len(receipt['assets']),native_binaries=expected,ros_versions=versions,pinocchio=pinocchio.__version__,hppfcl=hppfcl.__version__,numpy=numpy.__version__,motion_limit_exception=False)
Path('analysis/source-check.json').write_text(json.dumps(report,indent=2)+'\\n')
print(json.dumps(report))
PY
INSIDE
python3 "$rootfs/population-x500-20261022/pack-evidence.py"
tar czf /root/autodl-tmp/m710-l05-x500-20261022/population-x500-analysis.tar.gz -C "$rootfs/population-x500-20261022" analysis formal-v1/summary.json formal-v1/protocol.json formal-v1/verification.json formal-v1/initial-state-check.json focused-tests.log formal-v1.log formal-v1.exit
'''
(art/'analyze.sh').write_text(script,newline='\n')
