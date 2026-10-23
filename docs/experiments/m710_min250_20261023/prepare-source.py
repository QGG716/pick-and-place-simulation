"""Package the frozen source/assets and verified previous results for GPU transfer."""
from pathlib import Path
import hashlib,io,json,subprocess,tarfile

root=Path.cwd();out=root/'artifacts/m710_min250_20261023'
source=subprocess.check_output(['git','rev-parse','a300d23^{commit}'],text=True).strip()
work='work-min250-'+source[:7]
raw=subprocess.check_output(['git','archive',source,'src','tools','ros2','configs','tests','scripts','pyproject.toml','AGENTS.md',
    'docs/experiments/m710_mount_comparison_20261021/paired-inputs.json'])
with tarfile.open(fileobj=io.BytesIO(raw)) as t:
    data={m.name:t.extractfile(m).read() for m in t.getmembers() if m.isfile()}
manifest=dict(source_commit=source,sha256={p:hashlib.sha256(b).hexdigest() for p,b in data.items()
    if p.split('/')[0] in ('src','tools','ros2','configs')})
data['source-manifest.json']=(json.dumps(manifest,indent=2)+'\n').encode()
previous=root/'docs/experiments/m710_population_x500_20261022'
receipt=json.loads((previous/'source-receipt.json').read_text())
asset_archive=previous/'measured-source.tar.gz'
assert hashlib.sha256(asset_archive.read_bytes()).hexdigest()==receipt['archive_sha256']
with tarfile.open(asset_archive) as t:
    for p,sha in receipt['assets'].items():
        b=t.extractfile('work-population-x500-66604f4/'+p).read()
        assert hashlib.sha256(b).hexdigest()==sha,p
        data[p]=b
source_package=out/'prepared-source-v2.tar.gz'
with tarfile.open(source_package,'w:gz',compresslevel=6) as t:
    for p,b in sorted(data.items()):
        info=tarfile.TarInfo(work+'/'+p);info.size=len(b);info.mode=0o644
        t.addfile(info,io.BytesIO(b))
input_package=out/'previous-inputs-v2.tar.gz'
with tarfile.open(input_package,'w:gz',compresslevel=6) as t:
    t.add(out/'preparation-v1/previous-formal-v1',arcname='min250-20261023/previous-formal-v1')
formal=f'''#!/bin/bash
set -eo pipefail
export PYTHONPATH=/{work}/src PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/{work}
test -z "${{LD_PRELOAD:-}}"
out="${{1:-/min250-20261023/formal-v1}}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /{work}/tools/plan_m710_xy_top_population.py \\
  --output "$out" --worker-command '/bin/bash /worker.sh' \\
  --worker-binary /mount-build/m710_moveit_worker --source-commit {source} \\
  --source-manifest /{work}/source-manifest.json --mounting-variant x_minus_500_y_minus_100 \\
  --continue-min250-from /min250-20261023/previous-formal-v1
'''
(out/'formal-v1.sh').write_text(formal,encoding='utf-8',newline='\n')
result=dict(source_commit=source,status='PREPARED_NOT_DEPLOYED_NOT_NATIVE_PLANNED',source_files=len(manifest['sha256']),
    assets=receipt['assets'],packages={p.name:dict(sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size)
        for p in (source_package,input_package,out/'formal-v1.sh')})
(out/'source-manifest.json').write_bytes(data['source-manifest.json'])
(out/'source-receipt-v2.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(source_commit=source,packages=result['packages'])))
