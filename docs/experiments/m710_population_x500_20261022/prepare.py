from pathlib import Path
import hashlib,io,json,subprocess,tarfile

root=Path.cwd();out=root/'artifacts/m710_population_x500_20261022'
sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
work='work-population-x500-'+sha[:7]
tracked=subprocess.check_output(['git','ls-files','-z']).decode().split('\0')
paths=[p for p in tracked if p and (p.split('/')[0] in ('src','tools','ros2','configs','tests','scripts') or p=='pyproject.toml' or p=='docs/experiments/m710_mount_comparison_20261021/paired-inputs.json')]
data={p:subprocess.check_output(['git','show',sha+':'+p]) for p in paths}
manifest=dict(source_commit=sha,sha256={p:hashlib.sha256(b).hexdigest() for p,b in data.items() if p.split('/')[0] in ('src','tools','ros2','configs')})
data['source-manifest.json']=(json.dumps(manifest,indent=2)+'\n').encode()
previous=root/'docs/experiments/m710_l05_x500_20261022'
receipt=json.loads((previous/'source-receipt.json').read_text())
with tarfile.open(previous/'measured-source.tar.gz') as t:
    for p,h in receipt['assets'].items():
        b=t.extractfile('work/'+p).read();assert hashlib.sha256(b).hexdigest()==h,p
        data[p]=b
package=out/'measured-source.tar.gz'
with tarfile.open(package,'w:gz',compresslevel=6) as t:
    for p,b in sorted(data.items()):
        info=tarfile.TarInfo(work+'/'+p);info.size=len(b);info.mode=0o644
        t.addfile(info,io.BytesIO(b))
(out/'source-manifest.json').write_bytes(data['source-manifest.json'])
(out/'source-receipt.json').write_text(json.dumps(dict(source_commit=sha,source_files=len(manifest['sha256']),assets=receipt['assets'],archive_sha256=hashlib.sha256(package.read_bytes()).hexdigest(),archive_bytes=package.stat().st_size),indent=2)+'\n')
formal=f'''#!/bin/bash
set -eo pipefail
export PYTHONPATH=/{work}/src PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/{work}
test -z "${{LD_PRELOAD:-}}"
out="${{1:-/population-x500-20261022/formal-v1}}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /{work}/tools/plan_m710_xy_top_population.py \\
  --output "$out" --worker-command '/bin/bash /worker.sh' \\
  --worker-binary /mount-build/m710_moveit_worker --source-commit {sha} \\
  --source-manifest /{work}/source-manifest.json --mounting-variant x_minus_500_y_minus_100
'''
(out/'formal-v1.sh').write_text(formal,newline='\n')
deploy=f'''#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/{work}"
test ! -e "$rootfs/population-x500-20261022"
tar xzf "$base/population-x500-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/population-x500-20261022"
cp "$base/population-x500-formal.sh" "$rootfs/population-x500-20261022/formal-v1.sh"
chroot "$rootfs" /bin/bash -c 'cd /{work}; PYTHONPATH=src:tools PYTHONNOUSERSITE=1 /opt/planning-venv/bin/python -m pytest tests/test_mount_comparison.py -q' > "$rootfs/population-x500-20261022/focused-tests.log" 2>&1
test "$(sha256sum "$rootfs/mount-build/m710_moveit_worker" | cut -d' ' -f1)" = 0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a
test "$(sha256sum "$rootfs/mount-install/lib/libm710_search_trace.so" | cut -d' ' -f1)" = 1e612f9e38df55bf55ecaee3a7efd0151235e53db553360a1a4d9b00fee95892
set +e
chroot "$rootfs" /bin/bash /population-x500-20261022/formal-v1.sh > "$rootfs/population-x500-20261022/formal-v1.log" 2>&1
code=$?
printf '%s\\n' "$code" > "$rootfs/population-x500-20261022/formal-v1.exit"
exit "$code"
'''
(out/'deploy-and-run.sh').write_text(deploy,newline='\n')
print(json.dumps(dict(source_commit=sha,work=work,files=len(data),archive_bytes=package.stat().st_size)))
