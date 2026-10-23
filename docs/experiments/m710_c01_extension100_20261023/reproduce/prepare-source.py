from pathlib import Path
import hashlib,io,json,subprocess,tarfile
root=Path.cwd();out=root/'artifacts/m710_c01_extension100_20261023/v2';out.mkdir(exist_ok=True)
source='2f6e1ea069fea3f04578116f32d2d9388869c528';work='work-c01-ext100-v2-'+source[:7];round_name='c01-ext100-v2-20261023'
paths=['src','tools','ros2','configs','tests','scripts','pyproject.toml','AGENTS.md',
 'docs/experiments/m710_mount_comparison_20261021/paired-inputs.json',
 'docs/experiments/m710_min250_20261023/records/v2/summary.json',
 'docs/experiments/m710_min250_20261023/records/v2/protocol.json',
 'docs/experiments/m710_min250_20261023/records/v2/verification.json',
 'docs/experiments/m710_min250_20261023/raw/formal-v2.tar.xz',
 'docs/experiments/m710_lowprofile250_20261023/records/summary.json',
 'docs/experiments/m710_lowprofile250_20261023/records/protocol.json',
 'docs/experiments/m710_lowprofile250_20261023/records/verification.json',
 'docs/experiments/m710_lowprofile250_20261023/formal-v1-raw.tar.xz',
 'docs/experiments/m710_l01_tool_chassis_20261023/records/summary.json',
 'docs/experiments/m710_l01_tool_chassis_20261023/records/protocol.json',
 'docs/experiments/m710_l01_tool_chassis_20261023/records/verification.json',
 'docs/experiments/m710_l01_tool_chassis_20261023/formal-v1-raw.tar.xz']
raw=subprocess.check_output(['git','archive',source,*paths])
with tarfile.open(fileobj=io.BytesIO(raw)) as t:data={m.name:t.extractfile(m).read() for m in t if m.isfile()}
manifest=dict(source_commit=source,sha256={n:hashlib.sha256(b).hexdigest() for n,b in data.items() if n.split('/')[0] in ('src','tools','ros2','configs')})
receipt=json.loads((root/'artifacts/m710_min250_20261023/source-receipt-v3.json').read_text())
archive=root/'artifacts/m710_min250_20261023/prepared-source-v3.tar.gz'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==receipt['packages']['prepared-source-v3.tar.gz']['sha256']
with tarfile.open(archive) as t:
 for n,sha in receipt['assets'].items():
  b=t.extractfile('work-min250-3f449e8/'+n).read();assert hashlib.sha256(b).hexdigest()==sha;data[n]=b
data['source-manifest.json']=(json.dumps(manifest,indent=2)+'\n').encode()
package=out/'measured-source.tar.gz'
with tarfile.open(package,'w:gz',compresslevel=6) as t:
 for name,b in sorted(data.items()):
  info=tarfile.TarInfo(work+'/'+name);info.size=len(b);info.mode=0o644;t.addfile(info,io.BytesIO(b))
sha=hashlib.sha256(package.read_bytes()).hexdigest()
formal=f'''#!/bin/bash
set -euo pipefail
export PYTHONPATH=/{work}/src:/{work}/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/{work}
test -z "${{LD_PRELOAD:-}}"
out="${{1:-/{round_name}/formal-v1}}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /{work}/tools/plan_m710_xy_top_population.py \\
 --output "$out" --worker-command '/bin/bash /l01-tool-v2-worker.sh' --worker-binary /l01-tool-v2-build/m710_moveit_worker \\
 --source-commit {source} --source-manifest /{work}/source-manifest.json \\
 --mounting-variant x_minus_500_y_minus_100 --continue-l01-selected-routes-from /l01-tool-chassis-v2-20261023/formal-v1 --c01-finite-side-channel --tool-connector-extension-m 0.1 \\
 "${{@:2}}"
'''
(out/'formal-v1.sh').write_text(formal,encoding='utf-8',newline='\n')
deploy=f'''#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/{work}"
test ! -e "$rootfs/{round_name}"
test "$(sha256sum "$base/c01-ext100-v2-source.tar.gz" | cut -d' ' -f1)" = {sha}
test "$(sha256sum "$rootfs/l01-tool-v2-build/m710_moveit_worker" | cut -d' ' -f1)" = 17d9fcf691fe631ede46a6a92adcca20c90a05cf8b7bccee641080d79b604d38
test "$(sha256sum "$rootfs/l01-tool-v2-install/lib/libm710_search_trace.so" | cut -d' ' -f1)" = c095085d3d0fe806d6e8e1c3d35b11f881867d3f4e1d0a1f6ba9dfae44d1646b
tar xzf "$base/c01-ext100-v2-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/{round_name}"
cp "$base/c01-ext100-v2-formal.sh" "$rootfs/{round_name}/formal-v1.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/{work}/src:/{work}/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/{work}
cd /{work}
/opt/planning-venv/bin/python - <<'PY'
import json
from pathlib import Path
a=json.loads(Path('/work-l01-tool-e9dfef3/source-manifest.json').read_text())
b=json.loads(Path('/{work}/source-manifest.json').read_text())
native=lambda m:{{k:v for k,v in m['sha256'].items() if k.startswith('ros2/')}}
assert native(a)==native(b)
Path('/{round_name}/native-build-reuse.json').write_text(json.dumps(dict(status='PASS',
 original_compile_source=a['source_commit'],measured_source=b['source_commit'],all_native_source_files_unchanged=True,
 native_source_files=len(native(b)),native_source_hashes=native(b)),indent=2)+'\\n')
PY
/opt/planning-venv/bin/python -m pytest -q tests/test_tool_connector_extension.py tests/test_l01_selected_routes.py tests/test_l01_tool_chassis.py tests/test_right_receiver.py tests/test_low_top_receiver.py > /{round_name}/focused-tests.log 2>&1
bash /{round_name}/formal-v1.sh /{round_name}/production-check --check-only > /{round_name}/production-check.log 2>&1
cat /{round_name}/focused-tests.log
INSIDE
set +e
chroot "$rootfs" /bin/bash /{round_name}/formal-v1.sh > "$rootfs/{round_name}/formal-v1.log" 2>&1
code=$?
printf '%s\\n' "$code" > "$rootfs/{round_name}/formal-v1.exit"
exit "$code"
'''
(out/'deploy-and-run.sh').write_text(deploy,encoding='utf-8',newline='\n')
(out/'source-manifest.json').write_bytes(data['source-manifest.json'])
(out/'source-receipt.json').write_text(json.dumps(dict(source_commit=source,source_files=len(manifest['sha256']),
 assets=receipt['assets'],archive_sha256=sha,archive_bytes=package.stat().st_size),indent=2)+'\n',encoding='utf-8',newline='\n')
print(json.dumps(dict(source=source,work=work,archive_bytes=package.stat().st_size,sha256=sha)))
