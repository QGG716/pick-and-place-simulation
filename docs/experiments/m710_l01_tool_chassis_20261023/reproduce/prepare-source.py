from pathlib import Path
import hashlib,io,json,subprocess,tarfile
root=Path.cwd();out=root/'artifacts/m710_l01_tool_chassis_20261023';out.mkdir(exist_ok=True)
source=subprocess.check_output(['git','rev-parse','e9dfef3215b55bb9905ed0671d43ca4b9716a71a^{commit}'],text=True).strip();work='work-l01-tool-'+source[:7]
paths=['src','tools','ros2','configs','tests','scripts','pyproject.toml','AGENTS.md',
 'docs/experiments/m710_mount_comparison_20261021/paired-inputs.json',
 'docs/experiments/m710_min250_20261023/records/v2/summary.json',
 'docs/experiments/m710_min250_20261023/records/v2/protocol.json',
 'docs/experiments/m710_min250_20261023/records/v2/verification.json',
 'docs/experiments/m710_min250_20261023/raw/formal-v2.tar.xz',
 'docs/experiments/m710_lowprofile250_20261023/records/summary.json',
 'docs/experiments/m710_lowprofile250_20261023/records/protocol.json',
 'docs/experiments/m710_lowprofile250_20261023/records/verification.json',
 'docs/experiments/m710_lowprofile250_20261023/formal-v1-raw.tar.xz']
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
formal=f'''#!/bin/bash
set -euo pipefail
export PYTHONPATH=/{work}/src:/{work}/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/{work}
test -z "${{LD_PRELOAD:-}}"
out="${{1:-/l01-tool-chassis-v2-20261023/formal-v1}}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /{work}/tools/plan_m710_xy_top_population.py \\
 --output "$out" --worker-command '/bin/bash /l01-tool-v2-worker.sh' --worker-binary /l01-tool-v2-build/m710_moveit_worker \\
 --source-commit {source} --source-manifest /{work}/source-manifest.json \\
 --mounting-variant x_minus_500_y_minus_100 --continue-l01-tool-chassis-from /lowprofile250-20261023/formal-v1 \\
 "${{@:2}}"
'''
(out/'formal-v1.sh').write_text(formal,encoding='utf-8',newline='\n')
sha=hashlib.sha256(package.read_bytes()).hexdigest()
deploy=f'''#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/{work}"
test ! -e "$rootfs/l01-tool-chassis-v2-20261023"
test "$(sha256sum "$base/l01-tool-source.tar.gz" | cut -d' ' -f1)" = {sha}
test "$(sha256sum "$rootfs/mount-build/m710_moveit_worker" | cut -d' ' -f1)" = 0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a
test "$(sha256sum "$rootfs/mount-install/lib/libm710_search_trace.so" | cut -d' ' -f1)" = 1e612f9e38df55bf55ecaee3a7efd0151235e53db553360a1a4d9b00fee95892
tar xzf "$base/l01-tool-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/l01-tool-chassis-v2-20261023"
cp "$base/l01-tool-formal.sh" "$rootfs/l01-tool-chassis-v2-20261023/formal-v1.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/{work}/src:/{work}/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/{work}
cd /{work}
set +u
source /opt/ros/humble/setup.bash
set -u
cmake -S /{work}/ros2/m710_moveit_backend -B /l01-tool-v2-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/l01-tool-v2-install -DPython3_EXECUTABLE=/usr/bin/python3 > /l01-tool-chassis-v2-20261023/build.log 2>&1
cmake --build /l01-tool-v2-build --target m710_moveit_worker m710_process_policy_test -j2 >> /l01-tool-chassis-v2-20261023/build.log 2>&1
cmake --install /l01-tool-v2-build >> /l01-tool-chassis-v2-20261023/build.log 2>&1
export AMENT_PREFIX_PATH=/l01-tool-v2-install:${{AMENT_PREFIX_PATH:-}}
export LD_LIBRARY_PATH=/l01-tool-v2-install/lib:/l01-tool-v2-build:${{LD_LIBRARY_PATH:-}}
/l01-tool-v2-build/m710_process_policy_test > /l01-tool-chassis-v2-20261023/native-policy-tests.log 2>&1
/l01-tool-v2-build/m710_process_policy_test optimized >> /l01-tool-chassis-v2-20261023/native-policy-tests.log 2>&1
sha256sum /l01-tool-v2-build/m710_moveit_worker /l01-tool-v2-install/lib/libm710_search_trace.so > /l01-tool-chassis-v2-20261023/native-binary-sha256.txt
cat > /l01-tool-v2-worker.sh <<'WORKER'
#!/bin/bash
set -eo pipefail
test -z "${{LD_PRELOAD:-}}"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/l01-tool-v2-install:${{AMENT_PREFIX_PATH:-}}
export LD_LIBRARY_PATH=/l01-tool-v2-install/lib:/l01-tool-v2-build:${{LD_LIBRARY_PATH:-}}
exec /l01-tool-v2-build/m710_moveit_worker
WORKER
/opt/planning-venv/bin/python -m pytest -q tests/test_l01_tool_chassis.py tests/test_bottom_two_top.py tests/test_lowprofile250.py tests/test_contact_simulation_policy.py > /l01-tool-chassis-v2-20261023/focused-tests.log 2>&1
bash /l01-tool-chassis-v2-20261023/formal-v1.sh /l01-tool-chassis-v2-20261023/production-check --check-only > /l01-tool-chassis-v2-20261023/production-check.log 2>&1
/opt/planning-venv/bin/python - <<'PY'
from pathlib import Path
import json
r=json.loads(Path('/l01-tool-chassis-v2-20261023/production-check/initial-state-check.json').read_text())
print(json.dumps(dict(event='PRODUCTION_INITIAL_CHECK',passed=r['passed'],backend=r['backend'],cartons=len(r['root']['world_cartons']),height=r['root']['height_m'],history=len(r['root']['completed']))),flush=True)
PY
INSIDE
set +e
chroot "$rootfs" /bin/bash /l01-tool-chassis-v2-20261023/formal-v1.sh > "$rootfs/l01-tool-chassis-v2-20261023/formal-v1.log" 2>&1
code=$?
printf '%s\\n' "$code" > "$rootfs/l01-tool-chassis-v2-20261023/formal-v1.exit"
exit "$code"
'''
(out/'deploy-and-run.sh').write_text(deploy,encoding='utf-8',newline='\n')
(out/'source-manifest.json').write_bytes(data['source-manifest.json'])
(out/'source-receipt.json').write_text(json.dumps(dict(source_commit=source,source_files=len(manifest['sha256']),
 assets=receipt['assets'],archive_sha256=sha,archive_bytes=package.stat().st_size),indent=2)+'\n')
print(json.dumps(dict(source=source,work=work,archive_bytes=package.stat().st_size,sha256=sha)))
