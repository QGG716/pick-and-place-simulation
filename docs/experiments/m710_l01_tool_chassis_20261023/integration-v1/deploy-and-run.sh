#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-l01-tool-57e0957"
test ! -e "$rootfs/l01-tool-chassis-20261023"
test "$(sha256sum "$base/l01-tool-source.tar.gz" | cut -d' ' -f1)" = 0c13b5bd8684049e105d8683d9990a2b4a9580cae02e95775d0161c51602fdd1
test "$(sha256sum "$rootfs/mount-build/m710_moveit_worker" | cut -d' ' -f1)" = 0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a
test "$(sha256sum "$rootfs/mount-install/lib/libm710_search_trace.so" | cut -d' ' -f1)" = 1e612f9e38df55bf55ecaee3a7efd0151235e53db553360a1a4d9b00fee95892
tar xzf "$base/l01-tool-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/l01-tool-chassis-20261023"
cp "$base/l01-tool-formal.sh" "$rootfs/l01-tool-chassis-20261023/formal-v1.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-l01-tool-57e0957/src:/work-l01-tool-57e0957/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-l01-tool-57e0957
cd /work-l01-tool-57e0957
set +u
source /opt/ros/humble/setup.bash
set -u
cmake -S /work-l01-tool-57e0957/ros2/m710_moveit_backend -B /l01-tool-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/l01-tool-install -DPython3_EXECUTABLE=/usr/bin/python3 > /l01-tool-chassis-20261023/build.log 2>&1
cmake --build /l01-tool-build --target m710_moveit_worker m710_process_policy_test -j2 >> /l01-tool-chassis-20261023/build.log 2>&1
cmake --install /l01-tool-build >> /l01-tool-chassis-20261023/build.log 2>&1
export AMENT_PREFIX_PATH=/l01-tool-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/l01-tool-install/lib:/l01-tool-build:${LD_LIBRARY_PATH:-}
/l01-tool-build/m710_process_policy_test > /l01-tool-chassis-20261023/native-policy-tests.log 2>&1
/l01-tool-build/m710_process_policy_test optimized >> /l01-tool-chassis-20261023/native-policy-tests.log 2>&1
sha256sum /l01-tool-build/m710_moveit_worker /l01-tool-install/lib/libm710_search_trace.so > /l01-tool-chassis-20261023/native-binary-sha256.txt
cat > /l01-tool-worker.sh <<'WORKER'
#!/bin/bash
set -eo pipefail
test -z "${LD_PRELOAD:-}"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/l01-tool-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/l01-tool-install/lib:/l01-tool-build:${LD_LIBRARY_PATH:-}
exec /l01-tool-build/m710_moveit_worker
WORKER
/opt/planning-venv/bin/python -m pytest -q tests/test_l01_tool_chassis.py tests/test_bottom_two_top.py tests/test_lowprofile250.py tests/test_contact_simulation_policy.py > /l01-tool-chassis-20261023/focused-tests.log 2>&1
bash /l01-tool-chassis-20261023/formal-v1.sh /l01-tool-chassis-20261023/production-check --check-only > /l01-tool-chassis-20261023/production-check.log 2>&1
/opt/planning-venv/bin/python - <<'PY'
from pathlib import Path
import json
r=json.loads(Path('/l01-tool-chassis-20261023/production-check/initial-state-check.json').read_text())
print(json.dumps(dict(event='PRODUCTION_INITIAL_CHECK',passed=r['passed'],backend=r['backend'],cartons=len(r['root']['world_cartons']),height=r['root']['height_m'],history=len(r['root']['completed']))),flush=True)
PY
INSIDE
set +e
chroot "$rootfs" /bin/bash /l01-tool-chassis-20261023/formal-v1.sh > "$rootfs/l01-tool-chassis-20261023/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/l01-tool-chassis-20261023/formal-v1.exit"
exit "$code"
