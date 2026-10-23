#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-lowprofile-f17d636"
test ! -e "$rootfs/lowprofile250-20261023"
test "$(sha256sum "$base/lowprofile250-source.tar.gz" | cut -d' ' -f1)" = 9d90763ec98a0a74d1e0deb12ec18d7f2ba4984f34257dbab14f3ab9b7b6ac07
test "$(sha256sum "$rootfs/mount-build/m710_moveit_worker" | cut -d' ' -f1)" = 0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a
test "$(sha256sum "$rootfs/mount-install/lib/libm710_search_trace.so" | cut -d' ' -f1)" = 1e612f9e38df55bf55ecaee3a7efd0151235e53db553360a1a4d9b00fee95892
tar xzf "$base/lowprofile250-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/lowprofile250-20261023"
cp "$base/lowprofile250-formal.sh" "$rootfs/lowprofile250-20261023/formal-v1.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-lowprofile-f17d636/src:/work-lowprofile-f17d636/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-lowprofile-f17d636
cd /work-lowprofile-f17d636
set +u
source /opt/ros/humble/setup.bash
set -u
cmake -S /work-lowprofile-f17d636/ros2/m710_moveit_backend -B /lowprofile-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/lowprofile-install -DPython3_EXECUTABLE=/usr/bin/python3 > /lowprofile250-20261023/build.log 2>&1
cmake --build /lowprofile-build --target m710_moveit_worker m710_process_policy_test -j2 >> /lowprofile250-20261023/build.log 2>&1
cmake --install /lowprofile-build >> /lowprofile250-20261023/build.log 2>&1
export AMENT_PREFIX_PATH=/lowprofile-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/lowprofile-install/lib:/lowprofile-build:${LD_LIBRARY_PATH:-}
/lowprofile-build/m710_process_policy_test > /lowprofile250-20261023/native-policy-tests.log 2>&1
/lowprofile-build/m710_process_policy_test optimized >> /lowprofile250-20261023/native-policy-tests.log 2>&1
sha256sum /lowprofile-build/m710_moveit_worker /lowprofile-install/lib/libm710_search_trace.so > /lowprofile250-20261023/native-binary-sha256.txt
cat > /lowprofile-worker.sh <<'WORKER'
#!/bin/bash
set -eo pipefail
test -z "${LD_PRELOAD:-}"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/lowprofile-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/lowprofile-install/lib:/lowprofile-build:${LD_LIBRARY_PATH:-}
exec /lowprofile-build/m710_moveit_worker
WORKER
/opt/planning-venv/bin/python -m pytest -q tests/test_lowprofile250.py tests/test_contact_simulation_policy.py tests/test_selected_layer_top.py tests/test_low_top_receiver.py tests/test_min250_continuation.py > /lowprofile250-20261023/focused-tests.log 2>&1
bash /lowprofile250-20261023/formal-v1.sh /lowprofile250-20261023/production-check --check-only > /lowprofile250-20261023/production-check.log 2>&1
/opt/planning-venv/bin/python - <<'PY'
from pathlib import Path
import json
r=json.loads(Path('/lowprofile250-20261023/production-check/initial-state-check.json').read_text())
print(json.dumps(dict(event='PRODUCTION_INITIAL_CHECK',passed=r['passed'],backend=r['backend'],cartons=len(r['root']['world_cartons']),height=r['root']['height_m'],history=len(r['root']['completed']))),flush=True)
PY
INSIDE
set +e
chroot "$rootfs" /bin/bash /lowprofile250-20261023/formal-v1.sh > "$rootfs/lowprofile250-20261023/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/lowprofile250-20261023/formal-v1.exit"
exit "$code"
