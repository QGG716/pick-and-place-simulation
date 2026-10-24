#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-current-c04-dea9b2b"
test ! -e "$rootfs/current-c04-comparison-20261024"
test "$(sha256sum "$base/current-c04-source.tar.gz" | cut -d' ' -f1)" = 9f8c7e8214651a230c9e88e83f3875806eb17c1d1525aefeae6aec6cea9d31e9
tar xzf "$base/current-c04-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/current-c04-comparison-20261024"
cp "$base/current-c04-formal.sh" "$rootfs/current-c04-comparison-20261024/formal-v1.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-current-c04-dea9b2b/src:/work-current-c04-dea9b2b/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-current-c04-dea9b2b
cd /work-current-c04-dea9b2b
set +u
source /opt/ros/humble/setup.bash
set -u
/opt/planning-venv/bin/python - <<'PY'
import json
from pathlib import Path
a=json.loads(Path('/work-l01-hmax-48dcea9/source-manifest.json').read_text())
b=json.loads(Path('/work-current-c04-dea9b2b/source-manifest.json').read_text())
native=lambda m:{k:v for k,v in m['sha256'].items() if k.startswith('ros2/')}
assert native(a)==native(b)
Path('/current-c04-comparison-20261024/build.log').write_text(json.dumps(dict(status='REUSED_UNCHANGED_NATIVE_BUILD',
 compiled_source=a['source_commit'],measured_source=b['source_commit'],native_source_hashes=native(b)),indent=2)+'\n')
PY
export AMENT_PREFIX_PATH=/l01-hmax-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/l01-hmax-install/lib:/l01-hmax-build:${LD_LIBRARY_PATH:-}
/opt/planning-venv/bin/python tools/check_m710_l01_hmax.py --output /current-c04-comparison-20261024/native-fixtures.json
/l01-hmax-build/m710_coupled_receiver_test /current-c04-comparison-20261024/native-fixtures.json > /current-c04-comparison-20261024/native-policy-tests.log 2>&1
/l01-hmax-build/m710_process_policy_test >> /current-c04-comparison-20261024/native-policy-tests.log 2>&1
/l01-hmax-build/m710_process_policy_test optimized >> /current-c04-comparison-20261024/native-policy-tests.log 2>&1
sha256sum /l01-hmax-build/m710_moveit_worker /l01-hmax-install/lib/libm710_search_trace.so > /current-c04-comparison-20261024/native-binary-sha256.txt
cat > /l01-hmax-worker.sh <<'WORKER'
#!/bin/bash
set -eo pipefail
test -z "${LD_PRELOAD:-}"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/l01-hmax-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/l01-hmax-install/lib:/l01-hmax-build:${LD_LIBRARY_PATH:-}
exec /l01-hmax-build/m710_moveit_worker
WORKER
/opt/planning-venv/bin/python -m pytest -q tests/test_bottom_rotated.py tests/test_l00_top_continuation.py tests/test_c04_side_hmax.py tests/test_l01_hmax.py tests/test_tool_connector_extension.py tests/test_l01_selected_routes.py tests/test_l01_tool_chassis.py tests/test_right_receiver.py tests/test_low_top_receiver.py > /current-c04-comparison-20261024/focused-tests.log 2>&1
C04_FACE=side bash /current-c04-comparison-20261024/formal-v1.sh /current-c04-comparison-20261024/production-check --check-only > /current-c04-comparison-20261024/production-check.log 2>&1
cat /current-c04-comparison-20261024/focused-tests.log
cat /current-c04-comparison-20261024/native-policy-tests.log
INSIDE
for face in side front; do
set +e
chroot "$rootfs" /usr/bin/env C04_FACE="$face" /bin/bash /current-c04-comparison-20261024/formal-v1.sh /current-c04-comparison-20261024/$face > "$rootfs/current-c04-comparison-20261024/$face.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/current-c04-comparison-20261024/$face.exit"
test "$code" = 0 || exit "$code"
set -e
done
