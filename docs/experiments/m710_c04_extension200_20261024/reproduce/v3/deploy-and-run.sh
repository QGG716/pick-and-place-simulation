#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-c04-extension200-c88b5eb"
test ! -e "$rootfs/c04-extension200-v3-20261024"
test "$(sha256sum "$base/c04-extension200-v3-source.tar.gz" | cut -d' ' -f1)" = e3687705504f4474e9717156ed4038ba2963aa9bac8b464de2525e550c0c1986
tar xzf "$base/c04-extension200-v3-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/c04-extension200-v3-20261024"
cp "$base/c04-extension200-v3-formal.sh" "$rootfs/c04-extension200-v3-20261024/formal-v1.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-c04-extension200-c88b5eb/src:/work-c04-extension200-c88b5eb/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-c04-extension200-c88b5eb
cd /work-c04-extension200-c88b5eb
set +u
source /opt/ros/humble/setup.bash
set -u
/opt/planning-venv/bin/python - <<'PY'
import json
from pathlib import Path
a=json.loads(Path('/work-l01-hmax-48dcea9/source-manifest.json').read_text())
b=json.loads(Path('/work-c04-extension200-c88b5eb/source-manifest.json').read_text())
native=lambda m:{k:v for k,v in m['sha256'].items() if k.startswith('ros2/')}
assert native(a)==native(b)
Path('/c04-extension200-v3-20261024/build.log').write_text(json.dumps(dict(status='REUSED_UNCHANGED_NATIVE_BUILD',
 compiled_source=a['source_commit'],measured_source=b['source_commit'],native_source_hashes=native(b)),indent=2)+'\n')
PY
export AMENT_PREFIX_PATH=/l01-hmax-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/l01-hmax-install/lib:/l01-hmax-build:${LD_LIBRARY_PATH:-}
/opt/planning-venv/bin/python tools/check_m710_l01_hmax.py --output /c04-extension200-v3-20261024/native-fixtures.json
/l01-hmax-build/m710_coupled_receiver_test /c04-extension200-v3-20261024/native-fixtures.json > /c04-extension200-v3-20261024/native-policy-tests.log 2>&1
/l01-hmax-build/m710_process_policy_test >> /c04-extension200-v3-20261024/native-policy-tests.log 2>&1
/l01-hmax-build/m710_process_policy_test optimized >> /c04-extension200-v3-20261024/native-policy-tests.log 2>&1
sha256sum /l01-hmax-build/m710_moveit_worker /l01-hmax-install/lib/libm710_search_trace.so > /c04-extension200-v3-20261024/native-binary-sha256.txt
cat > /l01-hmax-worker.sh <<'WORKER'
#!/bin/bash
set -eo pipefail
test -z "${LD_PRELOAD:-}"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/l01-hmax-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/l01-hmax-install/lib:/l01-hmax-build:${LD_LIBRARY_PATH:-}
exec /l01-hmax-build/m710_moveit_worker
WORKER
/opt/planning-venv/bin/python -m pytest -q tests/test_c04_face_extension200.py tests/test_c04_side_hmax.py tests/test_l01_hmax.py tests/test_tool_connector_extension.py tests/test_l01_selected_routes.py tests/test_l01_tool_chassis.py tests/test_right_receiver.py tests/test_low_top_receiver.py > /c04-extension200-v3-20261024/focused-tests.log 2>&1
bash /c04-extension200-v3-20261024/formal-v1.sh top /c04-extension200-v3-20261024/production-check --check-only > /c04-extension200-v3-20261024/production-check.log 2>&1
cat /c04-extension200-v3-20261024/focused-tests.log
cat /c04-extension200-v3-20261024/native-policy-tests.log
INSIDE
for face in top; do
  trial="$rootfs/c04-extension200-v3-20261024/$face"
  mkdir "$trial"
  for name in focused-tests.log production-check.log formal-v1.sh build.log native-policy-tests.log native-binary-sha256.txt native-fixtures.json; do
    cp "$rootfs/c04-extension200-v3-20261024/$name" "$trial/$name"
  done
  cp -r "$rootfs/c04-extension200-v3-20261024/production-check" "$trial/production-check"
  set +e
  chroot "$rootfs" /bin/bash /c04-extension200-v3-20261024/formal-v1.sh "$face" "/c04-extension200-v3-20261024/$face/formal-v1" > "$trial/formal-v1.log" 2>&1
  code=$?
  set -e
  printf '%s\n' "$code" > "$trial/formal-v1.exit"
  printf '%s exit=%s\n' "$face" "$code"
  if [ "$code" -ne 0 ]; then cat "$trial/formal-v1.log"; fi
done
