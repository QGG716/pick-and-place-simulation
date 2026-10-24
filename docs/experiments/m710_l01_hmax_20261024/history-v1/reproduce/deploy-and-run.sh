#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-l01-hmax-48dcea9"
test ! -e "$rootfs/l01-hmax-20261024"
test "$(sha256sum "$base/l01-hmax-source.tar.gz" | cut -d' ' -f1)" = c4a9b440018f13bed99645769f2e6559dab2f5c147f36243a66fff98274ee654
tar xzf "$base/l01-hmax-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/l01-hmax-20261024"
cp "$base/l01-hmax-formal.sh" "$rootfs/l01-hmax-20261024/formal-v1.sh"
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-l01-hmax-48dcea9/src:/work-l01-hmax-48dcea9/tools PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-l01-hmax-48dcea9
cd /work-l01-hmax-48dcea9
set +u
source /opt/ros/humble/setup.bash
set -u
cmake -S /work-l01-hmax-48dcea9/ros2/m710_moveit_backend -B /l01-hmax-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/l01-hmax-install -DPython3_EXECUTABLE=/usr/bin/python3 > /l01-hmax-20261024/build.log 2>&1
cmake --build /l01-hmax-build --target m710_moveit_worker m710_coupled_receiver_test m710_process_policy_test -j2 >> /l01-hmax-20261024/build.log 2>&1
cmake --install /l01-hmax-build >> /l01-hmax-20261024/build.log 2>&1
export AMENT_PREFIX_PATH=/l01-hmax-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/l01-hmax-install/lib:/l01-hmax-build:${LD_LIBRARY_PATH:-}
/opt/planning-venv/bin/python tools/check_m710_l01_hmax.py --output /l01-hmax-20261024/native-fixtures.json
/l01-hmax-build/m710_coupled_receiver_test /l01-hmax-20261024/native-fixtures.json > /l01-hmax-20261024/native-policy-tests.log 2>&1
/l01-hmax-build/m710_process_policy_test >> /l01-hmax-20261024/native-policy-tests.log 2>&1
/l01-hmax-build/m710_process_policy_test optimized >> /l01-hmax-20261024/native-policy-tests.log 2>&1
sha256sum /l01-hmax-build/m710_moveit_worker /l01-hmax-install/lib/libm710_search_trace.so > /l01-hmax-20261024/native-binary-sha256.txt
cat > /l01-hmax-worker.sh <<'WORKER'
#!/bin/bash
set -eo pipefail
test -z "${LD_PRELOAD:-}"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/l01-hmax-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/l01-hmax-install/lib:/l01-hmax-build:${LD_LIBRARY_PATH:-}
exec /l01-hmax-build/m710_moveit_worker
WORKER
/opt/planning-venv/bin/python -m pytest -q tests/test_l01_hmax.py tests/test_tool_connector_extension.py tests/test_l01_selected_routes.py tests/test_l01_tool_chassis.py tests/test_right_receiver.py tests/test_low_top_receiver.py > /l01-hmax-20261024/focused-tests.log 2>&1
bash /l01-hmax-20261024/formal-v1.sh /l01-hmax-20261024/production-check --check-only > /l01-hmax-20261024/production-check.log 2>&1
cat /l01-hmax-20261024/focused-tests.log
cat /l01-hmax-20261024/native-policy-tests.log
INSIDE
set +e
chroot "$rootfs" /bin/bash /l01-hmax-20261024/formal-v1.sh > "$rootfs/l01-hmax-20261024/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/l01-hmax-20261024/formal-v1.exit"
exit "$code"
