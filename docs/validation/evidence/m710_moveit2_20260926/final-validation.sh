#!/bin/bash
set -euo pipefail
B=/root/autodl-tmp/m710-moveit2-clearance-20260926
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs
cd "$B/repo"
export PYTHONPATH=src M710_MOVEIT_ASSET_ROOT=/work-round2 M710_MOVEIT_COMMAND="$B/worker.sh" M710_MOVEIT_LOG="$B/evidence/worker-final.log" M710_MOVEIT_SEED=71070 M710_MOVEIT_STAGE_SECONDS=12 M710_MOVEIT_REQUEST_TIMEOUT=600
P=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
echo 'Final validation begins; commands execute serially; no Isaac.'
date -u
chroot "$R" /bin/bash -c 'source /opt/ros/humble/setup.bash; /work-round2/build-round2/m710_clearance_test' > "$B/evidence/synthetic-final.json" 2> "$B/evidence/synthetic-final.log"
$P -m pytest tests/test_moveit2_backend.py tests/test_layout_trajectory.py tests/test_isaac_bridge.py::test_native_duration_floor_reaches_existing_replay_exporter -q --tb=short > ../evidence/python-final.log 2>&1
cat ../evidence/python-final.log
$P tools/check_m710_moveit_clearance.py --output ../evidence/differential-final.json
$P tools/run_m710_moveit_integration.py --suite capability --output ../evidence/capability-final.json
$P tools/run_m710_moveit_integration.py --case empty_short --case linear_fixed_orientation --output ../evidence/small-final.json
$P tools/run_m710_moveit_integration.py --case loaded_fixed_transit --output ../evidence/loaded-final.json
date -u
echo FINAL_VALIDATION_COMPLETE
