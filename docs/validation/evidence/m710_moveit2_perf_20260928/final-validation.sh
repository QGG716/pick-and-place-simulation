set -e
D=/root/autodl-tmp/m710-clearance-perf-20260928
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs
E=$D/evidence/final2
mkdir -p "$E"
cd "$D/repo"
PY=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
export M710_MOVEIT_ASSET_ROOT=/work-round3 M710_MOVEIT_COMMAND="$D/worker.sh"
sha256sum "$R/work-round3/build-perf/m710_moveit_worker" "$R/work-round3/build-perf/m710_clearance_test" > "$E/binaries.sha256"
cp "$R/work-round3/build-perf/CMakeFiles/m710_moveit_worker.dir/flags.make" "$E/flags.make"
cp "$D/evidence/optimized-build.log" "$E/build.log"
{ date -u; uname -a; lscpu; cat /sys/fs/cgroup/cpu.max; free -m; nvidia-smi; } > "$E/environment.txt" 2>&1
chroot "$R" dpkg-query -W > "$E/packages.lock"
"$PY" - <<'PY'
from pathlib import Path
import hashlib,json
files=['ros2/m710_moveit_backend/src/worker.cpp','ros2/m710_moveit_backend/src/clearance.h','ros2/m710_moveit_backend/src/clearance_workspace.h','ros2/m710_moveit_backend/src/clearance_test.cpp','src/unloading_sim/moveit2_backend.py','src/unloading_sim/layout_single_carton.py','tools/profile_m710_clearance.py','tools/run_m710_moveit_integration.py','tools/check_m710_moveit_clearance.py','tests/fixtures/moveit2/frozen_candidate.json','tests/fixtures/moveit2/clearance_rejections.json','configs/validation/m710id70_proof_of_concept.yaml']
r={'review_baseline':'baeb2a7056e8af0397855e9f54c0d9d421472e6d','files':{f:hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in files},'build':'Release -O3 -DNDEBUG; no fast-math','resident_seed':71070,'loaded_request_seed':71072,'order':'synthetic reference, optimized; fixed_set reference, optimized; historical reference, optimized; loaded reference, optimized; optional one diagnostic; PTP, LIN, early TCP; focused pytest'}
Path('/root/autodl-tmp/m710-clearance-perf-20260928/evidence/final2/source-manifest.json').write_text(json.dumps(r,indent=2))
PY
for mode in reference optimized; do
 chroot "$R" /bin/bash -c "source /opt/ros/humble/setup.bash; /work-round3/build-perf/m710_clearance_test $mode" > "$E/synthetic-$mode.json" 2> "$E/synthetic-$mode.log"
done
for mode in reference optimized; do
 export M710_CLEARANCE_MODE=$mode M710_MOVEIT_LOG="$E/fixed-$mode-worker.log"
 "$PY" tools/profile_m710_clearance.py --output "$E/fixed-$mode.json" > "$E/fixed-$mode.log" 2>&1
done
for mode in reference optimized; do
 export M710_CLEARANCE_MODE=$mode M710_MOVEIT_LOG="$E/historical-$mode-worker.log"
 "$PY" tools/check_m710_moveit_clearance.py --output "$E/historical-$mode.json" > "$E/historical-$mode.log" 2>&1
done
for mode in reference optimized; do
 export M710_CLEARANCE_MODE=$mode M710_MOVEIT_LOG="$E/loaded-$mode-worker.log" M710_MOVEIT_REQUEST_LOG="$E/loaded-$mode-requests.jsonl"
 "$PY" tools/run_m710_moveit_integration.py --case loaded_fixed_transit --output "$E/loaded-$mode.json" --preserve-first-path "$E/first-accepted-loaded.json" > "$E/loaded-$mode.log" 2>&1
done
export M710_CLEARANCE_MODE=optimized
if "$PY" - <<'PY'
import json
p=json.load(open('/root/autodl-tmp/m710-clearance-perf-20260928/evidence/final2/loaded-optimized.json'))
f=p['results'][0]['failure']
raise SystemExit(0 if f and f['reason']=='MOVEIT2_SEARCH_EXHAUSTED' else 1)
PY
then
 export M710_MOVEIT_LOG="$E/diagnostic-worker.log" M710_MOVEIT_REQUEST_LOG="$E/diagnostic-requests.jsonl"
 "$PY" tools/run_m710_moveit_integration.py --case loaded_fixed_transit --diagnostic-only --output "$E/diagnostic-only.json" --preserve-first-path "$E/first-accepted-loaded.json" > "$E/diagnostic-only.log" 2>&1
fi
unset M710_MOVEIT_REQUEST_LOG
export M710_MOVEIT_LOG="$E/small-worker.log"
"$PY" tools/run_m710_moveit_integration.py --case empty_short --case linear_fixed_orientation --output "$E/small.json" > "$E/small.log" 2>&1
export M710_MOVEIT_LOG="$E/capability-worker.log"
"$PY" tools/run_m710_moveit_integration.py --suite capability --output "$E/capability.json" > "$E/capability.log" 2>&1
PYTHONPATH=src "$PY" -m pytest -q tests/test_moveit2_backend.py tests/test_isaac_bridge.py::test_native_duration_floor_reaches_existing_replay_exporter > "$E/pytest.log" 2>&1
printf 'COMPLETE\n' > "$E/COMPLETE"
