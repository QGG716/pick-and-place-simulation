set -e
D=/root/autodl-tmp/m710-tcp-lin-20260928
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs
E=$D/evidence/final2
mkdir -p "$E"
cp -n "$D/evidence/suffix.json" "$D/evidence/first-accepted-suffix-development.json"
tar xf "$D/update.tar" -C "$D/repo"
tar xf "$D/update.tar" -C "$R/work-round4"
chroot "$R" /bin/bash -c 'source /opt/ros/humble/setup.bash; cmake --build /work-round4/build -j2' > "$E/build.log" 2>&1
sha256sum "$R/work-round4/build/m710_moveit_worker" > "$E/binary.sha256"
cd "$D/repo"
export M710_MOVEIT_ASSET_ROOT=/work-round4 M710_MOVEIT_COMMAND="$D/worker.sh" M710_MOVEIT_LOG="$E/semantics-worker.log"
PY=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
"$PY" tools/check_m710_rotating_tcp.py --suite semantics --output "$E/semantics.json" > "$E/semantics.log" 2>&1
export M710_MOVEIT_LOG="$E/suffix-worker.log"
"$PY" tools/check_m710_rotating_tcp.py --suite suffix --output "$E/suffix.json" > "$E/suffix.log" 2>&1
# Full fixed candidate runs only after explicit suffix success, preserving all gates.
"$PY" -c 'import json,sys;d=json.load(open("/root/autodl-tmp/m710-tcp-lin-20260928/evidence/final/suffix.json"));sys.exit(0 if d["results"][0]["failure"] is None else 1)'
cp -n "$E/suffix.json" "$E/first-accepted-suffix.json"
export M710_MOVEIT_LOG="$E/task-worker.log" M710_MOVEIT_REQUEST_LOG="$E/task-requests.jsonl" M710_AUTHORITY_TRACE="$E/authority-paths.jsonl"
"$PY" -u tools/run_m710id70_layout_single_carton.py --backend moveit2 --target carton_l07_c02 --fixed-history-fixture tests/fixtures/moveit2/frozen_candidate.json --output "$E/full-task.json" > "$E/full-task.log" 2>&1

