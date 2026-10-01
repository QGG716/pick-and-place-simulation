set -e
D=/root/autodl-tmp/m710-execution-gate-20260928
E=$D/evidence/final
mkdir -p "$E"
tar xf "$D/gate5.tar" -C "$D/repo"
cd "$D/repo"
PY=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
"$PY" tools/check_m710_moveit_execution_tcp.py --bundle /root/autodl-tmp/m710-tcp-lin-20260928/evidence/final2/replay-bundle.json --legacy-requests /root/autodl-tmp/m710-tcp-lin-20260928/evidence/final2/task-requests.jsonl --derive-bundle "$E/derived-bundle.json" --output "$E/migration.json" > "$E/migration.log" 2>&1
export PYTHONPATH=src M710_GATE_TEST_BUNDLE=$E/derived-bundle.json
"$PY" -m pytest tests/test_m710_execution_tcp.py tests/test_moveit2_tcp.py tests/test_moveit2_backend.py tests/test_m710_replay_contract.py tests/test_isaac_bridge.py::test_build_fanuc_replay_is_deterministic_and_limit_audited tests/test_isaac_bridge.py::test_replay_inserts_stationary_vacuum_and_release_holds tests/test_isaac_bridge.py::test_native_duration_floor_reaches_existing_replay_exporter -q > "$E/pytest.txt" 2>&1
"$PY" -O -m pytest tests/test_m710_execution_tcp.py -k 'not actual_supervised' -q > "$E/pytest-optimized.txt" 2>&1
PYTHONOPTIMIZE=1 "$PY" -m pytest tests/test_m710_execution_tcp.py -k 'not actual_supervised' -q > "$E/pytest-env-optimized.txt" 2>&1
"$PY" tools/check_m710_moveit_execution_tcp.py --bundle "$E/derived-bundle.json" --output "$E/self-check.json" > "$E/self-check.log" 2>&1
"$PY" -O tools/check_m710_moveit_execution_tcp.py --bundle "$E/derived-bundle.json" --output "$E/self-check-optimized.json" > "$E/self-check-optimized.log" 2>&1
"$PY" - <<'PY'
import json
from pathlib import Path
E=Path('/root/autodl-tmp/m710-execution-gate-20260928/evidence/final')
b=json.loads((E/'derived-bundle.json').read_text())
(E/'derived-preflight.json').write_text(json.dumps(b['metadata']['m710_execution_preflight'],indent=2))
PY
"$PY" scripts/export_isaac_fanuc_replay.py --preflight "$E/derived-preflight.json" --output "$E/standard-export.json" > "$E/standard-export.log" 2>&1
"$PY" tools/check_m710_moveit_execution_tcp.py --bundle "$E/standard-export.json" --output "$E/standard-export-check.json" > "$E/standard-export-check.log" 2>&1
cat "$E/pytest.txt" "$E/pytest-optimized.txt" "$E/pytest-env-optimized.txt"
