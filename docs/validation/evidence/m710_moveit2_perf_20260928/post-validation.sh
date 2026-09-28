set -e
D=/root/autodl-tmp/m710-clearance-perf-20260928
E=$D/evidence/final2
while ! test -f "$E/COMPLETE"; do sleep 5; done
cd "$D/repo"
PY=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
"$PY" tools/check_m710_loaded_timing.py --accepted "$E/first-accepted-loaded.json" --output "$E/timing-contract.json" > "$E/timing-contract.log" 2>&1
"$PY" -m pip freeze > "$E/python-packages.lock"
"$PY" --version > "$E/python-version.txt"
"$PY" "$D/analyze.py" > "$E/summary.log" 2>&1
printf 'PASS\n' > "$E/AUDIT_COMPLETE"
