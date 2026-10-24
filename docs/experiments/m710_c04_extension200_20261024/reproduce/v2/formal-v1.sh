#!/bin/bash
set -euo pipefail
export PYTHONPATH=/work-c04-extension200-20b3ca7/src:/work-c04-extension200-20b3ca7/tools PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 M710_MOVEIT_ASSET_ROOT=/work-c04-extension200-20b3ca7
test -z "${LD_PRELOAD:-}"
face="${1:?top, side, or front required}"
case "$face" in top|side|front) ;; *) exit 2;; esac
out="${2:?new output path required}"
test ! -e "$out"
exec /opt/planning-venv/bin/python -u /work-c04-extension200-20b3ca7/tools/plan_m710_xy_top_population.py  --output "$out" --worker-command '/bin/bash /l01-hmax-worker.sh' --worker-binary /l01-hmax-build/m710_moveit_worker  --source-commit 20b3ca78b2a70e151b25d165c55b511c13667598 --source-manifest /work-c04-extension200-20b3ca7/source-manifest.json  --mounting-variant x_minus_500_y_minus_100 --continue-l01-c04-face-from /l01-hmax-v2-20261024/formal-v1  --tool-connector-extension-m 0.2 --c04-face "$face" "${@:3}"
