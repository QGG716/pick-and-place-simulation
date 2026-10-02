#!/bin/bash
set -eu
D=/root/autodl-tmp/m710-planning-only-20261002
cd "$D/repo"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-planning-only-20261002
exec timeout 7500 /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -u \
  tools/plan_m710_top_row_only.py \
  --output "$D/formal-once" \
  --worker-command "$D/worker.sh" \
  --worker-binary /root/autodl-tmp/m710-moveit2-20260922/rootfs/work-planning-only-20261002/build/m710_moveit_worker \
  --source-commit b1635ed3a47358f3480b320450a95d966db8d62b \
  --stage-budget-s 300 --box-budget-s 1800 --batch-budget-s 7200 --ipc-timeout-s 360 --seed 71070
