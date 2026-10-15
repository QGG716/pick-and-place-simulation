#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-approach-queue-20261015
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1
export ROS_HOME=/tmp/approach-queue-20261015/ros M710_MOVEIT_ASSET_ROOT=/work-static-prior-20261003
for spec in c04-A c04-B c02-B c02-A; do
  target=${spec%-*}; mode=${spec#*-}
  options=()
  if [ "$mode" = A ]; then options+=(--legacy-approach-layering); fi
  echo "START $spec"
  taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python -u tools/plan_m710_top_row_only.py \
    --output "/tmp/approach-queue-20261015/$spec" \
    --diagnostic-input "/tmp/approach-queue-20261015/$target-input.json" \
    --planning-mode static_prior_fast --static-prior /tmp/static-prior-20261003/static-prior-v3.json \
    --fast-phase-budget-s 1 --slow-completion \
    --worker-command /tmp/approach-queue-20261015/worker.sh \
    --worker-binary /tmp/approach-queue-20261015/build/m710_moveit_worker \
    --source-commit edf62178c579f29c7032792305fa5e487e533516 \
    --stage-budget-s 90 --box-budget-s 300 --batch-budget-s 900 --ipc-timeout-s 100 --seed 71070 \
    "${options[@]}" > "/tmp/approach-queue-20261015/$spec-stdout.txt" 2>&1
  echo "END $spec exit=$?"
done
