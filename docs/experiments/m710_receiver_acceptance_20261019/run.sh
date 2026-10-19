#!/bin/bash
source /opt/ros/humble/setup.bash
set -u
cd /work-single-contact-20261016
export PYTHONPATH=src PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=$PWD
root=/tmp/receiver-acceptance-20261019
failed=0
while read -r target seed; do
  out="$root/$target-s$seed"
  if test -e "$out"; then echo "REFUSE_OVERWRITE $out"; exit 3; fi
  echo "BEGIN $target seed=$seed"
  taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py \
    --through-lifted-release-ready --target "carton_l07_$target" --seed "$seed" \
    --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100 \
    --worker-command /tmp/lift-native-20261017/worker.sh \
    --worker-binary /tmp/lift-native-20261017/build/m710_moveit_worker \
    --source-commit 5dd484e59cac51fd3203cf52e83802408feec10a --output "$out" \
    > "$root/$target-s$seed-console.log" 2>&1
  code=$?
  echo "END $target seed=$seed exit=$code"
  if test "$code" != 0; then failed=1; fi
done <<'CASES'
c02 71070
c04 71070
c02 71071
c04 71071
CASES
echo "BATCH_COMPLETE exit=$failed"
exit "$failed"
