#!/bin/bash
source /opt/ros/humble/setup.bash
set -eu
cd /work-single-contact-20261016
export PYTHONPATH=src PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=$PWD
root=/tmp/right-inner-20261019
source_sha="$1"
/opt/planning-opt-20261003-venv/bin/python - <<'PY'
import json
from pathlib import Path
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.lifted_reception import bounds,layer_role
layout=load_workcell_layout(Path('configs/workcells/m710id70_unloading_layout_v1.yaml'))
boxes=layout.cartons();z=max(bounds(b)[0][2] for b in boxes)
row=sorted([b for b in boxes if abs(bounds(b)[0][2]-z)<1e-9],key=lambda b:b.center[1])
right,middle,left=row[0],row[len(row)//2],row[-1]
cases=[('right-open-s71070',right.name,71070,'right_inner',True),
 ('right-open-s71071',right.name,71071,'right_inner',True),
 ('right-blocked-s71070',right.name,71070,'right_inner',False),
 ('middle-front-s71070',middle.name,71070,'front',False),
 ('left-front-s71070',left.name,71070,'front',False)]
out=Path('/tmp/right-inner-20261019')
with (out/'protocol.json').open('x') as f:json.dump(dict(cases=cases,original_right_role=layer_role(right,boxes,boxes),
 declared_removed_middle_ids=[b.name for b in row[1:-1]],newly_unloaded_count=0,
 input='CONFIGURED_OFFLINE_STATIC_INITIAL_Q_NOT_PREVIOUS_PHYSICAL_FEEDBACK',
 task_s=300,stage_s=90,ipc_s=100,affinity=list(range(22)),threads=1,selection_fixed_before_runs=True),f,indent=2)
with (out/'cases.txt').open('x') as f:
 for label,target,seed,template,opened in cases:f.write(f'{label} {target} {seed} {template} {int(opened)}\n')
PY
while read -r label target seed template opened; do
  extra=()
  if test "$opened" = 1; then extra+=(--open-middle-channel); fi
  echo "BEGIN $label"
  set +e
  taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py \
    --through-lifted-release-ready --receiver-template "$template" "${extra[@]}" \
    --target "$target" --seed "$seed" --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100 \
    --worker-command "$root/worker.sh" --worker-binary "$root/build/m710_moveit_worker" \
    --source-commit "$source_sha" --output "$root/$label" > "$root/$label-console.log" 2>&1
  code=$?
  set -e
  echo "END $label exit=$code"
done < "$root/cases.txt"
echo BATCH_COMPLETE
