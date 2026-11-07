"""One explicit same-g order backtrack; retain all states, costs and cursors.

This is the actual host-side driver for the 2026-11-07 run, not a new physical
world. Run only after requesting the existing stop-after-trial checkpoint.
"""
import hashlib,json,os,subprocess,time
from pathlib import Path

base=Path('/root/m710-gap-search-20261107');root=base/'rootfs'
run=root/'gap-search-20261107/run-v1';path=run/'checkpoint.json'
while True:
    db=json.loads(path.read_text())
    if db['status']=='CHECKPOINTED_BETWEEN_TRIALS' and not db.get('pending'):break
    time.sleep(5)
assert db['contract']=='fixed40_gap_search_20261107_v1'
assert db['identity']['source_manifest_sha256']=='fdff0ac49b2b45bde9544687f4416def4284ce15b2ddf47319fd6117470f595e'
assert len(db['trials'])<74
record=db['layouts'][db['best']];assert record['gap_m']==.2
before=path.read_bytes();backup=run/'checkpoint-before-order-backtrack-v6.json'
assert not backup.exists();backup.write_bytes(before)
work='/work-gap-20261107-v1'
environment='export PYTHONPATH=src:tools:tests OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1; '
verify=subprocess.run(['chroot',str(root),'/bin/bash','-c',f'cd {work}; '+environment+
    '/opt/planning-venv/bin/python tools/verify_m710_joint_search.py /gap-search-20261107/run-v1'],capture_output=True,text=True)
(base/'pre-order-backtrack-verification.json').write_text(verify.stdout)
if verify.returncode:raise RuntimeError(verify.stderr)
assert path.read_bytes()==before,'ANOTHER_CHECKPOINT_WRITER'
best=record['nodes'][record['best_node']]
# Newly obtained progress before the stop keeps its active branch. Otherwise
# revisit the immediate accepted parent without altering any q, world or cost.
target=best if record['best_count']>3 else record['nodes'][best['parent']]
assert not target['closed']
event=dict(reason='EXPLICIT_FAIR_ORDER_BACKTRACK_AFTER_RETAINED_DEEPEST_BRANCH_FAILURES',
    from_node=record['active_node'],to_node=target['id'],preserved_best_node=record['best_node'],
    preserved_best_count=record['best_count'],selected_completed_count=len(target['root']['completed']),
    cursor_unchanged=target['cursor'],trials_retained=len(db['trials']),elapsed_search_s_retained=db['elapsed_search_s'],
    backup_sha256=hashlib.sha256(before).hexdigest(),driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    numerical_requests='RESTART_NO_RETAINED_NATIVE_TREE',state_and_motion_source='CURRENT_RUN_ACCEPTED_PARENT_CHAIN_ONLY',
    total_trial_allocation_unchanged=74,accepted_states_changed=False)
db.setdefault('branch_scheduler_events',[]).append(event);record['active_node']=target['id']
db['status']='BRANCH_REVISIT_READY'
tmp=path.with_suffix('.order-revisit.tmp');tmp.write_text(json.dumps(db,indent=2));os.replace(tmp,path)
(root/'gap-search-20261107/stop-after-trial').unlink()
remaining=74-len(db['trials'])
command=f'cd {work}; '+environment+(
    '/opt/planning-venv/bin/python tools/plan_m710_gap_search.py --output /gap-search-20261107/run-v1 '
    '--worker-command /gap-worker-v1.sh --worker-binary /gap-search-build-v1/m710_moveit_worker '
    '--source-manifest /gap-search-20261107/source-manifest-v6.json '
    f'--resume --max-trials {remaining} --trial-seconds 240 --prefix-seconds 90 --stage-seconds 120 '
    '--refine --focus-gap 0.2 --stop-file /gap-search-20261107/stop-after-trial')
print(json.dumps(dict(event=event,command=command)),flush=True)
with (base/'run-v6-order-branch.log').open('w') as log:
    result=subprocess.run(['chroot',str(root),'/bin/bash','-c',command],stdout=log,stderr=subprocess.STDOUT)
print(json.dumps({'search_exit':result.returncode}),flush=True)
if result.returncode:raise RuntimeError('REVISIT_SEARCH_EXIT_NONZERO')
with (base/'finalize-delivery.log').open('w') as log:
    final=subprocess.run(['python3',str(base/'finalize-delivery.py')],stdout=log,stderr=subprocess.STDOUT)
print(json.dumps({'finalization_exit':final.returncode}),flush=True)
