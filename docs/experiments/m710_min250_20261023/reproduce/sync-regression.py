"""Isolated regression of the exact pre-error root; never population credit."""
from pathlib import Path
from types import SimpleNamespace
import sys,json,hashlib,numpy as np
work=Path('/work-min250-3f449e8');sys.path[:0]=[str(work/'src'),str(work/'tools')]
from plan_m710_mount_comparison import make_context
from plan_m710_top_layer_lifted import run_seed,source_identity,save
from min250_continuation import restore_population_terminal,restore_before_world_sync_repair
from unloading_sim.coupled_batch import BatchState
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.low_top_receiver import BOX_TOP_LOW_DEFINITION
out=Path('/min250-20261023/sync-regression');out.mkdir(exist_ok=False)
a=SimpleNamespace(output=out,source_commit='3f449e89da518ac89b3c112b4a83eb9a83000b11',source_manifest=work/'source-manifest.json',development=False,worker_command='/bin/bash /worker.sh',task_budget_s=300.,stage_budget_s=90.,ipc_timeout_s=100.,direct_first=True,upper_arm_preference=True,receive_height_range=(.25,2.1),low_top_minimum_m=.25,low_definition=BOX_TOP_LOW_DEFINITION,include_receive_before_approach=True,right_front_eccentric=False,junction_only=False)
identity=source_identity(a)
original,built,audit=make_context('x_minus_500_y_minus_100',layout_path=work/'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250.yaml')
frozen=FrozenCartonPopulation.capture(original.cartons)
state=BatchState(frozen,tuple(original.cartons),np.asarray(original.snapshot['robot']['q_rad']),.6)
restore_population_terminal(Path('/min250-20261023/previous-formal-v1'),state,'x_minus_500_y_minus_100')
restore_before_world_sync_repair(Path('/min250-20261023/formal-v1'),state)
save(out/'protocol.json',dict(**identity,scope='ISOLATED_EXACT_WORLD_SYNC_REGRESSION_NO_POPULATION_CREDIT',target='carton_l02_c04',seed=71070,root=state.root(),no_historical_solution_hints=True,source_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
r=run_seed(a,original,built,frozen,71070,out,initial_state=state,target_ids=['carton_l02_c04'])
assert r['completed_count']==1
cycle=r['cycles'][0]
events=[e for e in cycle['events'] if e['event_type']=='EXTERNAL_AXIS_MOTION']
assert any(0<abs(e['from_height_m']-e['to_height_m'])<1e-12 and all(c['passed'] for c in e['checks']) for e in events)
save(out/'verification.json',dict(status='PASS_NATIVE_EXACT_WORLD_SYNC_AND_COMPLETE_CYCLE',source_commit=a.source_commit,backend=type(built.robot).__name__,population_completion_credit=0,isolated_regression_complete=1,planning_wall_s=r['planning_wall_s'],events=events,qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False))
print(json.dumps(dict(status='PASS_NATIVE_EXACT_WORLD_SYNC_AND_COMPLETE_CYCLE',population_completion_credit=0,seconds=r['planning_wall_s'])),flush=True)
