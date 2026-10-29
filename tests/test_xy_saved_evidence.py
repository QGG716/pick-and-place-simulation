"""Optional real-native-evidence regressions; run against a saved server probe."""
import hashlib,json,os,shutil
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import pytest
from plan_m710_joint_search import prepare_layout,restore_state
from verify_m710_xy_continuation import candidate,verify_move
from unloading_sim.moveit2_backend import digest

@pytest.fixture(scope='module')
def probe():
    path=os.environ.get('M710_XY_NATIVE_PROBE')
    prefix=os.environ.get('M710_XY_ACCEPTED_PREFIX')
    if not path or not prefix:pytest.skip('Requires saved native server evidence')
    folder=Path(path);db=json.loads((Path(prefix)/'checkpoint.json').read_text());record=db['layouts'][db['best']]
    source=candidate(record['candidate']);context,_=prepare_layout(SimpleNamespace(seed=71070,initial_draws=64),source)
    state=restore_state(record['nodes'][record['best_node']],context[2]);row=json.loads((folder/'xy-motion.json').read_text())
    return folder,row,state,context,source,candidate(row['to_layout'])

@pytest.mark.parametrize('mutation',['countertranslation','joint_reset','active_cup'])
def test_saved_native_success_cannot_authorize_a_changed_world_or_state(probe,tmp_path,mutation):
    folder,row,state,context,source,destination=probe
    for name in ('native-model.json','native-records.json'):shutil.copyfile(folder/name,tmp_path/name)
    rows=json.loads((folder/'native-checks.json').read_text());row=deepcopy(row);request=rows[0]['request']
    if mutation=='countertranslation':
        # Rehash the forgery so verification must check actual scene geometry.
        request['world'][0]['pose'][0][3]+=.1
        request['scene_fingerprint']=digest(request['world']);row['samples'][0]['evaluation_world_sha256']=digest(request['world'])
    elif mutation=='joint_reset':request['q_start'][0]+=.1;request['q_goal'][0]+=.1
    else:request['process_policy']['commanded_active_mask'][0]=True
    row['samples'][0]['native_request_sha256']=digest(request)
    (tmp_path/'native-checks.json').write_text(json.dumps(rows));row['native_checks_sha256']=hashlib.sha256((tmp_path/'native-checks.json').read_bytes()).hexdigest()
    (tmp_path/'xy-motion.json').write_text(json.dumps(row))
    with pytest.raises(AssertionError):verify_move(tmp_path,state,context[0],context[1],source,destination)
