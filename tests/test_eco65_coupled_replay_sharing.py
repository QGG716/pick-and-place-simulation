"""Large run export must retain exact request data while sharing geometry."""
from copy import deepcopy
import json

from export_eco65_coupled_replay import reuse_runtime_values, share_worlds


def test_repeated_native_contexts_are_shared_without_losing_trial_or_target():
    policy=dict(schema='test',robot_mast_exemption=dict(pairs=[['link_1','mast']]))
    context=dict(world=[dict(id='box',pose=[1.,2.,3.])],
        clearance_policy=dict(source_policy=policy),
        process_policy=dict(collision_policy=policy,contours=[dict(id='cup',points=[[0.,0.]])]))
    a=deepcopy(context);b=deepcopy(context)
    a.update(task_id='trial-a',target_id='first')
    b.update(task_id='trial-b',target_id='second')
    data=dict(attempts=[dict(request=a),dict(request=b)])
    expected=deepcopy(data)
    cache={}
    reuse_runtime_values(data,cache)
    assert a['world'] is b['world']
    assert a['clearance_policy']['source_policy'] is b['process_policy']['collision_policy']
    packed=json.loads(json.dumps(share_worlds(data)))
    def restore(value):
        if isinstance(value,list):return [restore(x) for x in value]
        if not isinstance(value,dict):return value
        if set(value)=={'shared_definition_ref'}:
            return restore(packed['shared_definitions'][value['shared_definition_ref']])
        result={k:restore(v) for k,v in value.items() if k!='world_ref'}
        if 'world_ref' in value:result['world']=restore(packed['shared_worlds'][value['world_ref']])
        return result
    assert restore(dict(attempts=packed['attempts']))==expected
