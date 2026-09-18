"""Small measured free-fall fixture; all production release/handoff gates run."""
import numpy as np
from unloading_sim.geometry import OBB
from unloading_sim.release_motion import predict_release, IDEAL_RECEPTION_RELEASE
from unloading_sim.m710_replay_physics import IdealReleaseHandoff


def measured_handoff(box, receiver, policy, time_s):
    dt=.02
    initial=OBB(box.center+[0,0,.5*9.81*dt*dt],box.half_extents,box.rotation,box.name,box.category)
    primitive=lambda b:dict(name=b.name,center_m=b.center.tolist(),size_m=(2*b.half_extents).tolist(),
        rotation_matrix=b.rotation.tolist(),category=b.category,dynamic=b.name==box.name)
    metadata=dict(target=box.name,release_mode=IDEAL_RECEPTION_RELEASE,
        release_prediction=predict_release(initial,[receiver],mode=IDEAL_RECEPTION_RELEASE),
        selected_place_support_names=[receiver.name],scene_primitives=[primitive(initial),primitive(receiver)])
    context=dict(world_id='fixture-world',task_id='fixture-task',receiver=receiver.name,transport_policy=policy)
    gate=IdealReleaseHandoff(metadata,**context)
    assert gate.accept_release(time_s=time_s-dt,position=initial.center,rotation=initial.rotation,
        linear_velocity=[0,0,0],angular_velocity=[0,0,0],current_cartons=[])['accepted']
    gate.confirm_independence(time_s=time_s,joint_present=False,translation_m=.0021,rotation_rad=0,
        minimum_translation_m=.002,minimum_rotation_rad=.01)
    assert gate.audit_takeover(metadata,**context,target=box.name,time_s=time_s,joint_present=False,
        position=box.center,rotation=box.rotation,linear_velocity=[0,0,-9.81*dt],
        angular_velocity=[0,0,0],current_cartons=[])['accepted']
    return gate
