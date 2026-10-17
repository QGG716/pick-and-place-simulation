"""Offline pose display only. Never imports a physics or hardware executor."""
import numpy as np
from .model import transform, robot_for
from .native_task import validate_plan_contract, STATUS
from .native_io import verify_snapshot
from ..timing import sample_quintic_knots


def replay_inputs(snapshot, plan, review):
    verify_snapshot(snapshot); validate_plan_contract(plan, snapshot)
    if (review['status'] != 'PASS' or review['plan_fingerprint'] != plan['fingerprint'] or
            review['snapshot_fingerprint'] != snapshot['fingerprint']):
        raise ValueError('OFFLINE_REPLAY_REQUIRES_SAME_REVIEWED_PLAN')
    robot = robot_for(snapshot['tool'], snapshot['scene'])
    # Validate both sides of semantic events before showing any frame.
    attach, release = plan['events']
    initial = transform(snapshot['scene']['box']['pose'])
    for event, pose in [(attach, initial), (release, np.asarray(release['pose']))]:
        actual = robot.named_link_frames(event['q'])['link_6']@np.asarray(attach['flange_from_target'])
        if not np.allclose(actual, pose, atol=1e-8, rtol=0): raise ValueError('REPLAY_TARGET_TELEPORT')
    return robot


def state_at(snapshot, plan, robot, time_s):
    duration = sum(s['times'][-1] for s in plan['stages'])
    if not np.isfinite(time_s) or time_s < 0 or time_s > duration+1e-9: raise ValueError('REPLAY_TIME_INVALID')
    offset = 0.
    for s in plan['stages']:
        if time_s <= offset+s['times'][-1]+1e-10: break
        offset += s['times'][-1]
    q, _ = sample_quintic_knots(np.array(s['times']), np.array(s['q']), [min(max(time_s-offset, 0), s['times'][-1])])
    frames = robot.named_link_frames(q[0]); target = snapshot['scene']['target_box_id']
    loaded = s['stage'] in {'extraction', 'transit', 'place'}
    cartons = {b['id']: transform(b['pose']) for b in snapshot['scene']['boxes']}
    if loaded: cartons[target] = frames['link_6']@np.asarray(plan['events'][0]['flange_from_target'])
    elif s['stage'] in {'withdrawal', 'residence'}: cartons[target] = np.asarray(plan['events'][1]['pose'])
    return dict(q=q[0], frames=frames, cartons=cartons, phase=s['stage'], attached=loaded, time_s=time_s,
                status='KINEMATIC_REPLAY', planning_status=STATUS, dynamics='NOT_EVALUATED')


def apply_display_state(stage, snapshot, state):
    """Update link parents once; keep their mesh and accessory transforms local."""
    from pxr import UsdGeom, Gf
    prefix = '/World/Workstation/'
    expected = {}
    def put(path, T):
        prim = stage.GetPrimAtPath(prefix+path)
        if not prim: raise ValueError('REPLAY_PRIM_MISSING:'+path)
        xf = UsdGeom.Xformable(prim); ops = xf.GetOrderedXformOps()
        if len(ops) != 1 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTransform:
            raise ValueError('REPLAY_TRANSFORM_STACK_CHANGED:'+path)
        ops[0].Set(Gf.Matrix4d(np.asarray(T).T.tolist())); expected[path] = np.asarray(T)
    for name, T in state['frames'].items():
        # The exporter stores FK on Robot/<link>, with identity mesh children.
        # Writing FK on those children again doubles the installation transform
        # and leaves J1-mounted accessories behind when the robot moves.
        put('Robot/'+name, T)
        components = snapshot['scene'].get('robot_collision_components', {}).get(name)
        roles = ['Visual']+(['Collision_'+row['id'] for row in components] if components else ['Collision'])
        for role in roles:
            path = 'Robot/'+name+'/'+role
            prim = stage.GetPrimAtPath(prefix+path)
            if not prim: raise ValueError('REPLAY_PRIM_MISSING:'+path)
            ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
            if (len(ops) != 1 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTransform
                    or not np.allclose(np.asarray(ops[0].Get()), np.eye(4), atol=1e-12, rtol=0)):
                raise ValueError('REPLAY_LINK_MESH_LOCAL_TRANSFORM_CHANGED:'+path)
            expected[path] = np.asarray(T)
    flange = state['frames']['link_6']; tool = snapshot['tool']; cad = flange@np.array(tool['T_flange_tool_cad'])
    for p in tool['visual_parts']: put('Tool/visual_'+p['id'], cad)
    for p in tool['collisions']: put('Tool/collision_'+p['id'], cad)
    for p in tool['adapter']:
        put('Adapter/'+p['id'], flange); put('Adapter/'+p['id']+'_collision', flange)
    for name, T in state['cartons'].items(): put('Cartons/'+name, T)
    for name, T in dict(state['frames'], flange=flange, tool_cad=cad,
                        tcp=flange@np.asarray(tool['T_flange_tcp'])).items():
        put('Frames/'+name, T)
    for row in snapshot['scene'].get('moving_attachments', []):
        path = 'Robot/'+row['parent_link']+'/NativeCollision/'+row['id']
        prim = stage.GetPrimAtPath(prefix+path)
        if not prim: raise ValueError('REPLAY_PERMANENT_ATTACHMENT_MISSING:'+row['id'])
        ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
        local = np.asarray(row['T_parent_local'])
        if (len(ops) != 1 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTransform
                or not np.allclose(np.asarray(ops[0].Get()).T, local, atol=1e-12, rtol=0)):
            raise ValueError('REPLAY_PERMANENT_ATTACHMENT_LOCAL_CHANGED:'+row['id'])
        expected[path] = state['frames'][row['parent_link']]@local
    for row in snapshot['scene'].get('native_world_objects', []):
        path = 'NativeWorldCollision/'+row['id']
        prim = stage.GetPrimAtPath(prefix+path)
        if not prim: raise ValueError('REPLAY_STATIC_CONVEX_MISSING:'+row['id'])
        expected[path] = np.asarray(row['pose'])
    cache = UsdGeom.XformCache(); W = np.asarray(snapshot['scene']['T_W_D']); maximum = 0.
    for path, T in expected.items():
        actual = np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(prefix+path))).T
        maximum = max(maximum, float(np.max(np.abs(actual-W@T))))
    if maximum > 1e-8: raise ValueError('REPLAY_USD_FK_OR_DOUBLE_WORLD_TRANSFORM')
    # TCP is a derived frame from the read-back flange; it is never an extra offset.
    actual_flange = np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(prefix+'Robot/link_6/Visual'))).T
    tcp_error = float(np.max(np.abs(actual_flange@np.array(tool['T_flange_tcp'])-W@flange@np.array(tool['T_flange_tcp']))))
    return dict(max_matrix_error=maximum, tcp_matrix_error=tcp_error, carton_count=len(state['cartons']))
