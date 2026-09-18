"""Asset-independent finite-task and analytic quintic acceptance checks."""
import copy,hashlib
import numpy as np
from .model import canonical_fingerprint,check_transform,URDF
from ..timing import JointMotionLimits

PHASES=('approach','contact','loaded_lift','loaded_transfer','place_contact','retreat')
FRONT_PHASES=('approach','contact','loaded_lift','front_separation','loaded_transfer','place_contact','retreat')
EVENTS=[('contact','ATTACHED'),('place_contact','SUPPORTED_RELEASE'),('retreat','RETREAT_COMPLETE')]

def runtime_fingerprint(w):
    h=hashlib.sha256(URDF.read_bytes())
    for value in (w.robot.base_transform,w.robot.tip_from_tcp,w.robot.joint_limits,w.model.mesh_vert,w.model.mesh_face,w.model.geom_size,w.model.geom_pos,w.model.geom_quat):
        h.update(np.asarray(value).tobytes())
    h.update(canonical_fingerprint(w.tool).encode())
    return h.hexdigest()

def motion_limits(scene,phase,official_velocity):
    cfg=scene.get('source_config',{}).get('task',scene)
    v=np.minimum(np.asarray(official_velocity,float),cfg['joint_velocity_rad_s'])
    a=np.full(len(v),cfg['joint_acceleration_rad_s2']);j=np.full(len(v),cfg['joint_jerk_rad_s3'])
    if phase in ('contact','place_contact'):
        v=np.minimum(v,.10);a=np.minimum(a,.3);j=np.minimum(j,1.)
    return JointMotionLimits(v,a,j,source='current scene limits capped by official URDF velocity')

def audit_task(artifact,declared,start,end,box_id,joint_limits,scene,official_velocity):
    """Reject malformed/full-task substitutions before querying CAD. No metadata timing trust."""
    errors=[];audits=[]
    try:
        a=copy.deepcopy(artifact);claimed=a.pop('trajectory_fingerprint')
        if claimed!=canonical_fingerprint(a):errors.append('artifact_hash')
        if a['box_id']!=box_id:errors.append('target_identity')
        for key in ('attachment_tcp_to_box','released_box_pose'):check_transform(a[key])
        phases=tuple(s['phase'] for s in a['segments'])
        if phases not in (PHASES,FRONT_PHASES):errors.append('task_phase_order_or_missing')
        # Legacy six-stage artifacts have implicit boundary events; version 2 declares them.
        if a.get('task_schema')==2:
            if [(e['after_phase'],e['event']) for e in a.get('required_events',[])]!=EVENTS:errors.append('required_events')
            if any(e['box_id']!=box_id for e in a['required_events']):errors.append('event_target_identity')
        limits=np.asarray(joint_limits,float);n=len(limits);previous=np.asarray(start,float)
        declared_q=[previous.tolist()]
        for s in a['segments']:
            q=np.asarray(s['q'],float);t=np.asarray(s['t'],float)
            if q.ndim!=2 or q.shape[1]!=n or len(q)<2 or t.shape!=(len(q),):raise ValueError('q_t_dimensions')
            if not np.isfinite(q).all() or not np.isfinite(t).all():raise ValueError('nonfinite_q_t')
            if abs(t[0])>1e-12 or np.any(np.diff(t)<=0):raise ValueError('timestamps')
            if not np.isfinite(s['duration_s']) or abs(s['duration_s']-t[-1])>1e-9:errors.append('duration_mismatch')
            if not np.allclose(previous,q[0],atol=1e-9,rtol=0):errors.append('segment_discontinuity')
            if np.any(q<limits[:,0]-1e-9) or np.any(q>limits[:,1]+1e-9):errors.append('joint_limits')
            # Rest-to-rest quintic is monotone in each joint. These are exact global edge extrema.
            dt=np.diff(t)[:,None];dq=np.abs(np.diff(q,axis=0));lim=motion_limits(scene,s['phase'],official_velocity)
            ratios=[float(np.max(f*dq/dt**order/bound)) for f,order,bound in [(1.875,1,lim.velocity),(10/np.sqrt(3),2,lim.acceleration),(60.,3,lim.jerk)]]
            if max(ratios)>1+1e-8:errors.append('independent_motion_limits:'+s['phase'])
            check_transform(s['target_tcp'])
            audits.append(dict(phase=s['phase'],velocity_ratio=ratios[0],acceleration_ratio=ratios[1],jerk_ratio=ratios[2],stopped_knots=True))
            declared_q.extend(q[1:].tolist());previous=q[-1]
        if not np.allclose(previous,end,atol=1e-9,rtol=0):errors.append('end_boundary')
        declared=np.asarray(declared,float);actual=np.asarray(declared_q,float)
        if declared.shape!=actual.shape or not np.isfinite(declared).all() or not np.allclose(declared,actual,atol=1e-9,rtol=0):errors.append('declared_trajectory_mismatch')
    except (KeyError,TypeError,ValueError,IndexError) as exc:errors.append('malformed:'+str(exc))
    return dict(valid=not errors,errors=errors,timing=audits,method='analytic quintic extrema from current limits; finite task state machine')


def current_kinematics_match_source(w):
    from .model import robot_for
    expected=robot_for(w.tool,w.scene)
    for probe in (np.zeros(len(expected.joint_limits)),np.linspace(.05,.3,len(expected.joint_limits))):
        actual=w.robot.named_link_frames(probe);reference=expected.named_link_frames(probe)
        if actual.keys()!=reference.keys() or any(not np.allclose(actual[k],reference[k],rtol=0,atol=1e-12) for k in actual):return False
    return np.allclose(w.robot.tip_from_tcp,expected.tip_from_tcp,rtol=0,atol=1e-12) and np.array_equal(w.robot.joint_limits,expected.joint_limits)


def measure_motion_envelope(w):
    """Conservative mesh-box envelope; exact vertices refine any desk-boundary escape."""
    if not hasattr(w,'_envelope_shapes'):
        shapes=[]
        for e in w.entries:
            if e['kind'] not in ('robot','visual','adapter','payload'):continue
            gid=e['gid'];kind=w.model.geom_type[gid];size=w.model.geom_size[gid];points=None
            if kind==w.mj.mjtGeom.mjGEOM_MESH:
                mid=w.model.geom_dataid[gid];a=w.model.mesh_vertadr[mid];n=w.model.mesh_vertnum[mid];points=w.model.mesh_vert[a:a+n]
                lo,hi=points.min(axis=0),points.max(axis=0);center=(lo+hi)/2;half=(hi-lo)/2
            elif kind==w.mj.mjtGeom.mjGEOM_CYLINDER:center=np.zeros(3);half=np.array([size[0],size[0],size[1]])
            else:center=np.zeros(3);half=size.copy()
            shapes.append((gid,e['name'],center,half,points,kind))
        w._envelope_shapes=shapes
    lows=[];highs=[];outside=[];desk=w.scene.get('source_config',{}).get('table',{}).get('bounds_xy_m')
    for gid,name,c,h,points,kind in w._envelope_shapes:
        R=w.data.geom_xmat[gid].reshape(3,3);p=w.data.geom_xpos[gid];center=R@c+p;extent=np.abs(R)@h
        if kind==w.mj.mjtGeom.mjGEOM_CYLINDER:extent=h[0]*np.sqrt(R[:,0]**2+R[:,1]**2)+h[2]*np.abs(R[:,2])
        lo,hi=center-extent,center+extent
        def escapes():return desk is not None and (lo[0]<desk[0]-1e-7 or hi[0]>desk[1]+1e-7 or lo[1]<desk[2]-1e-7 or hi[1]>desk[3]+1e-7)
        if escapes() and points is not None:
            actual=points@R.T+p;lo,hi=actual.min(axis=0),actual.max(axis=0)
        if escapes():outside.append(name)
        lows.append(lo);highs.append(hi)
    lo=np.min(lows,axis=0);hi=np.max(highs,axis=0)
    if not hasattr(w,'motion_envelope'):w.motion_envelope=dict(min_xyz_m=lo.tolist(),max_xyz_m=hi.tolist(),desk_escape_geometry=[],samples=0)
    report=w.motion_envelope;report['min_xyz_m']=np.minimum(report['min_xyz_m'],lo).tolist();report['max_xyz_m']=np.maximum(report['max_xyz_m'],hi).tolist()
    report['desk_escape_geometry']=sorted(set(report['desk_escape_geometry'])|set(outside));report['samples']+=1
    report['desk_xy_containment_checked']=desk is not None;report['inside_design_xy']=not report['desk_escape_geometry']
    report['scope']='robot, actual tool/adapter and cartons; sampled geometry, not personnel safety or desk load qualification'
    return report


def outfeed_preconditions(w,replay):
    """A completed event chain must belong to this actual world and this carton."""
    try:
        events=replay['events']
        if replay['status']!='GEOMETRIC_PICK_PLACE_COMPLETE' or replay['geometric_tasks_completed']!=1:return False
        if [e['event'] for e in events]!=['ATTACHED','SUPPORTED_RELEASE','RETREAT_COMPLETE']:return False
        if {e['box_id'] for e in events}!={w.box_id}:return False
        if replay['execution_id']!=getattr(w,'execution_id',None):return False
        if w.attached or not w.released or w.box_states[w.box_id]!='SUPPORTED_RELEASE':return False
        if not w.release_support['valid']:return False
        return bool(np.allclose(w.current_q,events[-1]['q'],atol=1e-9,rtol=0) and np.allclose(w.released_pose,events[1]['pose'],atol=1e-9,rtol=0))
    except (KeyError,AttributeError,TypeError,ValueError):return False
