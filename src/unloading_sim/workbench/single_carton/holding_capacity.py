"""Engineering suction estimate from actual CAD rings and sent-state FK.

This is not pull-off calibration or evidence of independent valve control.
Passive cups which are not anticipated to seal contribute zero force.
"""
import cv2
import numpy as np
from .geometry import transform
from .measured_envelope import measured_geometry

BASIS='工程估计，待现场拉脱试验标定'
GRAVITY_M_S2=9.80665


def settings(config):
    c=config.get('holding_capacity')
    if c is None:c=dict(friction_coefficient=.5,safety_factor=2.,effective_area_factor=.8,basis=BASIS)
    if not isinstance(c,dict) or not isinstance(c.get('basis'),str) or not c['basis'].strip():
        raise ValueError('HOLDING_CAPACITY_BASIS_REQUIRED')
    out=dict(c)
    for name,bound,direction in [('friction_coefficient',.5,'max'),('safety_factor',2.,'min'),('effective_area_factor',.8,'max')]:
        value=c.get(name)
        if (type(value) not in (float,int) or not np.isfinite(value) or value<=0 or
                (value>bound if direction=='max' else value<bound)):
            raise ValueError('HOLDING_CAPACITY_PARAMETER_INVALID:'+name)
    force=c.get('cup_suction_force_n')
    if force is not None:
        # User-stated per-cup suction force (2026-11-20: 42 N). It replaces the
        # area x pressure x area-factor estimate; the safety factor and friction
        # are unchanged. Bounded by CAD area x |grip threshold| in evaluate().
        if type(force) not in (float,int) or not np.isfinite(force) or force<=0:
            raise ValueError('HOLDING_CAPACITY_PARAMETER_INVALID:cup_suction_force_n')
        if not isinstance(c.get('cup_suction_force_basis'),str) or not c['cup_suction_force_basis'].strip():
            raise ValueError('HOLDING_CAPACITY_CUP_FORCE_BASIS_REQUIRED')
        grip=config.get('vacuum',{}).get('grip_below_kpa',-60.)
        out.update(cup_suction_force_n=float(force),grip_pressure_bound_pa=abs(float(grip))*1000)
    source=c.get('design_pressure_source','vacuum.hold_below_kpa')
    if source!='vacuum.hold_below_kpa':raise ValueError('HOLDING_CAPACITY_PRESSURE_SOURCE_INVALID')
    pressure=config.get('vacuum',{}).get('hold_below_kpa',-30.)
    if type(pressure) not in (float,int) or not np.isfinite(pressure) or not -30<=pressure<0:
        raise ValueError('HOLDING_CAPACITY_DESIGN_PRESSURE_INVALID')
    out.update(design_pressure_source=source,design_pressure_pa=abs(pressure)*1000)
    return out


def tier(margin):
    if not np.isfinite(margin) or margin<1:return -1
    return 2 if margin>=2 else 1 if margin>=1.5 else 0


def priority(clear,margin,posture_key):
    return (not clear,-tier(margin) if clear else 0,posture_key if clear else ())


def ring_area(ring):
    ring=np.asarray(ring,float)
    if ring.ndim!=2 or ring.shape[1]!=3 or len(ring)<3 or not np.isfinite(ring).all():
        raise ValueError('HOLDING_CAPACITY_INVALID_CAD_RING')
    area=float(np.linalg.norm(np.cross(ring,np.roll(ring,-1,axis=0)).sum(0))/2)
    if area<=0:raise ValueError('HOLDING_CAPACITY_ZERO_CAD_AREA')
    return area


def evaluate(box,candidate,tool,config,poses=None,times=None,contact_tcp=None):
    policy=settings(config);mass=config['carton_mass_kg']
    if type(mass) not in (int,float) or not np.isfinite(mass) or mass<=0:raise ValueError('CARTON_MASS_INVALID')
    active=candidate['active_ids'];seals={s['id']:s for s in tool['seals']}
    if not active or len(set(active))!=len(active) or not set(active)<=set(seals):
        raise ValueError('HOLDING_CAPACITY_ACTIVE_CUPS_INVALID')
    # Areas come from source CAD contours, not a nominal cup diameter or mesh
    # bounding rectangle. Candidate TCP rings place the same rigid contours.
    area=np.array([ring_area(seals[i]['ring_cad_m']) for i in active])
    force=area*policy['effective_area_factor']*policy['design_pressure_pa']
    force_basis='CAD_RING_AREA_X_EFFECTIVE_AREA_FACTOR_X_HOLD_PRESSURE'
    if policy.get('cup_suction_force_n') is not None:
        if np.any(policy['cup_suction_force_n']>area*policy['grip_pressure_bound_pa']+1e-9):
            raise ValueError('HOLDING_CAPACITY_CUP_FORCE_EXCEEDS_CAD_AREA_AT_GRIP_PRESSURE')
        force=np.full(len(active),policy['cup_suction_force_n']);force_basis='USER_STATED_PER_CUP_SUCTION_FORCE'
    centers=np.array([np.asarray(candidate['rings_tcp'][i]).mean(0) for i in active]);centroid=centers.mean(0)
    contact=np.asarray(candidate['tcp_pose'] if contact_tcp is None else contact_tcp,float)
    poses=np.asarray([contact] if poses is None else poses,float)
    if poses.ndim!=3 or poses.shape[1:]!=(4,4) or not np.isfinite(poses).all():raise ValueError('HOLDING_CAPACITY_FK_INVALID')
    n=-np.asarray(candidate['suction_normal_tcp'],float);n=n/np.linalg.norm(n)
    u=contact[:3,:3].T@np.asarray(candidate['face']['u']);u-=n*(n@u);u/=np.linalg.norm(u);v=np.cross(n,u)
    # Tipping footprint: the convex hull of every sealing CAD ring at contact,
    # in the face plane. A cup ring resists a moment across its own diameter;
    # point cups in one row would have no lever at all.
    rings=np.vstack([np.asarray(candidate['rings_tcp'][i],float) for i in active])
    planar=np.c_[(rings-centroid)@u,(rings-centroid)@v].astype(np.float32)
    hull=cv2.convexHull(planar).reshape(-1,2).astype(float)
    sf=policy['safety_factor'];mu=policy['friction_coefficient'];total=float(force.sum())
    worlds=[('collision_envelope',np.asarray(box['pose'])[:3,3]),('measured_box',np.asarray(measured_geometry(box)['pose'])[:3,3])]
    worst=None;max_accel=0.;accel_basis='STATIC_ONE_G'
    for name,com_world in worlds:
        com=transform(np.linalg.inv(contact),[com_world])[0]
        trajectory=np.einsum('nij,j->ni',poses[:,:3,:3],com)+poses[:,:3,3]
        accel=np.zeros_like(trajectory)
        if times is not None:
            t=np.asarray(times,float)
            if len(t)!=len(poses) or len(t)<3 or not np.isfinite(t).all() or np.any(np.diff(t)<=0):
                raise ValueError('HOLDING_CAPACITY_SENT_TIMES_INVALID')
            accel=np.gradient(np.gradient(trajectory,t,axis=0,edge_order=2),t,axis=0,edge_order=2)
            max_accel=max(max_accel,float(np.linalg.norm(accel,axis=1).max()))
            accel_basis='ACTUAL_SENT_STATE_FK_COM_SECOND_DIFFERENCE'
        load_world=mass*(np.array([0.,0.,-GRAVITY_M_S2])-accel)
        load=np.einsum('ni,nij->nj',load_world,poses[:,:3,:3])
        normal=np.abs(load@n);tangent=np.linalg.norm(load-(load@n)[:,None]*n,axis=1)
        moment=-np.cross(com-centroid,load)
        # Away from the tool = the side of the COM. Suction pulls the face
        # toward the tool at each cup centre; the load acts at the COM.
        away=n*(1. if (com-centroid)@n>=0 else -1.)
        # All suction forces share one direction, so their moment about each
        # hull edge is that of the resultant at the force-weighted centre.
        a2=hull;b2=np.roll(hull,-1,axis=0);keep=np.linalg.norm(b2-a2,axis=1)>1e-9;a2,b2=a2[keep],b2[keep]
        pa=centroid+np.outer(a2[:,0],u)+np.outer(a2[:,1],v)
        e=np.outer((b2-a2)[:,0],u)+np.outer((b2-a2)[:,1],v);e/=np.linalg.norm(e,axis=1)[:,None]
        weighted=(force[:,None]*centers).sum(0)
        suction=np.einsum('ej,ej->e',np.cross(weighted-total*pa,-away),e)
        over=np.einsum('nej,ej->ne',np.cross((com-pa)[None,:,:],load[:,None,:]),e)
        tipping=over*suction[None,:]<0
        m=np.where(tipping,abs(suction)[None,:]/np.maximum(sf*abs(over),1e-12),1e12)
        peel=m.min(1);edge_used=int(np.argmin(m.min(0))) if len(a2) else None
        demands=np.zeros((len(poses),len(active)));residual=np.zeros(len(poses))
        pull=np.divide(total,sf*normal,out=np.full(len(poses),1e12),where=normal>0)
        shear=np.divide(mu*(total-normal),sf*tangent,out=np.full(len(poses),1e12),where=tangent>0)
        margin=np.minimum.reduce([pull,shear,peel]);index=int(np.argmin(margin))
        record=dict(com_source=name,sent_state_index=index,normal_load_n=float(normal[index]),
            tangential_load_n=float(tangent[index]),normal_margin=float(pull[index]),
            tangential_margin=float(shear[index]),peel_margin=float(peel[index]),minimum_margin=float(margin[index]),
            peel_moment_tcp_nm=moment[index].tolist(),cup_allowed_force_n=(force/sf).tolist(),
            peel_model='TIPPING_ABOUT_CONVEX_HULL_OF_SEALING_CAD_RINGS',tipping_edge_index=edge_used,
            footprint_hull_face_m=hull.tolist(),
            com_tcp_m=com.tolist(),linear_acceleration_world_m_s2=accel[index].tolist())
        if worst is None or record['minimum_margin']<worst['minimum_margin']:worst=record
    valid=worst['minimum_margin']>=1
    return dict(status='PASS' if valid else 'HOLDING_CAPACITY_INSUFFICIENT',valid=valid,
        minimum_margin=worst['minimum_margin'],tier=tier(worst['minimum_margin']),worst=worst,
        active_ids=active,cad_ring_area_m2=area.tolist(),cup_design_force_n=force.tolist(),total_suction_n=total,
        cup_force_basis=force_basis,
        mass_kg=mass,policy=policy,maximum_linear_acceleration_m_s2=max_accel,acceleration_basis=accel_basis,
        hardware_validation='NOT_RUN')


def _axis_angle_batch(axis,values,joint_type):
    """robot._axis_angle_transform for many joint values: the same expressions per element."""
    t=np.zeros((len(values),4,4));t[:,0,0]=t[:,1,1]=t[:,2,2]=t[:,3,3]=1.
    axis=np.asarray(axis,dtype=float);norm=float(np.linalg.norm(axis))
    if norm<1e-12:raise ValueError('joint axis must be non-zero')
    axis=axis/norm
    if joint_type=='prismatic':
        t[:,:3,3]=axis*values[:,None];return t
    x,y,z=axis;c=np.cos(values);s=np.sin(values);c1=1.0-c
    t[:,0,0]=c+x*x*c1;t[:,0,1]=x*y*c1-z*s;t[:,0,2]=x*z*c1+y*s
    t[:,1,0]=y*x*c1+z*s;t[:,1,1]=c+y*y*c1;t[:,1,2]=y*z*c1-x*s
    t[:,2,0]=z*x*c1-y*s;t[:,2,1]=z*y*c1+x*s;t[:,2,2]=c+z*z*c1
    return t


def tcp_batch(model,qs):
    """``model.tcp(q)`` for every row of ``qs`` (2026-11-22).

    The same URDF chain, operands and operation order per state as
    CoordinateModel.tcp (bitwise identical results, see the directed test),
    with the same finiteness and official-limit checks, on stacked arrays.
    A model without the parsed chain (test doubles) is evaluated per state.
    """
    robot=getattr(model,'robot',None)
    if robot is None or not hasattr(robot,'joints') or getattr(robot,'validation_kernel',None) is not None:
        return np.asarray([model.tcp(q) for q in qs])
    qs=np.asarray(qs,dtype=float)
    if qs.ndim!=2 or qs.shape[1]!=6 or not np.isfinite(qs).all():raise ValueError('q_rad must be finite 6-vectors')
    limits=robot.joint_limits
    if not (np.all(qs>=limits[:,0]-1e-9) and np.all(qs<=limits[:,1]+1e-9)):
        raise ValueError("Measured or replay joints exceed official URDF limits")
    index={name:k for k,name in enumerate(robot.active_joint_names)}
    T=np.broadcast_to(robot.base_transform,(len(qs),4,4)).copy();link6=None
    for joint in robot.joints:
        T=T@joint.origin
        if joint.name in index:T=T@_axis_angle_batch(joint.axis,qs[:,index[joint.name]],joint.joint_type)
        if joint.child=='link_6':link6=T
    if link6 is None:raise ValueError('LINK_6_NOT_ON_CHAIN')
    return model.world_from_base@link6@model.tool.matrix


def sent_poses(planned,model):
    states=[];times=[];offset=0.
    for key in ('extraction','returning'):
        points=planned[key]['prepared']['points']
        for point in points:
            t=offset+point['t']
            if times and t<=times[-1]:continue
            times.append(t);states.append(point['q'])
        offset=times[-1]
    return tcp_batch(model,states),np.asarray(times)
