"""Engineering suction estimate from actual CAD rings and sent-state FK.

This is not pull-off calibration or evidence of independent valve control.
Passive cups which are not anticipated to seal contribute zero force.
"""
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
    centers=np.array([np.asarray(candidate['rings_tcp'][i]).mean(0) for i in active]);centroid=centers.mean(0)
    contact=np.asarray(candidate['tcp_pose'] if contact_tcp is None else contact_tcp,float)
    poses=np.asarray([contact] if poses is None else poses,float)
    if poses.ndim!=3 or poses.shape[1:]!=(4,4) or not np.isfinite(poses).all():raise ValueError('HOLDING_CAPACITY_FK_INVALID')
    n=-np.asarray(candidate['suction_normal_tcp'],float);n=n/np.linalg.norm(n)
    u=contact[:3,:3].T@np.asarray(candidate['face']['u']);u-=n*(n@u);u/=np.linalg.norm(u);v=np.cross(n,u)
    arms=np.cross(centers-centroid,n)
    A=np.vstack([np.ones(len(active)),arms@u,arms@v]);inverse=np.linalg.pinv(A)
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
        rhs=np.c_[normal,moment@u,moment@v];demands=rhs@inverse.T
        residual=np.linalg.norm(demands@A.T-rhs,axis=1)
        ratio=np.maximum(demands,0)*sf/force
        peel=np.divide(1.,ratio.max(1),out=np.full(len(poses),1e12),where=ratio.max(1)>0)
        peel[residual>1e-8]=0
        pull=np.divide(total,sf*normal,out=np.full(len(poses),1e12),where=normal>0)
        shear=np.divide(mu*(total-normal),sf*tangent,out=np.full(len(poses),1e12),where=tangent>0)
        margin=np.minimum.reduce([pull,shear,peel]);index=int(np.argmin(margin))
        record=dict(com_source=name,sent_state_index=index,normal_load_n=float(normal[index]),
            tangential_load_n=float(tangent[index]),normal_margin=float(pull[index]),
            tangential_margin=float(shear[index]),peel_margin=float(peel[index]),minimum_margin=float(margin[index]),
            peel_moment_tcp_nm=moment[index].tolist(),cup_tension_n=demands[index].tolist(),
            cup_allowed_tension_n=(force/sf).tolist(),unbalanced_peel_residual=float(residual[index]),
            com_tcp_m=com.tolist(),linear_acceleration_world_m_s2=accel[index].tolist())
        if worst is None or record['minimum_margin']<worst['minimum_margin']:worst=record
    valid=worst['minimum_margin']>=1
    return dict(status='PASS' if valid else 'HOLDING_CAPACITY_INSUFFICIENT',valid=valid,
        minimum_margin=worst['minimum_margin'],tier=tier(worst['minimum_margin']),worst=worst,
        active_ids=active,cad_ring_area_m2=area.tolist(),cup_design_force_n=force.tolist(),total_suction_n=total,
        mass_kg=mass,policy=policy,maximum_linear_acceleration_m_s2=max_accel,acceleration_basis=accel_basis,
        hardware_validation='NOT_RUN')


def sent_poses(planned,model):
    states=[];times=[];offset=0.
    for key in ('extraction','returning'):
        points=planned[key]['prepared']['points']
        for point in points:
            t=offset+point['t']
            if times and t<=times[-1]:continue
            times.append(t);states.append(model.tcp(point['q']))
        offset=times[-1]
    return np.asarray(states),np.asarray(times)
