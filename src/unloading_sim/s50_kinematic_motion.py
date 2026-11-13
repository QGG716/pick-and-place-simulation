"""Small numerical helpers for the user-directed S50 kinematic flow."""
import math
import numpy as np


def posture(robot, q):
    frames=robot.named_link_frames(q)
    upper=frames['s50_link_3'][:3,3]-frames['s50_link_2'][:3,3]
    forearm=frames['s50_link_4'][:3,3]-frames['s50_link_3'][:3,3]
    wrist=frames['s50_link_5'][:3,3]-frames['s50_link_4'][:3,3]
    wrist=wrist/np.linalg.norm(wrist)
    return dict(j2_deg=float(np.rad2deg(q[1])),j3_deg=float(np.rad2deg(q[2])),
        upper_arm_forward_tilt_deg=float(np.rad2deg(np.arctan2(upper[0],upper[2]))),
        upper_arm_definition='J2_TO_J3_WORLD_XZ_TILT_FROM_VERTICAL_POSITIVE_TOWARD_STACK',
        forearm_elevation_deg=float(np.rad2deg(np.arctan2(forearm[2],np.linalg.norm(forearm[:2])))),
        forearm_definition='J3_ORIGIN_TO_J4_ORIGIN_IN_WORLD',
        wrist_direction_world=wrist.tolist(),
        wrist_down_error_deg=float(np.rad2deg(np.arccos(np.clip(-wrist[2],-1,1)))))


def posture_cost(robot,q,preferences,reference=None):
    m=posture(robot,q);lo,hi=preferences['j2_preferred_deg']
    excess=max(lo-m['j2_deg'],0,m['j2_deg']-hi)
    angle=max(0,abs(m['forearm_elevation_deg'])-preferences['forearm_horizontal_max_deg'])
    # These are costs only: no preferred angle changes the actual joint limits
    # or turns a collision/IK failure into an accepted candidate.
    cost=4*(excess/40)**2+4*(angle/30)**2+8*(m['wrist_down_error_deg']/90)**2
    forward=max(0.,m['upper_arm_forward_tilt_deg']-preferences.get('upper_arm_forward_free_deg',30.))
    cost+=preferences.get('upper_arm_large_forward_weight',0.)*(forward/30.)**2
    if reference is not None:cost+=.015*float(np.linalg.norm(np.asarray(q)-reference)**2)
    return cost


def receiving_height(box_pose,half_extents,minimum=.55,maximum=1.5,gap=.1):
    T=np.asarray(box_pose);bottom=float(T[2,3]-(np.abs(T[:3,:3])@np.asarray(half_extents))[2])
    requested=bottom-gap;height=float(np.clip(requested,minimum,maximum))
    return dict(carton_bottom_m=bottom,requested_surface_height_m=requested,
        surface_height_m=height,requested_gap_m=gap,actual_gap_m=bottom-height,
        clamped=abs(height-requested)>1e-10,minimum_surface_height_m=minimum,maximum_surface_height_m=maximum)


def _hermite(points,stations,s):
    h=np.diff(stations);slopes=np.diff(points,axis=0)/h[:,None]
    tangents=np.empty_like(points);tangents[0]=slopes[0];tangents[-1]=slopes[-1]
    if len(points)>2:
        # Shape-preserving harmonic slopes prevent revolute overshoot and
        # preserve monotone coordinates through the solved IK knots.
        a,b=slopes[:-1],slopes[1:];same=a*b>0
        w1=(2*h[1:]+h[:-1])[:,None];w2=(h[1:]+2*h[:-1])[:,None]
        safe_a=np.where(same,a,1.);safe_b=np.where(same,b,1.)
        tangents[1:-1]=np.where(same,(w1+w2)/(w1/safe_a+w2/safe_b),0.)
    s=np.atleast_1d(s);index=np.clip(np.searchsorted(stations,s,side='right')-1,0,len(h)-1)
    length=h[index,None];u=((s-stations[index])/h[index])[:,None]
    a,b=points[index],points[index+1];ma,mb=tangents[index],tangents[index+1]
    q=(2*u**3-3*u**2+1)*a+(u**3-2*u**2+u)*length*ma+(-2*u**3+3*u**2)*b+(u**3-u**2)*length*mb
    dq=((6*u*u-6*u)*a+(3*u*u-4*u+1)*length*ma+(-6*u*u+6*u)*b+(3*u*u-2*u)*length*mb)/length
    ddq=((12*u-6)*a+(6*u-4)*length*ma+(-12*u+6)*b+(6*u-2)*length*mb)/(length*length)
    return q,dq,ddq,index


def continuous_samples(path,velocity=.35,acceleration=.5,stations=None,minimum_duration=0.):
    """C1 path through actual IK knots, one global rest-to-rest time law.

    Interior knots have shared velocities. Every returned state must still be
    checked against actual scene geometry by the caller before it is emitted.
    """
    points=np.asarray(path,float)
    if stations is None:
        stations=np.r_[0,np.cumsum(np.maximum(np.linalg.norm(np.diff(points,axis=0),axis=1),1e-8))]
    stations=np.asarray(stations,float);stations=(stations-stations[0])/(stations[-1]-stations[0])
    def evaluate(u):
        s=10*u**3-15*u**4+6*u**5;ds=30*u*u*(1-u)**2;dds=60*u*(1-u)*(1-2*u)
        q,qs,qss,index=_hermite(points,stations,s)
        return q,qs*ds[:,None],qss*ds[:,None]**2+qs*dds[:,None],index,s
    _,v,a,_,_=evaluate(np.linspace(0,1,4097))
    duration=max(.2,float(minimum_duration),1.03*float(np.max(abs(v)))/velocity,
        math.sqrt(1.03*float(np.max(abs(a)))/acceleration))
    count=max(2,math.ceil(duration/.06),math.ceil(np.sum(np.max(abs(np.diff(points,axis=0)),axis=1))/.025))
    u=np.linspace(0,1,count+1);q,v,a,index,s=evaluate(u)
    rows=[dict(time_s=float(t*duration),q=p,velocity=v0/duration,acceleration=a0/duration**2,
        knot_index=int(i),progress=float(progress)) for t,p,v0,a0,i,progress in zip(u,q,v,a,index,s)]
    return rows,dict(method='SHAPE_PRESERVING_HERMITE_THROUGH_IK_WITH_ONE_QUINTIC_CLOCK',
        duration_s=duration,interior_stop_required=False,maximum_sampled_velocity_rad_s=float(np.max(abs(v))/duration),
        maximum_sampled_acceleration_rad_s2=float(np.max(abs(a))/duration**2),solver_knots=len(points))
