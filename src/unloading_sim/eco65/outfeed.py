"""Finite ideal -Y/-X conveyance with actual inclined-surface support envelope.

Adapted from reference joint_search_outfeed at 4d191ec. The rigid discharge
tray is explicit geometry; no support plane is extended beyond a real belt.
"""
import math
import numpy as np
from .transverse_process import corners
from .model import check_transform


def support_height(pose,size,spec,state):
    v=corners(dict(pose=pose,size_m=size));lo,hi=v.min(0),v.max(0)
    bounds=np.asarray(state['receiving']['bounds_D_m']);upper=np.array(state['mechanism']['upper_endpoint_D_m']);lower=np.array(state['mechanism']['lower_endpoint_D_m'])
    values=[];ids=[];margin=.010
    if min(hi[0],bounds[1,0])-max(lo[0],bounds[0,0])>1e-9 and lo[1]>=bounds[0,1]+margin-1e-9 and hi[1]<=bounds[1,1]-margin+1e-9:
        values.append(state['height_m']);ids.extend(state['receiving']['support_ids'])
    half=spec['longitudinal_width_m']/2;y=spec['longitudinal_center_y_m']
    if lo[1]>=y-half+margin-1e-9 and hi[1]<=y+half-margin+1e-9:
        a,b=max(lo[0],lower[0]),min(hi[0],upper[0])
        if b-a>1e-9:
            slope=(upper[2]-lower[2])/(upper[0]-lower[0]);x=b if slope>=0 else a
            values.append(upper[2]+(x-upper[0])*slope)
            ids.extend(o['id'] for o in spec['belt_templates'] if o['owner']=='conveyor_longitudinal')
        if min(hi[0],lower[0])-max(lo[0],lower[0]-spec['config']['discharge_tray_length_m'])>1e-9:
            values.append(lower[2]);ids.append('eco65_discharge_tray')
    if not values:raise ValueError('IDEAL_OUTFEED_NO_REAL_SUPPORT_OVERLAP')
    return float(max(values)),sorted(ids)


def outfeed_path(pose,size,spec,state,config):
    p=check_transform(pose).copy();size=np.asarray(size,float)
    if not np.allclose(p[:3,:3],np.eye(3),atol=1e-6,rtol=0):raise ValueError('IDEAL_OUTFEED_LEVEL_ORIENTATION_REQUIRED')
    step=config['sample_step_m'];speed=config['display_speed_m_s']
    if not 0<step<=.010 or speed<=0:raise ValueError('IDEAL_OUTFEED_RESOLUTION_OR_SPEED')
    y=min(float(p[1,3]),spec['longitudinal_center_y_m'])
    exit_x=config['output_plane_x_m']-size[0]/2-.001
    vertices=[p[:3,3].copy(),np.array([p[0,3],y,p[2,3]]),np.array([exit_x,y,p[2,3]])]
    samples=[dict(t=0.,pose=p.tolist(),support_ids=support_height(p,size,spec,state)[1])];elapsed=0.
    stretch=math.sqrt(1+math.tan(state['tilt_rad'])**2)
    for a,b in zip(vertices,vertices[1:]):
        distance=np.linalg.norm(b[:2]-a[:2])
        if distance<1e-10:continue
        n=max(1,math.ceil(distance*stretch/step))
        for f in np.linspace(0,1,n+1)[1:]:
            q=p.copy();q[:2,3]=(a+f*(b-a))[:2]
            bottom,supports=support_height(q,size,spec,state);q[2,3]=bottom+size[2]/2
            elapsed+=np.linalg.norm(q[:3,3]-p[:3,3])/speed;p=q
            samples.append(dict(t=float(elapsed),pose=p.tolist(),support_ids=supports))
    if len(samples)>256:raise ValueError('IDEAL_OUTFEED_SAMPLE_RESOURCE_LIMIT')
    if max(corners(dict(pose=p,size_m=size))[:,0])>=config['output_plane_x_m']:raise ValueError('IDEAL_OUTFEED_FULL_ENVELOPE_NOT_OUTSIDE')
    return dict(schema='eco65_checked_ideal_outfeed_v1',samples=samples,duration_s=float(elapsed),
        maximum_step_m=step,output_plane_x_m=config['output_plane_x_m'],
        model='LEVEL_CARTON_SUPPORT_ENVELOPE_NO_DYNAMICS',tool_timing='WITHDRAW_THEN_WAIT_FULL_OUTFEED',
        physical_execution='NOT_RUN',physical_outfeed=False)
