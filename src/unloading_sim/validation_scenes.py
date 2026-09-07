"""V2-identical carton populations driven by the V3 effective configuration."""
import numpy as np
from .geometry import OBB


def box(name,center,size):
    return OBB(np.asarray(center,float),np.asarray(size,float)/2,np.eye(3),name,'carton')


def inclusive(lower,upper,step):
    values=lower+step*np.arange(int(np.floor((upper-lower)/step+1e-9))+1)
    return np.append(values,upper) if values[-1]<upper-1e-9 else np.r_[values[:-1],upper]


def inside(target,s):
    points=target.corners()
    return bool(np.min(points[:,1])>=-s['trailer_width_m']/2-1e-9 and np.max(points[:,1])<=s['trailer_width_m']/2+1e-9
                and np.min(points[:,2])>=-1e-9 and np.max(points[:,2])<=s['trailer_height_m']+1e-9)


def grid_tasks(s):
    for z in inclusive(0,s['trailer_height_m'],s['grid_step_m']):
        for y in inclusive(-s['trailer_width_m']/2,s['trailer_width_m']/2,s['grid_step_m']):
            for orientation,size in s['box_sizes_m'].items():
                target=box(f'target_{orientation}',[s['front_x_m']+size[0]/2,y,z],size)
                neighbors=[box(name,target.center+offset,size) for name,offset in (
                    ('left_neighbor',[0,size[1]+s['neighbor_gap_m'],0]),
                    ('right_neighbor',[0,-size[1]-s['neighbor_gap_m'],0]),
                    ('bottom_support',[0,0,-size[2]-s['neighbor_gap_m']]))]
                # Match V2's exact lower-support/side-bound filtering. Roof
                # validity is evaluated independently, not silently filtered.
                neighbors=[b for b in neighbors if abs(b.center[1])+b.half_extents[1]<=s['trailer_width_m']/2
                           and b.center[2]-b.half_extents[2]>=0]
                yield orientation,target,neighbors,inside(target,s)


def regular_scene(s):
    size=next(iter(s['box_sizes_m'].values()))
    return [box(f'regular_r{row}_c{col}',[s['front_x_m']+size[0]/2,y,size[2]/2+s['row_pitch_m']*row],size)
            for row in range(s['regular_rows']) for col,y in enumerate(np.linspace(*s['regular_y_limits_m'],s['regular_columns']))]


def random_scene(s,seed):
    rng=np.random.default_rng(seed);boxes=[];sizes=list(s['box_sizes_m'].values())
    for row in range(s['regular_rows']):
        cursor=s['random_start_y_m'];col=0
        while cursor<s['random_stop_y_m']:
            size=np.asarray(sizes[0] if rng.random()<s['random_type_a_probability'] else sizes[1])
            y=cursor+size[1]/2
            if y+size[1]/2>s['random_max_edge_y_m']:break
            if rng.random()>s['random_skip_probability']:
                front=s['front_x_m']+rng.uniform(*s['random_front_offset_limits_m'])
                boxes.append(box(f'random{seed}_r{row}_c{col}',[front+size[0]/2,y,size[2]/2+s['row_pitch_m']*row],size))
            cursor+=size[1]+rng.uniform(*s['random_gap_limits_m']);col+=1
    return boxes
