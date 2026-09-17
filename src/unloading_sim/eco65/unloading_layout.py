"""Design-only desktop unloading layout. No RRT, scheduler, transport or hardware."""
from __future__ import annotations
import copy,hashlib,json,subprocess,time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from .model import ROOT,URDF,LOCAL,World,asset_check,transform,check_transform,save_json,load_json,canonical_fingerprint,require_simulation
from ..ik import solve_ik

CONFIG=ROOT/'configs/workcells/eco65_desktop_unloading_layout_v1.yaml'
SOURCE_SHA='9db77cb8a51e9bf1821c6e84e90a7632fbce6b26'
FACE_AXES={'top':(2,1),'front':(0,-1),'right_facing':(1,1)}
FACE_ROTATIONS={
 'top':np.diag([1.,-1.,-1.]),
 'front':np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]]),
 'right_facing':np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])}

def bounds(obj):
    center=np.array(obj.get('position_m',obj.get('pose')),float);half=np.array(obj['size_m'])/2
    return center-half,center+half

def build_scene(config):
    c=copy.deepcopy(config);v=c['validation'];require_simulation(v)
    assert c['robot']['geometry_scale']==1.0 and c['robot']['mobility']=='fixed'
    assert all(c['trailer']['panels'].values()),'This layout requires all four enclosing panels'
    check_transform(c['world']['T_world_installation'])
    assert np.allclose(c['world']['T_world_installation'],np.eye(4)), 'Nonidentity installation frame requires explicit conversion'
    t=c['trailer'];f=t['floor_thickness_m'];z0=c['world']['tabletop_z_m'];floor=z0+f
    opening=t['opening_x_m'];rear=t['closed_end_inner_x_m'];half=t['inner_width_m']/2;th=t['wall_thickness_m'];height=t['inner_height_m']
    components=[];surfaces=[]
    def part(name,size,position,role,color,owner=None,support=False):
        o=dict(id=name,size_m=list(map(float,size)),position_m=list(map(float,position)),role=role,rgba=color,owner=owner or name)
        components.append(o)
        if support:surfaces.append(name)
        return o
    def rectangular(name,xy,bottom,top,role,color,owner=None,support=False):
        x0,x1,y0,y1=xy
        return part(name,[x1-x0,y1-y0,top-bottom],[(x0+x1)/2,(y0+y1)/2,(top+bottom)/2],role,color,owner,support)
    rectangular('trailer_floor',[opening,rear+th,-half-th,half+th],z0,floor,'floor',[.43,.46,.49,1],support=True)
    for side,y in [('left',half+th/2),('right',-half-th/2)]:
        part('trailer_'+side,[rear-opening,th,height],[(rear+opening)/2,y,floor+height/2],'wall',[.50,.65,.76,.15])
    part('trailer_rear',[th,2*(half+th),height],[rear+th/2,0,floor+height/2],'wall',[.50,.65,.76,.25])
    rectangular('trailer_roof',[opening,rear+th,-half-th,half+th],floor+height,floor+height+th,'roof',[.65,.75,.83,.10])
    plate=c['robot']['mounting_plate_size_m'];base=[*c['robot']['base_xy_m'],floor+plate[2]]
    part('robot_mount_plate',plate,[*base[:2],floor+plate[2]/2],'mount',[.38,.40,.42,1])
    belts=c['conveyors'];belt_z=belts['surface_z_m'];belt_th=belts['belt_thickness_m'];body_th=belts['body_thickness_m'];leg=belts['leg_width_m']
    def conveyor(name,xy,ground,color):
        x0,x1,y0,y1=xy;owner=name
        rectangular(name+'_belt',xy,belt_z-belt_th,belt_z,'belt',color,owner,True)
        rectangular(name+'_body',xy,belt_z-belt_th-body_th,belt_z-belt_th,'frame',[.28,.31,.34,1],owner)
        bottom=belt_z-belt_th-body_th
        assert bottom>ground
        for i,x in enumerate([x0+leg,x1-leg]):
            for j,y in enumerate([y0+leg,y1-leg]):
                part(f'{name}_support_{i}{j}',[leg,leg,bottom-ground],[x,y,(ground+bottom)/2],'leg',[.40,.42,.45,1],owner)
    conveyor('conveyor_transverse',belts['transverse']['bounds_xy_m'],floor,[.10,.55,.68,1])
    conveyor('conveyor_longitudinal',belts['longitudinal']['bounds_xy_m'],floor,[.15,.61,.39,1])
    rectangular('transfer_bridge',belts['transfer']['bridge_bounds_xy_m'],belt_z-belts['transfer']['bridge_thickness_m'],belt_z,'bridge',[.9,.7,.18,1],'conveyor_longitudinal',True)
    rectangular('outlet_bridge',belts['outlet']['bridge_bounds_xy_m'],belt_z-.006,belt_z,'bridge',[.9,.7,.18,1],'outlet',True)
    conveyor('outlet_catch',belts['outlet']['catch_bounds_xy_m'],z0,[.45,.67,.49,1])
    stack=c['carton_stack'];size=stack['carton_size_xyz_m'];assert stack['layer_gap_m']==0 and stack['depth_rows']==1
    boxes=[];cols=stack['width_columns'];layers=stack['height_layers']
    for layer in range(layers):
        for col in range(cols):
            ident=f'carton_r00_l{layer:02d}_c{col:02d}'
            below=f'carton_r00_l{layer-1:02d}_c{col:02d}' if layer else 'trailer_floor'
            above=[f'carton_r00_l{layer+1:02d}_c{col:02d}'] if layer<layers-1 else []
            boxes.append(dict(id=ident,size_m=list(size),pose=[stack['front_face_x_m']+size[0]/2,(col-(cols-1)/2)*(size[1]+stack['column_gap_m']),floor+(layer+.5)*size[2]],
                mass_kg=stack['mass_kg_assumed'],row=0,layer=layer,column=col,supported_by=[below],supports=above,
                evidence='DESIGN_CANDIDATE known-pose, no camera',rgba=[.69+.025*layer,.45+.04*layer,.23,1]))
    selected=next(b for b in boxes if b['layer']==layers-1 and b['column']==cols//2)
    lo=np.min([bounds(o)[0] for o in components],axis=0);hi=np.max([bounds(o)[1] for o in components],axis=0)
    # This is a required support envelope, not a measurement of the user's desk.
    rectangular('required_tabletop_envelope',[lo[0],hi[0],lo[1],hi[1]],z0-.03,z0,'table_envelope',[.58,.51,.42,1])
    routes={
        'A':dict(receiving='conveyor_transverse',process=['transverse','longitudinal'],centers_xy_m=[belts['transverse']['receive_center_xy_m'],[(sum(belts['transverse']['bounds_xy_m'][:2]))/2,sum(belts['longitudinal']['bounds_xy_m'][2:])/2],belts['outlet']['final_center_xy_m']]),
        'B':dict(receiving='conveyor_longitudinal',process=['longitudinal'],centers_xy_m=[belts['longitudinal']['receive_center_xy_m'],belts['outlet']['final_center_xy_m']])}
    return dict(schema=c['schema'],status='DESIGN_CANDIDATE',execution_mode='simulation',enable_hardware=False,
        axis_convention='+X into trailer,+Y left,+Z up',world=c['world'],base_position_m=base,robot_variant='ECO65-B',
        boxes=boxes,box=copy.deepcopy(selected),target_box_id=selected['id'],obstacles=components,support_surface_ids=surfaces,
        components_by_id={o['id']:dict(role=o['role'],owner=o['owner']) for o in components},routes=routes,
        regions=dict(conveyor_transverse=belts['transverse'],conveyor_longitudinal=belts['longitudinal'],transfer=belts['transfer'],outlet=belts['outlet']),
        occupancy={name:[] for name in surfaces if name!='trailer_floor'},wall_configuration=copy.deepcopy(t['panels']),
        floor_z_m=floor,roof_inner_z_m=floor+height,opening_x_m=opening,
        footprint=dict(bounds_xy_m=[float(lo[0]),float(hi[0]),float(lo[1]),float(hi[1])],minimum_static_size_m=(hi-lo)[:2].tolist(),
            recommended_table_size_m=((hi-lo)[:2]+2*c['installation']['recommended_edge_reserve_m']).tolist(),
            front_access_reserve_m=c['installation']['recommended_front_access_m'],desk_fit=c['installation']['desk_fit'],
            basis='housing/conveyors/outlet/mount static envelope; not a swept-motion or certified installation footprint'),
        margin_m=v['collision_margin_m'],mesh_error_reserve_m=v['mesh_error_reserve_m'],contact_tolerance_m=v['contact_tolerance_m'],
        source_config=c,transport='GEOMETRY_ONLY_NOT_EXECUTED',full_paths='NOT_EVALUATED',dynamics='NOT_EVALUATED')

def face_tcp(box_pose,size,face):
    axis,sign=FACE_AXES[face];local=np.zeros(3);local[axis]=sign*size[axis]/2
    return box_pose@transform(local,FACE_ROTATIONS[face])

def seal_check(tool,tcp,box_pose,size,face,edge=.003,tolerance=.0004):
    axis,sign=FACE_AXES[face];others=[i for i in range(3) if i!=axis];half=np.array(size)/2
    cad=check_transform(tcp)@np.linalg.inv(check_transform(tool['T_tool_cad_tcp']))
    local=np.linalg.inv(box_pose)@cad;normal=(np.linalg.inv(box_pose)@tcp)[:3,2]
    expected=np.zeros(3);expected[axis]=-sign
    angle=float(np.arccos(np.clip(normal@expected,-1,1)));worst=0.;reserve=1.
    for ring in tool['seals']:
        pts=np.array(ring['ring_cad_m'])@local[:3,:3].T+local[:3,3]
        worst=max(worst,float(np.max(np.abs(pts[:,axis]-sign*half[axis]))))
        reserve=min(reserve,float(np.min(half[others]-np.abs(pts[:,others]))))
    ok=angle<=tool['contact']['normal_tolerance_rad'] and worst<=tolerance and reserve>=edge
    return dict(fits=bool(ok),rings=len(tool['seals']),max_plane_error_m=worst,min_edge_clearance_m=reserve,normal_error_rad=angle,independent_control_assumed=False)

def rectangle_coverage(rect,rectangles):
    x0,x1,y0,y1=rect;clips=[]
    for a,b,c,d in rectangles:
        a,b,c,d=max(x0,a),min(x1,b),max(y0,c),min(y1,d)
        if a<b and c<d:clips.append([a,b,c,d])
    xs=sorted({x0,x1,*[v for r in clips for v in r[:2]]});ys=sorted({y0,y1,*[v for r in clips for v in r[2:]]})
    area=0.
    for a,b in zip(xs[:-1],xs[1:]):
        for c,d in zip(ys[:-1],ys[1:]):
            x,y=(a+b)/2,(c+d)/2
            if any(r[0]<=x<=r[1] and r[2]<=y<=r[3] for r in clips):area+=(b-a)*(d-c)
    return float(min(1.,area/((x1-x0)*(y1-y0))))

def functional_regions(scene):
    c=scene['source_config']['conveyors'];result={};dead=c['end_dead_zone_m']
    for key in ('transverse','longitudinal'):
        name='conveyor_'+key;xy=list(c[key]['bounds_xy_m']);usable=xy.copy();axis=1 if key=='transverse' else 0
        k=axis*2;usable[k]+=dead;usable[k+1]-=dead
        first=xy.copy();first[k+1]=first[k]+dead
        last=xy.copy();last[k]=last[k+1]-dead
        result[name]=dict(physical_support_bounds_xy_m=xy,receiving_bounds_xy_m=usable,end_zone_bounds_xy_m=[first,last],
            direction=c[key]['direction'],process_owner=c[key]['process_owner'],belt_id=name+'_belt',body_id=name+'_body',
            support_ids=[o['id'] for o in scene['obstacles'] if o['owner']==name and o['role']=='leg'],
            construction='simplified flat belt/body/supports; rollers, motors and true transfer mechanics not qualified')
    result['transfer']=dict(process_owner='longitudinal',bridge_id='transfer_bridge',bridge_bounds_xy_m=c['transfer']['bridge_bounds_xy_m'],
        mechanics=c['transfer']['mechanics'],occupancy_initial=[])
    result['outlet']=dict(bridge_id='outlet_bridge',catch_id='outlet_catch_belt',opening_x_m=scene['opening_x_m'],
        final_center_xy_m=c['outlet']['final_center_xy_m'],occupancy_initial=[],disappearance_allowed=False)
    return result

def support_at(scene,center_xy,size,surface_z):
    x,y=center_xy;rect=[x-size[0]/2,x+size[0]/2,y-size[1]/2,y+size[1]/2]
    surfaces=[];ids=[]
    for o in scene['obstacles']:
        lo,hi=bounds(o)
        if o['id'] not in scene['support_surface_ids'] or abs(hi[2]-surface_z)>1e-8:continue
        support=[lo[0],hi[0],lo[1],hi[1]]
        if min(rect[1],support[1])>max(rect[0],support[0]) and min(rect[3],support[3])>max(rect[2],support[2]):
            surfaces.append(support);ids.append(o['id'])
    return dict(coverage=rectangle_coverage(rect,surfaces),surface_ids=ids,mode='area union; geometric support only, not friction/transfer qualification')

def audit_layout(scene,tool):
    c=scene['source_config'];obs=scene['obstacles'];errors=[];penetrations=[]
    for i,a in enumerate(obs):
        alo,ahi=bounds(a)
        for b in obs[i+1:]:
            blo,bhi=bounds(b);overlap=np.minimum(ahi,bhi)-np.maximum(alo,blo)
            if np.all(overlap>1e-8):penetrations.append([a['id'],b['id'],overlap.tolist()])
    if penetrations:errors.append('static_component_penetration')
    boxes=scene['boxes'];ids=[b['id'] for b in boxes]
    assert len(ids)==len(set(ids))==9
    objects={o['id']:o for o in obs+boxes};supports=[]
    for b in boxes:
        lo,hi=bounds(b);parent=objects[b['supported_by'][0]];plo,phi=bounds(parent)
        good=abs(lo[2]-phi[2])<1e-8 and np.all(lo[:2]>=plo[:2]-1e-8) and np.all(hi[:2]<=phi[:2]+1e-8)
        supports.append(dict(id=b['id'],supported_by=b['supported_by'],supports=b['supports'],support_valid=bool(good),currently_removable=not b['supports']))
        if not good:errors.append('stack_support:'+b['id'])
    trans=c['conveyors']['transverse']['bounds_xy_m'];long=c['conveyors']['longitudinal']['bounds_xy_m'];stackfront=c['carton_stack']['front_face_x_m']
    assert scene['base_position_m'][0]<trans[0]<trans[1]<stackfront
    assert abs(trans[1]-long[1])<1e-10 and long[3]<trans[2]
    assert long[3]<0 and scene['opening_x_m']<long[0]
    size=boxes[0]['size_m'];T=transform(boxes[0]['pose'])
    faces={face:seal_check(tool,face_tcp(T,size,face),T,size,face) for face in ('top','front','right_facing')}
    if not all(d['fits'] for d in faces.values()):errors.append('seal_footprint')
    route_reports={};z=c['conveyors']['surface_z_m']
    for name,route in scene['routes'].items():
        samples=[]
        for a,b in zip(route['centers_xy_m'][:-1],route['centers_xy_m'][1:]):
            a,b=np.array(a),np.array(b)
            for s in np.linspace(0,1,max(2,int(np.linalg.norm(b-a)/.005)+1)):
                xy=a+(b-a)*s;samples.append(dict(xy_m=xy.tolist(),**support_at(scene,xy,size,z)))
        minimum=min(s['coverage'] for s in samples)
        route_reports[name]=dict(minimum_support_area_fraction=minimum,samples=len(samples),geometric_support_pass=minimum>1-1e-8,transport_physics='NOT_EVALUATED')
        if minimum<1-1e-8:errors.append('route_support:'+name)
    outlet=np.array(c['conveyors']['outlet']['final_center_xy_m']);assert outlet[0]+size[0]/2<scene['opening_x_m']
    return dict(status='PASS' if not errors else 'FAIL',errors=errors,component_penetrations=penetrations,stack=supports,face_compatibility=faces,
        transfer_gap_m=trans[2]-long[3],belts_front_edge_delta_m=trans[1]-long[1],routes=route_reports,footprint=scene['footprint'],
        functional_regions=functional_regions(scene),enclosure_collision_ids=['trailer_left','trailer_right','trailer_rear','trailer_floor','trailer_roof'],all_boxes_retained=True)

class UnloadingWorld(World):
    """All cartons and enclosure parts participate; only named geometric contacts are allowed."""
    def __init__(self,tool,scene):
        self.active_pose=transform(scene['box']['pose']);self.active_face=None
        super().__init__(tool,scene)
        self.object_info={o['id']:o for o in scene['obstacles']+scene['boxes']}
    def box_pose(self,q,phase):return self.active_pose
    def select_target(self,ident,pose=None,face=None):
        if ident not in self.boxes:raise KeyError(ident)
        self.box_id=ident;self.active_pose=transform(self.boxes[ident]['pose']) if pose is None else check_transform(pose)
        self.active_face=face
    def _pose(self,name):return self.active_pose if name==self.box_id else transform(self.object_info[name].get('pose',self.object_info[name].get('position_m')))
    def _support_pair(self,a,b):
        # Axis-aligned static fixture/carton contact, never blanket neighbor exemptions.
        for upper,lower in [(a,b),(b,a)]:
            if upper['kind']!='payload':continue
            if lower['kind']!='payload' and lower['name'] not in self.scene['support_surface_ids']:continue
            A=self._pose(upper['name']);B=self._pose(lower['name'])
            if not np.allclose(A[:3,:3],np.eye(3),atol=1e-8) or not np.allclose(B[:3,:3],np.eye(3),atol=1e-8):continue
            ah=np.array(self.object_info[upper['name']]['size_m'])/2;bh=np.array(self.object_info[lower['name']]['size_m'])/2
            delta=A[2,3]-ah[2]-(B[2,3]+bh[2])
            overlap=np.minimum(A[:2,3]+ah[:2],B[:2,3]+bh[:2])-np.maximum(A[:2,3]-ah[:2],B[:2,3]-bh[:2])
            if delta>=-self.scene['contact_tolerance_m'] and np.all(overlap>0):return True
        return False
    def valid(self,q,phase='pose',detail=False):
        self.calls+=1;self.last_failure=None
        if not np.isfinite(q).all() or not self.robot.within_limits(q):self.last_failure=dict(reason='joint_limits');return False
        self.set_state(q,phase);margin=self.scene['margin_m']+2*self.scene['mesh_error_reserve_m']
        centers=self.data.geom_xpos;distances=np.linalg.norm(centers[self.pair_a]-centers[self.pair_b],axis=1)-self.radii
        seal=None
        for index in np.flatnonzero(distances<=margin):
            a,b=self.pairs[index];pair={a['name'],b['name']};allowance=None
            if pair=={'robot_base_link','robot_mount_plate'}:allowance='fixed_mount_interface'
            elif pair=={'robot_link_6','adapter_flange_disk'}:allowance='flange_interface'
            elif self._support_pair(a,b):allowance='carton_bottom_support_top'
            if self.box_id in pair and self.active_face:
                other=b if a['name']==self.box_id else a
                if other['kind']=='terminal_contact_band':
                    if seal is None:seal=seal_check(self.tool,self.robot.fk(q),self.active_pose,self.boxes[self.box_id]['size_m'],self.active_face)
                    if seal['fits']:allowance='actual_target_seals_'+self.active_face
            threshold=-self.scene['contact_tolerance_m'] if allowance else margin
            distance=self.mj.mj_geomDistance(self.model,self.data,a['gid'],b['gid'],margin,None)
            if distance<threshold-1e-7:
                self.last_failure=dict(reason='collision',pair=[a['name'],b['name']],distance_m=float(distance),threshold_m=threshold,allowance=allowance)
                return False
        return True
    def attach(self,q):raise NotImplementedError('Layout pose checks only; no task execution in this round')
    def release(self,q):raise NotImplementedError('Layout pose checks only; no task execution in this round')

def inspect_pose(world,target,seed_q,rng,settings):
    start=time.perf_counter();deadline=start+settings['ik_seconds_per_pose'];found=[];best=None;solved=0
    seeds=[np.array(seed_q)]+[rng.uniform(world.robot.joint_limits[:,0],world.robot.joint_limits[:,1]) for _ in range(settings['ik_restarts_per_pose']-1)]
    for i,seed in enumerate(seeds):
        if time.perf_counter()>deadline:break
        result=solve_ik(world.robot,target,seed,max_iterations=settings['ik_iterations'],position_tolerance=.00003,orientation_tolerance=.0003,deadline_monotonic=deadline)
        if best is None or result.position_error+result.orientation_error<best['residual_score']:
            best=dict(q=np.array(result.q).tolist(),position_error_m=float(result.position_error),orientation_error_rad=float(result.orientation_error),residual_score=float(result.position_error+result.orientation_error))
        if not result.success:continue
        solved+=1;ok=world.valid(result.q)
        record=dict(seed_index=i,q=np.array(result.q).tolist(),position_error_m=float(result.position_error),orientation_error_rad=float(result.orientation_error),collision=copy.deepcopy(world.last_failure))
        if ok:return dict(status='POSE_VALID',elapsed_s=time.perf_counter()-start,attempts=i+1,ik_solutions=solved,**record)
        found.append(record)
    return dict(status='IK_FOUND_COLLISION' if solved else 'IK_NOT_FOUND_WITHIN_BUDGET',elapsed_s=time.perf_counter()-start,attempts=i+1,ik_solutions=solved,collision_solutions=found,best_numerical_attempt=best,
        qualification='Finite pose search only; not a proof of global unreachability; no full path searched')

def limited_reachability(world):
    s=world.scene;c=s['source_config'];v=c['validation'];rng=np.random.default_rng(v['seed'])
    previous=load_json(ROOT/'outputs/eco65_desktop_round1/round1/reports/home.json')['q']
    world.select_target(s['target_box_id'])
    if world.valid(previous):home=dict(status='POSE_VALID',q=previous,source='prior q rechecked in new fully enclosed layout')
    else:
        old_failure=copy.deepcopy(world.last_failure);target=transform([s['base_position_m'][0]+.25,s['base_position_m'][1]-.10,s['floor_z_m']+.38],FACE_ROTATIONS['top'])
        home=inspect_pose(world,target,previous,rng,v);home['previous_pose_failure']=old_failure
    if home['status']!='POSE_VALID':return dict(home=home,candidates=[],status='NO_VALID_INITIAL_POSE_FOUND',full_paths='NOT_EVALUATED')
    qhome=home['q'];cases=[];boxes=s['boxes'];highest=max(b['layer'] for b in boxes);midcol=max(b['column'] for b in boxes)//2
    for b in boxes:
        if b['layer']==highest:cases.append(dict(id='pick_front_'+b['id'],kind='pick',box_id=b['id'],face='front'))
    middle=next(b for b in boxes if b['layer']==highest and b['column']==midcol)
    cases.append(dict(id='pick_top_center',kind='pick',box_id=middle['id'],face='top'))
    lower=next(b for b in boxes if b['layer']==highest-1 and b['column']==midcol)
    cases.append(dict(id='pick_front_supported_middle',kind='pick',box_id=lower['id'],face='front'))
    for name,faces in [('transverse',['top','front']),('longitudinal',['top','right_facing'])]:
        for face in faces:
            cases.append(dict(id=f'place_{name}_{face}',kind='place',box_id=middle['id'],face=face,region='conveyor_'+name,
                position_m=[*c['conveyors'][name]['receive_center_xy_m'],c['conveyors']['surface_z_m']+middle['size_m'][2]/2]))
    results=[]
    for case in cases:
        b=world.boxes[case['box_id']];pose=transform(case.get('position_m',b['pose']));world.select_target(b['id'],pose,case['face'])
        target=face_tcp(pose,b['size_m'],case['face']);fit=seal_check(world.tool,target,pose,b['size_m'],case['face'])
        result=inspect_pose(world,target,qhome,rng,v) if fit['fits'] else dict(status='FOOTPRINT_INCOMPATIBLE')
        result.update(case);result['target_tcp']=target.tolist();result['box_pose']=pose.tolist();result['seal_geometry']=fit
        result['removal_status']='SUPPORT_DEPENDENCY_BLOCKED' if case['kind']=='pick' and b['supports'] else 'NO_BOX_SUPPORTED_ABOVE'
        result['supports_boxes']=b['supports'] if case['kind']=='pick' else []
        if case['kind']=='place':
            result['support']=support_at(s,pose[:2,3],b['size_m'],c['conveyors']['surface_z_m'])
            result['candidate_occupancy']={case['region']:[b['id']]}
        result['full_path_status']='NOT_EVALUATED';results.append(result)
        print('POSE',case['id'],result['status'],result['removal_status'],round(result.get('elapsed_s',0),2),'s',flush=True)
    world.select_target(s['target_box_id']);world.set_state(qhome,'pose')
    return dict(status='FINITE_POSE_CHECKS_COMPLETE',home=home,candidates=results,full_paths='NOT_EVALUATED',dynamics='NOT_EVALUATED',all_nine_box_ids=list(world.boxes),seed=v['seed'])

def source_mapping(config,scene):
    path=config['reference']['layout'];raw=subprocess.check_output(['git','show',SOURCE_SHA+':'+path],cwd=ROOT);original=yaml.safe_load(raw)
    A=np.array(original['assembly']['world_xyz_m']);old_belts={}
    for key in ('transverse','longitudinal'):
        b=original['conveyors'][key];center=A[:2]+b['center_xy_a_m'];half=np.array(b['size_xy_m'])/2
        old_belts[key]=[float(center[0]-half[0]),float(center[0]+half[0]),float(center[1]-half[1]),float(center[1]+half[1])]
    entries=[dict(item=key,source_bounds_xy_m=value,desktop_bounds_xy_m=config['conveyors'][key]['bounds_xy_m'],reason='preserve L topology; adjust width to full actual suction footprint and carton support') for key,value in old_belts.items()]
    entries += [dict(item='robot_base',source_xyz_m=[float(A[0]+original['robot']['base_origin_xy_a_m'][0]),float(A[1]+original['robot']['base_origin_xy_a_m'][1]),float(A[2]+original['robot']['mounting_surface_z_a_m'])],desktop_xyz_m=scene['base_position_m'],reason='official ECO65-B unchanged; fixed 12 mm mounting plate replaces chassis'),
        dict(item='carton',source_size_m=original['carton_stack']['carton_size_xyz_m'],desktop_size_m=config['carton_stack']['carton_size_xyz_m'],reason='initial user candidate checked against all CAD seal rings, no overall scaling'),
        dict(item='trailer',source=[original['trailer']['opening_x_m'],original['trailer']['closed_end_wall_x_m'],original['trailer']['inner_width_m'],original['trailer']['height_m']],desktop=[config['trailer'][k] for k in ['opening_x_m','closed_end_inner_x_m','inner_width_m','inner_height_m']],reason='recreate full work segment; shorten unused space behind stack; roof and both side walls retained'),
        dict(item='heights',source=dict(chassis_top_m=.6,body_thickness_m=.12),desktop=dict(floor_top_m=scene['floor_z_m'],robot_mount_m=scene['base_position_m'][2],belt_top_m=config['conveyors']['surface_z_m'],body_thickness_m=config['conveyors']['body_thickness_m']),reason='no chassis inheritance: thin floor/plate and short real conveyor supports'),
        dict(item='stack',source=[1,5,8],desktop=[1,3,3],reason='requested nine-carton candidate with neighbor and support relations'),
        dict(item='outlet',source='ideal downstream extension in historical report',desktop=config['conveyors']['outlet'],reason='explicit bridge and catch support; no disappearing carton or dynamic transport claim')]
    return dict(source_sha=SOURCE_SHA,source_path=path,source_sha256=hashlib.sha256(raw).hexdigest(),original_layout=original,entries=entries,
        adjustments=[dict(field='box',initial=[.24,.20,.16],selected=config['carton_stack']['carton_size_xyz_m'],reason='all 12 seals checked on top and front'),
            dict(field='inner_width',initial_range=[.80,.90],selected=.90,reason='280 mm longitudinal + 10 mm bridge + 540 mm transverse = 830 mm plus 35 mm side clearance each'),
            dict(field='rear_inner_x',one_third_initial=original['trailer']['closed_end_wall_x_m']/3,selected=config['trailer']['closed_end_inner_x_m'],reason='retain 60 mm behind 240 mm stack; omit unused rear development volume'),
            dict(field='outlet_support',initial='unspecified',selected='100 mm bridge + 320 mm catch table',reason='support the entire carton beyond the opening')])