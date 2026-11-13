"""Basic sampled geometry for the authorized kinematic simulation only."""
from itertools import combinations
import numpy as np
from .pinocchio_backend import PinocchioHppFclBackend, _transform
from .s50_collision_diagnostics import diagnostic_coal_obstacle, _world_geometry, coal_pair_query
from .conveyor_lift import coupled_conveyor_state
from .geometry import OBB
from .s50_kinematic_sim import tool_virtual


def bounds(local,T):
    lo,hi=local;center=T[:3,:3]@((lo+hi)/2)+T[:3,3];half=np.abs(T[:3,:3])@((hi-lo)/2)
    return center-half,center+half


class SimCollision:
    def __init__(self,c):
        self.c=c
        self.backend=PinocchioHppFclBackend(c.urdf,tip_frame='s50_flange',base_transform=c.world,
            tip_from_tcp=c.frames['nominal_compressed_contact'])
        self.coal=self.backend.coal;self.env_cache={};self.last=None;self.check_count=0;self.pair_queries=0
        self.adjacent={frozenset(p) for p in c.settings['collision']['adjacent_design_interfaces']}
        self.robot_local=[(v.min(0),v.max(0)) for v in self.backend._geometry_local_vertices]
        self.tool=[]
        from .s50_tool_mount import load_structure_meshes
        tool_meshes=load_structure_meshes(c.root,c.intrinsic)
        for kind,key in [('rigid','rigid_local_boxes_virtual_tcp_m'),('cup','compliant_local_boxes_virtual_tcp_m')]:
            for i,row in enumerate(c.intrinsic[key]):
                name=c.intrinsic.get(kind+'_body_ids',[f'tool_{kind}_{j}' for j in range(len(c.intrinsic[key]))])[i]
                row=np.asarray(row);shape=self.coal.Box(*row[3:])
                if name in tool_meshes:
                    mesh=tool_meshes[name];vertices=self.coal.StdVec_Vec3s();faces=self.coal.StdVec_Triangle()
                    for v in mesh['vertices']-row[:3]:vertices.append(v)
                    for a,b,d in mesh['faces']:faces.append(self.coal.Triangle(int(a),int(b),int(d)))
                    shape=self.coal.BVHModelOBBRSS();shape.beginModel(len(faces),len(vertices));shape.addSubModel(vertices,faces);shape.endModel()
                self.tool.append(dict(name=name,kind=kind,center=row[:3],half=row[3:]/2,shape=shape))
        self.fixed_shape_count=len(self.backend.geometry_model.geometryObjects)
        self.fk_crosscheck=max(np.max(np.abs(c.robot.fk(q)-self.backend.fk(q))) for q in [np.zeros(6),np.array([.1,.3,-.4,.2,.5,-.2])])
        if self.fk_crosscheck>1e-8:raise ValueError('SIM_CORE_PIN_FK_MISMATCH')

    def entity(self,box):
        shape,evidence=diagnostic_coal_obstacle(self.coal,box)
        return dict(name=box.name,kind=box.category,shape=shape,T=box.world_from_local,
            bounds=bounds((-box.half_extents,box.half_extents),box.world_from_local),evidence=evidence)

    def environment(self,height):
        active=tuple(getattr(self.c,'active_cartons',self.c.layout.cartons()))
        key=(round(float(height),8),self.c.settings['target'],tuple(b.name for b in active))
        if key not in self.env_cache:
            state=coupled_conveyor_state(self.c.layout,height,active)
            self.env_cache[key]=[self.entity(b) for b in state.boxes if b.name!=self.c.settings['target']]
        return self.env_cache[key]

    def near(self,a,b,gap=0.):
        return np.all(a['bounds'][0]<=b['bounds'][1]+gap) and np.all(b['bounds'][0]<=a['bounds'][1]+gap)

    def query(self,a,b,gap=0.,connection=None):
        if not self.near(a,b,gap):return None
        self.pair_queries+=1
        ta=_transform(self.coal,a['T'][:3,:3],a['T'][:3,3]);tb=_transform(self.coal,b['T'][:3,:3],b['T'][:3,3])
        req=self.coal.CollisionRequest();req.enable_contact=True;req.num_max_contacts=64
        result=self.coal.CollisionResult();self.coal.collide(a['shape'],ta,b['shape'],tb,req,result)
        if result.isCollision():
            contacts=list(result.getContacts())
            if connection is not None and contacts and all(connection(np.asarray(p.pos)) for p in contacts):return None
            depths=[float(p.penetration_depth) for p in contacts]
            # Coal mesh contacts can report signed negative values for an
            # intersection. A negative value must never be accepted as a gap.
            tolerance=self.c.settings['collision']['contact_tolerance_m']
            if connection is None and depths and all(abs(d)<=tolerance for d in depths):return None
            return dict(pair=[a['name'],b['name']],kind='INTERSECTION',raw_contact_depths_m=depths,
                contact_points=[np.asarray(p.pos).tolist() for p in contacts[:4]])
        if gap>0:
            req=self.coal.DistanceRequest();r=self.coal.DistanceResult()
            distance=float(self.coal.distance(a['shape'],ta,b['shape'],tb,req,r))
            if distance<gap:return dict(pair=[a['name'],b['name']],kind='GAP',distance_m=distance)
        return None

    def check(self,q,height=.6,payload=None,contact=False,phase='',record=False):
        self.check_count+=1;c=self.c
        if not c.robot.within_limits(q):self.last=dict(pair=[],kind='JOINT_LIMIT');return False
        geometries=_world_geometry(self.backend,q);robot=[]
        for i,g in enumerate(geometries):
            robot.append(dict(name=g['link'],shape=g['shape'],T=g['world'],bounds=bounds(self.robot_local[i],g['world'])))
        frames=c.robot.named_link_frames(q);axes=c.robot.joint_axis_frames(q)
        target=next(b for b in c.layout.cartons() if b.name==c.settings['target'])
        targetbox=target if payload is None else OBB(np.asarray(payload)[:3,3],target.half_extents,np.asarray(payload)[:3,:3],target.name,'carton')
        targetentity=self.entity(targetbox);target_active=target.name not in getattr(c,'outfed_carton_ids',())
        env=[*self.environment(height)]+([targetentity] if target_active else [])
        env_low=np.array([b['bounds'][0] for b in env]);env_high=np.array([b['bounds'][1] for b in env])
        robot_low=np.array([b['bounds'][0] for b in robot]);robot_high=np.array([b['bounds'][1] for b in robot])
        def nearby(a,items,low,high):
            return [items[i] for i in np.flatnonzero(np.all(a['bounds'][0]<=high+1e-6,axis=1)&np.all(low<=a['bounds'][1]+1e-6,axis=1))]
        def fail(hit):
            self.last=dict(**hit,q_rad=np.asarray(q).tolist(),phase=phase,height_m=height)
            return False
        for a,b in combinations(robot,2):
            if a['name']==b['name']:continue # original surface + local patch: one rigid body
            connection=None
            if frozenset((a['name'],b['name'])) in self.adjacent:
                child=max((a['name'],b['name']),key=lambda n:0 if n=='s50_base' else int(n.split('_')[-1]))
                idx=int(child.split('_')[-1]);origin,axis=axes[f's50_joint_{idx}']
                # Local joint housing only: CAD interface radial envelope 180mm,
                # axial envelope 360mm covers offset motor + bearing connection.
                connection=lambda p,o=origin,z=axis: abs((p-o)@z)<=.36 and np.linalg.norm((p-o)-((p-o)@z)*z)<=.18
            hit=self.query(a,b,connection=connection)
            if hit:return fail(hit)
        for a in robot:
            for b in nearby(a,env,env_low,env_high):
                if a['name']=='s50_base' and b['name']=='mount_plinth':continue
                hit=self.query(a,b)
                if hit:return fail(hit)
        virtual=tool_virtual(c,q);tool=[]
        for spec in self.tool:
            T=virtual.copy();T[:3,3]=virtual[:3,:3]@spec['center']+virtual[:3,3]
            a=dict(name=spec['name'],shape=spec['shape'],kind=spec['kind'],T=T,bounds=bounds((-spec['half'],spec['half']),T));tool.append(a)
            for b in nearby(a,env,env_low,env_high):
                if spec['kind']=='cup' and b['kind']=='carton':
                    if b['name']!=target.name or contact:continue
                hit=self.query(a,b)
                if hit:return fail(hit)
            for b in nearby(a,robot,robot_low,robot_high):
                if b['name']=='s50_link_6':continue # tool belongs to its ideal fixed flange
                hit=self.query(a,b)
                if hit:return fail(hit)
        if payload is not None and target_active:
            for b in nearby(targetentity,env[:-1],env_low[:-1],env_high[:-1]):
                hit=self.query(targetentity,b)
                if hit:return fail(hit)
        self.last=None
        return True

    def compactness(self,q,height,phase):
        from .s50_compactness import compactness_sample
        geometry=_world_geometry(self.backend,q)
        belt=next(b for b in coupled_conveyor_state(self.c.layout,height,()).boxes if b.name=='conveyor_transverse')
        shape,evidence=diagnostic_coal_obstacle(self.coal,belt);tf=_transform(self.coal,belt.rotation,belt.center)
        pairs=[]
        for g in geometry:
            T=g['world'];r=coal_pair_query(self.coal,g['shape'],_transform(self.coal,T[:3,:3],T[:3,3]),shape,tf)
            pairs.append(dict(kind='robot_environment',first=dict(link=g['link'],geometry=g['geometry']),second=dict(object=belt.name,**evidence),**r))
        return compactness_sample(geometry,pairs,self.c.layout.robot_base_transform(),q_rad=q,pose_id=phase,phase=phase,conveyor_height_m=height,
            safety_evidence=dict(scope='SAMPLED_KINEMATIC_SIMULATION_NOT_CERTIFIED'))
