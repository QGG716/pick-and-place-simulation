"""Bounded NumPy-native batch kinematics and conservative interval pair proofs.

The loop over configurations is inside NumPy's compiled matmul/ufunc kernels;
Python traverses the fixed URDF chain once per batch, not once per state.
Coal narrow phase and stage predicates retain their scalar reference semantics.
"""
from contextlib import contextmanager
from dataclasses import dataclass
from time import perf_counter
import numpy as np
from .motion_validation import LRU
from .geometry import OBB, _BOX_CORNER_SIGNS


class PreparedToolOBB(OBB):
    """Compatibility OBB with one immutable transform per immutable state."""
    @property
    def world_from_local(self):
        pose=getattr(self,'_prepared_transform',None)
        if pose is None:
            pose=super().world_from_local
            pose.setflags(write=False)
            object.__setattr__(self,'_prepared_transform',pose)
            self._kernel_statistics['tool_transform_constructions'] += 1
        return pose


class LazyToolBoxes:
    """Immutable state-owned arrays; materialize only requested compatibility OBBs."""
    def __init__(self, centers, halves, rotation, names, lower, upper, bounds, statistics):
        self.centers,self.halves,self.rotation,self.names=centers,halves,rotation,names
        self.lower,self.upper,self.bounds,self.statistics=lower,upper,bounds,statistics
        self.boxes={}
        self._plane_bounds=None

    def __len__(self): return len(self.names)

    def plane_bounds(self):
        if self._plane_bounds is None:
            local=self.halves[:,None,:]*_BOX_CORNER_SIGNS
            corners=(self.rotation @ local.transpose(0,2,1)).transpose(0,2,1)+self.centers[:,None,:]
            self._plane_bounds=(corners.min(1),corners.max(1))
        return self._plane_bounds

    def __getitem__(self,index):
        if isinstance(index,slice): return [self[i] for i in range(*index.indices(len(self)))]
        if index < 0: index += len(self)
        if not 0 <= index < len(self): raise IndexError(index)
        if index not in self.boxes:
            box=PreparedToolOBB(self.centers[index],self.halves[index],self.rotation,self.names[index],'robot')
            object.__setattr__(box,'_kernel_statistics',self.statistics)
            self.boxes[index]=box
            self.bounds[id(box)]=(self.lower[index],self.upper[index])
            self.statistics['tool_obb_constructions'] += 1
        return self.boxes[index]


@dataclass(frozen=True)
class KinematicState:
    links: dict
    joints: dict
    tcp: np.ndarray
    jacobian: np.ndarray
    geometry: np.ndarray | None = None
    mesh_frames: dict | None = None
    rigid: tuple = ()
    cups: tuple = ()
    compressed_cups: tuple = ()
    obstacle_aabbs: dict | None = None


def batch_kinematics(robot, states):
    """One native vectorized chain traversal. No Python per-state FK callback."""
    q = np.asarray(states, float)
    if q.ndim != 2 or q.shape[1] != robot.dof or not np.isfinite(q).all():
        raise ValueError('finite batch with the configured DOF required')
    n = len(q)
    t = np.broadcast_to(robot.base_transform, (n,4,4)).copy()
    links, joints = {robot.base_link:t}, {}
    axes, origins = {}, {}
    for joint in robot.joints:
        t = t @ joint.origin
        if joint.name in robot.active_joint_names:
            i = robot.active_joint_names.index(joint.name)
            axis = np.asarray(joint.axis)/np.linalg.norm(joint.axis)
            axes[i] = t[:,:3,:3] @ axis
            origins[i] = t[:,:3,3].copy()
            r = np.broadcast_to(np.eye(4),(n,4,4)).copy()
            if joint.joint_type in {'revolute','continuous'}:
                x,y,z = axis
                skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                c,s = np.cos(q[:,i,None,None]),np.sin(q[:,i,None,None])
                r[:,:3,:3] = c*np.eye(3)+(1-c)*np.outer(axis,axis)+s*skew
            elif joint.joint_type == 'prismatic':
                r[:,:3,3] = q[:,i,None]*axis
            else:
                raise ValueError('unsupported active joint')
            t = t @ r
            joints[joint.name] = t
        links[joint.child] = t
    tcp = t @ robot.tip_from_tcp
    jac = np.zeros((n,6,robot.dof))
    for i,joint in enumerate(robot.active_joints):
        if joint.joint_type == 'prismatic':
            jac[:,:3,i] = axes[i]
        else:
            jac[:,:3,i] = np.cross(axes[i],tcp[:,:3,3]-origins[i])
            jac[:,3:,i] = axes[i]
    return links,joints,tcp,jac


def point_motion_bound(delta, radii, prismatic=None):
    """Telescoping joint changes, full swept arc bounded by min(angle,2)*R.

    Each R encloses every point downstream of that joint for all configurations.
    Both bodies must contribute to a pair's relative displacement bound.
    """
    d,r = np.abs(np.asarray(delta,float)),np.asarray(radii,float)
    if d.shape != r.shape or not np.isfinite([d,r]).all() or np.any(r<0):
        return float('inf')
    terms = np.minimum(d,2.)*r
    if prismatic is not None:
        terms = np.where(prismatic,d,terms)
    return float(np.sum(terms))


def aabb_distance_lower(first, second):
    gap = np.maximum(np.maximum(second[0]-first[1],first[0]-second[1]),0.)
    return float(np.linalg.norm(gap))


def certify_pair(lower, first_motion, second_motion, required, epsilon=1e-9):
    values = np.asarray([lower,first_motion,second_motion,required,epsilon])
    return bool(np.isfinite(values).all() and np.all(values>=0)
                and lower-first_motion-second_motion > required+epsilon)


class ValidationKernel:
    mode = 'NUMPY_NATIVE_BATCH_FK_AND_TRANSFORMS_SCALAR_COAL'

    def __init__(self, connector, urdf, mesh):
        self.connector,self.urdf,self.mesh = connector,urdf,mesh
        self.cache = LRU(4096)
        self.active = {}
        self.statistics = dict(fk_states=0,native_batches=0,native_array_operations=0,
                               kinematics_seconds=0.,ordered_scalar_state_checks=0,
                               tool_obb_constructions=0,tool_template_preparations=0,
                               tool_transform_constructions=0)
        self._tool_template_key=None
        self.context_id = None
        urdf.validation_kernel = mesh.validation_kernel = self

    def get(self,q):
        q = np.asarray(q,float)
        return self.active.get(q.tobytes()) if q.shape == (self.urdf.dof,) else None

    def prepare_states(self, states):
        unique = {np.asarray(q,float).tobytes():np.asarray(q,float) for q in states
                  if np.asarray(q).shape == (self.urdf.dof,) and np.isfinite(q).all()
                  and self.urdf.within_limits(q)}
        output,missing = {},{}
        for key,q in unique.items():
            hit,value = self.cache.lookup((self.context_id,key))
            if hit: output[key]=value
            else: missing[key]=q
        if not missing: return output
        started = perf_counter()
        links,joints,tcp,jac = batch_kinematics(self.urdf,list(missing.values()))
        n = len(missing)
        world_base = np.broadcast_to(self.urdf.base_transform,(n,4,4))
        parents = {'universe':world_base, **joints}
        # Pinocchio geometry is relative to parentJoint, including fixed frames.
        geometry = np.stack([parents[str(self.mesh.model.names[g.parentJoint])] @
            self.mesh._matrix(g.placement) for g in self.mesh.geometry_model.geometryObjects],axis=1)
        frames = {str(f.name):parents[str(self.mesh.model.names[f.parentJoint])] @
                  self.mesh._matrix(f.placement) for f in self.mesh.model.frames}
        # Profiling identified per-tool _world_aabb as a primary hotspot. All
        # tool centers, extents and outward rounding now run as native arrays.
        rigid_rows=np.asarray(self.urdf.tool_collision_local_boxes)
        cup_rows=np.asarray(self.urdf.tool_compliant_collision_local_boxes)
        compression=self.connector.robot_state_validator.nominal_cup_compression_m
        template_key=(rigid_rows.tobytes(),cup_rows.tobytes(),compression)
        if template_key != self._tool_template_key:
            compressed=cup_rows.copy()
            compressed[:,2]-=compression/2;compressed[:,5]-=compression
            rows=np.concatenate([rigid_rows,cup_rows,compressed])
            halves=rows[:,3:]/2
            nr,nc=len(rigid_rows),len(cup_rows)
            names=tuple([f'tool_rigid_{i}' for i in range(nr)]+[f'tool_compliant_bellows_{i}' for i in range(nc)]*2)
            rows.setflags(write=False);halves.setflags(write=False)
            self._tool_template=(rows,halves,names,nr,nc)
            self._tool_template_key=template_key
            self.statistics['tool_template_preparations'] += 1
        rows,halves,names,nr,nc=self._tool_template
        centers=np.einsum('nij,kj->nki',tcp[:,:3,:3],rows[:,:3])+tcp[:,None,:3,3]
        world_half=np.einsum('nij,kj->nki',np.abs(tcp[:,:3,:3]),halves)
        pad=32*np.finfo(float).eps*(1+np.max(np.abs(centers),axis=2)+np.max(world_half,axis=2))
        lower=np.nextafter(centers-world_half-pad[:,:,None],-np.inf)
        upper=np.nextafter(centers+world_half+pad[:,:,None],np.inf)
        for array in (centers,lower,upper): array.setflags(write=False)
        for array in (tcp,jac,geometry,*links.values(),*joints.values(),*frames.values()):
            array.setflags(write=False)
        for i,key in enumerate(missing):
            bounds={}
            groups=[LazyToolBoxes(centers[i,s],halves[s],tcp[i,:3,:3],names[s],
                lower[i,s],upper[i,s],bounds,self.statistics)
                for s in (slice(0,nr),slice(nr,nr+nc),slice(nr+nc,None))]
            state = KinematicState({k:v[i] for k,v in links.items()},
                {k:v[i] for k,v in joints.items()},tcp[i],jac[i],geometry[i],
                {k:v[i] for k,v in frames.items()},*groups,bounds)
            # The records are owned by this bounded cache and never mutated.
            for value in [state.tcp,state.jacobian,state.geometry,*state.links.values(),*state.mesh_frames.values()]:
                value.setflags(write=False)
            self.cache.put((self.context_id,key),state); output[key]=state
        self.statistics['fk_states'] += n
        self.statistics['native_batches'] += 1
        self.statistics['native_array_operations'] += 2*len(self.urdf.joints)+len(frames)+len(geometry[0])+3*self.urdf.dof+12
        self.statistics['kinematics_seconds'] += perf_counter()-started
        return output

    @contextmanager
    def scope(self, states, proof=None):
        previous = self.active
        old_proof = getattr(self.mesh,'_interval_pairs',frozenset())
        self.active = self.prepare_states(states)
        pairs = frozenset((proof or {}).get('pairs',()))
        self.mesh._interval_pairs = pairs
        self.connector.robot_state_validator._interval_pairs = pairs
        self.connector._interval_pairs = pairs
        try: yield
        finally:
            self.active = previous
            self.mesh._interval_pairs = old_proof
            self.connector.robot_state_validator._interval_pairs = old_proof
            self.connector._interval_pairs = old_proof

    def check_states(self, states, scalar, proof=None):
        with self.scope(states,proof):
            result=[]
            for q in states:
                result.append(scalar(q))
                self.statistics['ordered_scalar_state_checks'] += 1
            return result

    def check_prefix(self, states, scalar, proof=None, interrupted=lambda: None):
        result=[]
        with self.scope(states,proof):
            for q in states:
                if interrupted(): break
                failure=scalar(q)
                self.statistics['ordered_scalar_state_checks'] += 1
                result.append(failure)
                if failure is not None: break
        return result

    def prepare_context(self, context, obstacles, **options):
        # Only the explicit POC Euclidean surface-distance policy is certified.
        # Legacy inflated-OBB and history-dependent contact predicates stay scalar.
        def proof(a,b):
            self.context_id = context.context_id
            policy = self.connector.collision_policy
            if not policy.poc_pair_clearance: return None
            with self.scope([a]):
                state = self.get(a)
                if state is None: return None
                bodies = []
                chain_length = sum(float(np.linalg.norm(j.origin[:3,3])) for j in self.urdf.joints)
                # This intentionally loose bound includes every upstream origin,
                # official mesh extent, tool mounting offset and payload corners.
                delta = np.abs(np.asarray(b)-a)
                if any(j.joint_type == 'prismatic' for j in self.urdf.active_joints): return None
                def body(name,bounds,pose,extra,upstream):
                    center=(bounds[0]+bounds[1])/2; half=(bounds[1]-bounds[0])/2
                    c=pose[:3,:3]@center+pose[:3,3]; h=np.abs(pose[:3,:3])@half
                    radius=chain_length+extra+float(np.linalg.norm(np.maximum(np.abs(bounds[0]),np.abs(bounds[1]))))
                    radii=np.zeros(self.urdf.dof); radii[:upstream]=radius
                    return name,(c-h-1e-9,c+h+1e-9),point_motion_bound(delta,radii)
                for i,g in enumerate(self.mesh.geometry_model.geometryObjects):
                    bounds=self.mesh._geometry_local_aabbs[i]
                    if bounds is None: continue
                    parent=int(g.parentJoint)
                    upstream=0 if parent==0 else self.urdf.active_joint_names.index(str(self.mesh.model.names[parent]))+1
                    bodies.append(body(str(g.name),bounds,state.geometry[i],
                        float(np.linalg.norm(self.mesh._matrix(g.placement)[:3,3])),upstream))
                rigid=list(self.urdf.tool_collision_obbs(a))
                cups=self.connector.robot_state_validator._compliant_boxes(a)
                tool_extra=float(np.linalg.norm(self.urdf.tip_from_tcp[:3,3]))
                tools=[]
                for box in [*rigid,*cups]:
                    offset=np.linalg.norm(box.center-state.tcp[:3,3])
                    tools.append(body(box.name,(-box.half_extents,box.half_extents),box.world_from_local,tool_extra+offset,self.urdf.dof))
                attachment=options.get('attachment')
                payload=[]
                if attachment is not None:
                    box=attachment.box_at(a)
                    offset=np.linalg.norm(box.center-state.tcp[:3,3])
                    payload=[body(box.name,(-box.half_extents,box.half_extents),box.world_from_local,tool_extra+offset,self.urdf.dof)]
                fixed=[(box.name,(box.corners().min(0)-1e-9,box.corners().max(0)+1e-9),0.) for box in obstacles]
                pairs=[]
                def certify(kind,x,y,margin):
                    lower=aabb_distance_lower(x[1],y[1])
                    if certify_pair(lower,x[2],y[2],margin): pairs.append((kind,x[0],y[0]))
                def certify_group(kind,xs,ys,margin):
                    if not xs or not ys: return
                    xb=np.asarray([x[1] for x in xs]);yb=np.asarray([y[1] for y in ys])
                    gap=np.maximum(np.maximum(yb[None,:,0]-xb[:,None,1],xb[:,None,0]-yb[None,:,1]),0.)
                    lower=np.linalg.norm(gap,axis=2)
                    motion=np.asarray([x[2] for x in xs])[:,None]+np.asarray([y[2] for y in ys])[None,:]
                    accepted=np.isfinite(lower-motion)&(lower-motion>margin+1e-9)
                    pairs.extend((kind,xs[i][0],ys[j][0]) for i,j in zip(*np.nonzero(accepted)))
                margin=policy.pair_clearance('external',self.connector.collision_margin_m)
                certify_group('mesh',bodies,[*fixed,*tools,*payload],margin)
                byname={x[0]:x for x in bodies}
                for pair in self.mesh.geometry_model.collisionPairs:
                    names=[str(self.mesh.geometry_model.geometryObjects[i].name) for i in (pair.first,pair.second)]
                    if all(n in byname for n in names): certify('self',byname[names[0]],byname[names[1]],0.)
                certify_group('obb',[*tools,*payload],fixed,margin)
                return dict(context_id=context.context_id,pairs=pairs,complete_contract=False,
                    distance_meaning='OUTWARD_AABB_EUCLIDEAN_LOWER_BOUND_MINUS_BOTH_SWEEP_BOUNDS',
                    required_clearance_m=margin,numerical_epsilon_m=1e-9)
        return proof
