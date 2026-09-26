"""Project-owned CUDA tensor collision cost plugged into pinned V2 rollouts.

OBBs replace outer spheres for every physical tool part and held carton.
Robot mesh spheres retain native self collision. No CPU state transfer occurs
inside cost/constraint evaluation. SAT separation is a conservative lower bound
on Euclidean box separation (not an exact distance or continuous collision test).
"""
import numpy as np
import torch
from curobo._src.cost.cost_base import BaseCost


def quaternion_matrix(q):
    w,x,y,z=q.unbind(-1)
    return torch.stack((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
                        2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
                        2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)),dim=-1).reshape(*q.shape[:-1],3,3)


def box_separation(ca,ra,ha,cb,rb,hb):
    # Paired boxes [...,3], column-axis rotations [...,3,3]. Parallel edge
    # axes carry -inf and cannot create a spurious separation/permission.
    delta=cb-ca
    axes_a=ra.transpose(-1,-2);axes_b=rb.transpose(-1,-2)
    axes_a,axes_b=torch.broadcast_tensors(axes_a,axes_b)
    cross=torch.linalg.cross(axes_a.unsqueeze(-2),axes_b.unsqueeze(-3),dim=-1).flatten(-3,-2)
    axes=torch.cat((axes_a,axes_b,cross),dim=-2)
    norm=torch.linalg.vector_norm(axes,dim=-1)
    axes=axes/norm.clamp_min(1e-10).unsqueeze(-1)
    extent_a=(torch.abs(axes@ra)*ha.unsqueeze(-2)).sum(-1)
    extent_b=(torch.abs(axes@rb)*hb.unsqueeze(-2)).sum(-1)
    gap=torch.abs((axes*delta.unsqueeze(-2)).sum(-1))-extent_a-extent_b
    return torch.where(norm>1e-7,gap,torch.full_like(gap,-1e6)).amax(-1)


def sphere_box_gap(spheres,centers,rotations,half):
    delta=spheres[...,None,:3]-centers[...,None,:,:]
    local=torch.einsum('...sbi,...bij->...sbj',delta,rotations)
    d=torch.abs(local)-half[...,None,:,:]
    return torch.linalg.vector_norm(torch.relu(d),dim=-1)+torch.minimum(d.amax(-1),torch.zeros_like(d[...,0]))-spheres[...,None,3]


class PairGeometry:
    def __init__(self,bundle,sphere_names,device='cuda'):
        self.bundle=bundle;self.sphere_names=sphere_names
        def tensor(x):return torch.as_tensor(np.asarray(x),dtype=torch.float32,device=device)
        payload=bundle['request']['payload'];policy=bundle['request']['collision_policy']
        self.gap=float(policy['required_pair_clearance_m'])
        self.moving=[*bundle['authority_tool'],dict(name=payload['object_id'],compliant=False,
                       dimensions_m=payload['dimensions_m'],parent_from_object=payload['flange_from_object'])]
        self.world=bundle['obstacles'];self.moving_names=[x['name'] for x in self.moving]
        self.local_c=tensor([np.asarray(x['parent_from_object'])[:3,3] for x in self.moving])
        self.local_r=tensor([np.asarray(x['parent_from_object'])[:3,:3] for x in self.moving])
        self.half=tensor([x['dimensions_m'] for x in self.moving])/2
        self.world_c=tensor([np.asarray(x['parent_from_object'])[:3,3] for x in self.world])
        self.world_r=tensor([np.asarray(x['parent_from_object'])[:3,:3] for x in self.world])
        self.world_h=tensor([x['dimensions_m'] for x in self.world])/2
        permissions=bundle['pair_permissions']
        base_pairs={tuple(p) for p in permissions['base_mount']}
        # Match the authority's exact fixed pair, not a broad link/world filter.
        if not base_pairs<={('base_link','chassis')}:raise ValueError('MODEL_MISMATCH: unknown installation permission')
        if not set(policy['wrist_tool_exempt_links'])<={'J5_link','J6_link'}:
            raise ValueError('MODEL_MISMATCH: unknown wrist ownership permission')
        robot_world=[[self.gap if (s,w['name']) not in base_pairs else -1e6 for w in self.world] for s in sphere_names]
        robot_moving=[[(-1e6 if s in policy['wrist_tool_exempt_links'] and i<len(bundle['tool']) else self.gap)
                       for i in range(len(self.moving))] for s in sphere_names]
        moving_world=[[(-1e6 if m.get('compliant') and policy['compliant_cup_neighbor_contact_mode']=='ignore'
                        and w['name'] in permissions['named_stack_cartons'] and w['name']!=payload['object_id'] else self.gap)
                       for w in self.world] for m in self.moving]
        self.robot_world=tensor(robot_world);self.robot_moving=tensor(robot_moving);self.moving_world=tensor(moving_world)
        self.tool_payload=tensor([-float(policy['maximum_compliant_cup_additional_compression_m'])
                                 if m.get('compliant') else self.gap for m in bundle['tool']])

    def moving_poses(self,state):
        index=state.tool_poses.tool_frames.index('flange')
        c=state.tool_poses.position[:,:,index].reshape(-1,3)
        r=quaternion_matrix(state.tool_poses.quaternion[:,:,index].reshape(-1,4))
        return c[:,None,:]+torch.einsum('bij,kj->bki',r,self.local_c),r[:,None]@self.local_r

    def terms(self,spheres,c,r):
        rw=sphere_box_gap(spheres,self.world_c,self.world_r,self.world_h)
        rm=sphere_box_gap(spheres,c,r,self.half)
        mw=box_separation(c[:,:,None],r[:,:,None],self.half[None,:,None],
                          self.world_c[None,None],self.world_r[None,None],self.world_h[None,None])
        tp=box_separation(self.local_c[:-1],self.local_r[:-1],self.half[:-1],
                          self.local_c[-1],self.local_r[-1],self.half[-1])[None].expand(len(c),-1)
        return ((rw,self.robot_world),(rm,self.robot_moving),(mw,self.moving_world),(tp,self.tool_payload))

    def cost(self,state,activation_extra=0.):
        c,r=self.moving_poses(state);s=state.robot_spheres.reshape(len(c),-1,4)
        outputs=[]
        # GPU AABB broadphase filters provably distant pairs before narrowphase.
        # nonzero gathers GPU indices; no robot states or geometry leave CUDA.
        for start in range(0,len(c),128):
            cc=c[start:start+128];rr=r[start:start+128];ss=s[start:start+128]
            value=torch.zeros(len(cc),device=cc.device,dtype=cc.dtype)
            for centers,rot,half,threshold in ((self.world_c,self.world_r,self.world_h,self.robot_world),
                                             (cc,rr,self.half,self.robot_moving)):
                if centers.ndim==2:
                    centers=centers[None].expand(len(cc),-1,-1);rot=rot[None].expand(len(cc),-1,-1,-1)
                extents=(rot.abs()@half[...,None]).squeeze(-1)
                lower=(torch.abs(ss[:,:,None,:3]-centers[:,None])-ss[:,:,None,3,None]-extents[:,None]).amax(-1)
                bi,si,oi=torch.nonzero(lower<threshold+activation_extra,as_tuple=True)
                chosen=ss[bi,si];delta=chosen[:,:3]-centers[bi,oi]
                local=torch.einsum('ni,nij->nj',delta,rot[bi,oi]);d=local.abs()-half[oi]
                gap=torch.linalg.vector_norm(torch.relu(d),dim=-1)+torch.minimum(d.amax(-1),torch.zeros_like(d[:,0]))-chosen[:,3]
                value=value.scatter_add(0,bi,torch.relu(threshold[si,oi]+activation_extra-gap))
            # All 203 moving OBBs vs every world obstacle; no box corners or
            # rigid inserts omitted. AABB is a lower bound, not the narrowphase.
            ma=(rr.abs()@self.half[...,None]).squeeze(-1)
            wa=(self.world_r.abs()@self.world_h[...,None]).squeeze(-1)
            lower=(torch.abs(cc[:,:,None]-self.world_c[None,None])-ma[:,:,None]-wa[None,None]).amax(-1)
            bi,mi,wi=torch.nonzero(lower<self.moving_world+activation_extra,as_tuple=True)
            gap=box_separation(cc[bi,mi],rr[bi,mi],self.half[mi],self.world_c[wi],self.world_r[wi],self.world_h[wi])
            value=value.scatter_add(0,bi,torch.relu(self.moving_world[mi,wi]+activation_extra-gap))
            # The rigid attachment makes these relative poses invariant. Keep
            # the constraint, including bounded cup compression, on the GPU.
            tp=box_separation(self.local_c[:-1],self.local_r[:-1],self.half[:-1],
                              self.local_c[-1],self.local_r[-1],self.half[-1])
            value=value+torch.relu(self.tool_payload+activation_extra-tp).sum()
            outputs.append(value)
        return torch.cat(outputs).reshape(state.robot_spheres.shape[:2])

    def diagnose(self,state):
        with torch.no_grad():
            c,r=self.moving_poses(state);s=state.robot_spheres.reshape(len(c),-1,4)
            result=[]
            labels=[(self.sphere_names,[x['name'] for x in self.world]),(self.sphere_names,self.moving_names),
                    (self.moving_names,[x['name'] for x in self.world]),(self.moving_names[:-1],[self.moving_names[-1]])]
            for i in range(len(c)):
                pairs=[]
                for kind,((g,t),(a,b)) in enumerate(zip(self.terms(s[i:i+1],c[i:i+1],r[i:i+1]),labels)):
                    gap=g[0].cpu().numpy();threshold=t.cpu().numpy()
                    if kind==3:gap=gap[:,None];threshold=threshold[:,None]
                    seen={}
                    for x,y in np.argwhere(gap<threshold+1e-6):
                        key=(a[x],b[y]);entry=dict(pair=list(key),surface_gap_or_SAT_lower_bound_m=float(gap[x,y]),required_m=float(threshold[x,y]),kind=kind)
                        if key not in seen or entry['surface_gap_or_SAT_lower_bound_m']<seen[key]['surface_gap_or_SAT_lower_bound_m']:seen[key]=entry
                    pairs.extend(seen.values())
                result.append(pairs)
            return result

    def focus_pairs(self,state):
        with torch.no_grad():
            c,r=self.moving_poses(state);s=state.robot_spheres.reshape(len(c),-1,4)
            result=[];world_names=[x['name'] for x in self.world]
            for i in range(len(c)):
                terms=self.terms(s[i:i+1],c[i:i+1],r[i:i+1]);rows=[]
                for kind,first,second in [(0,'base_link','chassis'),(3,'tool_rigid_0',self.moving_names[-1]),
                        (2,self.moving_names[-1],'carton_l06_c04'),(2,self.moving_names[-1],'conveyor_longitudinal')]:
                    if kind in (0,2) and second not in world_names:continue
                    if kind in (2,3) and first not in self.moving_names:continue
                    if kind==0:
                        ids=[j for j,n in enumerate(self.sphere_names) if n==first];j=world_names.index(second)
                        if not ids:continue
                        gap=terms[0][0][0,ids,j].min().item()
                        allowed=bool((self.robot_world[ids,j]<-100).all().item())
                        required=0. if allowed else self.gap
                        permission='existing fixed installation' if allowed else None
                    elif kind==3:
                        j=self.moving_names.index(first);gap=terms[3][0][0,j].item();required=terms[3][1][j].item();permission=None
                    else:
                        j=self.moving_names.index(first);k=world_names.index(second)
                        gap=terms[2][0][0,j,k].item();required=terms[2][1][j,k].item();permission=None
                    rows.append(dict(pair=[first,second],gpu_gap_m=gap,required_m=required,permission=permission,
                                     accepted=permission is not None or gap>=required))
                result.append(rows)
            return result


class PairCollisionCost(BaseCost):
    def __init__(self,config,geometry):
        super().__init__(config);self.geometry=geometry
        # Hard metrics use the 5 mm pair rule. Soft activation is distinct.
        self.extra=max(0.,float(config.activation_distance.item())-geometry.gap)
    def forward(self,state,idxs_env_query=None,trajectory_dt=None):
        return (self.geometry.cost(state,self.extra)*self._weight).unsqueeze(-1)


def install_pair_costs(planner,bundle,prepared):
    params=planner.kinematics.config.kinematics_config
    ids=params.link_sphere_idx_map.detach().cpu().numpy().reshape(-1)
    names={v:k for k,v in params.link_name_to_idx_map.items()}
    geometry=PairGeometry(bundle,[names[int(i)] for i in ids])
    rollouts=[]
    for component in (planner.ik_solver,planner.trajopt_solver,planner.graph_planner):
        rollouts.extend(component.get_all_rollout_instances())
        # Pinned 4ea77366 SolverCore excludes these from get_all_rollout_instances.
        # Its B-spline output check must use the same geometry and policy.
        core=getattr(component,'core',None)
        rollouts.extend(getattr(core,'additional_metrics_rollouts',{}).values())
    count=0
    for rollout in {id(r):r for r in rollouts}.values():
        for manager in rollout._cost_manager_list:
            old=manager.get_cost('scene_collision')
            if old is not None:
                manager.costs['scene_collision']=PairCollisionCost(old.config,geometry);count+=1
    if count==0:raise RuntimeError('No V2 rollout collision costs installed')
    geometry.installed_cost_count=count
    return geometry
