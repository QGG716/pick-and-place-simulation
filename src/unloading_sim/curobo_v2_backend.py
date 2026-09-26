"""Optional persistent cuRobo v0.8.0 worker. Imports CUDA only on construction.

Stdout protocol is one JSON line per candidate; native logs go to stderr.
The exact commit is checked against an installation provenance record.
"""
from __future__ import annotations
import contextlib
from dataclasses import asdict
from importlib.metadata import version
import json
from pathlib import Path
import sys
from time import perf_counter
import traceback
import numpy as np
from .stage_backend import fingerprint, pose_wxyz
from .stage_export import file_hash,worker_context_key

CUROBO_COMMIT = '4ea77366ca48ee453e7df139e39fa6532af49f3b'


GEOMETRY_SCHEMA = 'curobo_geometry_v3_robot_local_cover'
FIT_SETTINGS = dict(method='MORPHIT', num_spheres=64, iterations=100,
                    surface_samples=20000, local_cover_padding_m=0.000002)


def geometry_cache_key(bundle, settings=None):
    # No policy, limits, attachment, default q or assembled robot_cfg is cached.
    return fingerprint(dict(schema=GEOMETRY_SCHEMA, commit=CUROBO_COMMIT,
        meshes=[{k:v for k,v in x.items() if k!='path'} for x in bundle['meshes']],
        settings=FIT_SETTINGS if settings is None else settings, seed=bundle['request']['seed']))


def local_cover(centers, radii, points, padding):
    distances=np.linalg.norm(points[:,None,:]-centers[None,:,:],axis=-1)-radii
    owners=np.argmin(distances,axis=1)
    increments=np.zeros(len(radii))
    np.maximum.at(increments,owners,np.maximum(distances[np.arange(len(points)),owners]+padding,0))
    return radii+increments, increments


def prepare_geometry(bundle, directory, settings=None):
    from copy import deepcopy
    settings=deepcopy(FIT_SETTINGS if settings is None else settings)
    start=perf_counter();folder=Path(directory);folder.mkdir(parents=True,exist_ok=True)
    key=geometry_cache_key(bundle,settings);target=folder/(key+'.json')
    if target.exists():
        record=json.loads(target.read_text())
        if record.get('schema')!=GEOMETRY_SCHEMA or record.get('key')!=key or 'robot_cfg' in record:
            raise ValueError('MODEL_MISMATCH: invalid geometry cache schema')
        return record,dict(cache_hit=True,geometry_s=perf_counter()-start,key=key)
    import torch,trimesh
    from curobo._src.geom.sphere_fit import fit_spheres_to_mesh,SphereFitType
    from curobo._src.geom.sphere_fit.metrics import compute_sphere_fit_metrics
    from .geometry import rotation_matrix_from_rpy
    np.random.seed(bundle['request']['seed']);torch.manual_seed(bundle['request']['seed'])
    spheres,quality={},{}
    for item in bundle['meshes']:
        if file_hash(item['path'])!=item['sha256']:raise ValueError('MODEL_MISMATCH: mesh changed')
        print('Local sphere cover:',item['link'],file=sys.stderr,flush=True)
        mesh=trimesh.load(item['path'],force='mesh');mesh.apply_scale(item['scale'])
        m=np.eye(4);m[:3,:3]=rotation_matrix_from_rpy(*item['rpy']);m[:3,3]=item['xyz'];mesh.apply_transform(m)
        fit=fit_spheres_to_mesh(mesh,num_spheres=settings['num_spheres'],fit_type=SphereFitType.MORPHIT,
                                iterations=settings['iterations'],compute_metrics=True)
        centers=fit.centers.detach().cpu().numpy();radii=fit.radii.detach().cpu().numpy()
        points=np.concatenate([mesh.vertices,trimesh.sample.sample_surface(mesh,settings['surface_samples'])[0]])
        enlarged,increments=local_cover(centers,radii,points,settings['local_cover_padding_m'])
        after=compute_sphere_fit_metrics(mesh,centers,enlarged)
        spheres[item['link']]=[dict(center=x.tolist(),radius=float(r)) for x,r in zip(centers,enlarged)]
        quality[item['link']]=dict(source=item,before=asdict(fit.metrics),after=asdict(after),
            local_radius_increments_m=increments.tolist(),coverage_proven=False,
            method='nearest_sphere_local_sample_cover_no_radius_shrinking')
    torch.cuda.synchronize()
    record=dict(schema=GEOMETRY_SCHEMA,key=key,settings=settings,spheres=spheres,quality=quality,
                source_commit=CUROBO_COMMIT,fitting_s=perf_counter()-start)
    target.write_text(json.dumps(record,allow_nan=False))
    return record,dict(cache_hit=False,geometry_s=perf_counter()-start,key=key)


def assemble_runtime(bundle, geometry):
    from copy import deepcopy
    start=perf_counter();spheres=deepcopy(geometry['spheres'])
    def mass_fields(mass,com,tensor):
        inertia=np.asarray(tensor,float)
        return dict(link_mass=float(mass),link_com=list(com),
                    link_inertia=[float(inertia[i,j]) for i,j in ((0,0),(1,1),(2,2),(0,1),(0,2),(1,2))])
    tool_mass=bundle['tool_dynamics']
    extras = {'tool_mass':dict(parent_link_name=tool_mass['attach_to_link'],link_name='tool_mass',
        joint_name='tool_mass_fixed',joint_type='FIXED',fixed_transform=[0,0,0,1,0,0,0],
        **mass_fields(tool_mass['mass_kg'],tool_mass['com_xyz_m'],tool_mass['inertia_tensor_com_kg_m2']))}

    tool_links = []
    for i,item in enumerate(bundle['tool']):
        name = 'tool_part_'+str(i)
        tool_links.append(name)
        extras[name] = dict(parent_link_name='flange',link_name=name,joint_name=name+'_fixed',
                            joint_type='FIXED',fixed_transform=pose_wxyz(item['parent_from_object']),
                            **mass_fields(0.,[0.,0.,0.],np.zeros((3,3))))
        # All physical parts are exact OBBs in the GPU rollout collision cost.
    payload = bundle['request']['payload']
    extras['held_carton'] = dict(parent_link_name='flange',link_name='held_carton',joint_name='held_carton_fixed',
                                joint_type='FIXED',fixed_transform=pose_wxyz(payload['flange_from_object']),
                                **mass_fields(payload['mass_kg'],payload['com_xyz_m'],payload['inertia_tensor_com_kg_m2']))
    ignored = {}
    for a,b in bundle['self_collision_ignore']:
        ignored.setdefault(a,[]).append(b)
    # Tool/payload pairs use typed thresholds in the GPU OBB cost, not robot-self rules.
    d = bundle['request']
    # Source URDF is preserved byte-for-byte; package URIs are resolved using its package root.
    k = dict(format_version=2.0, urdf_path=bundle['urdf_path'],
             asset_root_path=str(Path(bundle['urdf_path']).parents[2]), base_link='base_link',
             tool_frames=['flange','fanuc_flange','tool0','held_carton','tool_mass'],
             collision_link_names=list(spheres), collision_spheres=spheres,
             collision_sphere_buffer=0., self_collision_buffer={name:0. for name in spheres},
             self_collision_ignore=ignored,extra_links=extras,
             cspace=dict(joint_names=d['joint_names'],default_joint_position=d['q_start'],
                         cspace_distance_weight=[1.]*6,null_space_weight=[1.]*6,
                         max_acceleration=d['limits']['acceleration'],max_jerk=d['limits']['jerk']))
    return dict(robot_cfg=dict(kinematics=k,load_dynamics=False),quality=geometry['quality'],
                geometry_schema=GEOMETRY_SCHEMA,assembly_s=perf_counter()-start,
                policy_fingerprint=d['collision_policy_fingerprint'],
                runtime_policy=deepcopy(d['collision_policy']),pair_permissions=deepcopy(bundle['pair_permissions']),
                exact_obb_count=len(bundle['tool'])+1,dynamics_constraints_enabled=False)


def prepare(bundle,directory):
    geometry,timing=prepare_geometry(bundle,directory)
    record=assemble_runtime(bundle,geometry);record['geometry_cache']=timing
    return record



class CuroboWorker:
    def __init__(self, bundle, cache_directory):
        import torch
        import curobo
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        from curobo._src.state.state_joint import JointState
        from curobo.content import get_task_configs_path
        import yaml
        self.torch,self.JointState = torch,JointState
        source = Path(curobo.__file__).resolve().parents[1]
        provenance = json.loads((source/'PINNED_SOURCE.json').read_text())
        if provenance['commit'] != CUROBO_COMMIT or version('nvidia-curobo').split('+')[0] != '0.8.0':
            raise ValueError('MODEL_MISMATCH: cuRobo version/commit')
        if file_hash(bundle['urdf_path']) != bundle['model_identity']['urdf_sha256']:
            raise ValueError('MODEL_MISMATCH: source URDF changed')
        for mesh in bundle['meshes']:
            if file_hash(mesh['path']) != mesh['sha256']:
                raise ValueError('MODEL_MISMATCH: mesh changed')
        from .stage_backend import StageRequest
        StageRequest(bundle['request'])
        if fingerprint({k:v for k,v in bundle.items() if k!='bundle_fingerprint'}) != bundle['bundle_fingerprint']:
            raise ValueError('MODEL_MISMATCH: bundle fingerprint')
        self.backend = dict(name='curobo_v2',version=version('nvidia-curobo'),commit=CUROBO_COMMIT,
                            import_path=curobo.__file__,python=sys.version,torch=torch.__version__,
                            cuda_runtime=torch.version.cuda,warp=version('warp-lang'),
                            gpu=torch.cuda.get_device_name(),capability=list(torch.cuda.get_device_capability()))
        before = perf_counter()
        self.prepared = prepare(bundle,cache_directory)
        self.d = bundle['request']
        self.bundle = bundle
        self.model_key = fingerprint([self.d[k] for k in ('robot_model_fingerprint','tool_fingerprint','payload_fingerprint','collision_policy_fingerprint')])
        scene = {'cuboid':{x['name']:dict(dims=x['dimensions_m'],pose=pose_wxyz(x['parent_from_object'])) for x in bundle['obstacles']}}
        graph = yaml.safe_load((Path(get_task_configs_path())/'graph_planner/exact_graph_planner.yml').read_text())
        graph['graph_planner']['sampler_seed'] = self.d['seed']
        metrics = yaml.safe_load((Path(get_task_configs_path())/'metrics_base.yml').read_text())
        gap = self.d['collision_policy']['required_pair_clearance_m']
        if gap is None:
            raise ValueError('MODEL_MISMATCH: only current POC pair clearance exported')
        metrics['rollout']['constraint_cfg']['scene_collision_cfg']['activation_distance'] = gap
        # v0.8.0 mutates dict rollout configs when building IK, then reuses
        # them for TrajOpt. A YAML path gives each factory a fresh dictionary.
        metrics_path=Path(cache_directory)/('metrics-'+fingerprint(metrics)+'.yml')
        metrics_path.write_text(yaml.safe_dump(metrics))
        from copy import deepcopy
        robot_cfg=deepcopy(self.prepared['robot_cfg'])
        robot_cfg['kinematics']['cspace']['default_joint_position']=self.d['q_start']
        for extra in robot_cfg['kinematics']['extra_links'].values():
            extra['link_com']=np.asarray(extra['link_com'])
            extra['link_inertia']=np.asarray(extra['link_inertia'])
        # Fixed v0.8.0 parser computes inertia_array but returns its default
        # inertia and applies the inertial origin translation twice. Correct the
        # parsed tensors from the hash-verified official XML before cloning into
        # IK, TrajOpt and graph rollout configs. No third-party source patch.
        import xml.etree.ElementTree as ET
        from curobo._src.types.robot import RobotCfg
        native_robot=RobotCfg.create({'robot_cfg':robot_cfg})
        params=native_robot.kinematics.kinematics_config
        self.parser_inertial_before={}
        for name,xml in bundle['model_identity']['inertials'].items():
            element=ET.fromstring(xml)
            if any(abs(float(x))>1e-12 for x in element.find('origin').get('rpy','0 0 0').split()):
                raise ValueError('MODEL_MISMATCH: rotated inertial origin needs explicit tensor transform')
            mass=float(element.find('mass').get('value'))
            com=[float(x) for x in element.find('origin').get('xyz').split()]
            tensor=[float(element.find('inertia').get(x)) for x in ('ixx','iyy','izz','ixy','ixz','iyz')]
            self.parser_inertial_before[name]=dict(mass_com=params.get_link_masses_com(name).detach().cpu().tolist(),
                                                   inertia_six=params.get_link_inertia(name).detach().cpu().tolist())
            params.update_link_mass(name,mass)
            params.update_link_com(name,torch.tensor(com,device='cuda',dtype=torch.float32))
            params.update_link_inertia(name,torch.tensor(tensor,device='cuda',dtype=torch.float32))
        cfg = MotionPlannerCfg.create(robot=native_robot,scene_model=scene,
               num_trajopt_seeds=self.d['resources']['num_seeds'], random_seed=self.d['seed'],
               graph_planner_config=graph, metrics_rollout=str(metrics_path.resolve()), use_cuda_graph=False, self_collision_check=True,
               optimizer_collision_activation_distance=.01)
        self.planner = MotionPlanner(cfg)
        from .curobo_collision import install_pair_costs
        self.pair_collision=install_pair_costs(self.planner,bundle,self.prepared)
        if self.planner.kinematics.joint_names != self.d['joint_names']:
            raise ValueError('MODEL_MISMATCH: GPU joint order')
        self.backend['configuration']=dict(num_trajopt_seeds=self.d['resources']['num_seeds'],use_cuda_graph=False,
            self_collision_check=True,hard_environment_gap_m=gap,optimizer_activation_distance_m=.01,
            graph_config_sha256=file_hash(Path(get_task_configs_path())/'graph_planner/exact_graph_planner.yml'),
            metrics_config_sha256=file_hash(metrics_path),dynamics_constraints_enabled=False)
        self.native_metric_observations=[]
        original_result=self.planner.trajopt_solver._get_result
        def observed_result(*args,**kwargs):
            result=original_result(*args,**kwargs)
            self.native_metric_observations.append(result)
            return result
        self.planner.trajopt_solver._get_result=observed_result
        self.graph_observations = []
        original = self.planner.graph_planner.find_path
        def observed(*args,**kwargs):
            result = original(*args,**kwargs)
            self.graph_observations.append(dict(success=result.success.detach().cpu().tolist()))
            return result
        self.planner.graph_planner.find_path = observed
        torch.cuda.synchronize()
        self.initialization_s = perf_counter()-before
        self.warmup_s = None
        self.initialization_reported = False
        self.consistency=self.check_consistency()
        if not self.consistency['passed']:
            raise ValueError('MODEL_MISMATCH: FK, inertial or joint-limit audit failed: '+str(self.consistency))

    def model_audit(self):
        import xml.etree.ElementTree as ET
        params=self.planner.kinematics.config.kinematics_config
        limits=self.planner.kinematics.get_joint_limits()
        records={}
        for name,xml in self.bundle['model_identity']['inertials'].items():
            source=ET.fromstring(xml)
            expected_mass=float(source.find('mass').get('value'))
            expected_com=np.array([float(x) for x in source.find('origin').get('xyz').split()])
            expected_inertia=np.array([float(source.find('inertia').get(x)) for x in ('ixx','iyy','izz','ixy','ixz','iyz')])
            mc=params.get_link_masses_com(name).detach().cpu().numpy()
            inertia=params.get_link_inertia(name).detach().cpu().numpy()
            records[name]=dict(mass_com=mc.tolist(),inertia_six=inertia.tolist(),
                passed=bool(np.allclose(mc,np.r_[expected_com,expected_mass],atol=2e-5,rtol=1e-6)
                            and np.allclose(inertia,expected_inertia,atol=2e-5,rtol=1e-6)))
        for name,source in (('tool_mass',self.bundle['tool_dynamics']),('held_carton',self.d['payload'])):
            mc=params.get_link_masses_com(name).detach().cpu().numpy()
            inertia=params.get_link_inertia(name).detach().cpu().numpy()
            tensor=np.asarray(source['inertia_tensor_com_kg_m2'])
            expected=[tensor[i,j] for i,j in ((0,0),(1,1),(2,2),(0,1),(0,2),(1,2))]
            records[name]=dict(mass_com=mc.tolist(),inertia_six=inertia.tolist(),
                passed=bool(np.allclose(mc,np.r_[source['com_xyz_m'],source['mass_kg']],atol=2e-5,rtol=1e-6)
                            and np.allclose(inertia,expected,atol=2e-5,rtol=1e-6)))
        return dict(joint_names=self.planner.kinematics.joint_names,
                    limits={key:getattr(limits,key).detach().cpu().tolist() for key in ('position','velocity','acceleration','jerk','effort')},
                    inertials=records,parser_inertial_before=self.parser_inertial_before,dynamics_constraints_enabled=False,
                    massless_frames='tool_part_* are collision frames; tool_mass carries the complete 20 kg inertia exactly once')

    def check_consistency(self):
        audit=self.model_audit()
        limits=audit['limits'];expected=self.d['limits']
        position=np.array([expected['lower'],expected['upper']])
        passed=bool(np.allclose(limits['position'],position,atol=2e-5,rtol=1e-6))
        for key in ('velocity','acceleration','jerk','effort'):
            passed=passed and bool(np.allclose(limits[key],[-np.asarray(expected[key]),expected[key]],atol=2e-5,rtol=1e-6))
        errors=[];ref=self.bundle['fk_reference'];actual=self.fk(ref['states'])
        for i,poses in enumerate(ref['poses']):
            for name,pose in poses.items():
                got=actual[name];p=np.asarray(got['position']).reshape(-1,3)[i]
                q=np.asarray(got['quaternion_wxyz']).reshape(-1,4)[i];exp=np.asarray(pose)
                errors.append(dict(state=i,frame=name,position_m=float(np.max(np.abs(p-exp[:3]))),
                    quaternion_error=float(min(np.linalg.norm(q-exp[3:]),np.linalg.norm(q+exp[3:])))))
        return dict(passed=passed and all(x['passed'] for x in audit['inertials'].values()) and
                    all(x['position_m']<ref['tolerance'] and x['quaternion_error']<ref['tolerance'] for x in errors),
                    limits_passed=passed,fk_errors=errors,collision_representation_equivalence_proven=False)

    def collision_diagnostics(self,states):
        js=self.JointState.from_position(self.torch.tensor(states,device='cuda',dtype=self.torch.float32),self.d['joint_names'])
        state=self.planner.compute_kinematics(js)
        spheres=state.get_link_spheres().detach().cpu().numpy().reshape(len(states),-1,4)
        params=self.planner.kinematics.config.kinematics_config
        indices=params.link_sphere_idx_map.detach().cpu().numpy().reshape(-1)
        idx_names={v:k for k,v in params.link_name_to_idx_map.items()}
        names=[idx_names[int(x)] for x in indices]
        ignored=self.prepared['robot_cfg']['kinematics']['self_collision_ignore']
        ignored_pairs={frozenset((a,b)) for a,bs in ignored.items() for b in bs}
        records=[]
        for row in spheres:
            env=[]
            for box in self.bundle['obstacles']:
                pose=np.asarray(box['parent_from_object']);half=np.asarray(box['dimensions_m'])/2
                local=(row[:,:3]-pose[:3,3])@pose[:3,:3]
                delta=np.abs(local)-half
                signed=np.linalg.norm(np.maximum(delta,0),axis=1)+np.minimum(np.max(delta,axis=1),0)-row[:,3]
                for link in sorted(set(names)):
                    selected=np.array([i for i,n in enumerate(names) if n==link and row[i,3]>0])
                    if len(selected) and np.min(signed[selected])<.005:
                        env.append(dict(link=link,object=box['name'],sphere_surface_gap_m=float(np.min(signed[selected]))))
            self_pairs=[]
            for i in range(len(row)):
                if row[i,3]<=0:continue
                distances=np.linalg.norm(row[i+1:,:3]-row[i,:3],axis=1)-row[i+1:,3]-row[i,3]
                for j0 in np.where(distances<0)[0]:
                    j=i+1+j0
                    if row[j,3]>0 and names[i]!=names[j] and frozenset((names[i],names[j])) not in ignored_pairs:
                        self_pairs.append(dict(links=[names[i],names[j]],sphere_surface_gap_m=float(distances[j0])))
            # Keep worst pair per link pair, not thousands of overlapping spheres.
            worst={}
            for pair in self_pairs:
                key=tuple(sorted(pair['links']))
                if key not in worst or pair['sphere_surface_gap_m']<worst[key]['sphere_surface_gap_m']:worst[key]=pair
            records.append(dict(environment=sorted(env,key=lambda x:x['sphere_surface_gap_m']),
                                self_collision=sorted(worst.values(),key=lambda x:x['sphere_surface_gap_m'])))
        return dict(scope='diagnostic_CPU_pair_analysis_of_GPU_transformed_spheres_not_authority',states=records)

    def fk(self,states):
        js = self.JointState.from_position(self.torch.tensor(states,device='cuda',dtype=self.torch.float32),self.d['joint_names'])
        out = self.planner.compute_kinematics(js).tool_poses.to_dict()
        return {k:dict(position=v.position.detach().cpu().tolist(),quaternion_wxyz=v.quaternion.detach().cpu().tolist()) for k,v in out.items()}

    def native_endpoints(self,states):
        q=self.torch.tensor(states,device='cuda',dtype=self.torch.float32)
        # Exactly the graph planner's native feasibility entry, also used to
        # reject start/end nodes and every candidate graph edge sample.
        feasible=self.planner.graph_planner.check_samples_feasibility(q)
        state=self.planner.compute_kinematics(self.JointState.from_position(q,self.d['joint_names']))
        accepted=feasible.detach().cpu().reshape(-1).tolist()
        return dict(feasible=accepted,pairs=self.pair_collision.diagnose(state),
                    self_pairs=[] if all(accepted) else [x['self_collision'] for x in self.collision_diagnostics(states)['states']],
                    focus_pairs=self.pair_collision.focus_pairs(state))

    def solve(self,attempt):
        torch = self.torch
        torch.cuda.synchronize(); start=perf_counter()
        endpoints=self.native_endpoints([self.d['q_start'],self.d['q_goal']])
        if not all(endpoints['feasible']):
            return dict(backend=self.backend,status='MODEL_MISMATCH',trajectory=None,
                error=dict(reason='NATIVE_COLLISION_REJECTS_AUTHORITY_ENDPOINT',native_endpoints=endpoints),
                timings=dict(worker_request_s=perf_counter()-start))
        if self.warmup_s is None:
            t=perf_counter(); self.native_endpoints([self.d['q_start'],self.d['q_goal']]); torch.cuda.synchronize()
            self.warmup_s=perf_counter()-t
        h2d=perf_counter()
        start_js=self.JointState.from_position(torch.tensor([self.d['q_start']],device='cuda',dtype=torch.float32),self.d['joint_names'])
        goal_js=self.JointState.from_position(torch.tensor([self.d['q_goal']],device='cuda',dtype=torch.float32),self.d['joint_names'])
        torch.cuda.synchronize(); h2d=perf_counter()-h2d
        self.graph_observations.clear(); self.native_metric_observations.clear()
        t=perf_counter(); a,b=torch.cuda.Event(enable_timing=True),torch.cuda.Event(enable_timing=True)
        a.record()
        # Each outer authority rejection advances to graph seeds; do not repeat
        # the identical deterministic direct candidate after an exact rejection.
        native=self.planner.plan_cspace(goal_js,start_js,max_attempts=1,enable_graph_attempt=1 if attempt==0 else 0)
        b.record(); b.synchronize(); solve=perf_counter()-t
        record=dict(backend=self.backend,status='BACKEND_NO_CANDIDATE',trajectory=None,
                    graph_enabled=attempt>0,graph_observations=list(self.graph_observations),
                    seeds=dict(optimizer=self.d['seed'],graph=self.d['seed'],attempt=attempt),
                    timings=dict(initialization_s=0. if self.initialization_reported else self.initialization_s,warmup_s=0. if self.initialization_reported else self.warmup_s,preparation=dict(geometry=self.prepared['geometry_cache'],configuration_assembly_s=self.prepared['assembly_s']),initialization_includes_preparation=True,warmup_scope='fixed_joint_FK_and_native_collision_no_pose_IK',
                                 h2d_s=h2d,solve_including_gpu_wait_s=solve,gpu_event_s=a.elapsed_time(b)/1000,
                                 native_total_s=None if native is None else native.total_time,
                                 native_solve_s=None if native is None else native.solve_time),
                    memory_bytes=dict(allocated=torch.cuda.memory_allocated(),peak=torch.cuda.max_memory_allocated()))
        self.initialization_reported=True
        if native is not None:
            record['native_success']=native.success.detach().cpu().tolist()
            record['native_status']=str(getattr(native,'status',None))
            def finite_tensor(value):
                if value is None:return None
                arr=value.detach().cpu().numpy() if hasattr(value,"detach") else np.asarray(value)
                return dict(shape=list(arr.shape),finite=bool(np.isfinite(arr).all()),
                    min=float(np.nan_to_num(arr).min()),max=float(np.nan_to_num(arr).max()),
                    nonzero=int(np.count_nonzero(arr)))
            detail={key:finite_tensor(getattr(native,key,None)) for key in
                ('feasible','cspace_error','position_error','rotation_error','minimum_trajectory_dt','maximum_trajectory_dt')}
            for attr in ('metrics','interpolated_metrics'):
                metrics=getattr(self.native_metric_observations[-1] if self.native_metric_observations else native,attr,None)
                if metrics is not None:
                    cc=metrics.costs_and_constraints
                    detail[attr]={}
                    for category in ('constraints','hybrid_costs_constraints','convergence'):
                        entries=getattr(metrics if category=='convergence' else cc,category,None)
                        if entries is not None:
                            detail[attr][category]={name:finite_tensor(v) for name,v in zip(entries.names,entries.values)}
            record['native_diagnostics']=detail
            if not bool(native.success.any().item()):
                record['error']=dict(reason='NATIVE_OPTIMIZATION_OR_CONVERGENCE_FAILED',native_endpoints=endpoints,native_constraints=detail)

            if bool(native.success.any().item()):
                t=perf_counter(); js=native.get_interpolated_plan()
                q=js.position.detach().cpu().numpy().reshape(-1,6)
                dt=float(self.planner.trajopt_solver.config.interpolation_dt)
                def derivative(name):
                    value=getattr(js,name,None)
                    return None if value is None else value.detach().cpu().numpy().reshape(-1,6).tolist()
                record['trajectory']=dict(joint_names=list(js.joint_names or self.d['joint_names']),q=q.tolist(),
                     time_s=(np.arange(len(q))*dt).tolist(),dq=derivative('velocity'),ddq=derivative('acceleration'),
                     valid_length=len(q),interpolation='linear_joint_samples',
                     native_interpolation='V2_evaluated_bspline_samples',
                     native_last_tstep=native.interpolated_last_tstep.detach().cpu().tolist())
                padded=native.interpolated_trajectory.position.detach().cpu().numpy().reshape(-1,6)
                record['native_output']=dict(control_points=native.solution.detach().cpu().tolist(),
                    knot_dt=native.js_solution.knot_dt.detach().cpu().tolist(),
                    optimized_dt=native.js_solution.dt.detach().cpu().tolist(),
                    interpolated_buffer_shape=list(native.interpolated_trajectory.position.shape),
                    interpolation_dt_s=dt,valid_length=len(q),padding_count=len(padded)-len(q),
                    valid_samples_match_native_buffer=bool(np.array_equal(q,padded[:len(q)])),
                    padding_max_position_delta_rad=float(np.max(np.abs(padded[len(q):]-q[-1]))) if len(padded)>len(q) else 0.,
                    start_max_error_rad=float(np.max(np.abs(q[0]-self.d['q_start']))),
                    goal_max_error_rad=float(np.max(np.abs(q[-1]-self.d['q_goal']))))
                record['trajectory']['derivative_semantics']='native evaluated B-spline derivatives; not acceleration/jerk proof for piecewise-linear delivery'
                record['status']='CANDIDATE_GENERATED'
                record['timings']['d2h_conversion_s']=perf_counter()-t
        else:
            record['error']=dict(reason='NATIVE_GRAPH_NO_SEED',native_endpoints=endpoints,graph_observations=list(self.graph_observations))
        record['timings']['worker_request_s']=perf_counter()-start
        return record


def main():
    bundle=json.loads(Path(sys.argv[1]).read_text())
    worker=None
    protocol=sys.stdout
    for line in sys.stdin:
        try:
            command=json.loads(line)
            with contextlib.redirect_stdout(sys.stderr):
                if worker is None:
                    worker=CuroboWorker(bundle,sys.argv[2])
                if command.get('op')=='fk':
                    result=dict(backend=worker.backend,fk=worker.fk(command['states']),model_audit=worker.model_audit(),consistency=worker.consistency,collision_diagnostics=worker.collision_diagnostics(command['states']),native_endpoints=worker.native_endpoints(command['states']),
                                preparation=worker.prepared,initialization_s=worker.initialization_s)
                else:
                    if 'request' in command:
                        from .stage_backend import StageRequest
                        incoming=StageRequest(command['request']).data
                        if worker_context_key(worker.bundle,incoming)!=worker_context_key(worker.bundle):
                            raise ValueError('MODEL_MISMATCH: worker context requires rebuild')
                        worker.d=incoming
                    result=worker.solve(command['attempt'])
        except Exception as exc:
            status=('MODEL_MISMATCH' if 'MODEL_MISMATCH' in str(exc) else
                    'DEPENDENCY_UNAVAILABLE' if isinstance(exc,(ImportError,FileNotFoundError)) else 'BACKEND_ERROR')
            result=dict(status=status,
                        backend=dict(name='curobo_v2',commit=CUROBO_COMMIT),trajectory=None,
                        error=dict(type=type(exc).__name__,message=str(exc),traceback=traceback.format_exc()))
        protocol.write(json.dumps(result,allow_nan=False)+'\n'); protocol.flush()

if __name__=='__main__':
    main()
