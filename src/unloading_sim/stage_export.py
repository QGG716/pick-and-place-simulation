"""Export the existing frozen M710 scene and actual TRANSIT attachment.

No independent layout is invented here. Robot source URDF and collision assets
are hash-bound; tool OBBs come from the same audited physical compound as CPU.
"""
from __future__ import annotations
from dataclasses import asdict
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from .stage_backend import SCHEMA_VERSION, StageRequest, fingerprint, pose_wxyz


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def box_record(box, parent_from_world=None):
    pose = box.world_from_local
    if parent_from_world is not None:
        pose = parent_from_world @ pose
    return dict(name=box.name, category=box.category, dimensions_m=np.round(2*box.half_extents,12).tolist(),
                parent_from_object=np.round(pose,12).tolist())


def box_spheres(dimensions, pitch=.06):
    """Cover the *entire* OBB by circumspheres of a tiled grid (including corners).

    This analytic box cover is deliberately conservative; protrusion is bounded
    by the cell diagonal. It is not a proof of original CAD coverage.
    """
    d = np.asarray(dimensions, float)
    if d.shape != (3,) or np.any(d <= 0) or not np.isfinite(d).all() or pitch <= 0:
        raise ValueError('invalid box cover')
    counts = np.maximum(1, np.ceil(d/pitch).astype(int))
    cell = d/counts
    axes = [-d[i]/2 + cell[i]*(np.arange(counts[i])+.5) for i in range(3)]
    centers = np.stack(np.meshgrid(*axes, indexing='ij'), -1).reshape(-1, 3)
    radius = float(np.linalg.norm(cell)/2 + 1e-7)
    return [dict(center=c.tolist(), radius=radius) for c in centers]


def export_request(scene, connector, attachment, start, goal, *, request_id, seed=716,
                   attempts=3, num_seeds=4):
    from .m710_dynamics import load_m710id70_dynamics
    dynamics = load_m710id70_dynamics()
    robot = connector.robot
    root = scene.policy.project_root
    urdf = connector.robot_state_validator.mesh_robot.urdf_path
    tree = ET.parse(urdf)
    joints = [tree.find(f"joint[@name='J{i}']") for i in range(1, 7)]
    if any(j is None or j.find('mimic') is not None for j in joints):
        raise ValueError('official independent joint identity mismatch')
    limits = {key: [float(j.find('limit').get(attr)) for j in joints]
              for key, attr in [('lower','lower'),('upper','upper'),('velocity','velocity'),('effort','effort')]}
    model = yaml.safe_load((root/'configs/robots/fanuc_m710id_70.yaml').read_text())
    limits['acceleration'] = model['joint_acceleration_limits_rad_s2']['values']
    limits['jerk'] = model['joint_jerk_limits_rad_s3']['values']
    q = np.asarray(start, float)
    flange = robot.named_link_frames(q)['flange']
    base = robot.base_transform
    payload = attachment.box_at(q)
    flange_from_payload = np.linalg.inv(flange) @ payload.world_from_local
    tool_robot = connector.robot_state_validator.tool_transform_robot
    compliant_names = {b.name for b in tool_robot.tool_compliant_collision_obbs(q)}
    tool = [{**box_record(b, np.linalg.inv(flange)), 'compliant': b.name in compliant_names}
            for b in tool_robot.tool_all_physical_obbs(q)]
    obstacles = [box_record(b, np.linalg.inv(base)) for b in scene.all_obstacles if b.name != payload.name]
    if sum(b.name == payload.name for b in scene.all_obstacles) != 1:
        raise ValueError('target must occur exactly once in frozen source world')
    # POC finite side-wall/floor boxes are already supplied by scene.all_obstacles.
    meshes = []
    for link in tree.findall('link'):
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                raise ValueError('unhandled official collision primitive')
            filename = mesh.get('filename')
            path = urdf.parents[2]/filename.removeprefix('package://')
            origin = collision.find('origin')
            meshes.append(dict(link=link.get('name'), path=str(path), sha256=file_hash(path),
                               scale=[float(x) for x in mesh.get('scale','1 1 1').split()],
                               xyz=[float(x) for x in origin.get('xyz','0 0 0').split()],
                               rpy=[float(x) for x in origin.get('rpy','0 0 0').split()]))
    srdf = root/'assets/robots/fanuc_m710id_70/official/m710id_70_official.srdf'
    ignored = [[x.get('link1'),x.get('link2')] for x in ET.parse(srdf).findall('disable_collisions')]
    inertials = {link.get('name'): ET.tostring(link.find('inertial'), encoding='unicode')
                 for link in tree.findall('link') if link.find('inertial') is not None}
    model_identity = dict(urdf_sha256=file_hash(urdf), meshes=[{k:v for k,v in x.items() if k!='path'} for x in meshes],
                          srdf_sha256=file_hash(srdf), inertials=inertials)
    pd = dict(object_id=payload.name, dimensions_m=(2*payload.half_extents).tolist(),
              mass_kg=dynamics.cartons.mass_kg_each, com_xyz_m=list(dynamics.cartons.com_xyz_m),
              inertia_tensor_com_kg_m2=np.asarray(dynamics.cartons.inertia_tensor_com_kg_m2).tolist(),
              flange_from_object=np.round(flange_from_payload,12).tolist())
    tool_dynamics={k:(str(v) if isinstance(v,Path) else v) for k,v in asdict(dynamics.tool).items()}
    tool_dynamics['config_path']=str(Path(tool_dynamics['config_path']).relative_to(root)).replace('\\','/')
    policy = connector.collision_policy.to_mapping()
    d = dict(schema_version=SCHEMA_VERSION, request_id=request_id, stage='TRANSIT',
             scene_revision=scene.snapshot.get('actual_state_context',{}).get('revision',scene.snapshot['scene_fingerprint']),
             scene_fingerprint=scene.snapshot['scene_fingerprint'], robot_model_fingerprint=fingerprint(model_identity),
             tool_fingerprint=fingerprint(dict(geometry=tool,dynamics=tool_dynamics)), payload_fingerprint=fingerprint(pd),
             collision_policy_fingerprint=fingerprint(policy), collision_policy=policy,
             joint_names=[f'J{i}' for i in range(1,7)], q_start=list(map(float,start)), q_goal=list(map(float,goal)),
             boundary_velocity=None, boundary_acceleration=None, limits=limits,
             transforms=dict(world_from_base=base.tolist(), flange_from_tcp=connector.flange_from_virtual_task_tcp.tolist(),
                             flange_from_contact=connector.flange_from_physical_contact.tolist()),
             payload=pd, seed=seed, resources=dict(attempts=attempts,num_seeds=num_seeds),
             deadline_monotonic=None, cancellation_token=None, goal_tolerance_rad=1e-4)
    request = StageRequest(d)
    bundle = dict(request=request.to_dict(), model_identity=model_identity, urdf_path=str(urdf),
                  meshes=meshes, self_collision_ignore=ignored, tool=tool, obstacles=obstacles,
                  tool_dynamics=tool_dynamics,
                  world_count_before=len(scene.all_obstacles), world_count_attached=len(obstacles),
                  snapshot=scene.snapshot, dynamics_constraints_enabled=False,
                  approximation_notes=['robot sphere fit requires numerical coverage audit',
                    'full tool and payload OBBs enter GPU feasibility and optimization',
                    'SAT separation is a conservative Euclidean distance lower bound; authority retained'])
    states=[np.asarray(start),np.asarray(goal),np.asarray(start)+np.array([0,0,.01,0,0,0])]
    reference=[]
    for state in states:
        frames=robot.named_link_frames(state)
        frames['held_carton']=attachment.box_at(state).world_from_local
        reference.append({name:pose_wxyz(np.linalg.inv(base)@frames[name])
                          for name in ('flange','fanuc_flange','tool0','held_carton')})
    bundle['fk_reference']=dict(states=[s.tolist() for s in states],poses=reference,
                                tolerance=2e-5,source='official_project_Pinocchio_and_actual_attachment')
    # Keep the original uncompressed source and its request identity. Collision
    # shapes must match the existing authority's current nominal cup compression.
    validator=connector.robot_state_validator
    actual_shapes={b.name:box_record(b,np.linalg.inv(flange)) for b in
                   [*tool_robot.tool_collision_obbs(q),*validator._compliant_boxes(q)]}
    bundle['authority_tool']=[{**actual_shapes[x['name']],'compliant':x['compliant']} for x in tool]
    bundle['authority_tool_source']=dict(provider='ExactM710LayoutStateValidator._compliant_boxes',
        nominal_cup_compression_m=validator.nominal_cup_compression_m,
        original_uncompressed_source_retained=True)
    bundle['pair_permissions']=dict(base_mount=[['base_link',connector.robot_state_validator.base_support_obstacle_name]],
        source='ExactM710LayoutStateValidator fixed installation pair only',
        named_stack_cartons=sorted(connector.robot_state_validator.stack_carton_names),
        target_id=payload.name,stage='transit')
    bundle['bundle_fingerprint'] = fingerprint(bundle)
    return request, bundle


def released_world(bundle, world_from_object):
    """Return the same identity as a world obstacle, removing the attachment."""
    from .stage_backend import transform
    d = bundle['request']
    base_inv = np.linalg.inv(transform(d['transforms']['world_from_base']))
    item = dict(name=d['payload']['object_id'],category='carton',dimensions_m=d['payload']['dimensions_m'],
                parent_from_object=(base_inv @ transform(world_from_object)).tolist())
    if any(x['name'] == item['name'] for x in bundle['obstacles']):
        raise ValueError('duplicate payload identity')
    return dict(obstacles=[*bundle['obstacles'],item], attached_object=None)


def worker_context_key(bundle, request=None):
    """All configuration dependencies, excluding per-request solve state only."""
    data=bundle['request'] if request is None else request
    per_solve={'request_id','q_start','q_goal','deadline_monotonic','cancellation_token'}
    return fingerprint(dict(configuration={k:v for k,v in bundle.items() if k not in
        {'request','bundle_fingerprint','fk_reference','snapshot'}},
        request_configuration={k:v for k,v in data.items() if k not in per_solve}))
