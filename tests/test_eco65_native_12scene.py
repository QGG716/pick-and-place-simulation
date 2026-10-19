"""Source-preservation and real MuJoCo checks with clearly synthetic fixtures."""
from copy import deepcopy
import hashlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from unloading_sim.eco65.model import transform
from unloading_sim.eco65.native_scene import adapt_static_scene, verify_scene, world_objects
from unloading_sim.eco65.home_candidate import make_candidate, verify_candidate
from unloading_sim.moveit2_backend import digest


def synthetic_twelve_source():
    boxes = []
    for layer in range(4):
        for col in range(3):
            boxes.append(dict(id=f'carton_r00_l{layer:02d}_c{col:02d}', size_m=[.25, .21, .1],
                pose=[.3334, (col-1)*.22, .23+layer*.1], row=0, layer=layer, column=col,
                supported_by=['cargo_pedestal' if layer == 0 else f'carton_r00_l{layer-1:02d}_c{col:02d}'],
                supports=[] if layer == 3 else [f'carton_r00_l{layer+1:02d}_c{col:02d}']))
    source = dict(schema='eco65_full_face_static_scene_v1', layout_id='eco65_desktop_4layer_v1',
        boxes=boxes, base_position_m=[-.36, .1, .18], T_W_D=transform([.32, -.1, .8]).tolist(),
        enable_hardware=False, environment_status='ALL_BOUNDS_INCLUDED_CONSERVATIVELY',
        environment=dict(asset_sha256={}),
        obstacles=[dict(id='robot_mount_plate', size_m=[.14, .14, .18], position_m=[-.36, .1, .09],
                        role='mount', owner='robot_mount_plate', rgba=[.4, .4, .4, 1]),
                   dict(id='cargo_pedestal', size_m=[.25, .65, .18], position_m=[.3334, 0, .09],
                        role='support', owner='cargo_pedestal', rgba=[.4, .4, .4, 1])],
        mast=dict(parts=[dict(id='synthetic_mast')], attachment='link_1',
                  T_link1_mast=transform([.2, 0, .25]).tolist()),
        conveyors=dict(items=[dict(id='conveyor_transverse'), dict(id='conveyor_longitudinal')]))
    source['scene_fingerprint'] = digest(source)
    snapshot = dict(schema='eco65_full_face_static_snapshot_v1', scene=source, config_sha256={})
    snapshot['fingerprint'] = digest(snapshot)
    return snapshot


def synthetic_scene(attachment_mesh='synthetic.obj', attachment_sha='synthetic_hash'):
    import trimesh
    source = synthetic_twelve_source()
    mesh = trimesh.creation.box([.9, .3, .02])
    fixed = dict(id='conveyor_longitudinal__synthetic_belt', category='environment',
        owner='conveyor_longitudinal', role='conveyor_belt', pose=transform([-.5, -.22, .17]).tolist(),
        convex_mesh=dict(vertices=mesh.vertices.tolist(), triangles=mesh.faces.tolist()))
    attachments = [dict(id='synthetic_mast', parent_link='link_1', mesh=attachment_mesh,
        mesh_sha256=attachment_sha, T_parent_local=transform([.2, 0, .25]).tolist(), role='permanent_mast')]
    receiving = dict(region_id='conveyor_longitudinal', support_ids=[fixed['id']],
        bounds_D_m=[[-.92, -.36, .16], [-.08, -.08, .18]], box_pose_D=transform([-.5, -.22, .23]).tolist())
    scene = adapt_static_scene(source, [fixed], attachments, receiving, [0.]*6)
    return source, scene


def test_twelve_scene_preserves_source_and_selects_physical_top_middle():
    source, scene = synthetic_scene()
    assert scene['boxes'] == source['scene']['boxes']
    assert scene['obstacles'] == source['scene']['obstacles']
    assert scene['mast'] == source['scene']['mast']
    assert scene['target_box_id'] == 'carton_r00_l03_c01'
    assert scene['box']['pose'] == [.3334, 0., .53]
    assert scene['box']['supported_by'] == ['carton_r00_l02_c01']
    assert len(world_objects(scene)) == 3
    assert 'convex_mesh' in world_objects(scene)[-1]
    assert source['scene']['schema'] == 'eco65_full_face_static_scene_v1'


@pytest.mark.parametrize('case', ['hardware', 'world_frame', 'target', 'omitted_carton', 'receiver', 'duplicate'])
def test_rehashed_invalid_twelve_scene_is_rejected(case):
    _, scene = synthetic_scene()
    if case == 'hardware': scene['enable_hardware'] = True
    elif case == 'world_frame': scene['T_W_D'][0][3] += .1
    elif case == 'target': scene['native_task']['target_id'] = 'carton_r00_l03_c00'
    elif case == 'omitted_carton': scene['boxes'].pop(0)
    elif case == 'receiver': scene['native_task']['receiving']['box_pose_D'][0][3] = 1.
    elif case == 'duplicate': scene['native_world_objects'].append(deepcopy(scene['native_world_objects'][0]))
    scene['scene_fingerprint'] = digest({k: v for k, v in scene.items() if k != 'scene_fingerprint'})
    with pytest.raises(ValueError): verify_scene(scene)


@pytest.mark.parametrize('field', ['native_world_objects', 'moving_attachments', 'native_task'])
def test_home_proposal_binds_permanent_meshes_and_task(field):
    _, scene = synthetic_scene(); tool = {'synthetic_tool': True}
    home = make_candidate([0.]*6, tool, scene, {'source': 'test fixture'})
    if field == 'native_world_objects': scene[field][0]['pose'][0][3] += .01
    elif field == 'moving_attachments': scene[field][0]['T_parent_local'][0][3] += .01
    else: scene[field]['receiving']['box_pose_D'][0][3] += .01
    with pytest.raises(ValueError, match='TOOL_OR_EQUIPMENT_CHANGED'):
        verify_candidate(home, tool, scene)


def test_actual_mujoco_includes_permanent_j1_geometry_and_rotated_world_mesh(tmp_path, monkeypatch):
    pytest.importorskip('mujoco'); trimesh = pytest.importorskip('trimesh')
    from unloading_sim.eco65.model import URDF
    if not URDF.is_file(): pytest.skip('Pinned official ECO65 URDF required for this numerical test')
    from unloading_sim.eco65 import native_review_world as review
    mesh_path = tmp_path/'attachment.obj'; trimesh.creation.box([.02, .03, .04]).export(mesh_path)
    sha = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
    _, scene = synthetic_scene(mesh_path.name, sha)
    # Off-origin vertices and a yawed authored frame cause MuJoCo to recenter
    # its mesh. Top-support bounds must remain in D, not the compiled frame.
    row = scene['native_world_objects'][0]
    vertices = np.asarray(row['convex_mesh']['vertices'])+[.11, -.07, .04]
    R = Rotation.from_euler('z', .35).as_matrix()
    row['convex_mesh']['vertices'] = vertices.tolist()
    row['pose'] = transform([-.5, -.22, .13], R).tolist()
    scene['scene_fingerprint'] = digest({k: v for k, v in scene.items() if k != 'scene_fingerprint'})
    monkeypatch.setattr(review, 'ROOT', tmp_path)
    tool = dict(T_flange_tcp=np.eye(4).tolist(), T_flange_tool_cad=np.eye(4).tolist(),
        T_tool_cad_tcp=np.eye(4).tolist(), visual_parts=[], collisions=[], adapter=[], mounting_contacts=[])
    world = review.NativeSceneWorld(tool, scene)
    try:
        qs = [np.zeros(6), np.array([.4, 0, 0, 0, 0, 0])]
        positions = []
        for q in qs:
            world.set_state(q, 'initial')
            expected = world.robot.named_link_frames(q)['link_1']@transform([.2, 0, .25])
            positions.append(world.data.geom_xpos[world.geoms['synthetic_mast']['gid']].copy())
            assert np.allclose(positions[-1], expected[:3, 3], atol=1e-8)
            assert len(world.boxes) == 12
        assert np.linalg.norm(positions[0]-positions[1]) > .05
        points = vertices@R.T+np.asarray(row['pose'])[:3, 3]
        expected_center = (points.min(0)+points.max(0))/2
        support_frame = world._pose(row['id'])
        assert np.allclose(support_frame, transform(expected_center), atol=1e-12)
        assert np.allclose(world.object_info[row['id']]['size_m'], np.ptp(points, axis=0))
        # Exercise the actual named support condition with an authored yaw
        # and off-origin vertices, not just the helper that returns a pose.
        target = world.geoms[world.box_id]; support = world.geoms[row['id']]
        world.phase = 'place_contact'
        world.active_pose = transform([*expected_center[:2], points[:, 2].max()+.05])
        assert world._support_pair(target, support)
        world.active_pose[2, 3] -= 2*scene['contact_tolerance_m']
        assert not world._support_pair(target, support)
        assert any({a['name'], b['name']} == {'synthetic_mast', 'carton_r00_l03_c01'} for a, b in world.pairs)
        assert any({a['name'], b['name']} == {row['id'], 'carton_r00_l03_c01'} for a, b in world.pairs)
    finally:
        world.close()


def test_named_j1_interface_still_rejects_real_penetration(tmp_path, monkeypatch):
    pytest.importorskip('mujoco'); trimesh = pytest.importorskip('trimesh')
    from unloading_sim.eco65.model import URDF
    if not URDF.is_file(): pytest.skip('Pinned official ECO65 URDF required for this numerical test')
    from unloading_sim.eco65 import native_review_world as review
    # Deliberately invalid test-only seat enclosing the link origin. A bound
    # interface must not make this real convex penetration disappear.
    path = tmp_path/'invalid_seat.obj'; trimesh.creation.box([.5, .5, .5]).export(path)
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    _, scene = synthetic_scene(path.name, sha)
    row = scene['moving_attachments'][0]
    row.update(id='eco65_mast_mounting_seat', T_parent_local=np.eye(4).tolist())
    source = dict(source='explicit_test_only_mount_definition', seat=dict(mesh=path.name, sha256=sha))
    scene['mast']['mount_interface'] = source
    scene['fixed_attachment_interfaces'] = [dict(parent_link='link_1', attachment_id=row['id'],
        source=deepcopy(source), maximum_penetration_m=.0004)]
    scene['scene_fingerprint'] = digest({k: v for k, v in scene.items() if k != 'scene_fingerprint'})
    monkeypatch.setattr(review, 'ROOT', tmp_path)
    tool = dict(T_flange_tcp=np.eye(4).tolist(), T_flange_tool_cad=np.eye(4).tolist(),
        T_tool_cad_tcp=np.eye(4).tolist(), visual_parts=[], collisions=[], adapter=[], mounting_contacts=[])
    world = review.NativeSceneWorld(tool, scene)
    try:
        names = frozenset(['robot_link_1', row['id']])
        pairs = [(a, b) for a, b in world.pairs if frozenset([a['name'], b['name']]) == names]
        assert len(pairs) == 1 and names in world.mounting_pairs
        # Isolate the named pair to prove its numerical gate, without treating
        # this synthetic arrangement as a complete accepted task scene.
        world.pairs = pairs
        world.use_group_bounds = False  # This test intentionally replaces the frozen pair inventory.
        world.pair_a = np.array([pairs[0][0]['gid']]); world.pair_b = np.array([pairs[0][1]['gid']])
        world.radii = world.model.geom_rbound[world.pair_a]+world.model.geom_rbound[world.pair_b]
        assert not world.valid(np.zeros(6))
        assert world.last_failure['distance_m'] < -.0004
        assert world.last_failure['threshold_m'] == -.0004
        assert world.last_failure['allowance'] == 'source_bound_J1_mount_interface'
    finally:
        world.close()


def test_source_usd_conveyors_retain_disconnected_and_planar_components(tmp_path, monkeypatch):
    pytest.importorskip('pxr'); trimesh = pytest.importorskip('trimesh')
    from pxr import Gf, Usd, UsdGeom, Vt
    import build_eco65_native_12carton as builder
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    items = []; belt_source = '/Source/Belt'
    expected_belt_bounds = {}
    for name in ('transverse', 'longitudinal'):
        path = tmp_path/(name+'.usda'); stage = Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageMetersPerUnit(stage, 1); UsdGeom.SetStageUpAxis(stage, 'Z')
        root = UsdGeom.Xform.Define(stage, '/Conveyor')
        root.AddTranslateOp().Set(Gf.Vec3d(.12, -.06, .02))
        belt = trimesh.creation.box([.6, .25, .02]); belt.apply_translation([0, 0, .15])
        left = trimesh.creation.box([.04, .04, .08]); left.apply_translation([-.25, 0, .04])
        right = left.copy(); right.apply_translation([.5, 0, 0])
        plane = trimesh.Trimesh(vertices=[[-.1, -.1, .05], [.1, -.1, .05], [.1, .1, .05], [-.1, .1, .05]],
            faces=[[0, 1, 2], [0, 2, 3]], process=False)
        structure = trimesh.util.concatenate([left, right, plane]); rows = []
        for ident, mesh, collision, source in [('Belt', belt, True, belt_source),
                ('Structure', structure, True, '/Source/Structure'), ('Decal', plane, False, '/Source/Decal')]:
            raw = np.asarray(mesh.vertices, np.float32)
            dest = UsdGeom.Mesh.Define(stage, '/Conveyor/'+ident)
            dest.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(raw))
            dest.CreateFaceVertexCountsAttr([3]*len(mesh.faces))
            dest.CreateFaceVertexIndicesAttr(mesh.faces.reshape(-1).tolist())
            rows.append(dict(prim='/Conveyor/'+ident, collision=collision, source_prim=source,
                points_sha256=hashlib.sha256(raw.tobytes()).hexdigest()))
        stage.GetRootLayer().Save()
        points = belt.vertices+[.12, -.06, .02]; bounds = np.array([points.min(0), points.max(0)])
        expected_belt_bounds['conveyor_'+name] = bounds
        receiving = bounds.copy(); receiving[0, 0] += .03; receiving[1, 0] -= .03
        items.append(dict(id='conveyor_'+name, usd_path=path.name, usd_sha256=builder.file_hash(path),
            meshes=rows, receiving_surface_bounds_D_m=receiving.tolist()))
    scene = dict(conveyors=dict(source=dict(belt_prim=belt_source), items=items))
    objects, records, surfaces = builder.prepare_conveyors(scene)
    assert len(objects) == 8  # Per conveyor: belt, two separate legs, planar plate.
    assert len([r for r in records if r['planar_envelope_thickness_m'] == 1e-6]) == 2
    assert all('Decal' not in r['source_prim'] for r in records)
    for ident, surface in surfaces.items():
        assert len(surface['support_ids']) == 1
        belt = next(r for r in objects if r['id'] == surface['support_ids'][0])
        assert belt['role'] == 'conveyor_belt'
        assert np.allclose(belt['bounds_D_m'], expected_belt_bounds[ident], atol=1e-7)
        assert surface['bounds_D_m'] == next(i for i in items if i['id'] == ident)['receiving_surface_bounds_D_m']


def test_permanent_camera_replaces_only_declared_parts_and_binds_source_hashes(tmp_path, monkeypatch):
    trimesh = pytest.importorskip('trimesh')
    import build_eco65_native_12carton as builder
    monkeypatch.setattr(builder, 'ROOT', tmp_path)
    def asset(name):
        path = tmp_path/(name+'.obj'); trimesh.creation.box([.02, .03, .04]).export(path)
        return path.name, builder.file_hash(path)
    def part(name):
        path, sha = asset(name)
        return dict(id=name, visual=path, visual_sha256=sha, collision=path, collision_sha256=sha)
    base = part('mast_part_001'); pole = part('mast_part_010'); old_camera = part('old_camera')
    seat, seat_sha = asset('seat'); source_path = tmp_path/'camera.step'; source_path.write_text('test-only CAD placeholder')
    manifest = tmp_path/'camera_manifest.json'; manifest.write_text('{"test_only": true}')
    M = transform([.2, .03, .1]); C = transform([0, .04, .15], Rotation.from_euler('x', .3).as_matrix())
    mast = dict(parts=[base, pole, old_camera], T_link1_mast=M.tolist(),
        mount_interface=dict(source='test-only connected mounting source', seat=dict(mesh=seat, sha256=seat_sha)))
    adapter = dict(name='pitch_plate', size_m=[.04, .03, .01], T_mast=transform([.01, .02, .04]).tolist())
    camera = dict(replaced_mast_parts=['old_camera'], T_mast_CAD=C.tolist(), parts=[part('new_camera')],
        source=source_path.name, source_sha256=builder.file_hash(source_path), cad_manifest=manifest.name,
        cad_manifest_sha256=builder.file_hash(manifest), mount=dict(source='test-only pitch mount'), pitch_adapter=[adapter])
    rows, assets, records, interfaces = builder.prepare_attachments(dict(mast=mast), camera, tmp_path/'prepared')
    by_id = {r['id']: r for r in rows}
    assert set(by_id) == {'eco65_mast_mast_part_001', 'eco65_mast_mast_part_010',
        'eco65_mast_mounting_seat', 'eco65_camera_new_camera', 'eco65_camera_pitch_plate'}
    assert np.allclose(by_id['eco65_camera_new_camera']['T_parent_local'], M@C)
    assert np.allclose(by_id['eco65_camera_pitch_plate']['T_parent_local'], M@np.asarray(adapter['T_mast']))
    assert assets[source_path.name] == camera['source_sha256']
    assert assets[manifest.name] == camera['cad_manifest_sha256']
    assert {r['attachment_id'] for r in interfaces} == {'eco65_mast_mast_part_001', 'eco65_mast_mounting_seat'}
    assert all(r['maximum_penetration_m'] == .0004 for r in interfaces)
    assert all(r['parent_link'] == 'link_1' for r in rows)
    for row in rows:
        assert builder.file_hash(tmp_path/row['mesh']) == row['mesh_sha256']
        assert trimesh.load(tmp_path/row['mesh'], force='mesh', process=False).is_convex
