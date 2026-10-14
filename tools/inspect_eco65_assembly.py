"""Private, reproducible CAD/installation/static checks; never a motion planner."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from unloading_sim.eco65.model import ROOT, URDF, load_json, save_json, transform, distance_accepted
from unloading_sim.eco65.native_contact import extract_cad_contours, check_contours
from unloading_sim.eco65.native_model import asset_inventory, build_native_model
from unloading_sim.eco65.native_scene import combined_scene
from unloading_sim.eco65.unloading_layout import UnloadingWorld


def mesh_plot(ax, mesh, pose, color, alpha=1.):
    v = np.asarray(mesh.vertices)@pose[:3, :3].T+pose[:3, 3]
    triangles = v[np.asarray(mesh.faces)]
    # All original mesh triangles; no generated image or geometric substitution.
    normals=mesh.face_normals@pose[:3,:3].T
    light=np.array([.3,-.6,.7]);light/=np.linalg.norm(light)
    colors=np.array(to_rgb(color))[None,:]*(.35+.65*np.maximum(0,normals@light))[:,None]
    ax.add_collection3d(Poly3DCollection(triangles, facecolor=colors, edgecolor='none', alpha=alpha, rasterized=True))
    return v


def limits(ax, vertices, padding=.12):
    pts = np.concatenate(vertices); lo, hi = pts.min(0), pts.max(0)
    mid = (lo+hi)/2; radius = max(hi-lo)*(1+padding)/2
    ax.set(xlim=(mid[0]-radius, mid[0]+radius), ylim=(mid[1]-radius, mid[1]+radius), zlim=(mid[2]-radius, mid[2]+radius))
    ax.set_box_aspect([1, 1, 1]); ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')


def axes(ax, pose, length=.025, name=''):
    for i, color in enumerate(('red', 'green', 'blue')):
        a = pose[:3, 3]; b = a+length*pose[:3, i]
        ax.plot(*np.stack([a, b]).T, color=color, linewidth=2,zorder=100)
    ax.text(*pose[:3, 3], name, fontsize=9,zorder=101)


def manual_fk(q, scene):
    """Independent direct XML chain multiplication, distinct from URDFRobot."""
    frames = {'base_link': transform(scene['base_position_m'])}
    for j, angle in zip(ET.parse(URDF).getroot().findall('joint'), q):
        origin = j.find('origin'); xyz = np.fromstring(origin.get('xyz'), sep=' ')
        rpy = np.fromstring(origin.get('rpy'), sep=' ')
        axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
        frames[j.find('child').get('link')] = frames[j.find('parent').get('link')]@transform(xyz, Rotation.from_euler('xyz', rpy).as_matrix())@transform(rotation=Rotation.from_rotvec(axis*angle).as_matrix())
    return frames


def distance_boundaries(cpp_probe=None):
    """Actual MuJoCo signed distances + shared C++ scalar acceptance policy.

    This deliberately does not label the scalar C++ probe as an FCL query.
    """
    import mujoco
    m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="box" size=".05 .05 .05"/><geom type="box" size=".05 .05 .05" pos=".2 0 0"/></worldbody></mujoco>')
    d=mujoco.MjData(m);rows=[]
    for threshold in (.0024,-.0004):
        for delta in (0.,-.5e-9,-2e-9,2e-9):
            m.geom_pos[1,0]=.1+threshold+delta;mujoco.mj_forward(m,d)
            distance=mujoco.mj_geomDistance(m,d,0,1,.1,None)
            row=dict(threshold_m=threshold,delta_m=delta,actual_mujoco_signed_distance_m=distance,
                     python_accepted=distance_accepted(distance,threshold))
            if row['python_accepted'] != (delta>=-1e-9):raise ValueError('MUJOCO_DISTANCE_BOUNDARY_MISMATCH')
            if cpp_probe:
                req=dict(op='distance_policy',distance=distance,threshold=threshold)
                result=subprocess.run([str(Path(cpp_probe).resolve())],input=json.dumps(req)+'\n',text=True,capture_output=True,check=True)
                row['cpp_policy_accepted']=json.loads(result.stdout)['accepted']
                if row['cpp_policy_accepted']!=row['python_accepted']:raise ValueError('CPP_DISTANCE_POLICY_MISMATCH')
            rows.append(row)
    return dict(scope='actual_MuJoCo_distance_and_CPP_scalar_policy_not_FCL_runtime',cases=rows)


def run(output, cpp_probe=None):
    out = Path(output).resolve(); out.relative_to(ROOT/'outputs'); out.mkdir(parents=True, exist_ok=False)
    timings = {}; started = time.perf_counter()
    tool = load_json(ROOT/'configs/local/eco65_desktop/tool.json'); cad = load_json(ROOT/'assets/tools/desktop_suction/derived/manifests/cad.json')
    geometry = extract_cad_contours(tool, ROOT); scene = combined_scene(); inventory = asset_inventory()
    if inventory['status'] != 'PASS': raise ValueError(inventory)
    save_json(out/'contact_geometry.json', geometry); save_json(out/'scene.json', scene)
    timings['asset_and_scene_s'] = time.perf_counter()-started
    started = time.perf_counter(); model = build_native_model(scene, out/'model'); save_json(out/'native_model_prepared.json', model)
    timings['native_model_preparation_s'] = time.perf_counter()-started
    started = time.perf_counter(); world = UnloadingWorld(tool, scene)
    timings['mujoco_initialization_s'] = time.perf_counter()-started
    started = time.perf_counter()
    seed = np.array(scene['source_equipment_config']['task']['initial_seed_q_rad'])
    probes = [np.zeros(6), seed, np.array([.1, -.2, .3, -.4, .5, -.6])]
    fk = []
    for q in probes:
        frames = world.robot.named_link_frames(q); direct = manual_fk(q, scene); cad_pose, _ = world.set_state(q, 'pose')
        error = max(float(np.max(abs(frames[k]-direct[k]))) for k in frames)
        body_errors = []
        for part in tool['visual_parts']:
            body = world.model.body('tool_'+part['id']).id
            actual = transform(world.data.xpos[body], world.data.xmat[body].reshape(3, 3))
            body_errors.append(float(np.max(abs(actual-cad_pose))))
        tcp = frames['link_6']@np.array(tool['T_flange_tool_cad'])@np.array(tool['T_tool_cad_tcp'])
        tcp_error = float(np.max(abs(tcp-world.robot.fk(q))))
        fk.append(dict(q=q.tolist(), flange=frames['link_6'].tolist(), tcp=tcp.tolist(),
            direct_xml_fk_max_error=error, tcp_chain_max_error=tcp_error, actual_mujoco_tool_body_max_error=max(body_errors)))
        if max(error, tcp_error, max(body_errors)) > 1e-8: raise ValueError('STATIC_TRANSFORM_INCONSISTENT')
    # This bounded diagnostic checks static candidates only; no IK or path search.
    rng = np.random.default_rng(6518); attempts = []
    candidates = [seed, np.zeros(6)] + [rng.uniform(world.robot.joint_limits[:, 0], world.robot.joint_limits[:, 1]) for _ in range(256)]
    home = None
    for q in candidates:
        valid = world.valid(q, 'pose'); attempts.append(dict(q=q.tolist(), valid=valid, failure=world.last_failure))
        if valid: home = q; break
    timings['static_fk_home_s'] = time.perf_counter()-started
    save_json(out/'home_search.json', dict(scope='local_static_candidate_checks_only_no_motion', attempts=attempts))
    shown = seed if home is None else home
    world.set_state(shown, 'pose')
    # Compare compiled MuJoCo supports to the explicit convex inputs exported
    # for FCL. Full worker/FCL runtime must still be checked on the ROS host.
    representation = [];started=time.perf_counter()
    frames = world.robot.named_link_frames(shown)
    for binding in model['convex_bindings']:
        ident = binding['id']; entry = world.geoms[ident]; gid = entry['gid']; mid = world.model.geom_dataid[gid]
        adr, count = world.model.mesh_vertadr[mid], world.model.mesh_vertnum[mid]
        mv = world.model.mesh_vert[adr:adr+count]@world.data.geom_xmat[gid].reshape(3, 3).T+world.data.geom_xpos[gid]
        mesh = trimesh.load(ROOT/binding['convex_path'], force='mesh', process=True)
        T = frames[ident.removeprefix('robot_')] if ident.startswith('robot_') else frames['link_6']@np.array(tool['T_flange_tool_cad'])
        vv = mesh.vertices@T[:3, :3].T+T[:3, 3]
        # Every facet-normal support must match in both directions; test the
        # MuJoCo hull normals too to avoid mere AABB/file-name equivalence.
        normals = np.vstack([mesh.face_normals@T[:3, :3].T, trimesh.convex.convex_hull(mv).face_normals])
        error = max(float(np.max(abs(np.max(mv@n.T, axis=0)-np.max(vv@n.T, axis=0)))) for n in np.array_split(normals, max(1, len(normals)//100)))
        representation.append(dict(id=ident, support_error_m=error, source_sha256=binding['source_sha256'], convex_sha256=binding['convex_sha256']))
    save_json(out/'convex_representation.json', dict(scope='compiled_MuJoCo_vs_prepared_FCL_convex_inputs', fcl_runtime='NOT_RUN', pairs=representation))
    timings['compiled_representation_comparison_s']=time.perf_counter()-started
    if max(r['support_error_m'] for r in representation)>1e-7:raise ValueError('COMPILED_CONVEX_REPRESENTATION_MISMATCH')
    save_json(out/'distance_boundaries.json',distance_boundaries(cpp_probe))
    # All twelve real CAD contours, including gap/penetration/edge/normal cases.
    started = time.perf_counter(); target = transform(scene['box']['pose']); size = scene['box']['size_m']
    top = target@transform([0, 0, size[2]/2], np.diag([1., -1., -1.]))
    base = top@np.linalg.inv(tool['T_flange_tcp']); cases = []; requests = []
    poses=[]
    for dz in (0., .0004, .00040001, -.0004, -.00040001):
        F = base.copy(); F[2, 3] += dz
        poses.append(('gap',dz,F))
    local=np.linalg.inv(target)@base@np.array(tool['T_flange_tool_cad'])
    points=np.concatenate([r['points_m'] for r in geometry['contours']])@local[:3,:3].T+local[:3,3]
    edge_shift=size[0]/2-.003-points[:,0].max()
    for delta in (-2e-8,0.,2e-8):
        F=base.copy();F[0,3]+=edge_shift+delta;poses.append(('edge',delta,F))
    turned=top.copy();turned[:3,:3]=turned[:3,:3]@Rotation.from_euler('x',.011).as_matrix()
    poses.append(('normal_tilt',.011,turned@np.linalg.inv(tool['T_flange_tcp'])))
    for kind,offset,F in poses:
        answer = check_contours(geometry, F, target, size, gap=.0004, penetration=.0004)
        cases.append(dict(kind=kind, offset=offset, python=answer))
        policy = dict(geometry, suction_edge_margin_m=.003, maximum_penetration_m=.0004,
                      max_attachment_gap_m=.0004, max_normal_misalignment_rad=.01,target_face='top')
        requests.append(dict(policy=policy, target=dict(id=scene['box']['id'], size=size, pose=target.tolist()), flange=F.tolist()))
    if cpp_probe:
        result = subprocess.run([str(Path(cpp_probe).resolve())], input=''.join(json.dumps(r)+'\n' for r in requests), text=True, capture_output=True, check=True)
        answers = [json.loads(line) for line in result.stdout.splitlines()]
        if len(answers) != len(cases): raise ValueError('CPP_PROBE_RESULT_COUNT')
        for case, answer in zip(cases, answers):
            case['cpp'] = answer
        save_json(out/'real_contour_checks.json', cases)
        for case in cases:
            answer=case['cpp']
            if answer['status'] != case['python']['status']: raise ValueError('REAL_CONTOUR_CPP_PARITY')
            for a,b in zip(answer['contours'],case['python']['contours']):
                if a['valid']!=b['valid'] or any(abs(a[k]-b[k])>1e-12 for k in ('min_gap_m','max_gap_m','edge_margin_m')):
                    raise ValueError('REAL_CONTOUR_CPP_NUMERIC_PARITY')
    save_json(out/'real_contour_checks.json', cases); timings['real_contour_checks_s'] = time.perf_counter()-started
    started = time.perf_counter()
    render_figures(tool,cad,geometry,world,shown,out,home)
    timings['four_figures_s'] = time.perf_counter()-started; world.close()
    report = dict(status='LOCAL_GEOMETRY_CHECKED', source_sha256=cad['source_sha256'], solids=len(cad['parts']),
        seals=len(geometry['contours']), asset_inventory=inventory, fk=fk, home_status='PASS_LOCAL_STATIC' if home is not None else 'NOT_FOUND',
        home_q=None if home is None else home.tolist(), convex_max_support_error_m=max(r['support_error_m'] for r in representation),
        native_worker='NOT_RUN', native_plan='NOT_RUN', selected_trajectory_independent_review='NOT_RUN',
        replay='NOT_RUN', dynamics='NOT_EVALUATED', full_warehouse='NOT_EVALUATED', timings=timings,
        image_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in out.glob('*.png')})
    save_json(out/'report.json', report); print(json.dumps({k:report[k] for k in ('status','solids','seals','home_status','convex_max_support_error_m','timings')}, indent=2))
    return report


def render_figures(tool,cad,geometry,world,shown,out,home):
    meshes = {p['id']: trimesh.load(ROOT/p['visual'], force='mesh', process=True) for p in tool['visual_parts']}
    flange_id = tool['mounting_evidence']['flange_part']; F_C = np.array(tool['T_flange_tool_cad'])
    for name, mounting in [('A_actual_assembly', False), ('B_end_mount_TCP', True)]:
        fig = plt.figure(figsize=(10, 8)); ax = fig.add_subplot(projection='3d'); vertices = []
        for p in tool['visual_parts']:
            color = '#d59b38' if p['id'] == flange_id else ('#267c87' if p['name'].startswith('FG42S') else '#9aa6b1')
            vertices.append(mesh_plot(ax, meshes[p['id']], F_C, color))
        if mounting:
            official = URDF.parent.parent/'meshes/link_6.STL'
            vertices.append(mesh_plot(ax, trimesh.load(official, force='mesh'), np.eye(4), '#656b78'))
            axes(ax, np.eye(4), name='F: official link_6'); axes(ax, np.array(tool['T_flange_tcp']), name='TCP')
        else: axes(ax, F_C, name='CAD frame')
        limits(ax, vertices); ax.view_init(elev=-14, azim=-90)
        if mounting:ax.text2D(.02,.94,'Axes: X red / Y green / Z blue\nF at the actual mating face; TCP at lip tangent plane',transform=ax.transAxes)
        ax.set_title(('Actual STEP assembly' if not mounting else 'Measured interface and TCP - simulation roll assumption')+'\n'+f'{len(cad["parts"])} solids; supplied flange in gold; no generated adapter')
        fig.tight_layout(); fig.savefig(out/(name+'.png'), dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(11, 8))
    projected=[]
    for p in tool['visual_parts']:
        tri=meshes[p['id']].triangles
        shade=(.25+.65*np.maximum(0,-meshes[p['id']].face_normals[:,1]))
        projected.extend((float(t[:,1].mean()),t[:,[0,2]],np.array([.65,.72,.75])*c) for t,c in zip(tri,shade))
    projected.sort(key=lambda item:-item[0])
    ax.add_collection(PolyCollection([t for _,t,_ in projected],facecolors=[c for _,_,c in projected],edgecolors='none',rasterized=True))
    for row in geometry['contours']:
        p = np.asarray(row['points_m']); ax.plot(*np.vstack([p[:, [0, 2]], p[0, [0, 2]]]).T, linewidth=1.8)
        ax.text(p[:, 0].mean(), p[:, 2].mean(), row['component'], ha='center', va='center', fontsize=9,bbox=dict(facecolor='white',alpha=.8,edgecolor='none'))
    ax.set(aspect='equal', xlabel='CAD X (m)', ylabel='CAD Z (m)', title='All 12 actual CAD lip tangent contours\nNormal CAD -Y; rounded torus lip, no planar annulus, no fitted-circle fallback')
    ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(out/'C_all_seal_contours.png', dpi=160); plt.close(fig)
    rgb = world.render(shown, 'pose', width=1280, height=720, azimuth=135, elevation=-24, distance=2.8, lookat=(-.08, 0, .04))
    fig, ax = plt.subplots(figsize=(12.8, 7.2)); ax.imshow(rgb); ax.axis('off')
    ax.set_title('Local four-carton equipment scene - '+('static HOME valid' if home is not None else 'diagnostic pose, HOME NOT FOUND')+'\nFull warehouse / native planning / dynamics NOT EVALUATED')
    fig.tight_layout(); fig.savefig(out/'D_four_carton_static.png', dpi=120); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--output', required=True); parser.add_argument('--cpp-probe')
    args = parser.parse_args(); run(args.output, args.cpp_probe)
