"""Offline simulation: unload one full face of 180x130x110 mm cartons onto the
longitudinal belt with the real-site single-carton planner (user 2026-11-27).

User request: "在仿真中把长180mm*宽130mm*高110mm的箱子铺满一整面，箱子长边都沿着
x轴。在横传送带和纵传送带都保持水平的条件下，你要尝试能否将所有的箱子都卸到纵带上。
一是上层优先正吸，中下层优先顶吸，不行再正吸。二是落箱时吸具平行于xy平面，落箱位置在
x方向上不能超过机械臂所在的yz平面，落箱高度你可以根据路径规划的需求来调整，三是在布局
上你仅可调整的是箱堆与机械臂的距离（该距离的最小值是：箱堆与剪叉机构-x方向的最末端相距
20mm），可增加该距离，但不可小于最小值。"

Everything is base_link metres of the field layout revision (layout 20261127).

* Scene: the STEP static layer exactly as planning loads it, with the
  inclined belt turned about its lower hinge until horizontal (simulation
  premise "纵带保持水平"; the mechanism model cannot reach it inside the lift
  range, see report). Transverse belts on the lift top are horizontal anyway.
* Stack: one carton deep, long side along base X, columns spread evenly
  between the two side panels, layers from the floor up to the top cover;
  the front face sits ``gap`` in front of the most -X point of the scissor
  lift (plates, interior envelope and the parts on its top plate).
* Each carton is a fully known cuboid (simulation, no camera): its front
  (+X) and top (+Z) faces carry dense 2 mm support grids.
* Order: highest layer first, centre column first (the continuous-job order).
  Suction preference: upper third of the layers FRONT then TOP, others TOP
  then FRONT (implementer's split of "上层 / 中下层").
* Release: release_pose TOOL_DOWN_SAME_POSITION (tool parallel to XY, cups
  down) at a fixed point on the belt with TCP x <= 0; start height = belt
  top + hanging carton extent + drop gap; the planner lowers it only if no
  IK exists.
* Each round plans from the previous release configuration; a carton that
  cannot be planned stays in place as an obstacle, and a carton with another
  carton still on top of it is not attempted (continuous-job rule).
* 2026-11-28 user review ("吸具横着的面积会大于单个箱子的正面 ... 把吸具竖起来吸一箱";
  "会尝试很多个吸取点 ... 很耗时"; "C5 ... 吸取点太靠右"; "仰泳 ... 不要再规划";
  "正吸的脱离方案只能是走直线"): front suction uses the code-default single
  contact (array standing up, lower two rows on the face, one sideways
  placement toward the robot), top suction one generic-search contact,
  reach-over states are rejected on every segment and front-suction detach
  is the straight OUTWARD line only. ``--wrist-cluster UP|DOWN`` keeps one
  wrist cluster; ``--screen-only`` maps endpoint feasibility of every carton
  per wrist cluster without path planning.

Offline only: no device, no permit, no motion or valve command.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from unloading_sim.workbench.single_carton.config import load  # noqa: E402

BOX = np.array([0.180, 0.130, 0.110])
FLOOR_Z = -0.789          # feet bottoms / CAD carton bottoms of the first revision (CAD y -0.111)
PANEL_Y = (-0.497, 0.210) # inner faces of layout_part_072 / layout_part_341
COVER_Z = 0.800           # underside of the top cover layout_part_125
REQUEST = ('在仿真中把长180mm*宽130mm*高110mm的箱子铺满一整面，箱子长边都沿着x轴。在横传送带和纵传送带都保持水平的条件下，'
           '你要尝试能否将所有的箱子都卸到纵带上。一是上层优先正吸，中下层优先顶吸，不行再正吸。二是落箱时吸具平行于xy平面，'
           '落箱位置在x方向上不能超过机械臂所在的yz平面，落箱高度你可以根据路径规划的需求来调整，三是在布局上你仅可调整的是箱堆与'
           '机械臂的距离（该距离的最小值是：箱堆与剪叉机构-x方向的最末端相距20mm），可增加该距离，但不可小于最小值。')


def extent(o):
    P = np.asarray(o['pose'], float); h = np.abs(P[:3, :3])@np.asarray(o['size'], float)/2
    return P[:3, 3]-h, P[:3, 3]+h


def rot_axis(axis, angle):
    k = np.asarray(axis, float)/np.linalg.norm(axis); K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*K@K


def flatten_belt(world, config, pad_m=0.002):
    """Rebuild every BELT_INCLINED box with the belt horizontal.

    The mechanism boxes are CAD-axis AABBs of the tilted belt; turning them
    would tilt those large boxes. Instead each part's visual mesh (CAD rest
    pose) is turned about the lower hinge (CAD X axis) until the hinge-to-
    hinge direction is horizontal, lifted with the top plate by h, and its
    CAD-axis AABB (exact for the now axis-aligned belt) plus ``pad_m`` is the
    new box. Returns the world and the evidence."""
    import trimesh
    from unloading_sim.eco65.site_mechanism import LiftMechanism, load_manifest, load_roles, cad_robot_base
    c = config['site_layout']
    manifest = load_manifest(ROOT/c['manifest']); roles = load_roles(ROOT/c['roles']); mech = LiftMechanism(manifest, roles)
    parts = {p['id']: p for p in manifest['parts']}
    T = cad_robot_base(roles); h = float(c['lift']['h_m']); state = mech.state(h)
    d = mech.angle0_rad; co, si = np.cos(d), np.sin(d)
    R = np.eye(4); R[1:3, 1:3] = [[co, -si], [si, co]]
    def tr(v): M = np.eye(4); M[:3, 3] = v; return M
    F = tr(mech.lower+np.array([0., h, 0.]))@R@tr(-mech.lower)
    upper = F@np.r_[mech.upper, 1.]; lower = F@np.r_[mech.lower, 1.]
    if abs(upper[1]-lower[1]) > 1e-9: raise RuntimeError('BELT_NOT_HORIZONTAL_AFTER_TURN')
    out = []; turned = 0; belt_boxes = []
    for o in world:
        if o.get('role') != 'BELT_INCLINED': out.append(o); continue
        pid = o.get('part_id') if o.get('part_id') in parts else (o.get('part_ids') or [None])[0]
        if pid not in parts: raise RuntimeError('BELT_PART_NOT_IN_MANIFEST:'+str(o['id']))
        v = np.asarray(trimesh.load(ROOT/parts[pid]['visual'], force='mesh', process=False).vertices, float)
        v = (F@np.c_[v, np.ones(len(v))].T).T[:, :3]
        lo, hi = v.min(0)-pad_m, v.max(0)+pad_m
        P = T.copy(); P[:3, 3] = T[:3, :3]@((lo+hi)/2)+T[:3, 3]
        o = dict(o, pose=P.tolist(), size=(hi-lo).tolist(), sim_premise='BELT_REBUILT_HORIZONTAL_ABOUT_LOWER_HINGE_FROM_VISUAL_MESH_AABB_PAD_2MM')
        out.append(o); turned += 1
        if 'BEZ01' in o.get('name', '') and ('_kj' in o.get('name', '') or '_pdd' in o.get('name', '')): belt_boxes.append(extent(o))
    lo = np.min([b[0] for b in belt_boxes], 0); hi = np.max([b[1] for b in belt_boxes], 0)
    hinge = T@lower
    return out, dict(premise='USER_CONDITION_2026_11_27 横传送带和纵传送带都保持水平: the inclined belt (role BELT_INCLINED) '
                     'rebuilt horizontal about its lower hinge (visual-mesh AABBs + 2 mm); BELT_SLIDER parts stay on their rails; '
                     'the mechanism model itself cannot make the belt horizontal inside the 160-300 mm lift range',
                     lift_h_m=h, mechanism_belt_angle_deg=state['belt_angle_deg'], cad_rest_angle_deg=float(np.degrees(d)),
                     hinge_base_m=hinge[:3].tolist(), turned_boxes=turned, belt_body_bounds_m=[lo.tolist(), hi.tolist()],
                     belt_top_z_m=float(hi[2]), belt_y_m=[float(lo[1]), float(hi[1])], belt_x_m=[float(lo[0]), float(hi[0])])


def raise_belts(world, belt, top_z):
    """Both belt tops at ``top_z`` (user 2026-11-28 "把横带和纵带都升到和机械臂底座相同的高度处").

    Transverse belts: every LIFT_TOP box moves up by the same amount (the top
    plate with everything on it) and the scissor interior envelope grows up
    to the raised plate. Longitudinal belt: the horizontal belt boxes move up
    until their top is at ``top_z``. BELT_SLIDER and FIXED parts stay."""
    trans = [o for o in world if o.get('role') == 'LIFT_TOP' and 'BAS01' in o.get('name', '') and ('_kj' in o['name'] or '_pdd' in o['name'])]
    if not trans: raise RuntimeError('TRANSVERSE_BELT_NOT_FOUND')
    t_top = max(extent(o)[1][2] for o in trans); dt = top_z-t_top; dl = top_z-belt['belt_top_z_m']
    out = []
    for o in world:
        role = o.get('role'); P = np.asarray(o['pose'], float).copy()
        if role == 'LIFT_TOP':
            P[2, 3] += dt; o = dict(o, pose=P.tolist(), sim_premise='LIFT_TOP_RAISED_TO_PUT_THE_TRANSVERSE_BELT_TOP_AT_ROBOT_BASE_HEIGHT')
        elif role == 'BELT_INCLINED':
            P[2, 3] += dl; o = dict(o, pose=P.tolist(), sim_premise='HORIZONTAL_LONGITUDINAL_BELT_TOP_AT_ROBOT_BASE_HEIGHT')
        elif role == 'LIFT_INTERIOR':
            size = np.asarray(o['size'], float).copy(); axis = int(np.argmax(np.abs(P[2, :3])))
            size[axis] += dt; P[2, 3] += dt/2
            o = dict(o, pose=P.tolist(), size=size.tolist(), sim_premise='SCISSOR_INTERIOR_ENVELOPE_UP_TO_THE_RAISED_TOP_PLATE')
        out.append(o)
    belt = dict(belt, belt_top_z_m=float(top_z), belt_body_bounds_m=[[*belt['belt_body_bounds_m'][0][:2], belt['belt_body_bounds_m'][0][2]+dl],
                                                                  [*belt['belt_body_bounds_m'][1][:2], belt['belt_body_bounds_m'][1][2]+dl]],
                raised=dict(top_z_m=float(top_z), transverse_top_before_m=float(t_top), transverse_raise_m=float(dt),
                            longitudinal_top_before_m=float(belt['belt_top_z_m']), longitudinal_raise_m=float(dl),
                            equivalent_overall_lift_height_mm=float(300.+dt*1000.),
                            premise='USER_REQUEST_2026_11_28 把横带和纵带都升到和机械臂底座相同的高度处: both belt tops at base_link z '
                                    f'{top_z} (robot base mounting plane); equivalent lift overall height about {300.+dt*1000.:.0f} mm, '
                                    'outside the 160-300 mm lift range of the mechanism model (simulation premise)'))
    return out, belt


def scissor_end_x(world):
    """Most -X point of the scissor lift: plates, interior envelope, top-plate parts."""
    xs = [extent(o)[0][0] for o in world if o.get('role') in ('LIFT_TOP', 'LIFT_INTERIOR') or 'YLYJ' in o.get('name', '')]
    return float(min(xs))


def walls(bounds):
    lo, hi = np.asarray(bounds, float); out = []
    for axis in range(3):
        for sign in (0, 1):
            a = lo.copy(); b = hi.copy(); a[axis] = (lo if sign == 0 else hi)[axis]-.04*(sign == 0); b[axis] = a[axis]+.04
            T = np.eye(4); T[:3, 3] = (a+b)/2
            out.append(dict(id=f'site_boundary_{axis}_{sign}', pose=T.tolist(), size=(b-a).tolist(), category='workspace_boundary'))
    return out


def stack_specs(front_x, removed_top_layers=0):
    columns = 5; full = int((COVER_Z-FLOOR_Z)//BOX[2]); layers = full-int(removed_top_layers)
    if layers < 1: raise SystemExit('NO_LAYER_LEFT')
    gap = (PANEL_Y[1]-PANEL_Y[0]-columns*BOX[1])/(columns+1)
    out = []
    for k in range(layers):
        for j in range(columns):
            c = np.array([front_x-BOX[0]/2, PANEL_Y[0]+gap*(j+1)+BOX[1]*(j+.5), FLOOR_Z+BOX[2]*(k+.5)])
            out.append(dict(id=f'sim_L{k+1:02d}_C{j+1}', layer=k+1, column=j+1, centre=c.tolist()))
    return out, dict(columns=columns, layers=layers, full_layers=full, removed_top_layers=int(removed_top_layers),
                     column_gap_m=gap, top_gap_to_cover_m=COVER_Z-(FLOOR_Z+layers*BOX[2]))


def face(centre, normal, u, v, size):
    n, u, v = (np.asarray(x, float) for x in (normal, u, v))
    a = np.arange(-size[0]/2+.001, size[0]/2, .002); b = np.arange(-size[1]/2+.001, size[1]/2, .002)
    uv = np.stack(np.meshgrid(a, b, indexing='ij'), -1).reshape(-1, 2)
    return dict(center_m=(np.asarray(centre)+n*0).tolist(), normal=n.tolist(), u=u.tolist(), v=v.tolist(), size_m=list(map(float, size)),
                observed=True, face='sim_known_face', support_points_uv_m=uv.tolist(), support_radius_m=.002)


def sim_box(spec, mode, index):
    """Known cuboid whose local frame is (u, v, n) of the gripped face (native
    single-carton binding: local +Z = face normal, the patch covers the face)."""
    c = np.asarray(spec['centre'], float)
    if mode == 'FRONT': u, v, n = np.eye(3)[1], np.eye(3)[2], np.eye(3)[0]
    else: u, v, n = np.eye(3)[0], np.eye(3)[1], np.eye(3)[2]
    R = np.column_stack((u, v, n)); size = np.abs(R.T@BOX)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = c; h = BOX/2
    corners = [(c+np.array([sx, sy, sz])*h).tolist() for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    f = face(c+n*size[2]/2, n, u, v, size[:2])
    geom = dict(pose=T.tolist(), size_m=size.tolist(), corners_m=corners, faces=[f])
    return dict(id=spec['id'], detection_index=index, **deepcopy(geom), measured_box=deepcopy(geom), measured_size_m=size.tolist(),
                collision_envelope={k: geom[k] for k in ('pose', 'size_m', 'corners_m')},
                observed_faces=['sim_known_front' if mode == 'FRONT' else 'sim_known_top'],
                inferred_faces=[], dimension_source='SIMULATION_KNOWN_CUBOID',
                quality=dict(simulation=True, full_extent_bounded=True, depth_measured=True),
                size_policy='SIMULATION_KNOWN', prior_applied=False)


def order(specs):
    return sorted(specs, key=lambda s: (-s['layer'], abs(s['column']-3), s['column']))


def preference(spec, layers):
    upper = spec['layer'] > layers-int(np.ceil(layers/3))
    return ('FRONT', 'TOP') if upper else ('TOP', 'FRONT')


def on_top(spec, remaining):
    """Ids of remaining cartons resting on ``spec`` (same column, next layer)."""
    return [o['id'] for o in remaining.values() if o['column'] == spec['column'] and o['layer'] == spec['layer']+1]


def contact_summary(candidate):
    sc = (candidate.get('contact_search') or {}).get('single_contact') or {}
    lateral = sc.get('lateral_shift') or {}
    return dict(active_ids=candidate['active_ids'], sealing=len(candidate['active_ids']), tcp_m=np.asarray(candidate['tcp_pose'])[:3, 3].tolist(),
                mode=sc.get('mode') or 'GENERIC_FACE_SEARCH', long_side_toward=sc.get('long_side_toward'),
                up_shift_in_face_m=sc.get('up_shift_in_face_m'), lateral_shift_m=lateral.get('chosen_shift_m'))


def plan_posture(pl, plan):
    """Wrist cluster / facing / upper arm at each segment end and the worst facing along it."""
    from unloading_sim.workbench.single_carton import posture
    pol = pl.posture_policy; out = []
    if pol is None: return out
    for seg in plan['segments']:
        Q = np.asarray([x['q'] for x in seg['prepared']['points']], float)
        m = posture.metrics(pol, Q[-1], Q[0])
        worst = max(posture.metrics(pol, x, Q[0]).get('arm_facing_deg', 0.) for x in Q[::max(1, len(Q)//100)])
        out.append(dict(stage=seg['stage'], end_q_deg=np.degrees(Q[-1]).round(2).tolist(), wrist_cluster_end=m.get('wrist_cluster'),
                        upper_from_up_deg_end=round(m['upper_from_up_deg'], 1), arm_facing_deg_end=round(m.get('arm_facing_deg', 0.), 1),
                        max_arm_facing_deg=round(worst, 1), seconds=round(len(Q)*0.005, 2)))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--config', default='outputs/workbench/single_carton.field_step_layout_v2_20261127.json')
    p.add_argument('--gap-m', type=float, default=0.020, help='stack front to scissor -X end (minimum 0.020)')
    p.add_argument('--release-x', type=float, default=-0.04)
    p.add_argument('--release-y', type=float, default=-0.31, help='TCP y: between the posts (y -0.437 / -0.196) and below the beams (y -0.437 / -0.154, underside z -0.108) over the belt')
    p.add_argument('--candidates', type=int, default=6, help='candidates planned per suction mode (in the planner order)')
    p.add_argument('--budget-s', type=float, default=240., help='planning deadline per suction attempt')
    p.add_argument('--ompl-budget-s', type=float, default=20., help='OMPL fallback budget (field file 5 s; a search setting only)')
    p.add_argument('--start-layer', type=int, help='probe only: cartons above this layer count as already unloaded')
    p.add_argument('--remove-top-layers', type=int, default=0, help='layers left out at the top of the full face')
    p.add_argument('--belt-top-z', type=float, help='raise both belt tops to this base z (0 = robot base mounting plane)')
    p.add_argument('--layer', type=int, help='plan only this layer (centre column first)')
    p.add_argument('--drop-gap-m', type=float, default=0.02)
    p.add_argument('--start-q-deg', type=float, nargs=6, default=[0., 8.6, -93.4, 78.3, 95.8, -8.9])
    p.add_argument('--only', help='comma-separated carton ids')
    p.add_argument('--max-cartons', type=int)
    p.add_argument('--wrist-cluster', choices=('ANY', 'UP', 'DOWN'), default='ANY', help='keep only this wrist cluster (J4->J5 up / down)')
    p.add_argument('--front-contact', choices=('code-default', 'horizontal-cup-grid'), default='code-default',
                   help='horizontal-cup-grid = the 2026-11-26 field default (long side along Y, 35 cup-grid shifts) for comparison')
    p.add_argument('--long-side', choices=('+Z', '-Z'), help='override single_contact.long_side_toward of the standing array')
    p.add_argument('--lateral', choices=('grid', 'max'), help="override single_contact.in_face_shift: grid = LATERAL_TOWARD_ROBOT, max = LATERAL_TOWARD_ROBOT_MAX")
    p.add_argument('--detach-clearance-m', type=float, help='detach_outward_clearance_m: straight pull = carton extent + this (field 0.01)')
    p.add_argument('--top-candidates', type=int, default=1, help='top-suction contacts (generic face search, best seal count first)')
    p.add_argument('--screen-only', action='store_true', help='endpoint screening per wrist cluster for every carton, no path planning')
    p.add_argument('--screen-clusters', default='UP,DOWN')
    p.add_argument('--output', required=True)
    a = p.parse_args()
    if a.gap_m < .020-1e-9: raise SystemExit('GAP_BELOW_USER_MINIMUM_20_MM')
    if a.release_x > 0: raise SystemExit('RELEASE_X_BEYOND_ROBOT_YZ_PLANE')
    out = Path(a.output).resolve(); out.mkdir(parents=True, exist_ok=False)
    base = load(Path(a.config))
    bounds = [[-0.95, -0.62, FLOOR_Z-0.001], [0.95, 0.62, 1.3]]
    from unloading_sim.workbench.single_carton.site_layout import load_layout
    probe = dict(base, site=dict(base['site'], workspace_bounds_m=bounds))
    layout_world = deepcopy(load_layout(probe, ROOT)['world'])
    world0, belt = flatten_belt(layout_world, base)
    if a.belt_top_z is not None: world0, belt = raise_belts(world0, belt, a.belt_top_z)
    end_x = scissor_end_x(world0); front_x = end_x-a.gap_m
    specs, grid = stack_specs(front_x, a.remove_top_layers)
    # Fixed structure the stack itself would overlap at this distance (not placeable).
    blocked = {}
    for s in specs:
        lo = np.asarray(s['centre'])-BOX/2; hi = np.asarray(s['centre'])+BOX/2
        hits = [o['id'] for o in world0 if np.all(extent(o)[1] > lo+1e-6) and np.all(extent(o)[0] < hi-1e-6)]
        if hits: blocked[s['id']] = hits
    release_y = float(a.release_y)
    if not belt['belt_y_m'][0] < release_y < belt['belt_y_m'][1]: raise SystemExit('RELEASE_Y_OFF_THE_BELT')
    cfg = json.loads(Path(a.config).read_text(encoding='utf-8'))
    site = json.loads((ROOT/cfg['site_scene']).read_text(encoding='utf-8'))
    site.update(workspace_bounds_m=bounds, workspace_bounds_source=cfg['site_scene'],
                workspace_bounds_basis='SIMULATION 2026-11-27: enclosure floor (carton bottoms) to above the cover; side panels, '
                                       'the stack and the horizontal belt inside; every other site value as the field file')
    site_path = out/'sim-site.json'; site_path.write_text(json.dumps(site, ensure_ascii=False, indent=1), encoding='utf-8')
    cfg['site_scene'] = str(site_path.relative_to(ROOT))
    cfg.pop('debug_near_target_obstacles', None)
    if a.detach_clearance_m is not None:
        cfg['detach_outward_clearance_m'] = a.detach_clearance_m
        cfg['detach_outward_clearance_basis'] = (f'SIMULATION 2026-11-28: straight front detach = carton extent + {a.detach_clearance_m} m '
                                                 '(field 0.01; "正吸的脱离方案只能是走直线", "脱垛后太着急翻腕将吸具朝下")')
    if a.wrist_cluster != 'ANY':
        cfg['posture_preference'] = dict(cfg['posture_preference'], wrist_cluster=a.wrist_cluster)
    if a.front_contact == 'horizontal-cup-grid':
        from unloading_sim.workbench.single_carton import optimization_defaults as od
        cfg['contact_search'] = dict(cfg['contact_search'], single_contact=dict(
            mode='FACE_CENTER_LONG_SIDE_ALONG_BASE_Y', long_side_toward='+Y', max_normal_from_x_deg=45.0,
            basis='SIMULATION COMPARISON: the 2026-11-26 code default', vertical_placement='LOWER_TWO_ROWS_ON_FACE',
            vertical_placement_basis='SIMULATION COMPARISON', in_face_shift='CUP_GRID', in_face_shift_basis='SIMULATION COMPARISON'))
    elif a.long_side or a.lateral:
        from unloading_sim.workbench.single_carton import optimization_defaults as od
        single = dict(od.SINGLE_CONTACT)
        if a.long_side: single['long_side_toward'] = a.long_side
        if a.lateral: single['in_face_shift'] = 'LATERAL_TOWARD_ROBOT' if a.lateral == 'grid' else 'LATERAL_TOWARD_ROBOT_MAX'
        cfg['contact_search'] = dict(cfg['contact_search'], single_contact=single)
    cfg['native_ompl_budget_s'] = a.ompl_budget_s
    cfg['native_ompl_budget_basis'] = (f'SIMULATION 2026-11-27: offline search budget {a.ompl_budget_s} s for the OMPL fallback '
                                       '(field file 5 s); a search setting, no check, clearance or limit changed')
    reports = []
    from unloading_sim.workbench.single_carton.planner import SinglePlanner
    from unloading_sim.workbench.controlled.trajectory import Authority
    from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    def release_for(mode):
        hang = BOX[2] if mode == 'TOP' else BOX[0]
        return dict(mode='TOOL_DOWN_SAME_POSITION', basis='USER_REQUEST_2026_11_27 落箱时吸具平行于xy平面，落箱高度可按路径规划需要调整',
                    position_xy_m=[a.release_x, release_y], start_height_m=belt['belt_top_z_m']+hang+a.drop_gap_m,
                    position_basis=f'SIMULATION: longitudinal belt centre line y {release_y:.3f}, TCP x {a.release_x} <= 0 '
                                   f'(robot YZ plane), belt top + hanging extent {hang} m + drop gap {a.drop_gap_m} m')
    def build_planner():
        c = deepcopy(cfg); c['release_pose'] = release_for('TOP')
        path = out/'sim-config.json'; path.write_text(json.dumps(c, ensure_ascii=False, indent=1), encoding='utf-8')
        conf = load(path); front_search = deepcopy(conf.get('contact_search'))
        inst = Installation.load(conf['execution']['installation'])
        model = CoordinateModel(URDF, URDF_SHA, inst.world_from_base, inst.tool)
        pl = SinglePlanner(conf, load_mapping(conf['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), model)
        pl.build(); return pl, front_search
    def set_mode(pl, front_search, mode):
        # FRONT: the field single face-centre contact; TOP: the generic face
        # search on the top face (single_contact only spans base Y/Z faces).
        pl.site['contact_search'] = deepcopy(front_search) if mode == 'FRONT' else dict(
            reference_face_roll_deg=0, j6_rotation_deg=180, lift_offsets_m=[0.0], max_candidates=a.top_candidates,
            precontact_m=front_search.get('precontact_m', 0.06),
            basis='SIMULATION 2026-11-28 ("会尝试很多个吸取点 ... 很耗时"): top face, array long side along the carton length, '
                  'best anticipated seal count first')
        pl.release_policy = release_for(mode); pl.release_for = None; pl.q_release = None
    pl, front_search = build_planner()
    remaining = {s['id']: s for s in specs if s['id'] not in blocked and (a.start_layer is None or s['layer'] <= a.start_layer)}
    q = np.deg2rad(a.start_q_deg); summary = []
    todo = order([s for s in specs if s['id'] in remaining])
    if a.only: todo = [s for s in todo if s['id'] in a.only.split(',')]
    if a.layer: todo = [s for s in todo if s['layer'] == a.layer]
    if a.max_cartons: todo = todo[:a.max_cartons]
    started = time.monotonic()
    if a.screen_only:
        try: summary = screen_all(pl, a, todo, specs, remaining, world0, bounds, q, out, set_mode, front_search, grid)
        finally:
            try: pl.close()
            except Exception: pass
        report = dict(scope='OFFLINE_SIMULATION_ENDPOINT_SCREEN_NOT_EXECUTABLE', hardware_execution='NOT_RUN', user_request=REQUEST,
                      config=a.config, gap_m=a.gap_m, scissor_end_x_m=end_x, stack_front_x_m=front_x, grid=grid, box_m=BOX.tolist(),
                      belt=belt, release=dict(x=a.release_x, y=release_y, drop_gap_m=a.drop_gap_m), clusters=a.screen_clusters.split(','),
                      results=summary, seconds=round(time.monotonic()-started, 1), blocked_by_fixed_structure=blocked)
        (out/'screen-report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str), encoding='utf-8')
        return
    try:
        for n, spec in enumerate(todo):
            above = on_top(spec, remaining)
            if above:
                summary.append(dict(id=spec['id'], layer=spec['layer'], column=spec['column'], centre=spec['centre'], attempts=[],
                                    q_start=q.tolist(), result='NOT_ATTEMPTED_CARTON_ON_TOP', on_top=above))
                print(f'[{n+1}/{len(todo)}] {spec["id"]} skipped: {above} on top', flush=True)
                (out/'progress.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding='utf-8')
                continue
            others = [dict(id=o['id'], size=BOX.tolist(), pose=sim_box(o, 'TOP', 0)['pose'], category='carton')
                      for o in remaining.values() if o['id'] != spec['id']]
            world = deepcopy(world0)+walls(bounds)+others
            row = dict(id=spec['id'], layer=spec['layer'], column=spec['column'], centre=spec['centre'],
                       preference=list(preference(spec, grid['layers'])), attempts=[], q_start=q.tolist())
            for mode in row['preference']:
                set_mode(pl, front_search, mode)
                box = sim_box(spec, mode, n)
                world_t = world+[dict(id=box['id'], size=box['size_m'], pose=box['pose'], category='carton')]
                folder = out/f'{n:02d}-{spec["id"]}-{mode.lower()}'; folder.mkdir()
                failures = []; t0 = time.monotonic()
                progress = lambda text, t0=t0: print(f'  [{time.monotonic()-t0:6.1f}s] {text}', flush=True)
                print(f'[{n+1}/{len(todo)}] {spec["id"]} {mode}', flush=True)
                found = None; error = None
                try:
                    pl.deadline = time.monotonic()+a.budget_s; pl.deadline_reference = time.monotonic()
                    pl.start_task_services(world_t, folder); pl.start_main_worker(folder); pl.acquire_workers(folder, [box])
                    found = pl.bounded_search(q, world_t, box, folder, progress, lambda: False, failures, set(range(a.candidates)))
                    if pl.evidence is not None: pl.evidence.flush(timeout=60.)
                except Exception as e:  # recorded, the job continues with the next mode/carton
                    import traceback
                    error = f'{type(e).__name__}: {e}'[:800]; (folder/'error.txt').write_text(traceback.format_exc(), encoding='utf-8')
                finally:
                    try: pl.release_workers(folder)
                    except Exception as e: error = (error or '')+f' | release_workers {type(e).__name__}: {e}'[:300]
                (folder/'planning-failures.json').write_text(json.dumps(failures, ensure_ascii=False, indent=1, default=str), encoding='utf-8')
                (folder/'sim-attempt.json').write_text(json.dumps(dict(target=box, q_start=q.tolist(), mode=mode, world=world_t,
                    release_policy=pl.release_policy, error=error, found=bool(found)), ensure_ascii=False, default=str), encoding='utf-8')
                attempt = dict(mode=mode, seconds=round(time.monotonic()-t0, 2), success=bool(found), error=error,
                               failures=[dict(candidate=f.get('candidate'), reason=str(f.get('reason'))[:600]) for f in failures][:12],
                               counts=pl.search_counts.get(box['id']))
                try:
                    cands = json.loads((folder/(box['id']+'-candidates.json')).read_text(encoding='utf-8'))
                    attempt['contacts'] = [dict(index=c['index'], **contact_summary(c)) for c in cands]
                except (OSError, ValueError, KeyError): pass
                if found:
                    try: attempt['posture'] = plan_posture(pl, found)
                    except Exception as e: attempt['posture_error'] = f'{type(e).__name__}: {e}'
                    attempt['motion_s'] = round(sum(len(seg['prepared']['points']) for seg in found['segments'])*0.005, 2)
                    attempt.update(candidate=found.get('index'), q_release=list(map(float, pl.q_release)),
                                   segments={k: v for k, v in found.items() if k in ('segments', 'segment_names')} or None,
                                   plan_keys=sorted(found.keys()))
                    plan_path = folder/'plan.json'
                    plan_path.write_text(json.dumps(found, default=lambda x: x.tolist() if hasattr(x, 'tolist') else str(x)), encoding='utf-8')
                    attempt['plan_file'] = str(plan_path.relative_to(out))
                row['attempts'].append(attempt)
                if found:
                    q = np.asarray(pl.q_release, float); remaining.pop(spec['id'])
                    row.update(result='UNLOADED', mode=mode); break
            else:
                row.update(result='NOT_UNLOADED')
            summary.append(row)
            print(f'  -> {row["result"]} {row.get("mode","")}', flush=True)
            (out/'progress.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding='utf-8')
    finally:
        try: pl.close()
        except Exception: pass
    report = dict(scope='OFFLINE_SIMULATION_NOT_EXECUTABLE', remove_top_layers=a.remove_top_layers, belt_top_z=a.belt_top_z, candidates_per_mode=a.candidates, budget_s_per_attempt=a.budget_s, ompl_budget_s=a.ompl_budget_s, start_layer=a.start_layer, hardware_execution='NOT_RUN', user_request=REQUEST,
                  config=a.config, gap_m=a.gap_m, scissor_end_x_m=end_x, stack_front_x_m=front_x, grid=grid, box_m=BOX.tolist(),
                  belt=belt, release=dict(x=a.release_x, y=release_y, drop_gap_m=a.drop_gap_m), workspace_bounds_m=bounds,
                  stack=specs, blocked_by_fixed_structure=blocked, order=[s['id'] for s in todo], results=summary,
                  unloaded=sum(r['result'] == 'UNLOADED' for r in summary), attempted=len(summary), total=len(specs),
                  seconds=round(time.monotonic()-started, 1), world_static=[dict(o) for o in world0])
    (out/'simulation-report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str), encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('gap_m', 'scissor_end_x_m', 'stack_front_x_m', 'grid', 'unloaded', 'attempted', 'total')}, ensure_ascii=False))


def screen_all(pl, a, todo, specs, remaining, world0, bounds, q, out, set_mode, front_search, grid):
    """Endpoint feasibility per carton, suction mode and wrist cluster (no path planning).

    For a carton of layer k every carton of layers <= k except the target is
    an obstacle (the layers above are taken as unloaded; the same layer is
    conservatively still full). The planner's own screen runs: release
    derivation, loaded release endpoint, approach IK branches through the
    upper-arm / facing / wrist-cluster gates, approach / contact / detach
    endpoint checks and the approach joint line."""
    from unloading_sim.workbench.single_carton.geometry import contact_candidates
    rows = []
    clusters = [c for c in a.screen_clusters.split(',') if c]
    for n, spec in enumerate(todo):
        others = [dict(id=o['id'], size=BOX.tolist(), pose=sim_box(o, 'TOP', 0)['pose'], category='carton')
                  for o in remaining.values() if o['id'] != spec['id'] and o['layer'] <= spec['layer']]
        world = deepcopy(world0)+walls(bounds)+others
        row = dict(id=spec['id'], layer=spec['layer'], column=spec['column'], modes={})
        for mode in ('FRONT', 'TOP'):
            set_mode(pl, front_search, mode)
            box = sim_box(spec, mode, n)
            world_t = world+[dict(id=box['id'], size=box['size_m'], pose=box['pose'], category='carton')]
            res = row['modes'][mode] = {}
            folder = out/f'{n:02d}-{spec["id"]}-{mode.lower()}'; folder.mkdir()
            try:
                cand = contact_candidates(box, pl.source['tool'], pl.site['topology'], pl.site.get('contact_search'))[0]
            except ValueError as e:
                res['error'] = str(e)[:300]; continue
            res['contact'] = contact_summary(cand)
            try:
                pl.deadline = time.monotonic()+a.budget_s; pl.deadline_reference = time.monotonic()
                pl.start_task_services(world_t, folder); pl.start_main_worker(folder); pl.acquire_workers(folder, [box])
                pl.q_release = None; pl.release_for = None
                pl.derive_tool_down_release(q, folder, None, np.asarray(cand['tcp_pose'], float))
                for cluster in clusters:
                    pl.posture_policy['wrist_cluster'] = cluster
                    sub = folder/cluster.lower(); sub.mkdir(); t0 = time.monotonic()
                    try:
                        screened = pl.screen_candidate(q, world_t, box, cand, sub, None, [])
                        sc = screened['screen']
                        branches = None
                        for r in sc.get('record', []):
                            if r.get('stage') == 'approach_wrist_cluster_gate': branches = r.get('kept')
                        res[cluster] = dict(conclusion=sc['conclusion'], reason=sc.get('reason'), ik_branches_total=sc.get('ik_branches'),
                                            cluster_branches=branches, approach_q_deg=None if sc.get('approach_q') is None else np.degrees(sc['approach_q']).round(1).tolist(),
                                            approach_line=sc.get('approach_path_screen'), seconds=round(time.monotonic()-t0, 2),
                                            first_failures=[dict(stage=r.get('stage'), pair=(r.get('failure') or {}).get('pair'))
                                                            for r in sc.get('record', []) if r.get('conclusion') not in ('PASS', 'GATE_APPLIED')][:4])
                    except Exception as e:
                        res[cluster] = dict(conclusion='ERROR', reason=f'{type(e).__name__}: {e}'[:400], seconds=round(time.monotonic()-t0, 2))
            except Exception as e:
                res['error'] = f'{type(e).__name__}: {e}'[:400]
            finally:
                pl.posture_policy['wrist_cluster'] = a.wrist_cluster
                try: pl.release_workers(folder)
                except Exception: pass
            print(f'[{n+1}/{len(todo)}] {spec["id"]} {mode}: ' + ', '.join(f'{c}={res.get(c, {}).get("conclusion")}' for c in clusters), flush=True)
        rows.append(row)
        (out/'progress.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1, default=str), encoding='utf-8')
    return rows


if __name__ == '__main__':
    main()
