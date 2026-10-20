"""Bounded real-contour placement of independent simulated cups on top/front."""
import numpy as np
from .model import check_transform
from .model import transform
from .native_contact import validate_geometry, check_contours, FACE_AXES
from ..moveit2_backend import digest


def grasp_candidates(geometry, target, size, flange_from_tcp, base_position,
                     *, target_id, faces=('top', 'front'), margin=.003, angle=.01, neighbor_boxes=()):
    validate_geometry(geometry)
    target = check_transform(target); F = check_transform(flange_from_tcp)
    C = np.linalg.inv(F)@check_transform(geometry['flange_from_contact_frame'])
    rings = [np.asarray(r['points_m'])@C[:3, :3].T+C[:3, 3] for r in geometry['contours']]
    centers = np.array([p.mean(0) for p in rings]); half = np.asarray(size)/2
    base = np.asarray(base_position); base_local = target[:3, :3].T@(base-target[:3, 3])
    groups = [(i,) for i in range(12)]
    for i in range(12):
        for j in np.argsort(np.linalg.norm(centers-centers[i], axis=1))[1:3]:
            pair = tuple(sorted([i, int(j)]))
            if pair not in groups: groups.append(pair)
    candidates = []; rejected = []; seen = set()
    for face in faces:
        axis, sign = FACE_AXES[face]; others = [i for i in range(3) if i != axis]
        # Both bases are proper rotations and aim TCP +Z into the selected face.
        R = np.diag([1., -1., -1.]) if face == 'top' else np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
        for roll in (0., np.pi/2, np.pi, 3*np.pi/2):
            co, si = np.cos(roll), np.sin(roll)
            rotation = R@np.array([[co,-si,0.],[si,co,0.],[0.,0.,1.]])
            for group in groups:
                points = np.concatenate([rings[i] for i in group])@rotation.T
                lo = -half[others]+margin-points[:,others].min(0)
                hi = half[others]-margin-points[:,others].max(0)
                if np.any(lo > hi): continue
                center = (lo+hi)/2; low = np.minimum(lo+.0002,center); high=np.maximum(hi-.0002,center)
                placements=[('group_center',center),('nearest_base',np.clip(base_local[others],low,high)),
                            ('face_center',np.clip(np.zeros(2),low,high))]
                if face=='front':
                    # A corner cup near the exposed upper edge can leave the
                    # other physical cups above the stack. Axis-aligned,
                    # face-centred groups were actually rejected on neighbors.
                    placements += [('exposed_top_edge',np.array([y,high[1]])) for y in (low[0],center[0],high[0])]
                else:
                    # The middle column has neighbors on both sides. Place a
                    # corner cup along the exposed front edge, including both
                    # lateral extremes, so inactive cups may remain outside
                    # the stack. The full physical tool is still checked.
                    placements += [('exposed_front_edge',np.array([low[0],y])) for y in (low[1],center[1],high[1])]
                for label, xy in placements:
                    local = np.eye(4); local[:3,:3]=rotation
                    local[axis,3]=sign*half[axis];local[others,3]=xy
                    tcp=target@local; key=(face,*np.round(tcp[:3].ravel(),9))
                    if key in seen: continue
                    seen.add(key); flange=tcp@np.linalg.inv(F)
                    report=check_contours(geometry,flange,target,size,gap=.0004,penetration=.0004,
                                          margin=margin,angle=angle,face=face)
                    active=report['actual_sealed_ids']
                    if not active:
                        rejected.append(dict(face=face,roll_rad=roll,reason='NO_FULL_CAD_SEAL'));continue
                    report=check_contours(geometry,flange,target,size,gap=.0004,penetration=.0004,
                                          margin=margin,angle=angle,face=face,active_ids=active)
                    if report['status']!='PASS':
                        rejected.append(dict(face=face,roll_rad=roll,reason='TOOL_FACE_EDGES_NOT_ALIGNED'))
                        continue
                    reserves=[r['edge_margin_m'] for r in report['contours'] if r['active']]
                    enabled_centers=np.array([centers[i] for i,r in enumerate(geometry['contours']) if r['id'] in active])
                    spread=float(np.linalg.norm(np.ptp(enabled_centers[:,:2],axis=0)))
                    near_neighbor=0;minimum_neighbor=None
                    for ring in rings:
                        points_world=ring@tcp[:3,:3].T+tcp[:3,3];nearest=float('inf')
                        for neighbor in neighbor_boxes:
                            if neighbor['id']==target_id:continue
                            B=transform(neighbor['pose']);local_points=(points_world-B[:3,3])@B[:3,:3]
                            distances=np.linalg.norm(np.maximum(np.abs(local_points)-np.array(neighbor['size_m'])/2,0),axis=1)
                            nearest=min(nearest,float(distances.min()))
                        if np.isfinite(nearest):
                            minimum_neighbor=nearest if minimum_neighbor is None else min(minimum_neighbor,nearest)
                            near_neighbor+=int(nearest<.0024)
                    binding=dict(target_id=target_id,target_face=face,active_ids=active,tcp_pose_D=tcp.tolist())
                    candidates.append(dict(kind=label,roll_rad=roll,target_face=face,active_ids=active,
                        geometrically_eligible_ids=active,nominal_contact_geometry=report,tcp_pose_D=tcp.tolist(),
                        candidate_id=digest(binding),minimum_edge_margin_m=min(reserves),
                        cup_group_spread_m=spread,
                        neighbor_contours_in_clearance_band=near_neighbor,
                        neighbor_contour_point_distance_hint_m=minimum_neighbor,
                        neighbor_hint_scope='POINT_SAMPLED_RANKING_ONLY_NOT_COLLISION_ACCEPTANCE',
                        flange_distance_from_base_m_heuristic=float(np.linalg.norm(flange[:3,3]-base)),
                        qualification='GEOMETRY_ONLY_NOT_IK_OR_PATH_VALIDATED'))
    for c in candidates:
        c['ranking_cost']=c['flange_distance_from_base_m_heuristic']-.025*len(c['active_ids'])-.1*c['cup_group_spread_m']-.3*c['minimum_edge_margin_m']+.25*c['neighbor_contours_in_clearance_band']
    candidates.sort(key=lambda c:(c['ranking_cost'],c['candidate_id']))
    return dict(candidates=candidates,rejected=rejected,checked_contour_count=12,
                edge_margin_m=margin,method='SINGLE_CUP_AND_LOCAL_PAIR_REAL_CONTOUR_PLACEMENTS',
                independent_control_assumed=True,hardware_independent_control_verified=False)
