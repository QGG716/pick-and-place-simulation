"""Explicit offline contact adapter. No cuboid completion, IK or execution admission."""
from dataclasses import asdict, dataclass
from pathlib import Path
import json

import numpy as np

from unloading_contracts import canonical_fingerprint
from unloading_sim.geometry import make_tool_rotation, make_transform, rotation_matrix_from_rpy
from unloading_sim.grasp import SuctionGraspCandidate, _expand_candidate_wrist_rolls, suction_cup_layout_geometry
from .algorithm_artifact import file_reference
from .geometry import validate_transform_parent_child
from .metric_faces import _binding, _support_mask, MetricFitConfig
from .metric_support import observation_support, backproject_pixels
from .planning_geometry import _surface
from .face_quality import patch_minimum_width


BLOCKERS = dict(volume='UNKNOWN', ik='NOT_EVALUATED', collision='NOT_EVALUATED',
                rigid_tool_clearance='NOT_EVALUATED', suction_force='NOT_EVALUATED',
                material_seal='NOT_EVALUATED', dynamics='NOT_EVALUATED', execution='BLOCKED')
SOURCE_KEYS = ('module_id', 'capture_id', 'source_instance_id', 'face_id')


def current_tool(project):
    """Read engineering configuration, never scene object transforms or GT boxes."""
    import yaml
    root = Path(project)
    config_path = root/'configs/validation/m710id70_v3.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    tool_path = (config_path.parent/config['tool']['config']).resolve()
    tool = yaml.safe_load(tool_path.read_text(encoding='utf-8'))
    asset_path = root/'assets/grippers/shanghai_wantai_three_zone/mass_properties.json'
    asset = json.loads(asset_path.read_text(encoding='utf-8'))
    t, p = config['tool'], config['planning']
    layout = dict(rows=t['cup_rows'], columns=t['cup_columns'], pitch_m=t['cup_pitch_m'],
                  cup_radius_m=t['cup_radius_m'], zone_count=len(asset['zones']),
                  minimum_sealed_cups=t['minimum_sealed_cups'])
    centers, _, _ = suction_cup_layout_geometry(layout)
    if (asset['cup_count'] != len(centers) or asset['cup_radius_from_step_mm']/1000 != layout['cup_radius_m']
            or not np.allclose(np.column_stack((centers[:, 1], -centers[:, 0])),
                               asset['cup_centers_tool_yz_m'], atol=1e-12, rtol=0)):
        raise ValueError('TOOL_LAYOUT_SOURCE_CONFLICT')
    mechanical = make_transform(tool['tcp_transform']['rotation_matrix'], tool['tcp_transform']['translation_xyz_m'])
    axes = make_transform(rotation_matrix_from_rpy(0, np.pi/2, 0))
    result = dict(name=tool['name'], layout=layout, edge_margin_m=t['suction_edge_margin_m'],
        standoff_m=p['pregrasp_standoff_m'], contact_gap_m=0.,
        contact_gap_source='validation_motion.evaluate_task -> fanuc_m710id70.target_pose: contact at face plane',
        grid_fractions=[-.42, 0., .42], grid_source='generate_suction_candidates default grid_fraction=0.42',
        roll_degrees=p['roll_candidates_deg'], flange_from_task_tcp=(mechanical@axes).tolist(),
        mechanical_tcp_from_task_tcp=axes.tolist(), rigid_geometry=tool['geometry'],
        zone_assignment=asset['zones'], zone_minimum_policy='total cup minimum only; no extra per-zone threshold',
        maximum_point_error_m=MetricFitConfig().maximum_mean_m,
        error_scope='Each covered depth sample <= existing plane mean limit; stricter local sample test, not seal/pose uncertainty',
        sources=[file_reference(v) for v in (config_path, tool_path, asset_path)],
        transform_source='validation_config.ValidationConfig.robot: mechanical_tcp @ Ry(pi/2)')
    validate_tool(result)
    result['identity'] = canonical_fingerprint(result)
    return result


def validate_tool(tool):
    centers, zones, minimum = suction_cup_layout_geometry(tool['layout'])
    values = [tool[k] for k in ('edge_margin_m', 'contact_gap_m', 'standoff_m', 'maximum_point_error_m')]
    if not np.isfinite(values).all() or min(values) < 0 or values[2] <= values[1] or values[3] <= 0:
        raise ValueError('INVALID_CONTACT_DISTANCES')
    for k in ('flange_from_task_tcp', 'mechanical_tcp_from_task_tcp'):
        validate_transform_parent_child(tool[k])
    grid, rolls = tool['grid_fractions'], tool['roll_degrees']
    if (not grid or not rolls or len(grid)**2*len(rolls)>256 or
            not np.isfinite(grid).all() or max(abs(x) for x in grid)>=1 or not np.isfinite(rolls).all()):
        raise ValueError('INVALID_BOUNDED_CONTACT_SEARCH')
    return centers, zones, minimum


class ContactSupportInsufficient(ValueError):
    """Valid evidence that cannot geometrically accommodate even one cup."""


@dataclass
class ContactCandidate:
    """The existing candidate is usable by geometry consumers; never an execution grant."""
    candidate: SuctionGraspCandidate
    evidence: dict

    def to_dict(self):
        fields = asdict(self.candidate)
        fields = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in fields.items()}
        return dict(candidate=fields, **self.evidence)


def prepare_support(surface, record, depth, mask, K):
    points, normal, _ = _surface(surface)
    K = np.asarray(K, float).reshape(3, 3)
    if (not np.isfinite(K).all() or min(K[0,0], K[1,1])<=0 or
            not np.array_equal(K[2], [0,0,1]) or K[0,1]!=0 or K[1,0]!=0):
        raise ValueError('INVALID_PINHOLE_CALIBRATION')
    if record.get('support_capture_binding') != _binding(depth, mask, K):
        raise ValueError('FROZEN_SUPPORT_INPUT_BINDING_MISMATCH')
    # Match the certified patch coordinates, not a convenient plane from the same instance.
    T = np.asarray(surface['T_W_C_at_capture'])
    camera_points = (points-T[:3,3])@T[:3,:3]
    matching = [f for f in record['camera_facing_faces'] if
                np.asarray(f.get('corners_3d_m', [])).shape==(4,3) and
                np.allclose(f['corners_3d_m'], camera_points, atol=1e-9, rtol=0)]
    if len(matching)!=1 or matching[0]['final_support']['status']!='PASS':
        raise ValueError('CERTIFIED_PATCH_RECORD_MISMATCH')
    f = matching[0]
    region = _support_mask(record['frozen_support_regions'][str(f['support_label'])], np.asarray(depth).shape)
    erosion=int(record.get('config',{}).get('erosion_px',2))
    parent=observation_support(depth,mask,mask,erosion_px=erosion)[1] if record.get('support_policy')=='MEASUREMENT_PARENT_V2' else None
    _, retained, audit = observation_support(depth, mask, region, erosion_px=erosion,parent_support=parent)
    n = T[:3,:3].T@normal
    d = float(surface['plane_offset_m']+normal@T[:3,3])
    projected = camera_points@K.T
    if (camera_points[:,2]<=0).any() or not np.allclose(projected[:,:2]/projected[:,2,None], surface['boundary_2d_px'], atol=1e-5, rtol=0):
        raise ValueError('PATCH_CAPTURE_PROJECTION_MISMATCH')
    return dict(depth=np.asarray(depth), retained=retained, K=K, T=T, normal_camera=n,
                offset_camera=d, audit=audit, label=f['support_label'])


def _cup_support(center, rotation, radius, points, normal, support, tolerance):
    # Analytic entire-circle inclusion in the measured patch, not physical box bounds.
    edges = np.roll(points,-1,axis=0)-points
    inward = np.cross(normal, edges)
    inward /= np.linalg.norm(inward,axis=1)[:,None]
    if np.min(np.sum((points.mean(axis=0)-points)*inward,axis=1))<0: inward *= -1
    clearance = float(np.min(np.sum((center-points)*inward,axis=1))-radius)
    if clearance < -1e-12: return dict(status='REJECTED', reason='CUP_LIP_OUTSIDE_OBSERVED_PATCH', margin_m=clearance)
    T,K=support['T'],support['K']; nc,dc=support['normal_camera'],support['offset_camera']
    # Project an enclosing square. Check every image pixel cell that can intersect
    # the disk. The metric radius below bounds each complete projective cell;
    # no sparse circumference sampling, convex-hull filling or depth-hole repair.
    square=np.array([center+radius*(u*rotation[:,0]+v*rotation[:,1]) for u,v in ((-1,-1),(1,-1),(1,1),(-1,1))])
    camera=(square-T[:3,3])@T[:3,:3]
    if (camera[:,2]<=0).any(): return dict(status='REJECTED',reason='CUP_BEHIND_CAMERA',margin_m=clearance)
    projection=camera@K.T; uv=projection[:,:2]/projection[:,2,None]
    lo=np.floor(uv.min(axis=0)-.5).astype(int); hi=np.ceil(uv.max(axis=0)+.5).astype(int)
    h,w=support['depth'].shape
    if (lo<0).any() or hi[0]>=w or hi[1]>=h:
        return dict(status='INSUFFICIENT_EVIDENCE',reason='CUP_IMAGE_CROPPED',margin_m=clearance)
    yy,xx=np.mgrid[lo[1]:hi[1]+1,lo[0]:hi[0]+1]; pixels=np.column_stack((xx.ravel(),yy.ravel()))
    def lift(p):
        rays=backproject_pixels(p,np.ones(len(p)),K); denom=rays@nc
        if (np.abs(denom)<1e-10).any(): raise ValueError('GRAZING_CAPTURE_RAYS')
        return rays*(-dc/denom)[:,None]
    pc=lift(pixels); corners=np.stack([lift(pixels+delta) for delta in ((-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5))])
    if (corners[:,:,2]<=0).any(): raise ValueError('INVALID_PIXEL_PLANE_INTERSECTION')
    cell_radius=np.max(np.linalg.norm(corners-pc,axis=2),axis=0)
    cc=(center-T[:3,3])@T[:3,:3]
    selected=np.linalg.norm(pc-cc,axis=1)<=radius+cell_radius
    pix=pixels[selected]; x,y=pix.T
    if not len(pix) or not support['retained'][y,x].all():
        return dict(status='INSUFFICIENT_EVIDENCE',reason='HOLE_OCCLUSION_INVALID_OR_ERODED_SUPPORT',margin_m=clearance,
                    checked_pixels=len(pix),unsupported_pixels=int(np.count_nonzero(~support['retained'][y,x])))
    actual=backproject_pixels(pix,support['depth'][y,x],K)
    residual=np.abs(actual@nc+dc)
    maximum=float(residual.max())
    passed=maximum<=tolerance
    return dict(status='GEOMETRIC_SUPPORT' if passed else 'REJECTED',
                reason=None if passed else 'LOCAL_DEPTH_PLANE_ERROR', margin_m=clearance,
                checked_pixels=len(pix),maximum_point_error_m=maximum)


def face_candidates(surface, record, depth, mask, K, tool, *, artifact, object_id):
    centers,zones,minimum=validate_tool(tool)
    points,normal,_=_surface(surface)
    support=prepare_support(surface,record,depth,mask,K)
    if record.get('support_policy')=='MEASUREMENT_PARENT_V2':
        width=patch_minimum_width(points)
        if width < 2*(tool['layout']['cup_radius_m']+tool['edge_margin_m']):
            raise ContactSupportInsufficient('PATCH_CANNOT_CONTAIN_ONE_COMPLETE_CUP_DISK')
    R=make_tool_rotation(-normal)
    uv=points@R[:,:2]; low,high=uv.min(axis=0),uv.max(axis=0)
    origin=points.mean(axis=0)  # Only an observed patch search origin; never a box centre.
    candidates=[]
    for fu in tool['grid_fractions']:
        for fv in tool['grid_fractions']:
            coordinate=(low+high)/2+np.array([fu,fv])*(high-low)/2
            contact=origin+R[:,:2]@(coordinate-origin@R[:,:2])
            base=SuctionGraspCandidate(object_id, contact, normal, 'observed_surface',
                make_transform(R,contact+normal*tool['standoff_m']),
                make_transform(R,contact+normal*tool['contact_gap_m']),0.)
            for roll,candidate in zip(tool['roll_degrees'],_expand_candidate_wrist_rolls([base],tool['roll_degrees'])):
                rotation=candidate.grasp_pose[:3,:3]
                cups=[_cup_support(contact+rotation[:,:2]@offset,rotation,
                      tool['layout']['cup_radius_m']+tool['edge_margin_m'],points,normal,support,
                      tool['maximum_point_error_m']) for offset in centers]
                indices=tuple(i for i,v in enumerate(cups) if v['status']=='GEOMETRIC_SUPPORT')
                candidate.sealed_cup_indices=indices
                candidate.sealed_cups_per_zone=tuple(sum(zones[i]==z for i in indices) for z in range(tool['layout']['zone_count']))
                candidate.score=float(len(indices))  # Supported cup count; deterministic generation-order ties.
                identity=dict(artifact=artifact,object_id=object_id,source={k:surface[k] for k in SOURCE_KEYS},
                              tool_identity=canonical_fingerprint(tool),grid=[fu,fv],roll_degrees=roll)
                candidates.append(ContactCandidate(candidate,dict(identity=canonical_fingerprint(identity),source=identity,
                    capture_time=surface['capture_time'],clock_domain=surface['clock_domain'],
                    contact_reference_pose=make_transform(rotation,contact).tolist(),
                    flange_pose=(candidate.grasp_pose@np.linalg.inv(tool['flange_from_task_tcp'])).tolist(),
                    mechanical_tcp_pose=(candidate.grasp_pose@np.linalg.inv(tool['mechanical_tcp_from_task_tcp'])).tolist(),
                    pose_status='CONSTRUCTED',support_status='GEOMETRIC_SUPPORT' if len(indices)>=minimum else 'INSUFFICIENT_SUPPORTED_CUPS',
                    required_cups=minimum,cups=cups,support_audit=support['audit'],
                    score_basis='number of completely supported cup disks; stable grid/roll order breaks ties',
                    sealed_field_semantics='geometric prediction only, no seal measurement',remaining=BLOCKERS.copy())))
    return sorted(candidates,key=lambda c:-c.candidate.score)


def representative_surfaces(cargo):
    """Use frozen fusion representatives; never merge support across cameras."""
    diagnostics=cargo.raw_result.get('fusion_diagnostics',{})
    reduction=diagnostics.get('face_reduction',{})
    if reduction.get('conflicts') or 'CONFLICT' in cargo.association_status or 'AMBIGUOUS' in cargo.association_status:
        return [], 'ASSOCIATION_CONFLICT_OR_AMBIGUITY'
    surfaces=list(cargo.observed_surfaces)
    keys=[tuple(s[k] for k in SOURCE_KEYS) for s in surfaces]
    if len(keys)!=len(set(keys)): raise ValueError('DUPLICATE_SURFACE_IDENTITY')
    if reduction:
        reps={tuple(s[k] for k in SOURCE_KEYS) for s in reduction['representatives']}
        if not reps.issubset(keys): raise ValueError('MISSING_FUSION_REPRESENTATIVE')
        surfaces=[s for s in surfaces if tuple(s[k] for k in SOURCE_KEYS) in reps]
    return surfaces, None if surfaces else 'NO_CERTIFIED_OBSERVED_FACE'
