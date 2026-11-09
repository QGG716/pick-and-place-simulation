"""Fixed station: gap and an explicitly selected 1000/900 mm mounting height.

Legacy search schemas are deliberately not accepted as evidence for this task.
"""
from copy import deepcopy
from dataclasses import dataclass
import numpy as np
from .geometry import OBB
from .workcell_layout import canonical_digest
from .joint_search_layout import BASELINE

SCHEMA = 'fixed40_gap_search_20261107_v1'
PROFILE = 'low_profile_200_outfeed20_v3'


def initial_stack_facing_evidence(physical_pose, cartons):
    """Check the physical cup working +Z axis, never a wrist or virtual TCP axis.

    The fixed stack faces -X. Its initial unloaded tool must point +X within
    one degree and its forward ray must hit the frozen stack's front face.
    This orientation check supplements, rather than replaces, full collision.
    """
    pose=np.asarray(physical_pose,float)
    if (pose.shape!=(4,4) or not np.all(np.isfinite(pose))
            or not np.allclose(pose[3],[0.,0.,0.,1.],atol=1e-9,rtol=0)
            or not np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-9,rtol=0)
            or not np.isclose(np.linalg.det(pose[:3,:3]),1.,atol=1e-9,rtol=0)):
        raise ValueError('INVALID_PHYSICAL_INITIAL_TOOL_POSE')
    if not cartons:raise ValueError('EMPTY_INITIAL_STACK')
    bounds=[(b.corners().min(axis=0),b.corners().max(axis=0)) for b in cartons]
    front=min(lo[0] for lo,hi in bounds);normal=pose[:3,2];origin=pose[:3,3]
    angle=float(np.arccos(np.clip(normal[0],-1.,1.)))
    distance=float((front-origin[0])/normal[0]) if normal[0]>0 else None
    point=origin+distance*normal if distance is not None and distance>0 else None
    hit=bool(point is not None and any(abs(lo[0]-front)<1e-9 and
        np.all(point[1:]>=lo[1:]-1e-9) and np.all(point[1:]<=hi[1:]+1e-9) for lo,hi in bounds))
    return dict(passed=bool(angle<=np.deg2rad(1.)+1e-12 and hit),
        physical_pose_world=pose.tolist(),physical_working_normal_world=normal.tolist(),
        required_normal_world=[1.,0.,0.],maximum_angle_rad=float(np.deg2rad(1.)),
        actual_angle_rad=angle,stack_front_x_m=float(front),forward_ray_hits_stack=hit,
        intersection_world_m=None if point is None else point.tolist())

@dataclass(frozen=True)
class GapCandidate:
    g: float
    mount_height: float = 1.

    def __post_init__(self):
        object.__setattr__(self, 'g', float(self.g))
        if not np.isfinite(self.g) or self.g < .005:
            raise ValueError('GAP_REQUIRES_AT_LEAST_5MM')
        object.__setattr__(self,'mount_height',float(self.mount_height))
        if self.mount_height not in (.9,1.):raise ValueError('UNAPPROVED_GAP_MOUNT_HEIGHT')

    @property
    def key(self):
        return canonical_digest(self.evidence())[:16]

    def evidence(self):
        return dict(schema=SCHEMA, gap_m=self.g,
            assembly_world_xyz_m=[-1.75-self.g,.35,0.],
            base_xyz_a_m=[.6,-.2,self.mount_height], base_xyz_world_m=[-1.15-self.g,.15,self.mount_height],
            mount_rpy_rad=[0.,0.,0.], mount_design='SOLID_PLINTH',
            plinth_height_m=round(self.mount_height-.6,9), mount_axis_to_chassis_front_m=.45,
            minimum_surface_height_m=.55, maximum_surface_height_m=1.5,
            initial_surface_height_m=.6, mechanism_profile=PROFILE, tool_extension_m=.2,
            upper_layer_original_bottom_threshold_m=1.5, upper_layer_faces=['FRONT'],
            lower_layer_face_priority=['TOP','FRONT','SIDE'],
            mechanical_reference_gap_m=.2, relief_design_maximum_height_m=2.1,
            lower_rail_x_bounds_a_m=[-2.,-.95], lower_rail_stations_x_a_m=[-1.18,-1.02],
            structural_strength='NOT_VERIFIED', physical_execution='NOT_RUN')

def candidate_data(baseline, candidate):
    d=deepcopy(baseline);e=candidate.evidence()
    if 'joint_search' in d or 'gap_search' in d:
        raise ValueError('GAP_BASELINE_MUST_BE_ORIGINAL_LAYOUT')
    d['assembly']['world_xyz_m']=e['assembly_world_xyz_m']
    d['robot'].update(base_origin_xy_a_m=[.6,-.2],mounting_surface_z_a_m=candidate.mount_height,
        base_front_edge_x_world_m=-.925-candidate.g)
    d['conveyors']['coupling'].update(minimum_surface_height_world_m=.55,
        maximum_surface_height_world_m=1.5,scissor_profile=PROFILE)
    d['evidence']['approval_basis']=SCHEMA
    d['gap_search']=e
    return d

def validate_search_layout(layout):
    import yaml
    baseline=yaml.safe_load((layout.config_path.parent/BASELINE).read_text(encoding='utf-8'))
    candidate=GapCandidate(layout.data['gap_search']['gap_m'],layout.data['gap_search']['base_xyz_a_m'][2])
    if layout.data!=candidate_data(baseline,candidate):
        raise ValueError('GAP_SEARCH_FROZEN_FIELDS_CHANGED')
    return candidate

def mounting_geometry(layout, original):
    candidate=validate_search_layout(layout)
    # Exact official base footprint; unchanged, solid 2100 x 1500 x 600 chassis.
    plinth=OBB([.54325,-.2,round((candidate.mount_height+.6)/2,9)],
        [.28175,.275,round((candidate.mount_height-.6)/2,9)],np.eye(3),
        'mount_plinth','mount_structure').transformed(layout.world_from_assembly)
    return (*original,plinth)

def floor_support_evidence(layout, coupled):
    """Do not confuse crossing the door with a proof of infeasibility.

    Full support footprints inside the existing floor are verified here.
    Outside material is UNKNOWN_EXTERNAL_SUPPORT, never silently a larger floor.
    """
    floor=next(b for b in coupled.boxes if b.name=='trailer_floor')
    lo=floor.corners().min(axis=0);hi=floor.corners().max(axis=0)
    records=[]
    for b in coupled.boxes:
        corners=b.corners()
        if not (b.name=='chassis' or '_foot_' in b.name or '_stand_' in b.name):continue
        if b.category=='carton' or abs(corners[:,2].min())>1e-9:continue
        if b.name=='trailer_floor':continue
        low=corners[:,:2].min(axis=0);high=corners[:,:2].max(axis=0)
        full=bool(np.all(low>=lo[:2]-1e-10) and np.all(high<=hi[:2]+1e-10))
        records.append(dict(id=b.name,footprint_min_xy_m=low.tolist(),footprint_max_xy_m=high.tolist(),
            status='VERIFIED_ON_MODELED_FLOOR' if full else 'UNKNOWN_EXTERNAL_SUPPORT'))
    return dict(status='VERIFIED_ON_MODELED_FLOOR' if all(r['status']=='VERIFIED_ON_MODELED_FLOOR' for r in records)
        else 'UNKNOWN_EXTERNAL_SUPPORT',chassis_rear_world_x_m=-2.8-layout.data['gap_search']['gap_m'],
        floor_x_bounds_m=[float(lo[0]),float(hi[0])],footprints=records,
        external_floor_added=False,global_infeasibility_proven=False)
