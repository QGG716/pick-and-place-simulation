"""Versioned fixed-layout search domain; historical layouts remain immutable."""
from copy import deepcopy
from dataclasses import dataclass
from itertools import product
import numpy as np
from .geometry import OBB
from .workcell_layout import canonical_digest

SCHEMA='fixed40_joint_search_20261028_v1'
BASELINE='m710id70_unloading_layout_v1_mount_x_minus_300_y_minus_200_min250_lowprofile_tool_extension200.yaml'

@dataclass(frozen=True)
class LayoutCandidate:
    x: float
    y: float
    z: float
    minimum: float=.25
    maximum: float=2.10
    profile: str='low_profile_250_v1'

    def __post_init__(self):
        # Refinement arithmetic returns NumPy scalars; YAML needs native floats.
        for name in ('x','y','z','minimum','maximum'):
            object.__setattr__(self,name,float(getattr(self,name)))

    def evidence(self):
        return dict(schema=SCHEMA,base_xyz_a_m=[self.x,self.y,self.z],
            base_xyz_world_m=[self.x-1.95,self.y+.35,self.z],mount_rpy_rad=[0.,0.,0.],
            mount_design='ORIGINAL_DECK' if self.z==.6 else 'SOLID_PLINTH' if self.z>.6 else 'RECESSED_DECK',
            minimum_surface_height_m=self.minimum,maximum_surface_height_m=self.maximum,
            mechanism_profile=self.profile,tool_extension_m=.2,
            structural_strength='NOT_VERIFIED',actuator_rating='NOT_VERIFIED',manufacturing_rating='NOT_VERIFIED')

    @property
    def key(self):return canonical_digest(self.evidence())[:16]

def support_domain(data):
    half=np.asarray(data['chassis']['size_xyz_m'])[:2]/2
    center=np.asarray(data['chassis']['center_a_m'])[:2]
    lo=np.asarray(data['robot']['base_support_bbox_min_xyz_m'])[:2]
    hi=np.asarray(data['robot']['base_support_bbox_max_xyz_m'])[:2]
    return np.column_stack((center-half-lo,center+half-hi))

def candidate_data(baseline,candidate):
    d=deepcopy(baseline);e=candidate.evidence();domain=support_domain(d)
    if (not np.isfinite([candidate.x,candidate.y,candidate.z,candidate.minimum,candidate.maximum]).all()
            or not all(domain[i,0]-1e-12<=v<=domain[i,1]+1e-12 for i,v in enumerate((candidate.x,candidate.y)))
            or not .25<=candidate.z<=1.10 or not .19<=candidate.minimum<=.6
            or candidate.maximum!=2.10 or candidate.profile not in ('low_profile_250_v1','low_profile_200_v1','low_profile_200_v2','low_profile_200_outfeed20_v3')):
        raise ValueError('JOINT_SEARCH_LAYOUT_DOMAIN')
    if candidate.profile=='low_profile_250_v1' and candidate.minimum<=.236:
        raise ValueError('ORIGINAL_SCISSOR_CONSTRUCTION_REQUIRES_HEIGHT_ABOVE_236MM')
    if candidate.profile in ('low_profile_200_v1','low_profile_200_v2','low_profile_200_outfeed20_v3') and candidate.minimum<=.190:
        raise ValueError('COMPACT_SCISSOR_CONSTRUCTION_RANGE')
    d['robot'].update(base_origin_xy_a_m=[candidate.x,candidate.y],mounting_surface_z_a_m=candidate.z,
        base_front_edge_x_world_m=candidate.x-1.95+.225)
    d['conveyors']['coupling'].update(minimum_surface_height_world_m=candidate.minimum,
        maximum_surface_height_world_m=candidate.maximum,scissor_profile=candidate.profile)
    d['evidence']['approval_basis']=SCHEMA
    d['joint_search']=e
    return d

def validate_search_layout(layout):
    import yaml
    baseline=yaml.safe_load((layout.config_path.parent/BASELINE).read_text(encoding='utf-8'))
    e=layout.data['joint_search']
    c=LayoutCandidate(*e['base_xyz_a_m'],e['minimum_surface_height_m'],e['maximum_surface_height_m'],e['mechanism_profile'])
    if layout.data!=candidate_data(baseline,c):raise ValueError('JOINT_SEARCH_FROZEN_FIELDS_CHANGED')
    return c

def mounting_geometry(layout,original):
    """Disjoint rectangular solids; recess removes only an explicitly named volume."""
    c=validate_search_layout(layout)
    if c.z==.6:return tuple(original)
    chassis=next(b for b in original if b.name=='chassis')
    out=[b for b in original if b.name!='chassis']
    robot=layout.data['robot'];lo=np.array([c.x,c.y])+np.asarray(robot['base_support_bbox_min_xyz_m'])[:2]
    hi=np.array([c.x,c.y])+np.asarray(robot['base_support_bbox_max_xyz_m'])[:2]
    def box(name,a,b,category='mount_structure'):
        a=np.asarray(a,float);b=np.asarray(b,float)
        if np.any(b-a<=1e-10):return
        out.append(OBB((a+b)/2,(b-a)/2,np.eye(3),name,category).transformed(layout.world_from_assembly))
    if c.z>.6:
        out.insert(0,chassis)
        box('mount_plinth',[*lo,.6],[*hi,c.z])
    else:
        # 25 mm radial installation clearance, clipped only at the chassis edge.
        lo=np.maximum(lo-.025,[-1.05,-.75]);hi=np.minimum(hi+.025,[1.05,.75])
        box('chassis',[-1.05,-.75,0],[1.05,.75,c.z],category='chassis')
        box('mount_recess_xminus',[-1.05,-.75,c.z],[lo[0],.75,.6])
        box('mount_recess_xplus',[hi[0],-.75,c.z],[1.05,.75,.6])
        box('mount_recess_yminus',[lo[0],-.75,c.z],[hi[0],lo[1],.6])
        box('mount_recess_yplus',[lo[0],hi[1],c.z],[hi[0],.75,.6])
    return tuple(out)

def qualified_mount_pairs(robot,obstacles):
    """Only the fixed base's nonpenetrating supporting plane; never J1 or tool."""
    spec=getattr(robot,'joint_search_mount',None)
    if not spec or spec['mount_design']!='SOLID_PLINTH':return set()
    support=next((b for b in obstacles if b.name=='mount_plinth'),None)
    if support is None:raise ValueError('MOUNT_PLINTH_MISSING_FROM_WORLD')
    low=support.center-support.half_extents;high=support.center+support.half_extents
    base=robot.base_transform[:3,3]
    if (not np.array_equal(support.rotation,np.eye(3)) or abs(high[2]-base[2])>1e-10
            or not np.allclose(low[:2],base[:2]+[-.3385,-.275],atol=1e-12,rtol=0)
            or not np.allclose(high[:2],base[:2]+[.225,.275],atol=1e-12,rtol=0)
            or abs(low[2]-.6)>1e-12):raise ValueError('MOUNT_PLINTH_CONTACT_GEOMETRY_CHANGED')
    return {('base_link','mount_plinth')}


def retained_chassis_contacts(layout,contacts):
    """Carry the original exterior assembly seam through a geometric split.

    Only unchanged chassis material on the original X/Y boundary qualifies.
    Runtime CPU/native separating-plane and nonpenetration checks still apply.
    This supplies no robot, tool, carton, plinth or interior-wall exemption.
    """
    if 'joint_search' not in layout.data:return list(contacts)
    c=validate_search_layout(layout)
    if c.z>=.6:return list(contacts)
    result=list(contacts);fixed=layout.fixed_components()
    lo=np.asarray(layout.data['chassis']['center_a_m'])-np.asarray(layout.data['chassis']['size_xyz_m'])/2
    hi=lo+np.asarray(layout.data['chassis']['size_xyz_m'])
    offset=layout.world_from_assembly[:3,3];lo=lo+offset;hi=hi+offset
    named={b.name:b for b in fixed}
    for pair in contacts:
        if 'chassis' not in pair:continue
        other=next(n for n in pair if n!='chassis')
        if other not in ('conveyor_transverse','conveyor_longitudinal'):continue
        belt=named[other];bl=belt.center-belt.half_extents;bh=belt.center+belt.half_extents
        for wall in fixed:
            if wall.name not in ('mount_recess_xminus','mount_recess_xplus','mount_recess_yminus','mount_recess_yplus'):continue
            wl=wall.center-wall.half_extents;wh=wall.center+wall.half_extents
            if not (np.all(wl>=lo-1e-12) and np.all(wh<=hi+1e-12)):raise ValueError('RETAINED_CHASSIS_OUTSIDE_ORIGINAL')
            if any((abs(wh[i]-hi[i])<1e-12 and abs(hi[i]-bl[i])<1e-12) or
                   (abs(wl[i]-lo[i])<1e-12 and abs(lo[i]-bh[i])<1e-12) for i in (0,1)):
                result.append([wall.name,other])
    return result

def initial_candidates(baseline,seed=71070):
    """Whole-domain XYZ combinations, with visible deterministic priority only."""
    domain=support_domain(baseline)
    preferred=[(.325,-.2,.6),(.625,-.25,.5),(.525,-.35,.4),(.725,-.35,.7),(.425,-.15,.8)]
    grid=[(float(x),float(y),float(z)) for x,y,z in product(
        np.unique(np.r_[np.arange(-.7,.826,.1),domain[0]]),
        np.unique(np.r_[np.arange(-.4,.476,.1),domain[1]]),np.arange(.3,1.101,.1))]
    rng=np.random.default_rng(seed);rng.shuffle(grid);seen=set()
    for x,y,z in preferred+grid:
        for minimum,profile in ((.25,'low_profile_250_v1'),(.24,'low_profile_250_v1'),(.20,'low_profile_200_v1'),(.20,'low_profile_200_v2'),(.20,'low_profile_200_outfeed20_v3')):
            c=LayoutCandidate(round(x,4),round(y,4),round(z,4),minimum,profile=profile)
            if c.key in seen:continue
            seen.add(c.key);yield c
