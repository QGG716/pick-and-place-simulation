"""Mixed recovery, ignored detections, and exact shadow ownership subtraction."""
from types import SimpleNamespace
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from unloading_sim.workbench.single_carton.geometry import transform,corners
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.single_carton.site_scene import build_scene
from unloading_sim.workbench.single_carton.size_policy import mark_incomplete_scene,unmodeled_cartons
from unloading_sim.workbench.single_carton.convex_clip import box_solid,subtract_solids

POLICY=dict(size_policy='MEASURED_PER_INSTANCE',size_policy_basis='SYNTHETIC_TWO_CARTONS',
    max_carton_extent_m=.4,max_carton_extent_basis='SYNTHETIC_UPPER_BOUND')


def fixture():
    shape=(350,400);y,x=np.indices(shape);rays=np.stack(((x-200)/450,(y-175)/450,np.ones(shape)),-1)
    depth=np.full(shape,1.4);owner=np.full(shape,-1)
    for index,(center,angle) in enumerate([([0,-.11,.7],30),([0,.11,.7],0)]):
        R=Rotation.from_euler('y',angle,degrees=True).as_matrix()@np.diag([1,-1,-1]);half=np.array([.12,.075,.05])
        origin=-np.asarray(center)@R;d=rays@R
        with np.errstate(divide='ignore',invalid='ignore'):
            a=(-half-origin)/d;b=(half-origin)/d
            near=np.minimum(a,b).max(-1);far=np.maximum(a,b).min(-1)
        take=(near<far)&(near>.1)&(near<depth);depth[take]=near[take];owner[take]=index
    # The lower carton has one visible face and missing background depth:
    # neither side measurement nor same-frame visibility can bound its back.
    depth[(y>175)&(owner==-1)]=0
    frame=SimpleNamespace(rgb=np.zeros((*shape,3),'uint8'),depth=depth,depth_scale_m=1.,observation_id='two-cartons',
        intrinsics=dict(width=400,height=350,fx=450,fy=450,ppx=200,ppy=175,model='SYNTHETIC_PINHOLE',coeffs=[0]*5))
    obs=SimpleNamespace(frame=frame,world_camera=np.eye(4),q_capture=[0.]*6)
    masks=[owner==i for i in (0,1)];result=SimpleNamespace(observation_id=frame.observation_id,masks=masks)
    site=dict(workspace_bounds_m=[[-.24,-.26,.4],[.24,.26,1.2]],voxel_m=.04,
        unknown_policy='USER_CONFIRMED_CLEAR',unknown_clearance_basis='SYNTHETIC_USER_CLEAR',fixed_obstacles=[],measured_clear_regions=[])
    return obs,result,site


def vertices(obj):
    if 'size' in obj:return corners(obj['pose'],obj['size'])
    return transform(obj['pose'],obj['convex_mesh']['vertices'])


def test_measured_target_and_unresolved_lower_neighbor_remain_plannable():
    obs,result,site=fixture()
    boxes,rejected=recover_boxes(obs.frame,np.eye(4),[{},{}],result.masks,POLICY,site['workspace_bounds_m'])
    assert [b['detection_index'] for b in boxes]==[0]
    assert [r['detection_index'] for r in rejected]==[1]
    scene=build_scene(obs,result,boxes,site,neighbor_depth_m=.4,unresolved_cartons=rejected)
    mark_incomplete_scene(scene,POLICY,rejected)
    assert not unmodeled_cartons(scene)
    shadow=[o for o in scene['world'] if o['category']=='neighbor_shadow']
    assert shadow and scene['unresolved_neighbor_extrusion'][0]['depth_m']==.4
    assert rejected[0]['evidence']['observed_face']['observed']
    owned=box_solid(boxes[0]['pose'],boxes[0]['size_m'])
    for obj in shadow:
        v=vertices(obj);eq=owned['equations']
        # A convex shadow lies wholly outside at least one target face.
        assert np.any(np.all(v@eq[:,:3].T+eq[:,3]>=-1e-9,axis=0))


def test_outside_detection_does_not_create_shadow_or_block_target():
    obs,result,site=fixture();site['workspace_bounds_m']=[[-.24,-.26,.4],[.24,-.01,1.2]]
    boxes,rejected=recover_boxes(obs.frame,np.eye(4),[{},{}],result.masks,POLICY,site['workspace_bounds_m'])
    assert boxes and rejected[0]['reason']=='OUTSIDE_WORKSPACE_IGNORED'
    scene=build_scene(obs,result,boxes,site,neighbor_depth_m=.4,unresolved_cartons=rejected)
    mark_incomplete_scene(scene,POLICY,rejected)
    assert scene['outside_workspace_ignored']==[1] and not unmodeled_cartons(scene)
    assert not scene['unresolved_neighbor_extrusion'] and scene['neighbor_shadow_voxels']==0
    assert len([o for o in scene['world'] if o['category']=='workspace_boundary'])==6


def test_exact_subtraction_preserves_all_unowned_volume_and_removes_robot():
    source=corners(np.eye(4),[.2,.2,.2]);robot=np.eye(4);robot[:3,3]=[.05,0,0]
    solid=box_solid(robot,[.1,.3,.3]);pieces=subtract_solids(source,[solid])
    assert abs(sum(ConvexHull(p).volume for p in pieces)-.004)<1e-12
    for p in pieces:
        eq=solid['equations'];assert np.any(np.all(p@eq[:,:3].T+eq[:,3]>=-1e-10,axis=0))


def test_scene_shadow_is_clipped_against_robot_solids():
    obs,result,site=fixture();T=np.eye(4);T[:3,3]=[.03,.11,.82]
    solid=box_solid(T,[.08,.08,.08]);eq=solid['equations']
    class Self:
        def contains(self,points,q,group_size=1):
            inside=np.all(np.asarray(points)@eq[:,:3].T+eq[:,3]<=1e-9,axis=1)
            return inside.reshape(-1,group_size).all(1)
        def world_solids(self,q):return [solid]
    boxes,rejected=recover_boxes(obs.frame,np.eye(4),[{},{}],result.masks,POLICY,site['workspace_bounds_m'])
    scene=build_scene(obs,result,boxes,site,Self(),neighbor_depth_m=.4,unresolved_cartons=rejected)
    assert scene['neighbor_shadow_clipped_pieces']>0
    for obj in scene['world']:
        if obj['category']=='neighbor_shadow':
            assert np.any(np.all(vertices(obj)@eq[:,:3].T+eq[:,3]>=-1e-9,axis=0))
