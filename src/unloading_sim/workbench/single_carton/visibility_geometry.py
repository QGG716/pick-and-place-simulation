"""Size-free bounds from the frozen RGB-D's observed free space.

The observed face defines a local cuboid half-space, not its hidden thickness.
An occluder, a missing depth or the image edge never closes that half-space.
Free-space witnesses can bound a convex front face and its inward extrusion;
without witnesses the extent stays unbounded. Static obstacles may be clipped
to the enforced planning workspace; an attached payload must NEVER be so clipped.
"""
from itertools import product
from types import SimpleNamespace
import cv2
import numpy as np
from ..detection import metric_points
from .geometry import corners, transform, rigid
from .site_scene import _project

FREE_MARGIN_M = .006  # Same RGB-D free-space margin as the site voxel test.
PIXEL_GUARD = 2


def face_bounds(frame, world_camera, evidence):
    """Bound a face by sensor-free barriers, including foreground occlusion.

    The whole connected non-free component is retained, not just the SAM mask.
    A component reaching any image border is explicitly unbounded. This works
    with an incomplete mask as long as actual background depth closes the face.
    """
    face = evidence['observed_face']; C = np.linalg.inv(rigid(world_camera))
    center = transform(C, [face['center_m']])[0]
    axes = C[:3, :3] @ np.column_stack([face[k] for k in ('u', 'v', 'normal')])
    unit = SimpleNamespace(rgb=frame.rgb, depth=np.ones_like(frame.depth),
        depth_scale_m=1., intrinsics=frame.intrinsics, observation_id=frame.observation_id)
    rays = metric_points(unit); den = rays @ axes[:, 2]; offset = center @ axes[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        z = offset / den
        far = z + FREE_MARGIN_M / np.abs(den)
    depth = np.asarray(frame.depth, float) * frame.depth_scale_m
    valid_plane = np.isfinite(z) & (z > .1) & (np.abs(den) > 1e-8)
    free = valid_plane & (depth < 3.) & (depth > far + FREE_MARGIN_M)
    free = cv2.erode(free.astype('uint8'), np.ones((2*PIXEL_GUARD+1,)*2, 'uint8')).astype(bool)
    _, labels = cv2.connectedComponents((~free).astype('uint8'), connectivity=8)
    support = np.asarray(face['support_points_uv_m'], float)
    points = center + support @ axes[:, :2].T
    uv = np.rint(_project(points, frame.intrinsics)).astype(int)
    h, w = depth.shape
    uv = uv[(uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)]
    ids = np.unique(labels[uv[:, 1], uv[:, 0]]) if len(uv) else np.array([], int)
    ids = ids[ids != 0]; component = np.isin(labels, ids)
    bounded = bool(ids.size and not (component[0].any() or component[-1].any() or
        component[:, 0].any() or component[:, -1].any()) and valid_plane[component].all())
    record = dict(method='RGBD_FREE_SPACE_CONNECTED_FACE_COMPONENT', bounded=bounded,
        component_pixels=int(component.sum()), free_margin_m=FREE_MARGIN_M,
        pixel_guard=PIXEL_GUARD, image_or_occlusion_boundary_is_box_edge=False,
        reason=None if bounded else 'FRONT_FACE_POSSIBLE_EXTENT_REACHES_UNOBSERVED_RAYS')
    if bounded:
        plane_points = rays[component] * z[component, None]
        local = (plane_points-center) @ axes[:, :2]
        # Cover the full pixel footprint as well as the fitted plane tolerance.
        pixel_m = float(np.max(z[component])) / min(frame.intrinsics['fx'], frame.intrinsics['fy'])
        pad = FREE_MARGIN_M + 2*pixel_m
        lo = np.minimum(local.min(0)-pad, -np.asarray(face['size_m'])/2)
        hi = np.maximum(local.max(0)+pad, np.asarray(face['size_m'])/2)
        record['bounds_uv_m'] = [lo.tolist(), hi.tolist()]
    return record


def inward_bound(frame, world_camera, evidence, *, search_limit_m=3., step_m=.01):
    """One observed face point must remain inside any cuboid's extrusion.

    If a guarded volume around that point at inward distance t is sensor-free,
    the carton cannot extend to t. The search limit is only a finite resource
    limit: no witness means NO upper bound, never a box of search_limit_m.
    """
    face=evidence['observed_face']; C=np.linalg.inv(rigid(world_camera))
    center=transform(C,[face['center_m']])[0]
    axes=C[:3,:3] @ np.column_stack([face[k] for k in ('u','v','normal')])
    support=np.asarray(face['support_points_uv_m'],float)
    support=support[::max(1,int(np.ceil(len(support)/128)))]
    seeds=center+support@axes[:,:2].T
    depth=np.asarray(frame.depth,float)*frame.depth_scale_m; h,w=depth.shape
    offsets=np.asarray(list(product((-1.,1.),repeat=3)))*FREE_MARGIN_M
    for distance in np.arange(step_m,search_limit_m+step_m/2,step_m):
        vertices=seeds[:,None,:]-distance*axes[:,2]+offsets
        uv=_project(vertices.reshape(-1,3),frame.intrinsics).reshape(-1,8,2)
        bounds=np.c_[np.floor(uv.min(1))-PIXEL_GUARD,np.ceil(uv.max(1))+PIXEL_GUARD].astype(int)
        view=(vertices[:,:,2].min(1)>.1)&(bounds[:,0]>=0)&(bounds[:,1]>=0)&(bounds[:,2]<w)&(bounds[:,3]<h)
        for i in np.flatnonzero(view):
            x0,y0,x1,y1=bounds[i]; patch=depth[y0:y1+1,x0:x1+1]
            if np.all((patch>vertices[i,:,2].max()+FREE_MARGIN_M)&(patch<3.)):
                return dict(bounded=True,upper_m=float(distance+FREE_MARGIN_M),
                    witness_inward_m=float(distance),witness_face_uv_m=support[i].tolist(),
                    witness_pixel_bounds=bounds[i].tolist(),witness_min_depth_m=float(patch.min()),
                    witness_max_query_depth_m=float(vertices[i,:,2].max()),
                    method='OBSERVED_FREE_SPACE_EXCLUDES_LONGER_CUBOID_EXTRUSION',
                    free_margin_m=FREE_MARGIN_M,pixel_guard=PIXEL_GUARD)
    return dict(bounded=False,upper_m=None,search_limit_m=search_limit_m,
        search_limit_is_size_bound=False,reason='NO_RGBD_FREE_SPACE_WITNESS_FOR_HIDDEN_THICKNESS')


def bound_visible_carton(frame, world_camera, evidence):
    """Return a conservative payload envelope, or explicit missing bounds.

    The same reliable fitted plane/normal model as observed_surface is used.
    No unique size, center or in-plane carton direction is claimed.
    """
    front=face_bounds(frame,world_camera,evidence)
    depth=inward_bound(frame,world_camera,evidence)
    record=dict(front=front,depth=depth,prior_applied=False,
        bound_basis='SAME_FRAME_RGBD_FREE_SPACE_AND_OBSERVED_CONVEX_CUBOID_FACE',
        observed_normal_model='RELIABLE_LOCAL_PLANE_FIT',workspace_used_as_size_bound=False)
    if not front['bounded'] or not depth['bounded']:return None,record
    face=evidence['observed_face']; lo,hi=np.asarray(front['bounds_uv_m'])
    lower=np.r_[lo,-depth['upper_m']]; upper=np.r_[hi,0.]
    reference=np.eye(4);reference[:3,:3]=np.column_stack([face[k] for k in ('u','v','normal')])
    reference[:3,3]=face['center_m']; pose=reference.copy()
    pose[:3,3]+=pose[:3,:3]@((lower+upper)/2); size=upper-lower
    # Existing native partial-envelope contract: the observed face is on +Z,
    # inside (not necessarily centered on) the complete collision envelope.
    uncertainty=dict(kind='BOUNDED_CUBOID_UNION',bound_method='RGBD_VISIBILITY_INTERVALS',
        reference_pose=reference.tolist(),bounds_face_frame_m=[lower.tolist(),upper.tolist()],
        hypotheses=[dict(kind='CONTINUOUS_SIZES_CENTERS_AND_IN_PLANE_ROTATIONS_INSIDE_RGBD_BOUNDS')],
        collision_envelope='ALL_CUBOIDS_INSIDE_SENSOR_BOUNDED_FACE_AND_EXTRUSION',
        unique_pose=False,unique_dimensions=False,image_or_occlusion_boundary_is_box_edge=False,
        visibility_evidence=record)
    box=dict(pose=pose.tolist(),size_m=size.tolist(),corners_m=corners(pose,size).tolist(),
        faces=[face],observed_faces=['local_depth_continuous_plane'],
        inferred_faces=['rgbd_bounded_unobserved_cuboid_surfaces'],nominal_size_m=None,
        dimension_source='RGBD_VISIBILITY_COLLISION_ENVELOPE_NOT_MEASURED_DIMENSIONS',
        uncertainty=uncertainty,prior_applied=False,full_dimensions_m=None,
        quality=dict(plane_rms_m=evidence['plane_rms_m'],plane_inliers=evidence['plane_inliers'],
            valid_depth_points=evidence['valid_depth_points'],observed_patch_size_m=face['size_m'],
            hidden_extent_measured=False,unseen_surfaces_observed=False,
            ambiguity='BOUNDED_EXTENTS_WITHOUT_EXACT_DIMENSIONS'))
    return box,record


def possible_static_cells(vertices, world_camera, evidence):
    """Cover a static unknown carton within the enforced workspace only.

    Workspace clipping is an obstacle query domain, NOT a measured carton end.
    A missing reliable face keeps the whole domain possible.
    """
    face=evidence.get('observed_face')
    if not face:return np.ones(len(vertices),bool)
    axes=np.column_stack([face[k] for k in ('u','v','normal')])
    local=(vertices-np.asarray(face['center_m']))@axes
    possible=local[:,:,2].min(1)<=FREE_MARGIN_M
    visibility=evidence.get('visibility_bounds') or {}
    front=visibility.get('front') or {}; depth=visibility.get('depth') or {}
    if front.get('bounded'):
        lo,hi=np.asarray(front['bounds_uv_m'])
        possible&=np.all(local[:,:,:2].max(1)>=lo,axis=1)&np.all(local[:,:,:2].min(1)<=hi,axis=1)
    if depth.get('bounded'):possible&=local[:,:,2].max(1)>=-depth['upper_m']
    return possible
