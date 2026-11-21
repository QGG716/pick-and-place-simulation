"""Task-local RGB-D collision scene in confirmed base coordinates.

Unknown voxels are occupied unless explicitly cleared by the user's field
confirmation. Neither that policy nor measured clear volumes erase observed
obstacles. A separate hash-bound user-confirmed artefact list may exclude
unclaimed returns only inside its named original volumes. No simulation world
is imported.

Near a reconstructed target, coarse grid cells are not allowed to manufacture
volume that was never measured: the target's own observed plane (including
pixels SAM missed at its border) belongs to the target envelope, mixed depth
pixels at the target's depth breaks are sensor artefacts, and remaining real
points use their measured extent instead of a whole 40 mm cell. The surface
observed directly below a target becomes one explicit support object whose
top is bound to the envelope bottom, so the existing detach-only payload/support
rule can apply; it never exempts robot or tool links.
"""
from itertools import product
import json
from pathlib import Path
import cv2
import numpy as np
from ..coordinates import rigid
from ..detection import metric_points
from .geometry import transform, obb_overlap_depth
from .. import timing

# Target-local scene rules (metres / pixels). Plane band is several times the
# recorded RANSAC RMS; the pixel band only reaches SAM border misses.
TARGET_PLANE_BAND_M=.004
TARGET_PIXEL_BAND_PX=15
ENVELOPE_CONTAINMENT_M=.002
FLYING_DEPTH_JUMP_M=.012
NEAR_TARGET_M=.06
MEASURED_EXTENT_PAD_M=.003
SUPPORT_BELOW_M=.015
SUPPORT_ABOVE_M=.004
SUPPORT_MIN_POINTS=30
SUPPORT_MIN_THICKNESS_M=.005
# A near-target run whose measured-extent box reaches a target's vicinity
# envelope is re-emitted as boxes over these sub-cells, each covering only
# its own measured points (+ pad), so one 4 cm cell's L-shaped point set no
# longer fills the empty corner that reaches into the envelope.
NEAR_TARGET_SUBCELL_M=.01
# Refinement changes representation, never the native pair clearance. Include
# separated runs too: an L-shaped cloud's bounding box can manufacture a tiny
# positive gap without intersecting the payload. Retain every point and pad.
# Every near-target run (within NEAR_TARGET_M of an envelope) is refined: the
# tool's off-face cups hang over neighbours there, and a whole world-aligned
# cell overshoots a rotated face by up to ~25 mm (field 7fc259a5: lips of the
# lower cup row against a coarse cell of the lower carton, real gap ~15 mm).
NEAR_TARGET_REFINE_GAP_M=NEAR_TARGET_M


def user_cleared_unknown(c):
    policy=c.get('unknown_policy')
    if policy=='OCCUPIED':return False
    if policy=='USER_CONFIRMED_CLEAR':
        basis=c.get('unknown_clearance_basis')
        if isinstance(basis,str) and basis.strip():return True
        raise ValueError('UNKNOWN_CLEARANCE_FIELD_BASIS_REQUIRED')
    raise ValueError('UNKNOWN_SPACE_POLICY_INVALID')


def target_envelope_points_assigned(c):
    """Whether unclaimed measured points inside a target's own envelope join the target.

    OBSTACLE (default) keeps them as world obstacles. TARGET_USER_CONFIRMED
    assigns them to the target only with a recorded field basis: the user has
    confirmed such points as false obstacles (e.g. box-edge pixels just outside
    the SAM mask). Points outside the envelope are never affected.
    """
    policy=c.get('target_envelope_points','OBSTACLE')
    if policy=='OBSTACLE':return False
    if policy=='TARGET_USER_CONFIRMED':
        basis=c.get('target_envelope_points_basis');margin=c.get('target_envelope_points_margin_m',0.)
        if not isinstance(basis,str) or not basis.strip():raise ValueError('TARGET_ENVELOPE_POINTS_FIELD_BASIS_REQUIRED')
        if not isinstance(margin,(int,float)) or not 0<=margin<=.02:raise ValueError('TARGET_ENVELOPE_POINTS_MARGIN_INVALID')
        return True
    raise ValueError('TARGET_ENVELOPE_POINTS_POLICY_INVALID')


def load_site(path):
    c=json.loads(Path(path).read_text())
    if c.get('schema')!='eco65_real_site_v1' or c.get('frame')!='base_link':
        raise ValueError('REAL_SITE_BASE_LINK_CONFIG_REQUIRED')
    lo,hi=np.asarray(c['workspace_bounds_m'],float)
    cell=float(c['voxel_m'])
    if (lo.shape!=(3,) or not np.isfinite([lo,hi]).all() or np.any(hi<=lo)
            or not .005<=cell<=.08 or np.prod(np.ceil((hi-lo)/cell))>300000):
        raise ValueError('INVALID_BOUNDED_SITE_WORKSPACE')
    user_cleared_unknown(c);target_envelope_points_assigned(c)
    from .pointcloud_artifacts import load_manifest
    load_manifest(c)
    ids=set()
    for b in c.get('fixed_obstacles',[]):
        if b['id'] in ids or not b.get('measurement_basis'):raise ValueError('SITE_OBSTACLE_ID_OR_MEASUREMENT_REQUIRED')
        ids.add(b['id']);rigid(b['pose'])
        if np.asarray(b['size_m']).shape!=(3,) or not np.isfinite(b['size_m']).all() or min(b['size_m'])<=0:
            raise ValueError('INVALID_SITE_BOX_SIZE')
    for r in c.get('measured_clear_regions',[]):
        a,b=np.asarray(r['bounds_m'],float)
        if not r.get('measurement_basis') or not np.isfinite([a,b]).all() or np.any(a>=b):
            raise ValueError('BLIND_CLEAR_VOLUME_REQUIRES_FIELD_MEASUREMENT')
    receiver=c.get('receiver')
    if receiver:
        if not receiver.get('support_ids') or not set(receiver['support_ids'])<=ids or not receiver.get('measurement_basis'):
            raise ValueError('RECEIVER_MUST_BIND_MEASURED_SITE_SUPPORT')
        a,b=np.asarray(receiver['bounds_m'],float)
        if not np.isfinite([a,b]).all() or np.any(a>=b) or not 0<=receiver.get('margin_m',.003)<.1:
            raise ValueError('INVALID_RECEIVER_BOUNDS')
    c['config_path']=str(Path(path).resolve())
    return c


def _box(ident,lo,hi,category):
    T=np.eye(4);T[:3,3]=(lo+hi)/2
    return dict(id=ident,pose=T.tolist(),size=(hi-lo).tolist(),category=category)


def _envelope_local(box,points):
    return transform(np.linalg.inv(np.asarray(box['pose'],float)),points.reshape(-1,3)).reshape(points.shape)


def _target_claims(box,mask,valid,world_points,jump,shared=None):
    """Pixels owned by one target, and mixed pixels at its depth breaks.

    Owned: inside the measured box and either in the SAM mask, on
    the target's own observed plane, or on a side plane through a visible patch
    edge, within a small image band around the mask. Other points (neighbors,
    supports, occluders) are retained.
    Flying: depth-discontinuous pixels in the band; a real obstacle keeps its
    depth-continuous interior pixels and therefore its voxel.
    """
    from .measured_envelope import measured_geometry,own_connected_surface
    envelope=box
    box=box.get('observed_side_geometry') or measured_geometry(box)
    half=np.asarray(box['size_m'],float)/2
    contained=np.all(abs(_envelope_local(box,world_points))<=half+ENVELOPE_CONTAINMENT_M,axis=-1)
    kernel=np.ones((2*TARGET_PIXEL_BAND_PX+1,)*2,np.uint8)
    band=cv2.dilate(mask.astype(np.uint8),kernel).astype(bool)
    observed=[f for f in box['faces'] if f.get('observed')]
    visibility_bound=box.get('uncertainty',{}).get('bound_method')=='RGBD_VISIBILITY_INTERVALS'
    on_plane=np.zeros(valid.shape,bool);on_side=np.zeros(valid.shape,bool)
    for face in observed:
        offset=world_points-np.asarray(face['center_m'])
        distance=offset@np.asarray(face['normal'])
        on_plane |= abs(distance)<=TARGET_PLANE_BAND_M
        if visibility_bound:continue  # A partial patch edge is not a side plane.
        # The carton's own side faces: perpendicular planes through the visible
        # patch edges, behind the face, within the other patch extent.
        coords=[offset@np.asarray(face[a]) for a in ('u','v')];half_patch=np.asarray(face['size_m'])/2
        for i in range(2):
            along=abs(coords[1-i])<=half_patch[1-i]+TARGET_PLANE_BAND_M
            on_side |= along&(distance<=TARGET_PLANE_BAND_M)&(abs(abs(coords[i])-half_patch[i])<=TARGET_PLANE_BAND_M)
    # The target's own plane may bulge a few millimetres past the fitted face
    # (cardboard) or continue past SAM's border; both stay within the band.
    loose=np.all(abs(_envelope_local(box,world_points))<=half+TARGET_PLANE_BAND_M,axis=-1)
    in_mask=valid&mask&contained
    surface=valid&band&loose&on_plane&~in_mask
    side=valid&band&contained&on_side&~in_mask&~surface
    # User confirmation 2026-11-20: own-mask returns depth-connected to the
    # measured surface are the carton itself; inside its collision envelope
    # (which conservative_envelope grew to contain them) they are its own.
    own=np.zeros(valid.shape,bool);extra=np.zeros(valid.shape,bool)
    if (envelope.get('uncertainty') or {}).get('kind')=='MEASURED_BOUNDARY_INTERVALS':
        inside=np.all(abs(_envelope_local(envelope,world_points))<=np.asarray(envelope['size_m'],float)/2+1e-9,axis=-1)
        # valid already excludes other instances for these envelopes.
        # Inside the already bounded envelope the own mask needs no depth-jump
        # cut (that cut only keeps background bleed from growing the envelope).
        # shared: pixels also labelled by another instance that carry this
        # carton's own face colour (user decisions 2026-11-20: own returns
        # depth-connected to the faces are the carton; RGB colour separates
        # the carton from its surroundings). Other-coloured pixels stay out.
        extra=np.zeros(valid.shape,bool) if shared is None else np.asarray(shared,bool)
        own=((own_connected_surface((valid|extra)&band&~jump,in_mask)|own_connected_surface((valid|extra)&mask,in_mask))
             &inside&~in_mask&~surface&~side)
    # Mixed pixels at depth breaks in the band, including those shared with
    # another instance under the colour/fattening rule above.
    flying=(valid|extra)&band&jump&~in_mask&~surface&~side&~own
    return in_mask|surface|side|own,flying,dict(mask_points_claimed=int(in_mask.sum()),
        own_connected_points_claimed=int(own.sum()),
        shared_instance_points_claimed=int((own&extra).sum()),
        border_surface_points_claimed=int(surface.sum()),side_face_points_claimed=int(side.sum()),
        flying_points_removed=int(flying.sum()))


def stack_seam_band_claims(boxes,stack_evidence,masks,available,world_points,evidence):
    """Returns of the seam band owned by the upper carton of a confirmed stack.

    Owned: valid, not yet claimed, inside the upper collision envelope (whose
    bottom is the stack support plane), at or below the upper measured bottom
    plus the support window, within the image band around the upper mask, and
    labelled by the upper or the lower instance of that stack or by no
    instance. Everything below the plane stays with the lower carton (its
    support body or ordinary obstacles); other instances are never claimed.
    """
    owned=np.zeros(available.shape,bool)
    labelled=np.zeros(available.shape,bool)
    for mask in masks:labelled|=np.asarray(mask,bool)
    for record in stack_evidence:
        rule=record.get('seam_band_ownership')
        if record.get('status')!='APPLIED' or not rule:continue
        box=next((b for b in boxes if b['id']==record['upper_id']),None)
        if box is None:continue
        upper=np.asarray(masks[rule['upper_index']],bool);lower=np.asarray(masks[rule['lower_index']],bool)
        band=cv2.dilate(upper.astype(np.uint8),np.ones((2*rule['image_band_px']+1,)*2,np.uint8)).astype(bool)
        half=np.asarray(box['size_m'],float)/2
        inside=np.all(abs(_envelope_local(box,world_points))<=half+1e-9,axis=-1)
        height=world_points@np.asarray(rule['normal_world'],float)
        seam=(available&band&inside&(height>=rule['plane_m'])&(height<=rule['band_top_m'])&
              (upper|lower|~labelled)&~owned)
        counts=dict(upper_instance=int((seam&upper).sum()),lower_instance=int((seam&lower&~upper).sum()),
                    unlabelled=int((seam&~labelled).sum()))
        record['seam_band_points_assigned_to_upper']=counts
        for item in evidence:
            if item.get('target_id')==box['id']:item['stack_seam_band_points_claimed']=counts
        owned|=seam
    return owned


def stack_pixels(masks,rule,labelled):
    """Upper-instance or unlabelled pixels within the upper mask's band."""
    upper=np.asarray(masks[rule['upper_index']],bool)
    near=cv2.dilate(upper.astype(np.uint8),np.ones((2*rule['image_band_px']+1,)*2,np.uint8)).astype(bool)
    return (upper|~labelled)&near


def _interface_support_pieces(body,record,boxes,observed,available,stack):
    """Stack returns directly under the upper envelope, at most SUPPORT_BELOW_M
    below the support plane, that the complete lower body does not enclose
    (its front is one fitted plane). Below the upper visible face is the lower
    (user decision 2026-11-20), so each 10 mm group becomes a support piece of
    the same confirmed support: oriented with the plane, top exactly on it,
    other sides at the measured extent pad. Robot and tool keep colliding
    with it; only the attached payload at detach uses the support rule."""
    rule=record['seam_band_ownership'];n=np.asarray(rule['normal_world'],float);d=float(rule['plane_m'])
    upper=next((b for b in boxes if b['id']==record['upper_id']),None)
    if upper is None or not len(observed):return []
    U=np.asarray(upper['pose'],float);half=np.asarray(upper['size_m'],float)/2
    vertical=int(np.argmax(abs(U[:3,:3].T@n)));lateral=[i for i in range(3) if i!=vertical]
    under=np.all(abs(((observed-U[:3,3])@U[:3,:3])[:,lateral])<=half[lateral],axis=1)
    below=d-observed@n
    chosen=np.flatnonzero(available&stack&under&(below>=0)&(below<=SUPPORT_BELOW_M))
    if not len(chosen):return []
    F=np.asarray(body['pose'],float)[:3,:3]            # x, y in the plane, z = normal
    local=observed[chosen]@F
    groups=np.floor(local[:,:2]/NEAR_TARGET_SUBCELL_M).astype(int)
    _,which=np.unique(groups,axis=0,return_inverse=True);which=which.ravel();pieces=[]
    for k in range(which.max()+1):
        member=chosen[which==k];pts=local[which==k]
        lo=pts.min(0)-MEASURED_EXTENT_PAD_M;hi=pts.max(0)+MEASURED_EXTENT_PAD_M;hi[2]=d
        lo[2]=min(lo[2],d-SUPPORT_MIN_THICKNESS_M)
        pose=np.eye(4);pose[:3,:3]=F;pose[:3,3]=F@((lo+hi)/2)
        piece=dict(id=f"{body['id']}_interface_{k}",category='confirmed_stack_support',pose=pose.tolist(),
            size=(hi-lo).tolist(),supports_target_id=body['supports_target_id'],detection_index=body['detection_index'],
            geometry_uncertain=True,support_plane=body['support_plane'],support_basis=body['support_basis'],
            observation_id=body['observation_id'],interface_piece_of=body['id'])
        mask=np.zeros(len(observed),bool);mask[member]=True;pieces.append((piece,mask))
    return pieces


def _cap_below_stack_plane(member,low,high,planes):
    """Top of a refined observed box whose points all lie under a confirmed
    stack support plane: its extent pad stops at that plane (where the upper
    payload rests), never below its own highest point. Returns the new z top."""
    top=float(high[2])
    for n,d,footprint in planes:
        if not np.all(member@n<=d):continue
        corners=np.array([[x,y] for x in (low[0],high[0]) for y in (low[1],high[1])])
        if not np.any(np.all((corners>=footprint[0])&(corners<=footprint[1]),axis=1)):continue
        plane_z=min((d-n[0]*x-n[1]*y)/n[2] for x,y in corners)
        top=min(top,max(plane_z,float(member[:,2].max())))
    return top


def _support_below(box,observed):
    """Observed surface directly under a target with a horizontal bottom.

    Returns (object, selected_point_mask, evidence) or (None, None, evidence).
    Its top equals the envelope bottom exactly so the native detach rule binds it.
    """
    T=np.asarray(box['pose'],float);R=T[:3,:3];half=np.asarray(box['size_m'],float)/2
    vertical=int(np.argmax(abs(R[2])))
    if abs(R[2,vertical])<np.cos(np.deg2rad(10)) or not len(observed):
        return None,None,dict(support='NONE_NO_HORIZONTAL_BOTTOM_OR_POINTS')
    # Same bottom value the planner uses to bind detach supports.
    bottom=float(np.asarray(box['corners_m'],float)[:,2].min())
    horizontal=[i for i in range(3) if i!=vertical]
    local=(observed-T[:3,3])@R
    near=np.all(abs(local[:,horizontal])<=half[horizontal]+NEAR_TARGET_M,axis=1)
    height=observed[:,2]
    # A plane measured during reconstruction (which already bounded the
    # envelope bottom) sets the window; its top noise band is absorbed.
    plane=(box.get('quality') or {}).get('observed_support_plane')
    level=plane['top_world_m'] if plane else bottom
    above=max(SUPPORT_ABOVE_M,4*plane['noise_m']) if plane else SUPPORT_ABOVE_M
    selected=near&(height>=level-SUPPORT_BELOW_M)&(height<=level+above)
    if selected.sum()<SUPPORT_MIN_POINTS:
        return None,None,dict(support='NONE_INSUFFICIENT_SUPPORT_POINTS',support_points=int(selected.sum()))
    axis=R[:,horizontal[0]].copy();axis[2]=0;axis/=np.linalg.norm(axis)
    frame=np.column_stack((axis,np.cross([0.,0.,1.],axis),[0.,0.,1.]))
    planar=(observed[selected]-T[:3,3])@frame
    lo=planar[:,:2].min(0)-MEASURED_EXTENT_PAD_M;hi=planar[:,:2].max(0)+MEASURED_EXTENT_PAD_M
    lowest=float(height[selected].min())-MEASURED_EXTENT_PAD_M
    thickness=max(bottom-lowest,SUPPORT_MIN_THICKNESS_M)
    pose=np.eye(4);pose[:3,:3]=frame
    pose[:3,3]=T[:3,3]+frame[:,:2]@((lo+hi)/2);pose[2,3]=bottom-thickness/2
    obj=dict(id='rgbd_support_'+box['id'],pose=pose.tolist(),size=[*(hi-lo).tolist(),thickness],category='observed_support')
    return obj,selected,dict(support=obj['id'],support_points=int(selected.sum()),
        points_above_envelope_bottom_clipped=int((height[selected]>bottom).sum()),
        top_world_m=bottom,measured_plane_top_world_m=plane['top_world_m'] if plane else None,
        support_top_below_measured_plane_m=float(plane['top_world_m']-bottom) if plane else None,
        envelope_bottom_tilt_rad=plane.get('tilt_rad') if plane else None,
        basis='OBSERVED_RGBD_SURFACE_BELOW_TARGET_TOP_BOUND_TO_ENVELOPE_BOTTOM')


def _project(points,k):
    from ..reference.camera_geometry import ProjectionInfo,opencv_brown_coefficients
    info=ProjectionInfo('realsense',k['width'],k['height'],k['fx'],k['fy'],k['ppx'],k['ppy'],
                        distortion=tuple(k['coeffs']),distortion_model=k['model'])
    coeff=(0.,)*5 if k['model']=='SYNTHETIC_PINHOLE' else opencv_brown_coefficients(info)
    K=np.array([[k['fx'],0,k['ppx']],[0,k['fy'],k['ppy']],[0,0,1.]])
    uv,_=cv2.projectPoints(points,np.zeros(3),np.zeros(3),K,np.asarray(coeff))
    return uv.reshape(-1,2)


def build_scene(observation,result,boxes,config,self_filter=None,neighbor_depth_m=None,unresolved_cartons=(),geometry=None):
    lap=timing.Laps('scene.')  # measurement only; no-op unless ECO65_TIMING=1
    cleared_unknown=user_cleared_unknown(config)
    frame=observation.frame
    if result.observation_id!=frame.observation_id:raise ValueError('SCENE_FRAME_MISMATCH')
    lo,hi=np.asarray(config['workspace_bounds_m'],float);cell=config['voxel_m']
    shape=np.ceil((hi-lo)/cell).astype(int);axes=[lo[i]+(np.arange(shape[i])+.5)*cell for i in range(3)]
    centers=np.stack(np.meshgrid(*axes,indexing='ij'),-1).reshape(-1,3)
    W=rigid(observation.world_camera);C=np.linalg.inv(W)
    offsets=np.asarray(list(product((-.5,.5),repeat=3)))*cell
    vertices=centers[:,None,:]+offsets
    camera=transform(C,vertices.reshape(-1,3)).reshape(-1,8,3)
    uv=_project(camera.reshape(-1,3),frame.intrinsics).reshape(-1,8,2)
    lap('voxel_corner_projection',voxels=len(centers))
    # Full projected voxel footprint must have valid, continuous farther depth.
    # Check every pixel in its bounding rectangle via integral min queries;
    # using only 8 corners would miss an intervening obstacle or depth hole.
    h,w=frame.depth.shape;depth=frame.depth.astype(float)*frame.depth_scale_m
    bounds=np.c_[np.floor(uv.min(1)),np.ceil(uv.max(1))].astype(int)
    free=np.zeros(len(centers),bool)
    in_view=(camera[:,:,2].min(1)>.1)&(bounds[:,0]>=0)&(bounds[:,1]>=0)&(bounds[:,2]<w)&(bounds[:,3]<h)
    for i in np.flatnonzero(in_view):
        x0,y0,x1,y1=bounds[i];patch=depth[y0:y1+1,x0:x1+1]
        free[i]=bool(patch.size and patch.min()>camera[i,:,2].max()+.006)
    lap('free_space_voxel_loop',voxels_in_view=int(in_view.sum()))
    sensor_free=free.copy()
    for region in config.get('measured_clear_regions',[]):
        a,b=np.asarray(region['bounds_m'])
        free |= np.all(centers-cell/2>=a,axis=1)&np.all(centers+cell/2<=b,axis=1)
    # Apart from the separately recorded named artefact exception below, every
    # valid depth not owned by a reconstructed target remains an obstacle,
    # including unreconstructed detections and unlabelled tables/floors/objects.
    # A failed/unexplained depth outside the full envelope stays an obstacle even
    # if SAM labelled it as the target; off-plane depth inside it also stays.
    lap('measured_clear_regions')
    points=metric_points(frame);valid=np.isfinite(points).all(-1)&(depth>.1)&(depth<3.)
    world_points=transform(W,points.reshape(-1,3)).reshape(points.shape)
    outside_ignored=[]
    for i,mask in enumerate(result.masks):
        selected=np.asarray(mask,bool)&valid
        inside=np.all((world_points>=lo)&(world_points<=hi),axis=-1)
        if selected.any() and not (selected&inside).any():
            outside_ignored.append(i);valid&=~np.asarray(mask,bool)
    z=np.where(valid,depth,0.).astype(np.float32);square=np.ones((3,3),np.uint8)
    jump=(cv2.dilate(z,square)-cv2.erode(z,square))>FLYING_DEPTH_JUMP_M
    lap('points_and_depth_jumps')
    # A confirmed stack sets the upper envelope bottom (in place) before any
    # point is claimed, so claims, supports and the planner use one bottom.
    from .stack_support import infer_supports
    stack_bodies,stack_evidence=infer_supports(observation,result,boxes,unresolved_cartons,config,geometry,
        world_points,valid,outside_ignored)
    claimed=np.zeros(valid.shape,bool);flying=np.zeros(valid.shape,bool);evidence=[]
    for box in boxes:
        claim_valid=valid;shared=None
        # A visibility envelope may enclose an occluder. Only this instance's
        # observed plane/mask owns its pixels, never other segmented instances
        # (except carton-coloured, depth-connected own surface, see below).
        if box.get('uncertainty'):
            others=np.zeros(valid.shape,bool)
            for index,mask in enumerate(result.masks):
                if index!=box['detection_index']:others|=np.asarray(mask,bool)
            claim_valid=valid&~others
            if (box.get('uncertainty') or {}).get('kind')=='MEASURED_BOUNDARY_INTERVALS':
                from .rgb_color_boundary import carton_coloured
                from .stereo_shadow import MAX_FATTENING_PX
                own_mask=np.asarray(result.masks[box['detection_index']],bool)
                colour=carton_coloured(frame.rgb,own_mask)
                # Stereo fattening: another instance's pixels within 6 px of
                # this mask carrying this carton's continuous depth (field
                # 9b51a37e: 344 black-object pixels 1-5 px off the side).
                near=cv2.distanceTransform((~own_mask).astype(np.uint8),cv2.DIST_L2,5)<=MAX_FATTENING_PX
                shared=valid&others&(near|(colour if colour is not None else False))
        owned,mixed,counts=_target_claims(box,np.asarray(result.masks[box['detection_index']],bool),claim_valid,world_points,jump,shared)
        claimed|=owned;flying|=mixed;evidence.append(dict(target_id=box['id'],**counts))
    # User decision 2026-11-20: in a confirmed stack the upper carton's
    # visible face decides the seam band (stack_support.infer_supports).
    claimed|=stack_seam_band_claims(boxes,stack_evidence,result.masks,valid&~claimed,world_points,evidence)
    flying&=~claimed
    assign_envelope=target_envelope_points_assigned(config)
    if assign_envelope:
        # User-confirmed: what is left inside a target's measured box is
        # that target (edge pixels SAM missed), not a separate world solid.
        margin=float(config.get('target_envelope_points_margin_m',0.))
        for box,record in zip(boxes,evidence):
            if box.get('uncertainty',{}).get('bound_method')=='RGBD_VISIBILITY_INTERVALS':
                record['envelope_points_assigned_user_confirmed']=0
                record['envelope_point_assignment']='NOT_APPLICABLE_TO_SIZE_FREE_POSSIBLE_VOLUME'
                continue
            from .measured_envelope import measured_geometry
            other_masks=np.zeros(valid.shape,bool)
            for index,mask in enumerate(result.masks):
                if index!=box['detection_index']:other_masks|=np.asarray(mask,bool)
            box=measured_geometry(box)
            half=np.asarray(box['size_m'],float)/2+margin
            # The margin never reaches below the envelope bottom: the surface the
            # target stands on stays a measured support, never part of the target.
            bottom=float(np.asarray(box['corners_m'],float)[:,2].min())
            inside=(valid&~claimed&~flying&~other_masks&(world_points[...,2]>=bottom)&
                    np.all(abs(_envelope_local(box,world_points))<=half,axis=-1))
            claimed|=inside;record['envelope_points_assigned_user_confirmed']=int(inside.sum())
    lap('target_point_claims',targets=len(boxes))
    from .pointcloud_artifacts import exclude_upper_returns
    eligible=valid&~claimed&~flying
    protected_support=np.zeros(valid.shape,bool)
    if config.get('upper_carton_pointcloud_artifacts',{}).get('mode','KEEP')!='KEEP':
        support_pixels=np.flatnonzero(eligible);support_points=world_points[eligible]
        for box in boxes:
            _,selected,_=_support_below(box,support_points)
            if selected is not None:protected_support.flat[support_pixels[selected]]=True
    ignored_artifacts,artifact_evidence=exclude_upper_returns(world_points,eligible,
        result.masks,boxes,unresolved_cartons,config,support_above_m=SUPPORT_ABOVE_M,
        protected_support=protected_support)
    artifact_evidence['observation_id']=frame.observation_id
    observed_pixels=np.flatnonzero(valid&~claimed&~flying&~ignored_artifacts)
    observed=world_points.reshape(-1,3)[observed_pixels]
    removed_self=0;owned_cells=np.zeros(len(centers),bool)
    if self_filter is not None:
        owned=self_filter.contains(observed,observation.q_capture)
        removed_self=int(owned.sum());observed=observed[~owned];observed_pixels=observed_pixels[~owned]
        lap('self_filter_points',points=len(owned))
        # A voxel is owned only when ALL corners are inside actual robot solids.
        # Partial/uncertain cells remain occupied unless field evidence covers them.
        owned_cells=self_filter.contains(vertices.reshape(-1,3),observation.q_capture,group_size=8)
        free |= owned_cells
        lap('self_filter_voxel_corners',corners=int(vertices.reshape(-1,3).shape[0]))
    for fixed in config.get('fixed_obstacles',[]):
        local=transform(np.linalg.inv(fixed['pose']),observed)
        keep=~np.all(abs(local)<=np.asarray(fixed['size_m'])/2+1e-9,axis=1)
        observed=observed[keep];observed_pixels=observed_pixels[keep]
    indices=np.floor((observed-lo)/cell).astype(int)
    inside=np.all(indices>=0,axis=1)&np.all(indices<shape,axis=1)
    observed=observed[inside];indices=indices[inside];observed_pixels=observed_pixels[inside]
    occupancy=(~free).reshape(tuple(shape));measured=np.zeros_like(occupancy)
    if len(indices):measured[tuple(indices.T)]=True
    occupancy |= measured
    # User-confirmed blind clearance affects ONLY unobserved cells. Real depth
    # obstacles, fixed measurements, target envelopes and boundaries remain.
    unknown=occupancy & ~measured
    # A detected but unreconstructed neighbour is evidence of a carton, so the
    # volume hidden behind its visible surface is not "unobserved clear space".
    # Under user-confirmed clearance, keep (full cells) each of its unclaimed
    # depth rays extended by one conservative carton depth. Reconstructed target
    # envelopes, robot-owned cells and sensor-proven free cells are excluded.
    lap('fixed_obstacles_indexing_occupancy')
    shadow=np.zeros(tuple(shape),bool);unreconstructed=[];extrusion_records=[]
    if neighbor_depth_m:
        reconstructed={b['detection_index'] for b in boxes}|set(stack_bodies)
        unreconstructed=[i for i in range(len(result.masks)) if i not in reconstructed and i not in outside_ignored]
        pick=np.zeros(valid.shape,bool)
        filtered_surfaces=[]
        for i in unreconstructed:
            mask=np.asarray(result.masks[i],bool);pick|=mask
            raw=mask&valid;selected=raw&~claimed&~flying&~jump
            surface=points[selected]
            if self_filter is not None and len(surface):
                surface=surface[~self_filter.contains(transform(W,surface),observation.q_capture)]
            status=('NEIGHBOR_SHADOW_REPRESENTED' if cleared_unknown else 'OCCUPIED_UNKNOWN_REPRESENTED')
            if not len(surface):status='ALL_SURFACE_POINTS_OWNED_OR_FLYING' if raw.any() else 'NO_VALID_SURFACE_UNMODELED'
            extrusion_records.append(dict(detection_index=i,status=status,valid_surface_points=int(raw.sum()),
                filtered_surface_points=len(surface),depth_m=float(neighbor_depth_m),
                direction='CAMERA_OPTICAL_RAY_AWAY_FROM_CAMERA',user_clearance_applied=False,
                basis='USER_2026_11_19_MAX_CARTON_EXTENT_STATIC_NEIGHBOR_SHADOW_NOT_PAYLOAD'))
            filtered_surfaces.append(surface)
        surface=np.concatenate(filtered_surfaces) if filtered_surfaces else np.empty((0,3))
        if len(surface):
            rays=surface/np.linalg.norm(surface,axis=1,keepdims=True)
            steps=np.arange(cell/2,float(neighbor_depth_m)+cell/2,cell/2)
            samples=transform(W,(surface[:,None,:]+rays[:,None,:]*steps[None,:,None]).reshape(-1,3))
            for box in boxes:
                half=np.asarray(box['size_m'],float)/2
                samples=samples[~np.all(abs(transform(np.linalg.inv(np.asarray(box['pose'],float)),samples))<=half,axis=1)]
            cells_hit=np.floor((samples-lo)/cell).astype(int)
            cells_hit=cells_hit[np.all(cells_hit>=0,axis=1)&np.all(cells_hit<shape,axis=1)]
            if len(cells_hit):shadow[tuple(cells_hit.T)]=True
        shadow &= (~sensor_free&~owned_cells).reshape(tuple(shape))&~measured
        if not cleared_unknown:
            unknown|=shadow  # explicit neighbor evidence overrides clear regions
            shadow[:]=False
    lap('neighbor_shadow')
    # A detected carton is not covered by the user's unknown-space clearance.
    # Its possible body remains an obstacle even when its exact dimensions are
    # unknown. Clip only this STATIC occupancy query to the enforced workspace;
    # never reuse the clipped volume as a payload size or an attachment.
    unresolved_grids=[];unresolved_records=[]
    if unresolved_cartons:
        from .visibility_geometry import possible_static_cells
        for item in unresolved_cartons:
            if item['detection_index'] in outside_ignored:continue
            index=item['detection_index']
            if index in stack_bodies:
                body,_=stack_bodies[index]
                local=(vertices-np.asarray(body['pose'])[:3,3])@np.asarray(body['pose'])[:3,:3]
                half=np.asarray(body['size'])/2
                possible=np.all(local.max(1)>=-half,axis=1)&np.all(local.min(1)<=half,axis=1)
                # This grid accounts for occupied volume but is not emitted:
                # one oriented full-volume box preserves the exact top plane.
                unresolved_grids.append((f'accounted_stack_{index}',possible.reshape(tuple(shape))))
                unresolved_records.append(dict(detection_index=index,possible_voxels=int(possible.sum()),
                    object_id=body['id'],representation_complete=True,
                    scope='STATIC_OBSTACLE_INTERSECTION_WITH_ENFORCED_WORKSPACE_NOT_PAYLOAD',
                    sensor_free_cells_excluded=False,user_clearance_applied=False,
                    carton_end_at_workspace_boundary_assumed=False,reliable_face=True))
                continue
            if neighbor_depth_m:continue
            possible=possible_static_cells(vertices,W,item.get('evidence') or {})&~sensor_free
            grid=possible.reshape(tuple(shape));index=item['detection_index']
            unresolved_grids.append((f'carton_possible_{index}',grid))
            unresolved_records.append(dict(detection_index=index,possible_voxels=int(possible.sum()),
                object_id_prefix=f'rgbd_carton_possible_{index}_',
                scope='STATIC_OBSTACLE_INTERSECTION_WITH_ENFORCED_WORKSPACE_NOT_PAYLOAD',
                sensor_free_cells_excluded=True,user_clearance_applied=False,
                carton_end_at_workspace_boundary_assumed=False,
                reliable_face=bool((item.get('evidence') or {}).get('observed_face'))))
    world=[dict(id=b['id'],pose=b['pose'],size=b['size_m'],category='measured_fixed') for b in config.get('fixed_obstacles',[])]
    world.extend(body for body,_ in stack_bodies.values())
    # Supports are explicit objects; their points leave the voxel cloud but the
    # cells stay measured, so they never turn into unknown space.
    represented=np.zeros(len(observed),bool);supports=[]
    labelled=np.zeros(valid.shape,bool)
    for mask in result.masks:labelled|=np.asarray(mask,bool)
    for index,(body,record) in stack_bodies.items():
        # Returns already enclosed by the lower's complete static body (below
        # the support plane) are represented by it: the lower instance, and
        # (user decision 2026-11-20: below the upper visible face is the lower)
        # the upper instance's and unlabelled returns there. Other detected
        # instances and points outside that body remain ordinary obstacles.
        # Keep measured occupancy above; no represented cell becomes free.
        rule=record.get('seam_band_ownership') or {}
        local=transform(np.linalg.inv(body['pose']),observed)
        inside=np.all(abs(local)<=np.asarray(body['size'])/2+1e-9,axis=1)
        selected=np.asarray(result.masks[index],bool).ravel()[observed_pixels]&inside
        if 'upper_index' in rule:
            # Only the interface band: <= SUPPORT_BELOW_M under the plane and
            # within the upper mask's pixel band. Deeper unlabelled objects
            # keep their own collider.
            upper=np.asarray(result.masks[rule['upper_index']],bool)
            near=cv2.dilate(upper.astype(np.uint8),np.ones((2*rule['image_band_px']+1,)*2,np.uint8)).astype(bool)
            stack=((upper|~labelled)&near).ravel()[observed_pixels]
            depth_below=rule['plane_m']-observed@np.asarray(rule['normal_world'],float)
            band=stack&inside&(depth_below<=SUPPORT_BELOW_M)&~selected
            record['interface_band_points_represented']=dict(count=int(band.sum()),
                labels='UPPER_INSTANCE_OR_UNLABELLED',max_below_plane_m=SUPPORT_BELOW_M,
                basis=rule.get('basis'))
            represented|=band
        represented|=selected;record['observed_instance_points_represented']=int(selected.sum())
        supports.append(body['id'])
        if 'upper_index' in rule:
            pieces=_interface_support_pieces(body,record,boxes,observed,~represented,
                (np.asarray(result.masks[index],bool)|stack_pixels(result.masks,rule,labelled)).ravel()[observed_pixels])
            for piece,members in pieces:
                world.append(piece);supports.append(piece['id']);represented|=members
            record['interface_support_pieces']=dict(count=len(pieces),points=int(sum(m.sum() for _,m in pieces)),
                rule='STACK_RETURNS_UNDER_THE_UPPER_ENVELOPE_WITHIN_SUPPORT_BELOW_M_TOP_AT_PLANE',basis=rule.get('basis'))
    for box,record in zip(boxes,evidence):
        visibility_bound=box.get('uncertainty',{}).get('bound_method')=='RGBD_VISIBILITY_INTERVALS'
        support,selected,info=(_support_below(box,observed[~represented]) if len(observed) and not visibility_bound
            else (None,None,dict(support='NONE_NO_MEASURED_BOTTOM' if visibility_bound else 'NONE_NO_POINTS')))
        record.update(info)
        if support is not None:
            keep=np.flatnonzero(~represented);represented[keep[selected]]=True
            world.append(support);supports.append(support['id'])
    lap('observed_supports')
    near=np.zeros(len(observed),bool)
    for box in boxes:
        half=np.asarray(box['size_m'],float)/2
        near|=np.all(abs(transform(np.linalg.inv(np.asarray(box['pose'],float)),observed))<=half+NEAR_TARGET_M,axis=1)
    cells=np.prod(shape);linear=np.ravel_multi_index(indices.T,tuple(shape)) if len(indices) else np.zeros(0,int)
    rest=~represented;emit=np.zeros(cells,bool);emit[linear[rest]]=True
    tight=np.zeros(cells,bool);tight[linear[rest&near]]=True
    low=np.full((cells,3),np.inf);high=np.full((cells,3),-np.inf)
    np.minimum.at(low,linear[rest],observed[rest]);np.maximum.at(high,linear[rest],observed[rest])
    # Points of every near-target cell, for sub-cell refinement below.
    cell_points={}
    if len(linear):
        chosen=np.flatnonzero(rest&tight[linear])
        chosen=chosen[np.argsort(linear[chosen],kind='stable')]
        keys,starts=np.unique(linear[chosen],return_index=True)
        for key,group in zip(keys,np.split(chosen,starts[1:])):cell_points[int(key)]=observed[group]
    envelopes=[(np.asarray(b['pose'],float),np.asarray(b['size_m'],float)/2) for b in boxes]
    # Confirmed stack planes with the upper envelope's xy footprint (for pads).
    stack_planes=[]
    for _,record in stack_bodies.values():
        upper_box=next((b for b in boxes if b['id']==record.get('upper_id')),None)
        if upper_box is None:continue
        c=np.asarray(upper_box['corners_m'],float)
        stack_planes.append((np.asarray(record['plane']['normal_world'],float),float(record['plane']['offset_m']),
                             (c[:,:2].min(0)-NEAR_TARGET_M,c[:,:2].max(0)+NEAR_TARGET_M)))
    from .convex_clip import box_solid,subtract_solids,mesh
    shadow_owned=[box_solid(T,2*h) for T,h in envelopes]
    if self_filter is not None and hasattr(self_filter,'world_solids'):
        shadow_owned+=self_filter.world_solids(observation.q_capture)
    shadow_clipped_pieces=0
    refined_runs=0;refined_boxes=0
    emit=emit.reshape(tuple(shape));tight=tight.reshape(tuple(shape))
    low=low.reshape(*shape,3);high=high.reshape(*shape,3);tight_runs=0;camera_side_runs=0;camera_origin=W[:3,3]
    lap('cell_extents_and_near_target_points')
    grids=[('observed',emit)]
    if not cleared_unknown:grids.append(('unknown',unknown))
    else:grids.append(('neighbor_shadow',shadow))
    grids.extend((kind,grid) for kind,grid in unresolved_grids if not kind.startswith('accounted_stack_'))
    for ix in range(shape[0]):
        for iy in range(shape[1]):
            # Merge consecutive occupied cells along Z with identical ownership.
            for kind,grid in grids:
                row=grid[ix,iy];edges=np.diff(np.r_[False,row,False].astype(int))
                for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
                    lower=lo+np.array([ix,iy,a])*cell;upper=lo+np.array([ix+1,iy+1,b])*cell
                    if kind=='observed' and tight[ix,iy,a:b].any():
                        # Near a target: measured extent only, never the whole cell.
                        lower=np.maximum(lower,low[ix,iy,a:b].min(0)-MEASURED_EXTENT_PAD_M)
                        upper=np.minimum(upper,high[ix,iy,a:b].max(0)+MEASURED_EXTENT_PAD_M);tight_runs+=1
                        run=np.eye(4);run[:3,3]=(lower+upper)/2
                        if any(obb_overlap_depth(run,(upper-lower)/2,T,h)>=-NEAR_TARGET_REFINE_GAP_M for T,h in envelopes):
                            pts=np.concatenate([cell_points[k] for k in (np.ravel_multi_index((ix,iy,z),tuple(shape)) for z in range(a,b)) if k in cell_points])
                            groups=np.floor((pts-lower)/NEAR_TARGET_SUBCELL_M).astype(int)
                            _,which=np.unique(groups,axis=0,return_inverse=True);which=which.ravel()
                            for j in range(which.max()+1):
                                member=pts[which==j]
                                top=np.minimum(upper,member.max(0)+MEASURED_EXTENT_PAD_M)
                                top[2]=_cap_below_stack_plane(member,np.maximum(lower,member.min(0)-MEASURED_EXTENT_PAD_M),top,stack_planes)
                                world.append(_box(f'rgbd_{kind}_{ix}_{iy}_{a}_s{j}',np.maximum(lower,member.min(0)-MEASURED_EXTENT_PAD_M),
                                                  top,kind))
                            refined_runs+=1;refined_boxes+=int(which.max())+1
                            continue
                    elif kind=='observed' and cleared_unknown:
                        # Camera-facing sides only: between the camera and the
                        # first measured return lies ray-traversed space or space
                        # the user confirmed clear; no surface was measured there.
                        # The far side (object interior) keeps the whole cell.
                        near_side=low[ix,iy,a:b].min(0)-MEASURED_EXTENT_PAD_M;far_side=high[ix,iy,a:b].max(0)+MEASURED_EXTENT_PAD_M
                        before=(lower.copy(),upper.copy())
                        lower=np.where(camera_origin<lower,np.maximum(lower,near_side),lower)
                        upper=np.where(camera_origin>upper,np.minimum(upper,far_side),upper)
                        camera_side_runs+=int(np.any(lower!=before[0]) or np.any(upper!=before[1]))
                    category='unresolved_carton_volume' if kind.startswith('carton_possible_') else kind
                    obj=_box(f'rgbd_{kind}_{ix}_{iy}_{a}',lower,upper,category)
                    if kind=='neighbor_shadow' and shadow_owned:
                        vertices=np.array(list(product(*zip(lower,upper))))
                        pieces=subtract_solids(vertices,shadow_owned)
                        if len(pieces)==1 and np.array_equal(pieces[0],vertices):world.append(obj)
                        else:
                            for k,part in enumerate(pieces):
                                world.append(dict(id=obj['id']+f'_clip{k}',pose=np.eye(4).tolist(),
                                    convex_mesh=mesh(part),category=kind,support_contact_qualified=False))
                            shadow_clipped_pieces+=len(pieces)
                        continue
                    if category=='unresolved_carton_volume':
                        obj.update(geometry_uncertain=True,detection_index=int(kind.removeprefix('carton_possible_')),
                            support_contact_qualified=False)
                    world.append(obj)
    lap('voxel_column_merge',objects=len(world))
    # Finite boundary walls keep paths inside the region actually classified.
    for axis in range(3):
        for sign in (0,1):
            a=lo.copy();b=hi.copy();a[axis]=(lo if sign==0 else hi)[axis]-.04*(sign==0);b[axis]=a[axis]+.04
            world.append(_box(f'site_boundary_{axis}_{sign}',a,b,'workspace_boundary'))
    lap('workspace_boundaries')
    possible_union=np.zeros(tuple(shape),bool)
    for _,grid in unresolved_grids:possible_union|=grid
    return dict(schema='eco65_real_rgbd_scene_v1',frame='base_link',world=world,
        observation_id=frame.observation_id,q_capture=list(observation.q_capture),T_base_camera=W.tolist(),
        unknown_policy=config['unknown_policy'],unknown_voxels=int(unknown.sum()),
        unknown_collision_voxels=0 if cleared_unknown else int(unknown.sum()),
        user_confirmed_clear_unknown_voxels=int((unknown&~shadow&~possible_union).sum()) if cleared_unknown else 0,
        unreconstructed_detections=unreconstructed,neighbor_shadow_voxels=int(shadow.sum()),
        outside_workspace_ignored=outside_ignored,unresolved_neighbor_extrusion=extrusion_records,
        neighbor_shadow_clipped_pieces=shadow_clipped_pieces,
        unresolved_carton_obstacles=unresolved_records,
        confirmed_stack_support=stack_evidence,
        neighbor_shadow_depth_m=float(neighbor_depth_m) if cleared_unknown and neighbor_depth_m else None,
        target_envelope_points=config.get('target_envelope_points','OBSTACLE'),
        target_envelope_points_basis=config.get('target_envelope_points_basis') if assign_envelope else None,
        target_envelope_points_margin_m=float(config.get('target_envelope_points_margin_m',0.)) if assign_envelope else None,
        target_envelope_points_evidence='USER_FIELD_CONFIRMATION_FALSE_OBSTACLES_NOT_SENSOR_OBSERVATION' if assign_envelope else None,
        unknown_clearance_basis=config.get('unknown_clearance_basis') if cleared_unknown else None,
        unknown_clearance_evidence='USER_FIELD_CONFIRMATION_NOT_SENSOR_OBSERVATION' if cleared_unknown else None,
        observed_voxels=int(measured.sum()),target_local_scene=evidence,observed_supports=supports,
        upper_carton_pointcloud_artifacts=artifact_evidence,
        near_target_measured_extent_runs=tight_runs,near_target_refined_runs=refined_runs,near_target_refined_boxes=refined_boxes,
        near_target_subcell_m=NEAR_TARGET_SUBCELL_M,near_target_refine_gap_m=NEAR_TARGET_REFINE_GAP_M,
        camera_facing_measured_extent_runs=camera_side_runs,
        camera_facing_rule='USER_CONFIRMED_CLEAR_ONLY_CAMERA_SIDE_TO_MEASURED_EXTENT_FAR_SIDE_FULL_CELL' if cleared_unknown else 'NOT_APPLIED_OCCUPIED_POLICY_FULL_CELLS',target_local_rules=dict(plane_band_m=TARGET_PLANE_BAND_M,
            pixel_band_px=TARGET_PIXEL_BAND_PX,flying_depth_jump_m=FLYING_DEPTH_JUMP_M,near_target_m=NEAR_TARGET_M,
            measured_extent_pad_m=MEASURED_EXTENT_PAD_M,support_window_m=[-SUPPORT_BELOW_M,SUPPORT_ABOVE_M]),
        measured_clear_regions=config.get('measured_clear_regions',[]),
        self_returns_removed=removed_self,self_filter_pose='q_capture',
        imported_simulation_objects=0,hardware_execution='NOT_RUN')
