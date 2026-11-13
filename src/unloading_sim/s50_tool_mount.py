"""S50-only direct mount of the existing Wantai gripper backplate.

The shared FANUC assets, connector extension and historical snapshots are not
modified. Stable source body IDs survive removal of the three bracket plates.
"""
from copy import deepcopy
from pathlib import Path
import json,hashlib
import numpy as np
from .workcell_layout import canonical_digest

DIRECT_PROFILE='S50_DIRECT_TO_GRIPPER_BACKPLATE'
SHORT_PROFILE='S50_SHORT_RIGID_MOUNT_TO_BACKPLATE'
CONNECTOR_IDS=('tool_rigid_11','tool_rigid_12','tool_rigid_13')


def bind_s50_tool_mount(frozen,settings,root):
    profile=settings.get('tool_mount_profile','INHERITED_WANTAI_CONNECTOR')
    if profile=='INHERITED_WANTAI_CONNECTOR':return frozen
    if profile not in (DIRECT_PROFILE,SHORT_PROFILE):raise ValueError('UNKNOWN_S50_TOOL_MOUNT_PROFILE:'+profile)
    standoff=float(settings.get('tool_mount_standoff_m',0.)) if profile==SHORT_PROFILE else 0.
    spacer=float(settings.get('tool_flange_spacer_m',0.)) if profile==SHORT_PROFILE else 0.
    millimetres=round(standoff*1000)
    if profile==SHORT_PROFILE and (not .05-1e-12<=standoff-spacer<=.15+1e-12 or not 0<=spacer<=.1 or abs(standoff-millimetres/1000)>1e-12):
        raise ValueError('S50_SHORT_MOUNT_GEOMETRY_NOT_PREPARED')
    scene=deepcopy(frozen);intrinsic=scene['tool_intrinsic'];rigid=np.asarray(intrinsic['rigid_local_boxes_virtual_tcp_m'])
    extension=intrinsic['connector_extension_m']
    expected=np.array([[0.,-.105,-.17-extension/2,.2,.01,.14+extension],
                       [0., .105,-.17-extension/2,.2,.01,.14+extension],
                       [0.,0.,-.244-extension,.2,.22,.012]])
    if rigid.shape!=(130,6) or not np.allclose(rigid[[11,12,13]],expected,atol=1e-7,rtol=0):
        raise ValueError('S50_DIRECT_MOUNT_CONNECTOR_SOURCE_CHANGED')
    # Source rigid body 0 is the 288 x 576 x 24 mm suction backplate.
    # Its rear plane is z=-100 mm in the unchanged virtual TCP frame.
    if not np.allclose(rigid[0],[0,0,-.088,.288,.576,.024],atol=1e-7,rtol=0):
        raise ValueError('S50_DIRECT_MOUNT_BACKPLATE_SOURCE_CHANGED')
    new_virtual_distance=.1+standoff
    old_virtual_distance=float(np.asarray(intrinsic['intrinsic_frames']['T_tool_mount_reference_virtual_task_tcp'])[0,3])
    shortening=old_virtual_distance-new_virtual_distance
    kept=[i for i in range(len(rigid)) if i not in (11,12,13)]
    intrinsic['rigid_local_boxes_virtual_tcp_m']=rigid[kept].tolist()
    intrinsic['rigid_body_ids']=[f'tool_rigid_{i}' for i in kept]
    intrinsic['cup_body_ids']=[f'tool_cup_{i}' for i in range(len(intrinsic['compliant_local_boxes_virtual_tcp_m']))]
    for value in intrinsic['intrinsic_frames'].values():value[0][3]-=shortening
    intrinsic.update(connector_extension_m=0.,frame_convention='S50_FLANGE_COAXIAL_WITH_ORIGINAL_SUCTION_BACKPLATE',
        mount_profile=profile,removed_body_ids=list(CONNECTOR_IDS),original_tool_body_count=202,retained_tool_body_count=199,
        axial_shortening_from_inherited_mount_m=shortening,direct_mount_plane_in_virtual_tcp_m=-.1,
        mount_standoff_m=standoff,flange_to_nominal_compressed_contact_m=.0675+standoff,flange_to_uncompressed_cup_plane_m=.0775+standoff,
        mass_model_status='UNCHANGED_20KG_BOOKKEEPING_NOT_REQUALIFIED_AFTER_BRACKET_REMOVAL',
        tool_mesh_scope='UNCHANGED_SOURCE_BOUNDS_WITH_ONLY_USER_REMOVED_CONNECTOR_BODIES_EXCLUDED')
    # Preserve the original no-spacer snapshot identity for retained 105 mm
    # worlds; only a physical spacer adds this geometry field.
    if spacer:intrinsic['flange_spacer_m']=spacer
    intrinsic['definition'].update(connector_extension_m=0.,s50_mount_profile=profile)
    manifest_path=Path(root)/'assets/grippers/shanghai_wantai_three_zone/s50_direct_mount/structure_meshes.json'
    manifest=json.loads(manifest_path.read_text(encoding='utf8'))
    if manifest['removed_body_ids']!=list(CONNECTOR_IDS):raise ValueError('DIRECT_TOOL_MESH_PART_SELECTION_CHANGED')
    intrinsic['structure_meshes']=manifest
    if profile==SHORT_PROFILE:
        suffix=f'_spacer{round(spacer*1000)}mm' if spacer else ''
        mount=json.loads((Path(root)/f'assets/grippers/shanghai_wantai_three_zone/s50_short_mount_{millimetres}mm{suffix}/mount_meshes.json').read_text(encoding='utf8'))
        if not np.isclose(mount['standoff_m'],standoff,atol=1e-12):raise ValueError('S50_MOUNT_STANDOFF_MISMATCH')
        if abs(mount.get('flange_spacer_m',0.)-spacer)>1e-12:raise ValueError('S50_MOUNT_SPACER_MISMATCH')
        intrinsic['mount_meshes']=mount
        intrinsic['added_mount_body_ids']=[part['body_id'] for part in mount['parts']]
        intrinsic['rigid_body_ids']+=intrinsic['added_mount_body_ids']
        intrinsic['rigid_local_boxes_virtual_tcp_m'] += [part['local_box_virtual_tcp_m'] for part in mount['parts']]
        intrinsic['retained_tool_body_count']+=len(mount['parts'])
    intrinsic['definition']['s50_mount_standoff_m']=standoff
    scene['layout_fields']['tool'].update(connector_extension_m=0.,s50_mount_profile=profile)
    scene['layout_fields']['tool']['s50_mount_standoff_m']=standoff
    scene['parameters']['tool_extension_m']=0.
    scene['parameters']['s50_tool_mount_profile']=profile
    scene['source_environment_fingerprint_before_s50_tool_change']=scene.pop('environment_fingerprint')
    scene['environment_fingerprint']=canonical_digest(scene)
    return scene


def load_structure_meshes(root,intrinsic):
    result={}
    for key in ['structure_meshes','mount_meshes']:
        manifest=intrinsic.get(key)
        if manifest is None:continue
        path=Path(root)/manifest['mesh_file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=manifest['mesh_sha256']:raise ValueError('DIRECT_TOOL_MESH_CHANGED')
        with np.load(path,allow_pickle=False) as data:
            result.update({p['body_id']:dict(vertices=data[p['vertex_key']],faces=data[p['face_key']]) for p in manifest['parts']})
    return result
