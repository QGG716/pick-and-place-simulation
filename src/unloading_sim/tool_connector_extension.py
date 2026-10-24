"""User-selected axial extension of the two Wantai connector plates.

The audited flange plate stays fixed. The two existing axial side plates grow,
and the rest of the original tool translates with its virtual/contact frames.
No collider is removed, made compliant, or exempted from environment checks.
"""
import numpy as np


def bind_connector_frames(policy):
    """Bind the existing frame contract to the explicitly selected layout once."""
    from copy import deepcopy
    from dataclasses import replace
    from .layout_single_carton import _load_tool_frame_contract
    extension=connector_extension_m(policy.layout_validation.layout.data['tool'])
    if not extension:return policy
    data=deepcopy(policy.data)
    frame=data['tool_frame_contract']
    current=float(np.asarray(frame['T_flange_virtual_task_tcp'])[0,3])
    if np.isclose(current,.25,atol=1e-12,rtol=0):
        _load_tool_frame_contract(frame)
        for name in ('T_flange_virtual_task_tcp','T_flange_uncompressed_cup_plane',
                     'T_flange_nominal_compressed_contact'):
            transform=np.array(frame[name],dtype=float,copy=True)
            transform[0,3]+=extension
            frame[name]=transform.tolist()
    frames=_load_tool_frame_contract(frame,connector_extension_m=extension)
    return replace(policy,data=data,tool_frames=frames)


def connector_extension_m(tool):
    length=float(tool.get('connector_extension_m',0.))
    if not np.isfinite(length) or length not in (0.,.1,.2):
        raise ValueError('UNAPPROVED_WANTAI_CONNECTOR_EXTENSION')
    return length


def extended_connector_boxes(rows,extension):
    boxes=np.asarray(rows,dtype=float).copy()
    if extension==0:return boxes
    connector_extension_m({'connector_extension_m':extension})
    # Stable owned rigid IDs from the hash-verified source solid order. Validate
    # their literal dimensions before applying the user engineering change.
    expected=np.array([[0.,-.105,-.17,.2,.01,.14],
                       [0.,.105,-.17,.2,.01,.14],
                       [0.,0.,-.244,.2,.22,.012]])
    if boxes.shape!=(130,6) or not np.allclose(boxes[[11,12,13]],expected,atol=1e-7,rtol=0):
        raise ValueError('WANTAI_CONNECTOR_SOURCE_BINDING_CHANGED')
    # Rows use virtual-TCP axes (+Z along flange +X). The new TCP itself moves
    # by the extension, so these changes keep the rear anchors at their flange.
    boxes[[11,12],2]-=extension/2
    boxes[[11,12],5]+=extension
    boxes[13,2]-=extension
    return boxes


def connector_extension_evidence(extension):
    if extension==0:return None
    connector_extension_m({'connector_extension_m':extension})
    return dict(extension_m=extension,axis_in_flange=[1.,0.,0.],
        stretched_owned_bodies=['tool_rigid_11','tool_rigid_12'],fixed_owned_bodies=['tool_rigid_13'],
        old_plate_axial_length_m=.14,new_plate_axial_length_m={.1:.24,.2:.34}[extension],
        original_tool_body_count=202,tool_bodies_removed=0,
        original_cad_bounds_hash_verified=True,representation='USER_ENGINEERING_AXIAL_EXTRUSION_WITH_SOURCE_CROSS_SECTIONS',
        source_cad_modified=False,mass_model='EXISTING_20KG_ENGINEERING_INPUT_NOT_REQUALIFIED',physical_execution='NOT_RUN')
