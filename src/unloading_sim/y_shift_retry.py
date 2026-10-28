"""The exact user-requested 100 mm -Y mounting change, with frozen equipment."""
from copy import deepcopy
import numpy as np
from .current_population_recipe import LAYOUT as BASE_LAYOUT
from .moveit2_backend import box_message

LAYOUT='m710id70_unloading_layout_v1_mount_x_minus_300_y_minus_200_min250_lowprofile_tool_extension200.yaml'
APPROVAL='user_20261027_l05_l04_mount_yminus100'
TARGETS=('carton_l05_c03','carton_l04_c03')


def mount_evidence(previous,shifted):
    expected=deepcopy(previous.data)
    expected['robot']['base_origin_xy_a_m']=[.325,-.2]
    expected['evidence']['approval_basis']=APPROVAL
    if shifted.data!=expected:raise ValueError('Y_SHIFT_CHANGED_UNAUTHORIZED_LAYOUT_FIELDS')
    a=previous.robot_base_transform();b=shifted.robot_base_transform()
    if (not np.allclose(a[:3,3],[-1.625,.25,.6],atol=1e-12,rtol=0)
            or not np.allclose(b[:3,3],[-1.625,.15,.6],atol=1e-12,rtol=0)
            or not np.array_equal(a[:3,:3],b[:3,:3])):
        raise ValueError('Y_SHIFT_BASE_TRANSFORM_CHANGED')
    if ([box_message(x) for x in previous.fixed_components()]!=[box_message(x) for x in shifted.fixed_components()]
            or [box_message(x) for x in previous.cartons()]!=[box_message(x) for x in shifted.cartons()]):
        raise ValueError('Y_SHIFT_FIXED_EQUIPMENT_OR_CARTONS_CHANGED')
    return dict(approval=APPROVAL,old_layout_file='configs/workcells/'+BASE_LAYOUT,
        new_layout_file='configs/workcells/'+LAYOUT,old_base_world_m=[-1.625,.25,.6],
        new_base_world_m=[-1.625,.15,.6],translation_world_m=[0.,-.1,0.],
        old_layout_fingerprint=previous.layout_fingerprint,new_layout_fingerprint=shifted.layout_fingerprint,
        fixed_equipment_and_cartons_unchanged=True,tool_extension_m=.2,
        operation='NEW_PLANNING_WORLD_WITH_CHANGED_ROBOT_MOUNT',physical_mount_motion_executed=False)


def verify_mount_protocol(protocol,root_path):
    from .workcell_layout import load_workcell_layout
    old=load_workcell_layout(root_path/'configs/workcells'/BASE_LAYOUT)
    new=load_workcell_layout(root_path/'configs/workcells'/LAYOUT)
    assert protocol['authorized_mount_change']==mount_evidence(old,new)
    assert protocol['active_layout_file']=='configs/workcells/'+LAYOUT
    allowed=(('carton_l04_c02','carton_l04_c03') if protocol.get('scope')=='ORDERED_L04_C02_THEN_C03_AFTER_ACCEPTED_L05' else TARGETS)
    assert set(protocol['scoped_target_ids'])<=set(allowed)
    assert protocol['no_historical_solution_hints'] and protocol['all_joint_rate_limits_retained']
    assert protocol['task_budget_s']==300. and protocol['stage_budget_s']==90.
