from pathlib import Path
import json,sys
import numpy as np
work=Path('/work-c01-ext100-v2-2f6e1ea');sys.path[:0]=[str(work/'src'),str(work/'tools')]
from plan_m710_mount_comparison import make_context
from unloading_sim.coupled_batch import message_box
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.moveit2_backend import native_tool_collision_boxes,box_message
from unloading_sim.lifted_reception import directional_contact,bounds,translated
from unloading_sim.junction_reception import reception_classification
from unloading_sim.geometry import OBB
context,c,_=make_context('x_minus_500_y_minus_100',layout_path=work/'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250_lowprofile_tool_extension100.yaml',robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
assert type(c.robot).__name__=='PinocchioHppFclBackend'
s=json.loads(Path('/c01-ext100-v2-20261023/formal-v1/summary.json').read_text());root=s['initial'];q=np.array(root['q_rad'])
boxes={b['id']:message_box(b) for b in root['world_cartons']};target=boxes['carton_l01_c01'];neighbor=boxes['carton_l01_c04']
world={b.name:b for b in coupled_conveyor_state(context.policy.layout_validation.layout,root['height_m'],()).boxes}
tb=world['conveyor_transverse'];lb=world['conveyor_longitudinal']
release=translated(target,[tb.center[0]-target.center[0],0,0]);classification=reception_classification(release,tb,lb)
yboundary=max(v[1] for v in classification['influence_polygon_world_xy_m'])+(target.center[1]-bounds(target)[0][1])
turn_y=yboundary+2*c.ik['position_tolerance_m'];dy=turn_y-target.center[1]
tools,ownership=native_tool_collision_boxes(c,q)
actual_contact=c.physical_from_virtual(c.robot.fk(q));local=[b.transformed(np.linalg.inv(actual_contact)) for b in tools]
pose=np.array(directional_contact(target,[0,1,0])['contact_pose']);contact_tools=[b.transformed(pose) for b in local]
depth=max(-bounds(b)[0][2] for b in local)
standoff=c.budget.pregrasp_standoff_m
def swept(b,delta):
    points=np.vstack([b.corners(),b.corners()+delta]);a=points.min(axis=0);z=points.max(axis=0)
    return OBB((a+z)/2,(z-a)/2,np.eye(3),b.name,b.category)
rows=[]
for stage,start,delta in [('pregrasp_to_contact',contact_tools,np.array([0,standoff,0])),('positive_y',contact_tools,np.array([0,dy,0])),('negative_x',[translated(t,[0,dy,0]) for t in contact_tools],np.array([tb.center[0]-target.center[0],0,0]))]:
    gaps=[(swept(b,delta).signed_distance_obb(neighbor),b.name) for b in start]
    gap,name=min(gaps);rows.append(dict(stage=stage,minimum_conservative_tool_c04_gap_m=gap,body=name,delta_world_m=delta.tolist()))
data=dict(source_commit=s['source_commit'],backend=type(c.robot).__name__,scope='STATIC_FIXED_TOOL_POSE_ENVELOPE_NOT_ROBOT_PATH',
    target='carton_l01_c01',neighbor='carton_l01_c04',target_center_m=target.center.tolist(),target_size_m=(target.half_extents*2).tolist(),
    inlet_y_range_m=[min(v[1] for v in classification['influence_polygon_world_xy_m']),max(v[1] for v in classification['influence_polygon_world_xy_m'])],
    first_nonjunction_turn_y_m=turn_y,positive_y_distance_m=dy,minimum_ideal_positive_y_m=yboundary-target.center[1],
    tool_body_count=len(tools),tool_depth_from_physical_plane_m=depth,pregrasp_standoff_m=standoff,
    side_exposure_depth_with_gap_m=depth+standoff+c.collision_policy.free_space_clearance_m,
    target_to_c04_initial_face_gap_m=bounds(neighbor)[0][1]-bounds(target)[1][1],tool_sweeps=rows,
    target_to_c04_turn_gap_m=bounds(neighbor)[0][1]-bounds(translated(target,[0,dy,0]))[1][1],
    physical_execution='NOT_RUN',continuous_swept_proof=False)
points=np.vstack([b.corners() for b in contact_tools]);lo=points.min(axis=0);hi=points.max(axis=0)
envelope=OBB((lo+hi)/2,(hi-lo)/2,np.eye(3),'tool_preview_envelope','tool')
data.update(preview_contact_tool_envelope=box_message(envelope),preview_turn_tool_envelope=box_message(translated(envelope,[0,dy,0])),preview_turn_box=box_message(translated(target,[0,dy,0])),preview_release_box=box_message(translated(target,[tb.center[0]-target.center[0],dy,0])))
Path('/c01-ext100-v2-20261023/geometry.json').write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps(data))
