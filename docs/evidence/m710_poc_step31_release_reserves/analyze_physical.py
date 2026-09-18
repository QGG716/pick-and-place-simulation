from pathlib import Path
import json,hashlib,numpy as np
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input
from unloading_sim.geometry import OBB
from unloading_sim.serial_unloading import rotation_from_actual_quaternion,apply_actual_motion_state
from unloading_sim.unloading_sequence import RowUnloadingState,RowSequencePolicy
root=Path.cwd();p=root/'outputs/isaac_step31_once';d=json.loads((p/'result.json').read_text());events=json.loads((p/'execution_events.json').read_text())['events']
keys=['target','runtime_stop_reason','new_target_execution_counts','execution_counts','workflow_cycle_completed','physical_cycle_completed','simulation_execution_ready','simulation_execution_qualified','execution_qualified','machine_qualified','qualification_failures','replayed_simulation_seconds','replay_wall_seconds','physics_hz','physics_steps','replay_video_frame_count','replay_video_physical_time_scale','replay_preview_speed','receiver_top_owners','archive_initialization','peak_joint_error_rad','peak_payload_attachment_position_error_m','peak_payload_attachment_rotation_error_rad','joint_positions_within_official_limits','minimum_joint_position_margin_rad','drive_effort_output_qualified','actual_free_transit_gate','target_cup_release_clearance_gate','first_unexpected_runtime_robot_contact','poc_pair_query_record_scope','payload_release_open_confirmed','actual_reception_succeeded','payload_actual_contact_cup_count_at_attach']
r={k:d.get(k) for k in keys};r['release_attachment_events']=[e for e in events if e.get('event') in ('actual_attachment_capture','ideal_release_actual_height_check','actual_constraint_removed') or 'reception' in str(e.get('event','')).lower() or 'OUTFED' in str(e.get('event','')) or 'ASSUMED' in str(e.get('event','')) or 'IDEAL' in str(e.get('event',''))]
policy=load_layout_motion_policy('configs/validation/m710id70_step31_recovery.yaml');scene=build_verified_motion_input(policy);receivers=[b for b in scene.fixed_components if b.category=='conveyor']
from unloading_sim.geometry import rotation_vector_from_matrix
capture=next((e for e in events if e.get('event')=='actual_attachment_capture'),None)
if capture:
 segment=json.loads(Path('outputs/recovery_delivery/first_feasible/motion.json').read_text())['selected_trajectory_segment'];nominal=np.array(segment['contact']['physical_contact_from_box']);measured=np.array(capture['physical_contact_from_box']);half=next(b.half_extents for b in scene.cartons if b.name==d['target'])
 boxes=[OBB(a[:3,3],half,a[:3,:3],d['target'],'carton') for a in (nominal,measured)]
 r['attachment_capture_comparison']=dict(scope='EXACT_MEASURED_CAPTURE_VERSUS_PLANNED_ATTACHMENT_IN_PHYSICAL_CONTACT_FRAME_NOT_HOLDING_DRIFT',translation_difference_m=float(np.linalg.norm(nominal[:3,3]-measured[:3,3])),rotation_difference_rad=float(np.linalg.norm(rotation_vector_from_matrix(nominal[:3,:3].T@measured[:3,:3]))),maximum_corresponding_corner_difference_m=float(np.linalg.norm(boxes[0].corners()-boxes[1].corners(),axis=1).max()))
from unloading_sim.pair_clearance import obb_surface_distance
frames=json.loads((p/'actual_frame_states.json').read_text())['states'];minima={b.name:dict(distance_m=float('inf')) for b in receivers};count=0
for f in frames:
 if not f['attached'] or f['stage']!='transit':continue
 a=next(c for c in f['cartons'] if c['name']==d['target']);box=OBB(a['center_m'],np.array(a['size_m'])/2,rotation_from_actual_quaternion(a['quaternion_wxyz']),a['name'],'carton');count+=1
 for b in receivers:
  distance=obb_surface_distance(box,b)
  if distance<minima[b.name]['distance_m']:minima[b.name]=dict(distance_m=distance,time_s=f['time_s'],trajectory_time_s=f['trajectory_time_s'],q_rad=f['q_rad'],q_target_rad=f['q_target_rad'],actual_box_pose=box.world_from_local.tolist())
r['observed_actual_pose_transit_receiver_clearance']=dict(scope='ALL_SAVED_ATTACHED_TRANSIT_FRAMES_AT_5_FPS_NOT_ALL_PHYSICS_STEPS_OR_ALL_PAIRS',frames=count,minimum_by_receiver=minima)
rows=RowUnloadingState(RowSequencePolicy(row_height_fraction=float(policy.data['search_strategy'].get('row_height_fraction',.05))));rows.rank(scene.cartons,support_graph=scene.support_graph);original=json.loads((p/'initialized_actual_state.json').read_text());scene=apply_actual_motion_state(scene,original,row_state=rows)
actual=json.loads((p/'actual_remaining_state.json').read_text())
try:
 updated=apply_actual_motion_state(scene,actual,row_state=rows);again=apply_actual_motion_state(updated,actual,row_state=rows)
 r['actual_state_chain']=dict(accepted=True,initial_state_source='SAME_NEW_WORLD_INITIALIZED_ACTUAL_STATE',remaining_ids=list(updated.removable_cartons),target_returned_to_pending=d['target'] in updated.removable_cartons,repeat_idempotent=updated.removable_cartons==again.removable_cartons,completed_ids=actual['completed_carton_ids'],ideal_received_ids=actual['ideal_received_ids'],processed_ids=actual['processed_carton_ids'],outfed_ids=actual['handed_off_ids'])
except Exception as exc:r['actual_state_chain']=dict(accepted=False,error=type(exc).__name__+': '+str(exc))
r['output_sha256']={name:hashlib.sha256((p/name).read_bytes()).hexdigest() for name in ('result.json','actual_remaining_state.json','execution_events.json','replay.mp4','initial_archive_binding.json')}
r['joint_effort_check_detail']=d['qualification_check_details']['joint_efforts_within_limit'];r['new_target_transport_events']=[e for e in d['post_landing_transport']['events'] if e.get('carton_id')==d['target']];r['physics_contract']=d['physics_contract'];r['simulation_profile']=d['simulation_profile'];r['initial_state_time_s']=json.loads((p/'initialized_actual_state.json').read_text())['time_s'];r['post_landing_state']=d['post_landing_transport']['states'].get(d['target'])
Path('outputs/physical_summary.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:r[k] for k in ('new_target_execution_counts','runtime_stop_reason','workflow_cycle_completed','observed_actual_pose_transit_receiver_clearance','actual_state_chain')},indent=2))
