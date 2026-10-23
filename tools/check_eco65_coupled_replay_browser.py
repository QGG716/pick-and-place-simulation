"""Directed browser checks for the self-contained coupled ECO65 viewer."""
import argparse
import html
import json
from pathlib import Path
import re
import subprocess
from time import perf_counter

CHECK = r'''
<script>
setTimeout(()=>{const result={checks:[],status:'RUNNING'};
const check=(name,value)=>{if(!value)throw Error(name);result.checks.push(name);};
const close=(a,b,eps=1e-8)=>a.length===b.length&&a.every((v,i)=>Math.abs(v-b[i])<=eps);
const same=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
const pairKeys=pairs=>(pairs||[]).map(pair=>JSON.stringify(pair.slice().sort())).sort();
const objectIds=world=>world.map(b=>b.id).sort();
const policyCheck=(name,display,request,response,requireNative=false)=>{
const source=request.clearance_policy?.source_policy||request.process_policy?.collision_policy;
check(name+'_POLICY_SCHEMA',display.policy_schema===source?.schema);
if(source?.rigid_tool_carton_contact){
check(name+'_RIGID_CARTON_DESCRIPTOR',same(display.rigid_tool_carton_contact,source.rigid_tool_carton_contact));
const native=response.process_policy?.rigid_tool_carton_contact||response.process?.rigid_tool_carton_contact;
if(requireNative)check(name+'_RIGID_CARTON_NATIVE_PRESENT',!!native);
if(native)check(name+'_RIGID_CARTON_ACTUAL_PAIRS',same(pairKeys(display.rigid_tool_carton_pairs),pairKeys(native.pairs)));
check(name+'_RIGID_CARTON_NO_ROBOT_OR_LIP',display.rigid_tool_carton_pairs.every(p=>source.rigid_tool_carton_contact.rigid_tool_links.includes(p[0])&&!p[0].endsWith('_lip')));
}
if(source?.robot_tool_exemption){
check(name+'_ROBOT_TOOL_POLICY_DESCRIPTOR',same(display.robot_tool_exemption,source.robot_tool_exemption));
check(name+'_EXACT_ROBOT_TOOL_PAIR',same(source.robot_tool_exemption.exempt_pairs,[['link_3','part_026']]));
}
check(name+'_POLICY_DESCRIPTOR',same(display.extraction_carton_contact??null,source?.extraction_carton_contact??null));
if(source?.cup_neighbor_contact){
check(name+'_CUP_POLICY_DESCRIPTOR',same(display.cup_neighbor_contact,source.cup_neighbor_contact));
const cups=new Set(source.cup_neighbor_contact.terminal_band_links),target=request.process_policy.target_id;
const world=new Set(request.world.map(b=>b.id)),remaining=new Set(request.process_policy.task_contract.stack_carton_ids);
check(name+'_CUP_NAMED_NEIGHBORS_ONLY',display.cup_neighbor_pairs.every(p=>cups.has(p[0])&&p[1]!==target&&world.has(p[1])&&remaining.has(p[1])&&!(request.completed_ids||[]).includes(p[1])));
const cupNative=response.process_policy?.cup_neighbor_contact||response.process?.cup_neighbor_contact;
if(requireNative)check(name+'_CUP_NATIVE_EVIDENCE_PRESENT',!!cupNative);
if(cupNative){check(name+'_CUP_NATIVE_PAIRS',same(pairKeys(display.cup_neighbor_pairs),pairKeys(cupNative.pairs)));
check(name+'_CUP_NATIVE_EVIDENCE',same(display.cup_neighbor_native_evidence,cupNative));}
}
if(!source?.extraction_carton_contact)return;
const native=response.process_policy?.extraction_carton_contact||response.process?.extraction_carton_contact;
if(requireNative)check(name+'_NATIVE_POLICY_EVIDENCE_PRESENT',!!native&&typeof native.active==='boolean'&&Array.isArray(native.pairs));
if(native&&typeof native.active==='boolean'){
check(name+'_NATIVE_POLICY_EVIDENCE',same(display.native_evidence,native));
check(name+'_NATIVE_PERMITTED_PAIRS',same(pairKeys(display.permitted_pairs),pairKeys(native.pairs)));
check(name+'_NATIVE_ACTIVE_MEANS_NONEMPTY_PAIRS',native.active===(native.pairs.length>0));}
if(display.permitted_pairs.length){
const allowed=new Set(source.extraction_carton_contact.original_carton_ids);
const remaining=new Set(request.process_policy.task_contract.stack_carton_ids);
const world=new Set(request.world.map(b=>b.id));
check(name+'_EXTRACTION_ONLY',request.stage==='extraction'&&request.attachment?.id===request.process_policy.target_id);
check(name+'_NAMED_REMAINING_WORLD_CARTONS',display.permitted_pairs.every(pair=>
pair[0]===request.attachment.id&&pair[1]!==pair[0]&&allowed.has(pair[1])&&remaining.has(pair[1])&&world.has(pair[1])&&!(request.completed_ids||[]).includes(pair[1])));
}
if(request.stage==='transit'){
check(name+'_FREE_TRANSIT_RESTORES_CHECKS',display.request_scope_applies===false&&display.permitted_pairs.length===0);
if(native)check(name+'_NATIVE_FREE_TRANSIT_NO_EXEMPTION',native.active===false&&native.pairs.length===0);}
};
try{
check('VIEWER_READY',window.REPLAY_READY===true);
check('EMPTY_NATIVE_PAIR_NOT_LOCALIZED',pairOf({pair:['','']}).length===0);
if(DATA.perception_binding){
const calibration=DATA.perception_binding.calibration;
check('TCP_CALIBRATION_PANEL',!document.getElementById('perceptionPanel').hidden);
check('TCP_METHOD',calibration.calibration_method==='TCP_FK_REFERENCED_CAMERA_EXTRINSICS');
check('FRESH_IMAGES_EMBEDDED',DATA.perception_images.length===calibration.tcp_observations.photos.length+1&&DATA.perception_images.every(r=>r.data_url.startsWith('data:image/png;base64,')));
check('OFFLINE_CALIBRATION_IMAGE_LOADED',document.getElementById('perceptionPhoto').naturalWidth>0);
}
check('TWELVE_PHYSICAL_CUPS',model.cup_ids.length===12);
let maximum=0;
for(const fixture of DATA.fk_fixtures){const frames=window.replay.fk(fixture.q);
for(const [name,reference]of Object.entries(fixture.frames)){const columns=reference[0].map((_,i)=>reference.map(row=>row[i])).flat();
maximum=Math.max(maximum,...columns.map((v,i)=>Math.abs(v-frames[name][i])));}}
result.maximum_fk_matrix_error=maximum;check('PYTHON_FK_MATCH',maximum<1e-8);
if(model.base_translation){
const p=new T.Vector3().setFromMatrixPosition(fk(currentQ).base_link);
check('TRANSLATED_BASE_RENDERED_AT_DECLARED_POSITION',close(p.toArray(),model.base_translation.new_base_position_m,1e-12));
}
check('REAL_HEIGHT_PREVIEWS',DATA.previews.length>=3);
if(DATA.receiver_hardware_range_m)check('HARDWARE_BOUNDARY_PREVIEWS',DATA.receiver_hardware_range_m.every(h=>DATA.previews.some(p=>Math.abs(p.height_m-h)<1e-9)));
if(DATA.receiver_hardware_range_m)check('PREVIEWS_INSIDE_HARDWARE_RANGE',DATA.previews.every(p=>p.height_m>=DATA.receiver_hardware_range_m[0]-1e-12&&p.height_m<=DATA.receiver_hardware_range_m[1]+1e-12));
for(let i=0;i<DATA.previews.length;i++){
preview(i);const expected=DATA.previews[i].state;
check('PREVIEW_HEIGHT_'+i,state.height_m===expected.height_m);
check('PREVIEW_Q_'+i,close(currentQ,expected.q));
let error=0;
for(const b of expected.world){const objects=worldObjects.filter(o=>o.userData.id===b.id);
check('PREVIEW_OBJECT_PRESENT_'+i+'_'+b.id,objects.length===2);
for(const object of objects)error=Math.max(error,...matrix(b.pose).elements.map((v,j)=>Math.abs(v-object.matrix.elements[j])));}
check('SAME_NATIVE_WORLD_POSES_'+i,error<1e-12);
// Independently inspect the rendered mesh vertices. Matching the source pose
// alone misses a local mesh that was converted from world coordinates twice.
const belts=worldObjects.filter(o=>o.userData.layer==='visual'&&o.userData.role==='conveyor_belt');
check('TWO_CONVEYOR_ASSEMBLIES_'+i,new Set(belts.map(o=>o.userData.owner)).size===2);
for(const owner of ['conveyor_transverse','conveyor_longitudinal']){
const own=belts.filter(o=>o.userData.owner===owner);check('BELT_PRESENT_'+i+owner,own.length>0);
const yMin=Math.min(...own.map(o=>new T.Box3().setFromBufferAttribute(o.geometry.getAttribute('position')).applyMatrix4(o.matrix).min.y));
check('ACTUAL_MESH_YMIN_'+i+owner,Math.abs(yMin+.3)<1e-6);
const frames=worldObjects.filter(o=>o.userData.layer==='visual'&&o.userData.owner===owner&&o.userData.role==='conveyor_frame');
check('BELT_FRAME_ALIGNMENT_'+i+owner,frames.length===1&&Math.abs(new T.Box3().setFromBufferAttribute(frames[0].geometry.getAttribute('position')).applyMatrix4(frames[0].matrix).min.y-yMin)<1e-6);
}
check('NO_OLD_STATIC_CONVEYORS_'+i,!model.static_visual.some(o=>o.id.startsWith('conveyor_')));
}
const collision=document.querySelector('input[data-layer="collision"]');collision.click();
check('COLLISION_LAYER_VISIBLE',robotObjects.filter(r=>r.item.layer==='collision').every(r=>r.object.visible));collision.click();
const accessory=model.joints.filter(j=>j.child.startsWith('eco65_mast_')||j.child.includes('camera'));
check('J1_ACCESSORY_PARENTAGE',accessory.length>0&&accessory.every(j=>{let p=j.parent;while(p!=='link_1'&&p!=='world'){const edge=model.joints.find(k=>k.child===p);if(!edge)return false;p=edge.parent;}return p==='link_1';}));
document.getElementById('top').click();check('TOP_VIEW',Math.abs(camera.position.x-controls.target.x)<1e-9);document.getElementById('reset').click();
if(model.camera_definition?.axis_rotation){
const C=model.camera_definition,rotation=C.axis_rotation;
const p=new T.Vector3(...rotation.point_mast_m);
check('CAMERA_ACTUAL_POLE_AXIS_INVARIANT',p.distanceTo(p.clone().applyMatrix4(matrix(rotation.T_new_from_old_mast)))<1e-10);
const previous=matrix(rotation.old_T_mast_CAD).invert().multiply(matrix(rotation.old_T_mast_optical));
const changed=matrix(C.T_mast_CAD).invert().multiply(matrix(C.T_mast_optical));
check('CAMERA_CAD_OPTICAL_RIGID_RELATION',close(previous.elements,changed.elements,1e-10));
const M=matrix(C.T_link1_mast).multiply(matrix(C.T_mast_optical)),f=new T.Vector3().setFromMatrixColumn(M,2);
check('CAMERA_THIRTY_DEGREE_DOWNWARD_PITCH',Math.abs(f.z+.5)<1e-10);
if(C.layout_remount){
const old=matrix(C.layout_remount.old_T_link1_optical),oldR=new T.Matrix4().makeRotationZ(C.layout_remount.old_base_rpy_rad[2]);
const newR=new T.Matrix4().makeRotationZ(C.layout_remount.new_base_rpy_rad[2]);
const oldWorld=oldR.multiply(old),newWorld=newR.multiply(M);
if(C.layout_remount.optical_orientation==='WORLD_HEADING_AT_REFERENCE_J1'){
const reference=new T.Matrix4().makeRotationZ(C.layout_remount.reference_raw_j1_rad).multiply(newWorld);
check('CAMERA_HORIZONTAL_PLUS_Y_DOWN30_AT_REFERENCE_J1',new T.Vector3().setFromMatrixColumn(reference,2).distanceTo(new T.Vector3(0,Math.sqrt(3)/2,-.5))<1e-10);
const oldMount=matrix(model.layout_installation.old_T_link1_mast),newMount=matrix(model.layout_installation.new_T_link1_mast);
const rotation=newMount.clone().multiply(oldMount.clone().invert()),expected=new T.Matrix4().makeRotationZ(-Math.PI/2);
check('MAST_BASE_CLOCKWISE_90',close(rotation.elements,expected.elements,1e-10));
const horizontal=new T.Vector3().setFromMatrixColumn(new T.Matrix4().makeRotationZ(C.layout_remount.new_base_rpy_rad[2]).multiply(newMount),0);
check('MAST_BASE_HORIZONTAL_AXIS_PARALLEL_X',horizontal.distanceTo(new T.Vector3(-1,0,0))<1e-10);
}else check('CAMERA_WORLD_ORIENTATION_PRESERVED',[0,1,2].every(i=>new T.Vector3().setFromMatrixColumn(oldWorld,i).distanceTo(new T.Vector3().setFromMatrixColumn(newWorld,i))<1e-10));
const base=fk(currentQ).base_link,switchNormal=new T.Vector3(1,0,0).transformDirection(base);
check('BASE_SWITCH_FACE_MINUS_X',switchNormal.distanceTo(new T.Vector3(-1,0,0))<1e-10);
}else check('CAMERA_REVERSED_OPTICAL_HEADING',f.y<0);
const visual=robotObjects.find(r=>r.item.id==='camera_camera_part_001'&&r.item.layer==='visual');
const native=fk(currentQ)['eco65_camera_camera_part_001'];
check('CAMERA_DISPLAY_NATIVE_SAME_TRANSFORM',!!visual&&!!native&&close(visual.object.matrix.elements,native.elements,1e-9));
check('NEW_EXTRINSICS_NOT_CALIBRATED',C.calibration.prior_calibration_valid===false);
check('CAMERA_DIAGNOSTICS_VISIBLE',document.getElementById('cameraInfo').textContent.includes('30.0000'));
check('OFFICIAL_RAW_J1_LIMITS',close(model.joint_limits_rad[0],[-3.106,3.106],1e-12));
const failed=DATA.attempts.findIndex(a=>a.request.stage==='contact'&&a.response.status==='STATE_REJECTED');
if(failed>=0){
document.getElementById('mode').value='search';document.getElementById('request').value=failed;chooseRequest();
check('ACTUAL_REJECTED_CONTACT_EDGE_DIAGNOSTIC',document.getElementById('edgeInfo').textContent.includes('实际 FK'));
check('CANDIDATE_NOT_REPORTED_AS_ACCEPTED',!active.adopted);
}
}
if(DATA.segments.length){
document.getElementById('mode').value='cycle';modeChanged();
for(const segment of DATA.segments){
setTime(segment.start_s+segment.duration_s*.37);
const attempt=DATA.attempts.find(a=>a.adopted&&a.request.stage_id===segment.record_id);
check('RECORD_HAS_OWN_NATIVE_REQUEST_'+segment.record_id,!!attempt);
policyCheck('RECORD_'+segment.record_id,segment.display_policy,attempt.request,attempt.response,true);
check('RECORD_POLICY_PANEL_'+segment.record_id,same(JSON.parse(document.getElementById('policyEvidence').textContent),segment.display_policy));
check('RECORD_STATE_POLICY_'+segment.record_id,same(segment.state_before.collision_policy,attempt.request.clearance_policy.source_policy)&&same(segment.state_before.collision_policy,segment.state_after.collision_policy));
const expected=(segment.kind==='event'&&segment.event_type!=='NOMINAL_RECEIVE')?segment.state_after:segment.state_before;
check('RECORD_RENDERED_OWNERSHIP_'+segment.record_id,same(objectIds(state.world),objectIds(expected.world))&&state.attachment?.id===expected.attachment?.id&&state.target_id===expected.target_id&&same(state.completed_ids,expected.completed_ids));
if(segment.kind==='external_axis')check('EXTERNAL_AXIS_HOLDS_Q_'+segment.record_id,close(currentQ,segment.state_before.q,1e-12));
if(segment.kind==='robot_motion')check('ROBOT_HOLDS_HEIGHT_'+segment.record_id,state.height_m===segment.state_before.height_m);
if(segment.retreat_geometry?.strategy==='EMPTY_TOOL_UP'){
check('EMPTY_UP_RETREAT_RETAINS_RECEIVED_BOX_'+segment.record_id,!state.attachment&&state.world.some(b=>b.id===state.target_id));
check('EMPTY_UP_RETREAT_LABEL_'+segment.record_id,document.getElementById('hud').textContent.includes('空载工具沿 +Z'));
}
if(segment.event_type==='ATTACH'&&['left','right'].includes(state.target_face))check('SIDE_CONTACT_EDGE_DISPLAY_'+segment.record_id,document.getElementById('edgeInfo').textContent.includes('边方向 PASS'));
if(state.attachment){check('NO_DUPLICATE_ATTACHMENT_'+segment.record_id,!state.world.some(b=>b.id===state.attachment.id));const actual=window.replay.payload.matrix.elements,expected=fk(currentQ)[model.flange_link].clone().multiply(matrix(state.attachment.pose)).elements;check('LOCAL_ATTACHMENT_'+segment.record_id,close(actual,expected,1e-12));}
}
setTime(DATA.duration_s);check('TERMINAL_Q',close(currentQ,DATA.terminal_state.q,1e-12));
check('TERMINAL_OWNERSHIP',state.world.length===DATA.terminal_state.world.length&&state.attachment?.id===DATA.terminal_state.attachment?.id);
const originalIds=new Set(DATA.previews[0].state.world.filter(b=>b.category==='carton').map(b=>b.id));
const worldIds=state.world.filter(b=>originalIds.has(b.id)).map(b=>b.id),attachedIds=state.attachment?[state.attachment.id]:[],completedIds=state.completed_ids||[];
result.terminal_carton_counts={world:worldIds.length,attached:attachedIds.length,completed:completedIds.length};
check('TERMINAL_CARTON_IDS_PARTITION',new Set([...worldIds,...attachedIds,...completedIds]).size===originalIds.size&&worldIds.length+attachedIds.length+completedIds.length===originalIds.size&&[...worldIds,...attachedIds,...completedIds].every(id=>originalIds.has(id)));
check('TERMINAL_COUNTS_MATCH_VALIDATED_PLAN',worldIds.length===DATA.validation.world_cartons&&attachedIds.length===DATA.validation.attached_cartons&&completedIds.length===DATA.validation.completed_cartons);
if(DATA.plan_summary.complete){
if(DATA.plan_summary.completion_goal==='ALL_CARTONS_DRAINED')check('ALL_CARTONS_DRAINED_EMPTY_WORLD_AND_ATTACHMENT',completedIds.length===originalIds.size&&attachedIds.length===0&&worldIds.length===0);
else check('ONE_COMPLETE_NEXT_ONLY_ATTACHED',completedIds.length===1&&completedIds[0]===DATA.plan_summary.first_target_id&&attachedIds.length===1&&attachedIds[0]===DATA.plan_summary.next_target_id&&!completedIds.includes(DATA.plan_summary.next_target_id));
}
result.complete_task=DATA.plan_summary.complete?'TESTED':'PARTIAL_ONLY';
}else result.complete_task='NOT_RUN_NO_PLAN';
if(DATA.attempts.length){
document.getElementById('mode').value='search';modeChanged();
const transitionIndex=DATA.attempts.findIndex(a=>a.failure_samples.some(f=>f.state));
if(transitionIndex>=0){
document.getElementById('request').value=transitionIndex;chooseRequest();
const index=failures.findIndex(f=>f.state);inspectFailure(index);
check('TRANSITION_FAILURE_ACTUAL_WORLD',state.world===failures[index].state.world);
check('TRANSITION_FAILURE_ACTUAL_HEIGHT',state.height_m===failures[index].state.height_m);
check('TRANSITION_FAILURE_HOLDS_ACTUAL_Q',close(currentQ,failures[index].q,1e-12));
chooseRequest();check('TRANSITION_REQUEST_RESTORES_START_WORLD',state.world===active.request.world);
}
const policyIndices=[DATA.attempts.findIndex(a=>a.request.stage==='extraction'&&a.display_policy?.native_evidence?.active),DATA.attempts.findIndex(a=>a.request.stage==='transit'&&a.display_policy?.native_evidence)];
for(const i of [...new Set(policyIndices.filter(i=>i>=0))]){
document.getElementById('request').value=i;chooseRequest();policyCheck('SEARCH_'+i,active.display_policy,active.request,active.response);
check('SEARCH_POLICY_PANEL_'+i,same(JSON.parse(document.getElementById('policyEvidence').textContent),active.display_policy));
check('SEARCH_POLICY_OWN_WORLD_'+i,state.world===active.request.world&&state.attachment===active.request.attachment);
}
const index=DATA.attempts.findIndex(a=>a.trace?.planners?.some(p=>p.nodes.length));
document.getElementById('request').value=Math.max(0,index);chooseRequest();
check('REQUEST_OWNS_WORLD',state.world===active.request.world);
check('REQUEST_OWNS_ATTACHMENT',state.attachment===active.request.attachment);
check('REQUEST_HEIGHT',state.height_m===active.request.mechanism_state?.height_m||state.height_m===DATA.mechanisms[active.mechanism_ref]?.height_m);
if(planner?.nodes.length){
document.getElementById('event').value=planner.events.length;drawGraph();renderer.render(scene,camera);
let clicked=false;
for(const n of drawnNodes){const v=tcp(n.q).clone().project(camera);if(Math.abs(v.x)>.95||Math.abs(v.y)>.95)continue;const r=renderer.domElement.getBoundingClientRect();
renderer.domElement.dispatchEvent(new MouseEvent('click',{clientX:r.left+(v.x+1)*r.width/2,clientY:r.top+(1-v.y)*r.height/2,bubbles:true}));
if(document.getElementById('selection').textContent.startsWith('真实 PlannerData 节点')){clicked=true;break;}}
check('REAL_NODE_CANVAS_CLICK',clicked);
if(failures.length){inspectFailure(0);check('FAILED_SAMPLE_ACTUAL_Q',close(currentQ,failures[0].q,1e-12));}
if(planner.events.length){document.getElementById('event').value=0;document.getElementById('step').click();check('SEARCH_EVENT_STEP',document.getElementById('event').value==='1');}
}
}else result.search='NOT_RUN_NO_NATIVE_REQUESTS';
document.getElementById('mode').value=DATA.segments.length?'cycle':'preview';modeChanged();
if(DATA.segments.length)setTime(DATA.duration_s);else preview(DATA.previews.length-1);
if(!DATA.segments.length&&DATA.attempts.length){
const rejected=DATA.attempts.findIndex(a=>a.request.stage==='contact'&&a.response.status==='STATE_REJECTED');
if(rejected>=0){document.getElementById('mode').value='search';document.getElementById('request').value=rejected;modeChanged();if(failures.length)inspectFailure(0);}
}
if(typeof openFirstFailure==='function'&&failedRequests.length){
check('FAILURE_CATALOGUE_MATCHES_NATIVE_STATUSES',failedRequests.length===DATA.attempts.filter(a=>a.response.status!=='SUCCESS').length);
document.getElementById('failureFilter').value='collision';document.getElementById('failureFilter').dispatchEvent(new Event('change'));
check('GEOMETRIC_FAILURE_FILTER_USES_REAL_WITNESSES',filteredFailures.length===failedRequests.filter(({a})=>hasPair(a)).length&&filteredFailures.every(({a})=>hasPair(a)));
if(filteredFailures.length>1){const previous=active;document.getElementById('nextFailure').click();check('NEXT_FAILURE_SELECTS_OWN_REQUEST',active!==previous&&state.world===(failures[Number(document.getElementById('failure').value)]?.state?.world||active.request.world));}
document.getElementById('failureFilter').value='all';fillFailureCatalogue();
const pairIndex=failedRequests.find(({a})=>hasPair(a))?.index;
if(pairIndex!==undefined){
openFailure(pairIndex);
check('FAILURE_PANEL_VISIBLE',!document.getElementById('diagnosis').hidden);
const selected=failures[Number(document.getElementById('failure').value)],pair=pairOf(selected.witness);
check('DIAGNOSTIC_USES_ACTUAL_REJECTED_Q',close(currentQ,selected.q,1e-12));
check('DIAGNOSTIC_USES_REQUEST_OR_SAMPLE_WORLD',state.world===(selected.state?.world||active.request.world));
check('EXACT_FAILURE_OBJECTS_LOCATED',same(window.replay.highlightedIds,pair));
check('FAILURE_COLLIDERS_VISIBLE_WITH_COLLISION_LAYER_OFF',!layers.collision&&diagnosticObjects.length>0&&diagnosticObjects.every(o=>o.visible));
check('REASON_NAMES_EXACT_OBJECT_IDS',pair.every(id=>document.getElementById('diagnosisPairs').textContent.includes(id)));
check('RIGID_CUP_NEIGHBOR_SCOPE_EXPLAINED',!pair.some(id=>id.endsWith('_rigid'))||document.getElementById('diagnosisPolicy').textContent.includes('刚性'));
const sentinel=failures.findIndex(f=>(f.witness.original||f.witness).surface_distance_m<0);
if(sentinel>=0){inspectFailure(sentinel);check('NEGATIVE_DISTANCE_NOT_FAKE_PENETRATION_DEPTH',document.getElementById('diagnosisReason').textContent.includes('未记录穿透深度')&&!document.getElementById('diagnosisReason').textContent.includes('-1000'));}
}
for(const status of ['NATIVE_IK_FAILED','NATIVE_SOLVER_DEADLINE_EXPIRED']){
const index=failedRequests.find(({a})=>a.response.status===status&&!witnesses(a).length)?.index;
if(index!==undefined){openFailure(index);check(status+'_NO_INVENTED_COLLISION',window.replay.highlightedIds.length===0&&document.getElementById('diagnosisPairs').children.length===0);check(status+'_ACTUAL_SEED_CONTEXT',state.world===active.request.world&&close(currentQ,active.request.q_start,1e-12));check(status+'_NO_FALSE_INFEASIBILITY_CLAIM',document.getElementById('diagnosisOutcome').textContent.includes(status==='NATIVE_IK_FAILED'?'不能据此断言':'超时不代表'));}
}
document.getElementById('backToCycle').click();check('CYCLE_CLEARS_FAILURE_OVERLAY',document.getElementById('diagnosis').hidden&&window.replay.highlightedIds.length===0);
if(DATA.segments.length){document.getElementById('completedEnd').click();
check('COMPLETED_END_BUTTON_RESTORES_TERMINAL',close(currentQ,DATA.terminal_state.q,1e-12)&&same(state.completed_ids,DATA.terminal_state.completed_ids));}
if(document.documentElement.dataset.startInFailures==='true'){
openFirstFailure();document.querySelector('aside').scrollTop=0;controls.update();updateDiagnosticLabels();renderer.render(scene,camera);
result.failure_view={request_index:DATA.attempts.indexOf(active),status:active.response.status,pair:window.replay.highlightedIds,reason:document.getElementById('diagnosisReason').textContent,failed_requests:failedRequests.length};
}
}
const checkedContact=DATA.attempts.find(a=>a.request.op==='check_state_offline'&&a.request.stage==='contact'&&a.response.status==='SUCCESS');
if(checkedContact){
selectInspectionTarget(targetOf(checkedContact),false);targetQuick('contact_ik');
check('CONTACT_IK_BUTTON_SHOWS_CHECKED_CANDIDATE',active===checkedContact&&close(currentQ,checkedContact.request.q_start,1e-12));
check('CONTACT_IK_DOES_NOT_INVENT_ATTACHMENT',state.attachment===checkedContact.request.attachment);
targetQuick('approach');
if(active.request.stage==='pregrasp'&&hasPair(active))check('APPROACH_COLLISION_BUTTON_OWNS_WORLD',state.world===active.request.world&&window.replay.highlightedIds.length===2);
}
if(DATA.plan_summary?.inventory_ids){
check('ALL_INVENTORY_TARGETS_VISIBLE',DATA.plan_summary.inventory_ids.every(id=>[...document.getElementById('diagnosticTarget').options].some(o=>o.value===id)));
const untouched=DATA.plan_summary.inventory_ids.find(id=>!DATA.attempts.some(a=>targetOf(a)===id));
if(untouched&&DATA.segments.length){selectInspectionTarget(untouched);
check('UNSEARCHED_TARGET_HAS_NO_INVENTED_REQUEST',document.getElementById('targetEvidence').options.length===0);
check('UNSEARCHED_TARGET_USES_TERMINAL_WORLD',same(state.completed_ids,DATA.terminal_state.completed_ids)&&close(currentQ,DATA.terminal_state.q,1e-12));}
const heightRejected=(DATA.plan_summary.continuation_search||[]).flatMap(s=>s.targets).find(t=>t.candidate_count===0&&t.process_options.rejected.some(r=>r.reason==='PROCESS_RECEIVE_HEIGHT_OUT_OF_RANGE'));
if(heightRejected){selectInspectionTarget(heightRejected.target_id,false);const text=document.getElementById('targetSummary').textContent;
check('HEIGHT_REJECTION_SEPARATES_FROZEN_BOTTOM_AND_HARDWARE_RANGE',text.includes('冻结箱底')&&DATA.receiver_hardware_range_m.every(h=>text.includes((h*1000).toFixed(0))));}
}
result.status='PASS';
}catch(error){result.status='FAIL';result.error=error.stack;}
const pre=document.createElement('pre');pre.id='browser-check-result';pre.textContent=JSON.stringify(result);document.body.append(pre);
window.BROWSER_CHECK_RESULT=result;
},500);
</script>
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--html', type=Path, required=True)
    parser.add_argument('--browser', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    start = perf_counter()
    page = args.output/'browser-test.html'
    page.write_text(args.html.read_text(encoding='utf-8')+CHECK, encoding='utf-8')
    process = subprocess.run([str(args.browser), '--headless', '--no-first-run',
        '--user-data-dir='+str((args.output/'profile').resolve()),
        '--window-size=1440,1000', '--virtual-time-budget=4000', '--dump-dom',
        '--screenshot='+str((args.output/'checked.png').resolve()), page.resolve().as_uri()],
        capture_output=True, timeout=120)
    match = re.search(r'<pre id="browser-check-result">(.*?)</pre>',
                      process.stdout.decode('utf-8', 'replace'), re.S)
    if not match:
        (args.output/'browser-stderr.txt').write_bytes(process.stderr)
        raise RuntimeError('BROWSER_TEST_RESULT_MISSING:'+str(process.returncode))
    result = json.loads(html.unescape(match.group(1)))
    result['wall_s'] = perf_counter()-start
    (args.output/'checks.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({**{k:v for k,v in result.items() if k!='checks'},
                      'check_count':len(result['checks'])}))
    if result['status'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
