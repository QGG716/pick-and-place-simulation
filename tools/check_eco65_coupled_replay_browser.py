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
check('TWELVE_PHYSICAL_CUPS',model.cup_ids.length===12);
let maximum=0;
for(const fixture of DATA.fk_fixtures){const frames=window.replay.fk(fixture.q);
for(const [name,reference]of Object.entries(fixture.frames)){const columns=reference[0].map((_,i)=>reference.map(row=>row[i])).flat();
maximum=Math.max(maximum,...columns.map((v,i)=>Math.abs(v-frames[name][i])));}}
result.maximum_fk_matrix_error=maximum;check('PYTHON_FK_MATCH',maximum<1e-8);
check('THREE_REAL_HEIGHT_PREVIEWS',DATA.previews.length===3);
for(let i=0;i<DATA.previews.length;i++){
preview(i);const expected=DATA.previews[i].state;
check('PREVIEW_HEIGHT_'+i,state.height_m===expected.height_m);
check('PREVIEW_Q_'+i,close(currentQ,expected.q));
let error=0;
for(const b of expected.world){const objects=worldObjects.filter(o=>o.userData.id===b.id);
check('PREVIEW_OBJECT_PRESENT_'+i+'_'+b.id,objects.length===2);
for(const object of objects)error=Math.max(error,...matrix(b.pose).elements.map((v,j)=>Math.abs(v-object.matrix.elements[j])));}
check('SAME_NATIVE_WORLD_POSES_'+i,error<1e-12);}
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
check('CAMERA_REVERSED_OPTICAL_HEADING',f.y<0);
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
if(state.attachment){check('NO_DUPLICATE_ATTACHMENT_'+segment.record_id,!state.world.some(b=>b.id===state.attachment.id));const actual=window.replay.payload.matrix.elements,expected=fk(currentQ)[model.flange_link].clone().multiply(matrix(state.attachment.pose)).elements;check('LOCAL_ATTACHMENT_'+segment.record_id,close(actual,expected,1e-12));}
}
setTime(DATA.duration_s);check('TERMINAL_Q',close(currentQ,DATA.terminal_state.q,1e-12));
check('TERMINAL_OWNERSHIP',state.world.length===DATA.terminal_state.world.length&&state.attachment?.id===DATA.terminal_state.attachment?.id);
const originalIds=new Set(DATA.previews[0].state.world.filter(b=>b.category==='carton').map(b=>b.id));
const worldIds=state.world.filter(b=>originalIds.has(b.id)).map(b=>b.id),attachedIds=state.attachment?[state.attachment.id]:[],completedIds=state.completed_ids||[];
result.terminal_carton_counts={world:worldIds.length,attached:attachedIds.length,completed:completedIds.length};
check('TERMINAL_CARTON_IDS_PARTITION',new Set([...worldIds,...attachedIds,...completedIds]).size===originalIds.size&&worldIds.length+attachedIds.length+completedIds.length===originalIds.size&&[...worldIds,...attachedIds,...completedIds].every(id=>originalIds.has(id)));
check('TERMINAL_COUNTS_MATCH_VALIDATED_PLAN',worldIds.length===DATA.validation.world_cartons&&attachedIds.length===DATA.validation.attached_cartons&&completedIds.length===DATA.validation.completed_cartons);
if(DATA.plan_summary.complete)check('ONE_COMPLETE_NEXT_ONLY_ATTACHED',completedIds.length===1&&completedIds[0]===DATA.plan_summary.first_target_id&&attachedIds.length===1&&attachedIds[0]===DATA.plan_summary.next_target_id&&!completedIds.includes(DATA.plan_summary.next_target_id));
result.complete_task=DATA.plan_summary.complete?'TESTED':'PARTIAL_ONLY';
}else result.complete_task='NOT_RUN_NO_PLAN';
if(DATA.attempts.length){
document.getElementById('mode').value='search';modeChanged();
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
