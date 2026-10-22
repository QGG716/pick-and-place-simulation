"""Directed UI checks for selecting one carton's recorded failure evidence."""
import check_eco65_coupled_replay_browser as browser

browser.CHECK=r'''
<script>setTimeout(()=>{const result={checks:[],status:'RUNNING'};
const check=(key,value)=>{if(!value)throw Error(key);result.checks.push(key);};
const same=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
const context=()=>{check('OWN_TARGET',state.target_id===selectedFailureTarget);
check('OWN_WORLD',state.world===active.request.world);check('OWN_ATTACHMENT',state.attachment===active.request.attachment);
check('OWN_HEIGHT',state.height_m===active.request.mechanism_state.height_m);};
const click=kind=>document.querySelector('[data-target-case="'+kind+'"]').click();
try{
check('READY',window.REPLAY_READY===true);
check('EXACT_TARGET_SELECTED',selectedFailureTarget==='carton_r00_l03_c02');
check('TARGET_REQUEST_COUNT',targetRows().length===376);
check('TARGET_FAILURE_FILTER',filteredFailures.length>0&&filteredFailures.every(({a})=>targetOf(a)===selectedFailureTarget));
context();check('DEFAULT_LOADED_COLLISION',active.request.stage==='transit'&&active.response.status==='STATE_REJECTED');
check('EXACT_PAIR',same(window.replay.highlightedIds,['link_2','part_008_lip']));
check('ACTUAL_REJECTED_POSE',same(currentQ,active.failure_samples[0].q));
check('GEOMETRY_VISIBLE',diagnosticObjects.length>0&&diagnosticObjects.every(o=>o.visible));
check('POLICY_SCOPE_EXPLAINED',document.getElementById('diagnosisPolicy').textContent.includes('机械臂自身'));
check('FOLDED_UPPER_ARM_MEASURED',Math.abs(displayedPosture.upper_arm_tilt_deg-74.086)<.01&&displayedPosture.wrist_forward_of_elbow_m<-.04);
click('contact');context();setRequestTime(active.native_path.at(-1).t);
check('ACTUAL_UPRIGHT_CONTACT',active.request.stage_id==='contact-362'&&Math.abs(displayedPosture.upper_arm_tilt_deg-9.429)<.01&&displayedPosture.wrist_forward_of_elbow_m>.29);
click('direction');context();const f=witnesses(active).find(f=>(f.witness.original||f.witness).from_q),w=f.witness.original||f.witness;
check('J1_REQUEST',active.request.stage_id==='extraction-364');
check('EXACT_RAW_J1_EDGE',w.from_q[0]===0.06046834454753859&&w.to_q[0]===0.060538323456379765&&w.direction===-1);
document.getElementById('targetEdgeStart').click();check('EDGE_START',same(currentQ,w.from_q));context();
document.getElementById('targetEdgeEnd').click();check('EDGE_END',same(currentQ,w.to_q));
check('NO_FALSE_J1_COLLISION',window.replay.highlightedIds.length===0);
click('extraction');context();check('REAL_SUCCESSFUL_CANDIDATE',active.request.stage_id==='extraction-370'&&active.response.status==='SUCCESS'&&!active.adopted);
check('CANDIDATE_NOT_TASK_SUCCESS',document.getElementById('diagnosisOutcome').textContent.includes('未纳入'));
document.getElementById('targetPath').click();check('LOCAL_NATIVE_PLAY',requestPlaying);
requestPlaying=false;setRequestTime(active.native_path.at(-1).t);check('LOCAL_NATIVE_ENDPOINT',same(currentQ,active.native_path.at(-1).q));context();
const second=DATA.attempts.findIndex(a=>a.request.stage_id==='attach-637');openTargetEvidence(second);context();
check('SECOND_TRIAL_ORIGINAL_WORLD',state.completed_ids.length===2);
click('transfer');context();check('ACTUAL_TRANSFER_FAILURE',active.request.stage_id==='transit-399'&&active.response.solver_message==='INVALID_MOTION_PLAN');
click('budget');context();check('ACTUAL_CHECK_TIMEOUT',active.error==='NATIVE_ABSOLUTE_DEADLINE_EXPIRED');
check('TIMEOUT_NOT_COLLISION',window.replay.highlightedIds.length===0&&document.getElementById('diagnosisReason').textContent.includes('预算耗尽'));
check('SCHEDULER_RECORDS_DISTINCT',document.getElementById('targetBudget').textContent.includes('独立原生失败姿态'));
click('collision');context();document.querySelector('aside').scrollTop=0;controls.update();updateDiagnosticLabels();renderer.render(scene,camera);
result.selected_request={index:DATA.attempts.indexOf(active),target:state.target_id,pair:window.replay.highlightedIds,q:currentQ,height_m:state.height_m};
result.status='PASS';
}catch(error){result.status='FAIL';result.error=error.stack;}
const pre=document.createElement('pre');pre.id='browser-check-result';pre.textContent=JSON.stringify(result);document.body.append(pre);
},500);</script>'''

if __name__=='__main__':browser.main()
