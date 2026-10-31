// Presentation of recorded native failures only. No IK or collision calculation here.
const diagnosticGroup=new T.Group();scene.add(diagnosticGroup);
let diagnosticObjects=[],diagnosticBoxes=[],diagnosticWitness=null,filteredFailures=[];
let selectedFailureTarget='';
let displayedPosture=null;
const targetOf=a=>a.request.process_policy?.target_id||a.request.process_policy?.task_contract?.target_id;
const failureColors=[0xff506d,0xffc857];
const failedRequests=DATA.attempts.map((a,index)=>({a,index})).filter(({a})=>a.response.status!=='SUCCESS');
const statusLabels={NATIVE_IK_FAILED:'本次 IK 未求得解',NATIVE_PLANNING_FAILED:'原生运动规划失败',NATIVE_PATH_REJECTED:'原生路径检查拒绝',NATIVE_SOLVER_DEADLINE_EXPIRED:'原生求解达到截止时间',STATE_REJECTED:'状态检查拒绝'};
const stageLabels={pregrasp:'接近预抓取位',contact:'抓取接触',extraction:'脱垛抽离',transit:'带载转运',departure:'工具退出',return:'返回下一箱'};
function showPosture(frames,q){
  const shoulder=position(frames.link_2),elbow=position(frames.link_3),wrist=position(frames.link_6),tool=position(frames[model.tcp_link]);
  const boxes=DATA.previews[0].state.world.filter(b=>b.category==='carton');
  if(!boxes.length)return;
  const center=new T.Vector3();boxes.forEach(b=>center.add(position(matrix(b.pose))));center.multiplyScalar(1/boxes.length);
  const toward=center.sub(shoulder);toward.z=0;toward.normalize();
  const upper=elbow.clone().sub(shoulder).normalize();
  displayedPosture={upper_arm_tilt_deg:Math.acos(Math.max(-1,Math.min(1,upper.z)))*180/Math.PI,
    wrist_forward_of_elbow_m:wrist.clone().sub(elbow).dot(toward),tcp_forward_of_elbow_m:tool.clone().sub(elbow).dot(toward)};
  const p=displayedPosture;
  $('postureMetrics').textContent=`实际 FK 构型\n上臂（肩→肘）偏离 +Z：${p.upper_arm_tilt_deg.toFixed(2)}°\n腕部相对肘部朝箱堆方向：${(p.wrist_forward_of_elbow_m*1000).toFixed(1)} mm\n吸具 TCP 相对肘部：${(p.tcp_forward_of_elbow_m*1000).toFixed(1)} mm\n负值表示折回肩侧；原始 J2 ${(q[1]*180/Math.PI).toFixed(2)}°。\n${(active?.request?.ik_posture_preference||DATA.plan_summary?.ik_posture_preference)?'本次使用肩肘展开偏好（软排序）；显示的仍是原始记录姿态。':'这是已有姿态的 FK 测量，不代表旧规划已使用新的构型偏好。'}`;
}
function statusLabel(a){return statusLabels[a.response.status]||a.response.status||'未记录结果';}
function witnesses(a){return [...(a.failure_samples||[]),...(a.trace?.planners||[]).flatMap(p=>(p.checked_samples||[]).filter(s=>!s[1]).map(s=>({q:s[0],witness:s[2]})))];}
function hasPair(a){return witnesses(a).some(f=>pairOf(f.witness).length===2);}
function failureCategory(a){return a.response.status==='NATIVE_IK_FAILED'?'ik':a.response.status==='NATIVE_SOLVER_DEADLINE_EXPIRED'?'timeout':'other';}
function objectLabel(id){
  const r=robotObjects.find(r=>r.item.layer==='collision'&&(r.item.id===id||r.item.link===id));
  if(r){
    if(id.endsWith('_rigid'))return '吸盘刚性内件';
    if(id.endsWith('_lip'))return '吸盘柔性唇';
    if(model.joint_names.some((_,i)=>id==='link_'+(i+1))||id==='base_link')return '机械臂连杆';
    return '工具 / 安装件';
  }
  const b=state?.world.find(b=>b.id===id);
  if(b?.category==='carton')return id===state.target_id?'目标纸箱':'邻箱';
  if(b?.owner==='conveyor_transverse')return '横向输送机';
  if(b?.owner==='conveyor_longitudinal')return '纵向输送机';
  if(state?.attachment?.id===id)return '被抓箱体';
  return b?.role||'场景对象';
}
function clearDiagnosticGeometry(){
  clear(diagnosticGroup);diagnosticBoxes=[];diagnosticObjects=[];diagnosticWitness=null;
  for(const object of [...robotObjects.map(r=>r.object),...worldObjects]){
    const m=object.material;
    if(!object.userData.appearance)object.userData.appearance={opacity:m.opacity,transparent:m.transparent,depthWrite:m.depthWrite,depthTest:m.depthTest,renderOrder:object.renderOrder};
    const original=object.userData.appearance;Object.assign(m,{opacity:original.opacity,transparent:original.transparent,depthWrite:original.depthWrite,depthTest:original.depthTest});object.renderOrder=original.renderOrder;
  }
  for(const id of ['pairTagA','pairTagB'])$(id).hidden=true;
}
function highlightWitness(witness){
  diagnosticWitness=witness||null;const pair=pairOf(witness);if(pair.length!==2)return;
  scene.updateMatrixWorld(true);
  for(const object of [...robotObjects.map(r=>r.object),...worldObjects])if(object.userData.layer==='visual'){
    object.material.color.set(0x728797);object.material.transparent=true;object.material.opacity=Math.min(object.material.opacity,.24);object.material.depthWrite=false;
  }
  pair.forEach((id,index)=>{
    // Exact collider ID or owning physical link, never a broad name prefix.
    const objects=[...robotObjects.filter(r=>r.item.layer==='collision'&&(r.item.id===id||r.item.link===id)).map(r=>r.object),...worldObjects.filter(o=>o.userData.layer==='collision'&&o.userData.id===id)];
    const box=new T.Box3();
    for(const object of objects){
      object.material.color.set(failureColors[index]);object.material.opacity=.85;object.material.depthTest=false;object.material.depthWrite=false;object.renderOrder=1001;
      diagnosticObjects.push(object);box.union(new T.Box3().setFromObject(object));
    }
    if(box.isEmpty())return;
    const helper=new T.Box3Helper(box,failureColors[index]);helper.material.depthTest=false;helper.material.transparent=true;helper.renderOrder=1002;helper.userData.disposable=true;diagnosticGroup.add(helper);
    diagnosticBoxes.push({id,index,box});
  });
}
function updateDiagnosticLabels(){
  for(const tag of ['pairTagA','pairTagB'])$(tag).hidden=true;
  if($('mode').value!=='search')return;
  for(const {index,box}of diagnosticBoxes){
    const p=box.getCenter(new T.Vector3()).project(camera),tag=$(index?'pairTagB':'pairTagA');
    if(Math.abs(p.x)>1||Math.abs(p.y)>1||p.z>1||p.z< -1)continue;
    tag.hidden=false;tag.textContent=index?'B · 邻近对象':'A · 被检查对象';
    tag.style.left=((p.x+1)*view.clientWidth/2)+'px';tag.style.top=((1-p.y)*view.clientHeight/2+(index?24:-32))+'px';
  }
}
function focusFailure(){
  const box=new T.Box3();diagnosticBoxes.forEach(r=>box.union(r.box));if(box.isEmpty())return;
  const center=box.getCenter(new T.Vector3()),size=box.getSize(new T.Vector3()),radius=Math.max(.19,size.length()*.65);
  const direction=new T.Vector3(-1.1,-1.4,.85).normalize(),distance=radius/Math.sin(camera.fov*Math.PI/360)*1.8;
  // Leave room for the reason card to the right without moving any scene object.
  const right=new T.Vector3().crossVectors(direction,camera.up).negate().normalize();
  controls.target.copy(center).addScaledVector(right,distance*.16);camera.position.copy(controls.target).addScaledVector(direction,distance);controls.update();
}
function renderDiagnosis(witness=null,poseLabel='当前姿态仅供请求上下文参考；未定位到失败样本'){
  const visible=$('mode').value==='search'&&!!active;$('diagnosis').hidden=!visible;if(!visible)return;
  const a=active,r=a.response,w=witness?.original||witness,pair=pairOf(witness),index=DATA.attempts.indexOf(a);
  $('failedRequest').value=index;
  if(poseLabel==='{')poseLabel=a.request.op==='ik'?'当前显示本次 IK 种子，尚未选择失败样本。':'当前显示本请求起点，尚未选择失败样本。';
  $('diagnosisTitle').textContent=statusLabel(a);$('diagnosisStatus').textContent=`请求 #${index+1} · ${stageLabels[a.request.stage]||a.request.stage||a.request.op} · ${a.request.planner_id||a.request.op}`;
  let outcome=r.status==='NATIVE_IK_FAILED'?'本次原生 IK 未求得解。不能据此断言目标不可达；本请求未记录碰撞原因。':r.status==='NATIVE_SOLVER_DEADLINE_EXPIRED'?'本次求解达到截止时间。超时不代表已证明无可行路径。':r.status==='SUCCESS'?'此请求原生返回成功；下面的拒绝样本若存在，仅属于求解过程。':r.status==='STATE_REJECTED'?'该候选状态被原生检查拒绝。':'原生规划未返回可接受的完整运动。';
  if(r.solver_message)outcome+='\n返回：'+r.solver_message;
  $('diagnosisOutcome').textContent=outcome;
  $('diagnosisPose').textContent=witness?'当前为记录中的真实拒绝采样姿态（不是实际执行碰撞）':poseLabel;
  if(w?.reason==='NATIVE_ABSOLUTE_DEADLINE_EXPIRED')$('diagnosisPose').textContent='记录中的检查超时状态；未证明该姿态发生碰撞。';
  $('diagnosisPairs').replaceChildren();
  pair.forEach((id,i)=>{const row=document.createElement('div');row.className='pairRow '+(i?'pairB':'pairA');const label=document.createElement('b');label.textContent=(i?'B':'A')+' · '+objectLabel(id);const code=document.createElement('code');code.textContent=id;row.append(label,code);$('diagnosisPairs').append(row);});
  let reason='本请求没有当前姿态的碰撞见证；不生成碰撞位置。';
  if(w){
    if(w.reason==='CLEARANCE_INSUFFICIENT')reason='采样拒绝原因：间隙不足';
    else if(w.reason==='INTERSECTION_OR_CONTACT')reason='采样拒绝原因：检测到接触或相交';
    else if(w.reason==='J1_DIRECTION_VIOLATION')reason='采样拒绝原因：J1 运动方向违反要求';
    else if(w.reason==='NATIVE_ABSOLUTE_DEADLINE_EXPIRED')reason='检查达到截止时间，未获得该状态的完整检查结论。';
    else reason='采样拒绝原因：'+(w.reason||'未记录具体原因');
    if(w.from_q&&w.to_q){
      reason+=`\n原始 J1 ${w.from_q[0].toFixed(7)} → ${w.to_q[0].toFixed(7)} rad\n变化 ${(w.to_q[0]-w.from_q[0]).toExponential(4)} rad`;
      if(w.direction)reason+='\n要求：'+(w.direction<0?'顺时针 / 原始角非增':'逆时针 / 原始角非减');
    }
    const gap=w.surface_distance_m,required=w.required_pair_clearance_m;
    if(Number.isFinite(gap)&&gap>=0)reason+=`\n实测间隙 ${(gap*1000).toFixed(3)} mm`;
    else if(Number.isFinite(gap)&&gap<0)reason+='\n返回值是相交标记，未记录穿透深度。';
    if(Number.isFinite(required))reason+=`\n要求间隙 ${(required*1000).toFixed(3)} mm`;
    if(Number.isFinite(gap)&&gap>=0&&Number.isFinite(required)&&gap<required)reason+=` · 差 ${(1000*(required-gap)).toFixed(3)} mm`;
    if(w.phase)reason+='\n检查位置：'+({search:'搜索路径采样',initial:'起始状态',endpoint:'端点检查'}[w.phase]||w.phase);
  }
  $('diagnosisReason').textContent=reason;
  const rigid=pair.find(id=>id.endsWith('_rigid')),neighbor=pair.find(id=>id!==state.target_id&&state.world.some(b=>b.id===id&&b.category==='carton'));
  $('diagnosisPolicy').textContent=a.display_policy?.rigid_tool_carton_contact?'本请求已豁免具名刚性工具件与原有纸箱（含目标、附着及释放箱）的碰撞；实际吸盘唇密封接触及其他环境检查仍保留。':rigid&&neighbor&&a.display_policy?.cup_neighbor_contact?'此处是吸盘刚性内件。当前邻箱接触许可覆盖柔性吸盘唇，未覆盖该刚性件。':'';
  if(pair.some(id=>/^link_[1-6]$/.test(id))&&pair.some(id=>id.endsWith('_lip')))
    $('diagnosisPolicy').textContent='此处是吸盘与机械臂自身的接触。允许吸盘接触邻箱的策略不豁免这一对象对。';
  if(a.error){$('diagnosisOutcome').textContent+='\n记录错误：'+a.error;
    if(a.error==='NATIVE_ABSOLUTE_DEADLINE_EXPIRED')$('diagnosisReason').textContent='检查预算耗尽，未取得完整检查返回；不能认定此姿态碰撞或目标不可达。';}
  if(r.status==='SUCCESS')$('diagnosisOutcome').textContent+='\n'+(a.adopted?'该段纳入最终采用流程。':'该段属于未完成箱的候选分支，未纳入最终采用流程。');
  const round=Number.isFinite(a.round_trip_s)?a.round_trip_s.toFixed(3)+' s':'未记录';
  $('diagnosisContext').textContent=`目标 ${state.target_id||'未记录'}\n横带 ${Number(state.height_m).toFixed(3)} m · ${state.attachment?'已附着 '+state.attachment.id:'空载'}\n原始 J1 ${currentQ[0].toFixed(6)} rad · 本请求往返墙钟 ${round}`;
  $('diagnosisNote').textContent=pair.length?'彩色实体与外框标出记录中的两个对象；环境半透明辅助观察。未采集接触点，不绘制虚构接触位置。':'当前仅显示该请求自己的场景与姿态参考，不是推测出的失败接触位。';
  const missing=pair.filter(id=>!diagnosticBoxes.some(b=>b.id===id));if(missing.length)$('diagnosisNote').textContent+='\n几何未定位：'+missing.join(', ');
  $('focusFailure').disabled=!diagnosticBoxes.length;
  $('diagnosisRaw').textContent=JSON.stringify({request_index:index,request_id:a.request.request_id,stage_id:a.request.stage_id,status:r.status,solver_message:r.solver_message,error_code:r.error_code??r.moveit_error_code,budget:r.budget,failure:r.failure,witness:witness||null,q:currentQ,height_m:state.height_m,trace_status:a.trace_status},null,2);
}
function fillFailureCatalogue(){
  const filter=$('failureFilter').value,old=$('failedRequest').value;
  filteredFailures=failedRequests.filter(({a})=>(!selectedFailureTarget||targetOf(a)===selectedFailureTarget)&&(filter==='all'||(filter==='collision'?hasPair(a):failureCategory(a)===filter)));
  $('failedRequest').replaceChildren(...filteredFailures.map(({a,index})=>new Option(`#${index+1} · ${stageLabels[a.request.stage]||a.request.stage||a.request.op} · ${statusLabel(a)}${hasPair(a)?' · 有几何见证':''}`,index)));
  if(filteredFailures.some(r=>String(r.index)===old))$('failedRequest').value=old;
  $('failurePosition').textContent=`${selectedFailureTarget||'全部箱体'} · 筛选 ${filteredFailures.length} 个失败请求 · 几何见证可与超时重叠`;
  $('previousFailure').disabled=$('nextFailure').disabled=!filteredFailures.length;
}
function openFailure(index,focus=true){
  if(!DATA.attempts[index])return;
  $('mode').value='search';$('request').value=index;modeChanged();$('failedRequest').value=index;
  // Every sample retains its recorded request world or explicit transition world.
  let sample=failures.findIndex(f=>pairOf(f.witness).length===2);
  if(sample<0&&failures.length)sample=0;
  if(sample>=0){inspectFailure(sample);if(focus){if(diagnosticBoxes.length)focusFailure();else reset();}}
  else {reset();renderDiagnosis(null,active.request.op==='ik'?'当前显示本次 IK 种子；没有返回解，也没有失败碰撞姿态。':'当前显示本请求起点；未采集可定位的失败状态。');}
}
function stepFailure(delta){const index=filteredFailures.findIndex(r=>String(r.index)===$('failedRequest').value);if(filteredFailures.length)openFailure(filteredFailures[(index+delta+filteredFailures.length)%filteredFailures.length].index);}
function openFirstFailure(){
  $('failureFilter').value='all';fillFailureCatalogue();const first=filteredFailures.find(({a})=>hasPair(a))||filteredFailures[0];
  layers.trajectory=false;document.querySelector('[data-layer="trajectory"]').checked=false;
  if(first)openFailure(first.index);
}

// Target inspection navigates original requests individually; it never stitches
// rejected branches into the accepted task trajectory or recalculates IK.
function targetRows(){return DATA.attempts.map((a,index)=>({a,index})).filter(({a})=>!selectedFailureTarget||targetOf(a)===selectedFailureTarget);}
function targetEvidenceRows(){return targetRows().filter(({a})=>
  a.request.op==='plan'||
  (a.request.op==='check_state_offline'&&a.request.stage==='contact'&&a.response.status==='SUCCESS')||
  a.request.transition?.event_type==='ATTACH'||
  (a.request.stage==='transit'&&a.response.status!=='SUCCESS'&&a.request.op!=='ik'));}
function targetQuick(kind){
  const rows=targetRows(),predicates={
    collision:a=>a.request.stage==='transit'&&hasPair(a),
    approach:a=>a.request.stage==='pregrasp'&&hasPair(a),
    contact_ik:a=>a.request.op==='check_state_offline'&&a.request.stage==='contact'&&a.response.status==='SUCCESS',
    direction:a=>witnesses(a).some(f=>(f.witness.original||f.witness).reason==='J1_DIRECTION_VIOLATION'),
    transfer:a=>a.request.stage==='transit'&&a.request.op==='plan'&&a.response.status!=='SUCCESS',
    budget:a=>a.error==='NATIVE_ABSOLUTE_DEADLINE_EXPIRED',
    extraction:a=>a.request.stage==='extraction'&&a.request.op==='plan'&&a.response.status==='SUCCESS',
    contact:a=>a.request.stage==='contact'&&a.request.op==='plan'&&a.response.status==='SUCCESS'};
  const row=rows.find(({a})=>predicates[kind](a));if(row)openTargetEvidence(row.index);
}
function openTargetEvidence(index){
  const a=DATA.attempts[index];if(!a||targetOf(a)!==selectedFailureTarget)return;
  $('targetEvidence').value=index;layers.trajectory=false;document.querySelector('[data-layer="trajectory"]').checked=false;
  openFailure(index);
  if(a.response.status==='SUCCESS'&&a.native_path?.length)setRequestTime(a.native_path.at(-1).t);
  $('targetEdge').hidden=!witnesses(a).some(f=>(f.witness.original||f.witness).from_q);
  $('targetPath').disabled=!a.native_path?.length;
  $('targetEvidenceNote').textContent=`请求 #${index+1} · ${a.request.stage_id||a.request.op}\n此前已完成 ${a.request.completed_ids?.length||0} 箱 · ${statusLabel(a)}\n${a.adopted?'纳入最终流程':'候选分支；未纳入主回放'}。每个请求单独使用其原始场景。`;
}
function inspectTargetEdge(end){
  const f=witnesses(active).find(f=>(f.witness.original||f.witness).from_q);if(!f)return;
  const w=f.witness.original||f.witness;restoreRequestWorld();pose(w[end],end==='from_q'?'记录中违规边起点':'记录中违规边终点',f.witness);
  $('diagnosisPose').textContent=end==='from_q'?'显示原生记录的违规边起点。':'显示原生记录的违规边终点。';summaryState('J1 违规边 · '+(end==='from_q'?'起点':'终点'));
}
function selectInspectionTarget(id,open=true){
  selectedFailureTarget=id;$('diagnosticTarget').value=id;$('failureFilter').value='all';fillFailureCatalogue();
  const rows=targetRows(),success=(stage,event)=>rows.filter(({a})=>a.response.status==='SUCCESS'&&(event?a.request.transition?.event_type===event:a.request.op==='plan'&&a.request.stage===stage)).length;
  $('targetSummary').textContent=`${id}\n${rows.length} 条原生请求 · 附着成功 ${success(null,'ATTACH')} 次 · 抽离成功 ${success('extraction')} 次 · 释放成功 ${success(null,'RELEASE')} 次\n成功候选段不等于整箱完成。此处保留未被最终采用的尝试。`;
  $('targetEvidence').replaceChildren(...targetEvidenceRows().map(({a,index})=>new Option(`#${index+1} · ${a.request.stage_id||a.request.op} · ${statusLabel(a)}`,index)));
  const budget=(DATA.plan_summary?.failures||[]).filter(f=>f.target_id===id&&String(f.error).includes('BUDGET'));
  $('targetBudget').textContent=budget.length?`${budget.length} 条调度预算耗尽记录（不是 ${budget.length} 个独立原生失败姿态）。\n`+JSON.stringify(budget,null,2):'已接受前缀中没有该箱的调度预算耗尽记录。';
  const dispatch=(DATA.plan_summary?.continuation_search||[]).flatMap(s=>s.targets.filter(t=>t.target_id===id).map(t=>({...t,after_completed:s.completed_ids.length,attempts:s.attempts.filter(a=>a.target_id===id)})));
  const last=dispatch.at(-1);
  if(last){
    const reasons={PROCESS_RECEIVE_HEIGHT_OUT_OF_RANGE:'接收高度超出机构范围',LATERAL_CHANNEL_BLOCKED:'侧移通道被邻箱挡住',FACE_NOT_EXPOSED:'抓面未暴露',NO_INWARD_LATERAL_SPACE:'内侧移空间不足'};
    const frozen=last.process_options.frozen;
    if(frozen&&DATA.receiver_hardware_range_m){
      const permission=frozen.top_permission==='EXPOSURE_SEAL_NATIVE_COLLISION_COMPLETE_CYCLE'
        ?'无高度门槛；仍须满足顶面暴露、实际密封和完整周期碰撞检查'
        :(frozen.top_allowed?'历史策略允许':'历史策略不允许（箱底 ≥ 100 mm）');
      $('targetSummary').textContent+=`\n冻结箱底 ${(frozen.z_b_m*1000).toFixed(1)} mm；横带范围 ${DATA.receiver_hardware_range_m.map(h=>(h*1000).toFixed(0)).join('–')} mm；顶吸：${permission}。`;
    }
    const rejected=last.process_options.rejected.map(r=>`${r.template||'工艺'}：${reasons[r.reason]||r.reason}${Number.isFinite(r.height_m)?'（候选 '+(r.height_m*1000).toFixed(1)+' mm）':''}${r.blocker_ids?.length?'；阻挡 '+r.blocker_ids.join(', '):''}`);
    $('targetSummary').textContent+=`\n完成 ${last.after_completed} 箱后的调度：${last.candidate_count} 个几何候选；本轮尝试 ${last.attempts.length} 次。\n`+rejected.join('\n');
    $('targetBudget').textContent+='\n'+JSON.stringify(dispatch,null,2);
  }
  if(!rows.length){
    $('targetSummary').textContent+='\n未发起该箱原生请求；这不是 IK 不可达的证明。';
    if(open){
      $('mode').value='cycle';modeChanged();setTime(DATA.duration_s);
      const box=new T.Box3();worldObjects.filter(o=>o.userData.id===id&&o.userData.layer==='visual').forEach(o=>{o.material.color.set(0xffc857);box.union(new T.Box3().setFromObject(o));});
      if(!box.isEmpty()){const helper=new T.Box3Helper(box,0xffc857);helper.userData.disposable=true;diagnosticGroup.add(helper);}
      summaryState('终态箱体 · 未完成 '+id);
    }
  }
  if(open){const first=rows.find(({a})=>a.request.stage==='transit'&&hasPair(a))||rows.find(({a})=>hasPair(a))||rows.find(({a})=>a.response.status!=='SUCCESS');if(first)openTargetEvidence(first.index);}
}
function initTargetInspection(){
  const ids=[...new Set([...(DATA.plan_summary?.inventory_ids||[]),...DATA.attempts.map(targetOf).filter(Boolean)])];if(!ids.length)return;
  $('targetInspection').hidden=false;$('diagnosticTarget').replaceChildren(...ids.map(id=>new Option(id,id)));
  $('diagnosticTarget').onchange=()=>selectInspectionTarget($('diagnosticTarget').value);
  $('targetEvidence').onchange=()=>openTargetEvidence(Number($('targetEvidence').value));
  document.querySelectorAll('[data-target-case]').forEach(b=>b.onclick=()=>targetQuick(b.dataset.targetCase));
  $('targetEdgeStart').onclick=()=>inspectTargetEdge('from_q');$('targetEdgeEnd').onclick=()=>inspectTargetEdge('to_q');
  $('targetPath').onclick=()=>{if(active?.native_path?.length){setRequestTime(0);requestPlaying=true;}};
  const id=document.documentElement.dataset.focusTarget||(ids.length===1?ids[0]:null);
  if(id&&ids.includes(id)){selectInspectionTarget(id);document.querySelector('aside').scrollTop=0;}
  else $('targetSummary').textContent='选择箱体，查看其独立候选请求和失败证据。';
}
function initFailureDiagnostics(){
  const counts=new Map();failedRequests.forEach(({a})=>counts.set(statusLabel(a),(counts.get(statusLabel(a))||0)+1));
  $('failureSummary').textContent=(DATA.plan_summary?.complete?'本次完整流程已成功。以下是规划期间失败的候选请求。':'本次流程未完成。以下为实际记录的失败请求。')+`\n${DATA.attempts.length} 个请求 · ${failedRequests.length} 个失败`;
  $('failureCounts').textContent=[...counts].map(([name,n])=>`${name} ${n}`).join(' · ');
  fillFailureCatalogue();$('openFailures').disabled=!failedRequests.length;
  $('openFailures').onclick=openFirstFailure;$('backToCycle').onclick=()=>{$('mode').value=DATA.segments.length?'cycle':'preview';modeChanged();reset();};
  $('failureFilter').onchange=()=>{fillFailureCatalogue();if(filteredFailures.length)openFailure(Number($('failedRequest').value));};
  $('failedRequest').onchange=()=>openFailure(Number($('failedRequest').value));
  $('previousFailure').onclick=()=>stepFailure(-1);$('nextFailure').onclick=()=>stepFailure(1);
  $('failure').onchange=()=>{inspectFailure(Number($('failure').value));focusFailure();};
  $('focusFailure').onclick=focusFailure;
}
