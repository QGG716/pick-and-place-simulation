// Presentation of recorded native failures only. No IK or collision calculation here.
const diagnosticGroup=new T.Group();scene.add(diagnosticGroup);
let diagnosticObjects=[],diagnosticBoxes=[],diagnosticWitness=null,filteredFailures=[];
const failureColors=[0xff506d,0xffc857];
const failedRequests=DATA.attempts.map((a,index)=>({a,index})).filter(({a})=>a.response.status!=='SUCCESS');
const statusLabels={NATIVE_IK_FAILED:'本次 IK 未求得解',NATIVE_PLANNING_FAILED:'原生运动规划失败',NATIVE_SOLVER_DEADLINE_EXPIRED:'原生求解达到截止时间',STATE_REJECTED:'状态检查拒绝'};
const stageLabels={pregrasp:'接近预抓取位',contact:'抓取接触',extraction:'脱垛抽离',transit:'带载转运',departure:'工具退出',return:'返回下一箱'};
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
  $('diagnosisPairs').replaceChildren();
  pair.forEach((id,i)=>{const row=document.createElement('div');row.className='pairRow '+(i?'pairB':'pairA');const label=document.createElement('b');label.textContent=(i?'B':'A')+' · '+objectLabel(id);const code=document.createElement('code');code.textContent=id;row.append(label,code);$('diagnosisPairs').append(row);});
  let reason='本请求没有当前姿态的碰撞见证；不生成碰撞位置。';
  if(w){
    if(w.reason==='CLEARANCE_INSUFFICIENT')reason='采样拒绝原因：间隙不足';
    else if(w.reason==='INTERSECTION_OR_CONTACT')reason='采样拒绝原因：检测到接触或相交';
    else reason='采样拒绝原因：'+(w.reason||'未记录具体原因');
    const gap=w.surface_distance_m,required=w.required_pair_clearance_m;
    if(Number.isFinite(gap)&&gap>=0)reason+=`\n实测间隙 ${(gap*1000).toFixed(3)} mm`;
    else if(Number.isFinite(gap)&&gap<0)reason+='\n返回值是相交标记，未记录穿透深度。';
    if(Number.isFinite(required))reason+=`\n要求间隙 ${(required*1000).toFixed(3)} mm`;
    if(Number.isFinite(gap)&&gap>=0&&Number.isFinite(required)&&gap<required)reason+=` · 差 ${(1000*(required-gap)).toFixed(3)} mm`;
    if(w.phase)reason+='\n检查位置：'+({search:'搜索路径采样',initial:'起始状态',endpoint:'端点检查'}[w.phase]||w.phase);
  }
  $('diagnosisReason').textContent=reason;
  const rigid=pair.find(id=>id.endsWith('_rigid')),neighbor=pair.find(id=>id!==state.target_id&&state.world.some(b=>b.id===id&&b.category==='carton'));
  $('diagnosisPolicy').textContent=rigid&&neighbor&&a.display_policy?.cup_neighbor_contact?'此处是吸盘刚性内件。当前邻箱接触许可覆盖柔性吸盘唇，未覆盖该刚性件。':'';
  const round=Number.isFinite(a.round_trip_s)?a.round_trip_s.toFixed(3)+' s':'未记录';
  $('diagnosisContext').textContent=`目标 ${state.target_id||'未记录'}\n横带 ${Number(state.height_m).toFixed(3)} m · ${state.attachment?'已附着 '+state.attachment.id:'空载'}\n原始 J1 ${currentQ[0].toFixed(6)} rad · 本请求往返墙钟 ${round}`;
  $('diagnosisNote').textContent=pair.length?'彩色实体与外框标出记录中的两个对象；环境半透明辅助观察。未采集接触点，不绘制虚构接触位置。':'当前仅显示该请求自己的场景与姿态参考，不是推测出的失败接触位。';
  const missing=pair.filter(id=>!diagnosticBoxes.some(b=>b.id===id));if(missing.length)$('diagnosisNote').textContent+='\n几何未定位：'+missing.join(', ');
  $('focusFailure').disabled=!diagnosticBoxes.length;
  $('diagnosisRaw').textContent=JSON.stringify({request_index:index,request_id:a.request.request_id,stage_id:a.request.stage_id,status:r.status,solver_message:r.solver_message,error_code:r.error_code??r.moveit_error_code,budget:r.budget,failure:r.failure,witness:witness||null,q:currentQ,height_m:state.height_m,trace_status:a.trace_status},null,2);
}
function fillFailureCatalogue(){
  const filter=$('failureFilter').value,old=$('failedRequest').value;
  filteredFailures=failedRequests.filter(({a})=>filter==='all'||(filter==='collision'?hasPair(a):failureCategory(a)===filter));
  $('failedRequest').replaceChildren(...filteredFailures.map(({a,index})=>new Option(`#${index+1} · ${stageLabels[a.request.stage]||a.request.stage||a.request.op} · ${statusLabel(a)}${hasPair(a)?' · 有几何见证':''}`,index)));
  if(filteredFailures.some(r=>String(r.index)===old))$('failedRequest').value=old;
  $('failurePosition').textContent=`筛选 ${filteredFailures.length} / ${failedRequests.length} 个失败请求 · 几何见证可与超时重叠`;
  $('previousFailure').disabled=$('nextFailure').disabled=!filteredFailures.length;
}
function openFailure(index,focus=true){
  if(!DATA.attempts[index])return;
  $('mode').value='search';$('request').value=index;modeChanged();$('failedRequest').value=index;
  // Every sample retains its recorded request world or explicit transition world.
  let sample=failures.findIndex(f=>pairOf(f.witness).length===2);
  if(sample<0&&failures.length)sample=0;
  if(sample>=0){inspectFailure(sample);if(focus)focusFailure();}
  else {reset();renderDiagnosis(null,active.request.op==='ik'?'当前显示本次 IK 种子；没有返回解，也没有失败碰撞姿态。':'当前显示本请求起点；未采集可定位的失败状态。');}
}
function stepFailure(delta){const index=filteredFailures.findIndex(r=>String(r.index)===$('failedRequest').value);if(filteredFailures.length)openFailure(filteredFailures[(index+delta+filteredFailures.length)%filteredFailures.length].index);}
function openFirstFailure(){
  $('failureFilter').value='all';fillFailureCatalogue();const first=failedRequests.find(({a})=>hasPair(a))||failedRequests[0];
  layers.trajectory=false;document.querySelector('[data-layer="trajectory"]').checked=false;
  if(first)openFailure(first.index);
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
