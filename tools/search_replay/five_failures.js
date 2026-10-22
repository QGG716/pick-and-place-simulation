// Read-only annotations over the existing native-q renderer. No solver, motion or FK replacement.
const diagnosticStyle=document.createElement('style');
diagnosticStyle.textContent=`main{grid-template-columns:360px 1fr}aside>section,aside>details{display:none}aside>#failurePanel{display:block;border:0}#hud{top:auto;bottom:58px;font-size:13px;max-width:520px}#failurePanel h2{font-size:20px;margin:14px 0 8px}#failurePanel p{line-height:1.65;margin:8px 0}#failurePanel .case-buttons{display:grid;gap:6px}#failurePanel .case-buttons button{text-align:left;padding:9px 10px}#failurePanel .case-buttons button.active{border-color:#69d9ea;background:#19485b}#failurePanel .stage-row{display:flex;flex-wrap:wrap;gap:5px;margin:14px 0}.stage-chip{padding:4px 6px;border-radius:4px;font-size:12px;background:#263346}.stage-chip.ok{color:#71e5b5}.stage-chip.bad{color:#ffb575}.stage-chip.idle{color:#8b9aaf}#failurePanel .legend{font-size:12px;color:#b6c9dc}#failurePanel select{width:100%}#failurePanel details{font-size:12px;margin-top:14px}#failurePanel .counter{font-size:12px;color:#a9bfd4}header h1{font-size:20px}#notice{font-size:11px}.evidence-title{color:#ffc789}#failurePanel #regionControls[hidden]{display:none}@media(max-width:900px){main{grid-template-columns:300px 1fr}aside{padding:10px}}`;
document.head.append(diagnosticStyle);
document.title=DATA.inspection_title||'5 箱失败原因 · 本轮原生记录';
document.querySelector('h1').textContent=DATA.inspection_title||'5 箱失败原因 · 三维检查';
document.querySelector('header .badge').innerHTML=DATA.inspection_subtitle||'本次规划记录 · 非物理执行<br>机器人安装 −X / −Y 各 100 mm';
$('identity').textContent='实测源码 '+DATA.raw.source_commit.slice(0,12)+' · 拖动旋转 / 滚轮缩放 / 右键平移';
const failurePanel=document.createElement('section');failurePanel.id='failurePanel';
failurePanel.innerHTML=`<div class="case-buttons">${DATA.inspection_cases.map((c,i)=>`<button data-case="${i}">${c.target_id.replace('carton_','')} · ${c.kind==='collision'?'工具碰撞':c.kind==='region'?'抽离未完成':'IK 未求解'}</button>`).join('')}</div>
<h2 id="failureTitle"></h2><p id="failureDetail"></p><div id="failureStages" class="stage-row"></div><p id="failureMetrics" class="counter"></p>
<div id="regionControls" hidden><label>本次实际派发的接收位置 <input id="regionSample" type="range" min="0" value="0"></label><div id="regionSampleInfo" class="counter"></div><p class="legend">青色点：尝试过的箱心；线框箱：选中的请求终点。虚线仅示意期望 −X 抽离，不是生成轨迹。</p></div>
<p class="legend">橙色：目标箱<br>红色：有本次见证的碰撞对象<br>青色：请求位置或实际搜索端点</p>
<button id="focusFailure">聚焦失败处</button> <button id="wholeFailure">全局视角</button>
<label><input type="checkbox" id="diagnosticLabels" checked> 显示三维标签</label>
<label><input type="checkbox" id="diagnosticGhosts" checked> 显示请求位置</label>
<label>背景透明度 <input id="failureOpacity" type="range" min="0" max=".8" step=".05" value=".15"></label>
<details><summary>本次原始状态与证据</summary><pre id="failureEvidence"></pre></details>
<p class="counter">${DATA.inspection_summary||'5 箱都未完成完整抓放。图中每个箱的环境取自它当时的请求，不是将最终剩余场景套到所有任务上。'}</p>`;
document.querySelector('aside').prepend(failurePanel);
const annotationGroup=new T.Group(),labelGroup=new T.Group(),sampleGroup=new T.Group();scene.add(annotationGroup,labelGroup,sampleGroup);
let failureIndex=0;
function diagnosticLabel(text,point,color='#e2edf7'){
  const canvas=document.createElement('canvas');canvas.width=768;canvas.height=96;const ctx=canvas.getContext('2d');
  ctx.fillStyle='#0b1425ee';ctx.fillRect(0,0,768,96);ctx.fillStyle=color;ctx.font='bold 32px "Microsoft YaHei",sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,384,48);
  const texture=new T.CanvasTexture(canvas),sprite=new T.Sprite(new T.SpriteMaterial({map:texture,depthTest:false}));
  sprite.position.copy(point);sprite.scale.set(1.0,.125,1);sprite.renderOrder=10;labelGroup.add(sprite);return sprite;
}
function targetCenter(){return position(matrix(DATA.inspection_cases[failureIndex].target_box.pose));}
function styleFailureWorld(){
  const c=DATA.inspection_cases[failureIndex],pair=c.collision?.pair||[];
  for(const o of obstacles){const selected=o.userData.name===c.target_id,bad=pair.includes(o.userData.name);o.material.opacity=selected?.78:bad?.76:Number($('failureOpacity').value);o.material.color.set(bad?0xff304d:selected?0xf9a35b:o.userData.baseColor);}
  for(const [o,l] of modelObjects){if(pair.includes(l.link))o.material.color.set(0xff304d);}
  if(payload)payload.material.opacity=.8;
}
function focusFailure(){
  const c=DATA.inspection_cases[failureIndex],p=targetCenter();
  if(c.kind==='collision'){camera.position.set(-1.7,p.y-2.8,2.65);controls.target.set(.15,p.y,1.4);}
  else if(c.kind==='region'){camera.position.set(-3.8,p.y-3.6,3.3);controls.target.set(-.75,p.y,p.z-.1);}
  else{camera.position.set(-3.8,-4.2,3.2);controls.target.set(-.6,-.45,1.15);}
  controls.update();
}
function showRegionSample(index){
  const c=DATA.inspection_cases[failureIndex];clear(sampleGroup);if(c.kind!=='region')return;
  const sample=c.region_samples[index],point=new T.Vector3(...sample.center_world_m),start=targetCenter();
  const ghost=new T.LineSegments(new T.EdgesGeometry(new T.BoxGeometry(...c.target_box.size)),new T.LineBasicMaterial({color:0x66e3ed}));ghost.position.copy(point);sampleGroup.add(ghost);
  sampleGroup.add(line([start,point],0x66e3ed,.8,true));
  $('regionSampleInfo').textContent=`${index+1} / ${c.region_samples.length} · 箱心 (${sample.center_world_m.map(v=>v.toFixed(4)).join(', ')}) m\n${sample.native_result?.solver_message||sample.native_result?.status} · 未到达`;
  sampleGroup.visible=$('diagnosticGhosts').checked;
}
function showFailure(index,updateHash=true){
  failureIndex=index;const c=DATA.inspection_cases[index];
  attempts=DATA.raw.attempts.map(a=>({a,p:null}));$('attempt').innerHTML=attempts.map((_,i)=>`<option value="${i}">${i}</option>`).join('');$('attempt').value=index;
  stage=attempts[index].a.request.stage;chooseAttempt();
  pose(c.display_q,c.state_label);clear(framesGroup);clear(annotationGroup);clear(labelGroup);clear(sampleGroup);
  styleFailureWorld();const p=targetCenter();
  diagnosticLabel(c.target_id.replace('carton_','')+' · 目标箱',p.clone().add(new T.Vector3(.38,0,-.20)),'#ffc17a');
  if(c.kind==='collision'){
    const blocker=obstacles.find(o=>(c.collision.pair||[]).includes(o.userData.name));
    if(blocker)diagnosticLabel(blocker.userData.name.replace('carton_','')+' · 碰撞箱',position(blocker.matrix).add(new T.Vector3(.1,0,.28)),'#ff7186');
    const rigid=modelObjects.find(([o,l])=>c.collision.pair.includes(l.link));
    if(rigid)diagnosticLabel('tool_rigid_0 · 刚性工具',position(rigid[0].matrix).add(new T.Vector3(-.35,0,.32)),'#ff7186');
  }else if(c.kind==='region'){
    const points=c.region_samples.map(r=>new T.Vector3(...r.center_world_m));
    annotationGroup.add(new T.Points(new T.BufferGeometry().setFromPoints(points),new T.PointsMaterial({color:0x66e3ed,size:.018})));
    for(const polygon of c.receiving_region?.polygons_world_m||[])annotationGroup.add(line([...polygon,polygon[0]].map(x=>new T.Vector3(...x)),0x64dfad));
    diagnosticLabel('接收端点：已搜索，未到达',points[0].clone().add(new T.Vector3(-.05,0,.34)),'#7de7f0');
    $('regionSample').max=c.region_samples.length-1;$('regionSample').value=c.region_samples.length-1;showRegionSample(c.region_samples.length-1);
  }else{
    const g=matrix(active.request.goal_pose),axis=new T.AxesHelper(.22);axis.matrixAutoUpdate=false;axis.matrix.copy(g);annotationGroup.add(axis);
    const point=position(g),marker=new T.Mesh(new T.SphereGeometry(.025,12,8),new T.MeshBasicMaterial({color:0x66e3ed}));marker.position.copy(point);annotationGroup.add(marker);
    diagnosticLabel('IK 请求 TCP · 没有解',point.clone().add(new T.Vector3(0,0,.28)),'#7de7f0');
  }
  annotationGroup.visible=sampleGroup.visible=$('diagnosticGhosts').checked;labelGroup.visible=$('diagnosticLabels').checked;
  $('failureTitle').textContent=c.title;$('failureDetail').textContent=c.detail;
  const stages=[['数值 IK',(c.counts.numerical_ik_success||0)>0],['接近',!!c.counts.pregrasp],['contact',!!c.counts.contact],['抽离',false],['完整抓放',false]];
  const failAt=c.kind==='region'?3:0;
  $('failureStages').innerHTML=stages.map(([name,ok],i)=>`<span class="stage-chip ${ok?'ok':i===(c.kind==='collision'?1:failAt)?'bad':'idle'}">${ok?'✓':'○'} ${name}</span>`).join('');
  $('failureMetrics').textContent=`本箱用时 ${c.task_wall_s.toFixed(1)} s · 带高 ${c.height_m.toFixed(2)} m · delta ${c.delta_m.toFixed(3)} m\n数值 IK 成功 ${c.counts.numerical_ik_success||0} · 命令杯 ${c.commanded_cups} · 实际 contact ${c.actual_contact_cups===null?'未到达':c.actual_contact_cups+' 杯'}${c.kind==='region'?'\n346 个区域端点；另有 3 个流程选项未探索':''}`;
  $('failureMetrics').style.whiteSpace='pre-line';$('regionControls').hidden=c.kind!=='region';
  $('failureEvidence').textContent=JSON.stringify({source_commit:DATA.raw.source_commit,display_state:c.state_label,q_rad:c.display_q,stage_id:active.request.stage_id,scene_fingerprint:active.request.scene_fingerprint,collision:c.collision,native_response:c.native_response,final_failure:c.failure.reason,termination:c.failure.termination||'STAGE_BUDGET_EXHAUSTED',no_global_unreachability_claim:true},null,2);
  $('hud').innerHTML=`<b>${c.target_id.replace('carton_','')} · ${c.state_label}</b><br>${c.kind==='region'?'规划请求中已附着 · 48 杯有效 · 尚未抽离':'未附着 · contact 未到达'} · 非物理执行`;
  $('notice').textContent=c.kind==='collision'?'红色来自本次原生碰撞见证；显示被拒绝的静态目标，不代表机器人实际运动到这里。未记录接触点，不绘制推测碰撞点。':'没有本次碰撞见证。请求位置不是可达证明；未生成的运动不补动画。预算内未完成不等于全局不可行。';
  for(const b of failurePanel.querySelectorAll('[data-case]'))b.classList.toggle('active',Number(b.dataset.case)===index);
  if(updateHash)history.replaceState(null,'','#'+c.target_id.replace('carton_',''));focusFailure();
}
for(const b of failurePanel.querySelectorAll('[data-case]'))b.onclick=()=>showFailure(Number(b.dataset.case));
$('regionSample').oninput=e=>showRegionSample(Number(e.target.value));$('focusFailure').onclick=focusFailure;$('wholeFailure').onclick=reset;
$('failureOpacity').oninput=styleFailureWorld;$('diagnosticLabels').onchange=e=>labelGroup.visible=e.target.checked;
$('diagnosticGhosts').onchange=e=>annotationGroup.visible=sampleGroup.visible=e.target.checked;
window.failureInspection={show:showFailure,get index(){return failureIndex;},get q(){return currentQ;},get case(){return DATA.inspection_cases[failureIndex];}};
const initialCase=DATA.inspection_cases.findIndex(c=>c.target_id==='carton_'+location.hash.slice(1));showFailure(initialCase>=0?initialCase:Math.min(2,DATA.inspection_cases.length-1),false);resize();
