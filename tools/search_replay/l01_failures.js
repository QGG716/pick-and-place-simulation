// Recorded states only, over the existing native-model FK renderer.
const l01Style=document.createElement('style');
l01Style.textContent=`main{grid-template-columns:370px 1fr}aside>section,aside>details{display:none}aside>#l01Panel{display:block;border:0}header h1{font-size:20px}#l01Panel p{line-height:1.65;margin:10px 0}#l01Panel h2{font-size:21px;margin:15px 0 8px}.case-grid{display:flex;flex-wrap:wrap;gap:6px}.case-grid button{padding:9px 12px}.l01-tabs{display:grid;gap:6px;margin:12px 0}.l01-tabs button{text-align:left;padding:8px}#l01Panel button.active{background:#205165;border-color:#6dddeb}.subtle{font-size:12px;color:#b6c8dd}.gap-card{padding:12px;background:#263348;border-left:3px solid #ff8e9b;line-height:1.6}.gap-card b{font-size:23px;color:#ffbec6}.gap-card small{display:block;color:#bfd0e0}.stage-tags{display:flex;gap:5px;flex-wrap:wrap}.stage-tags span{font-size:12px;padding:4px 6px;border-radius:4px;background:#233449}.stage-tags .ok{color:#7ee6b8}.stage-tags .bad{color:#ffba83}#l01Panel details{font-size:12px}#hud{top:auto;bottom:55px;max-width:540px;font-size:12px}#notice{font-size:12px}#l01Panel label{margin-top:10px}#l01Panel [hidden]{display:none!important}@media(max-width:1000px){main{grid-template-columns:320px 1fr}}`;
document.head.append(l01Style);
const l01Compact=document.createElement('style');l01Compact.textContent='.case-grid{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:4px}.case-grid button{padding:9px 2px;font-size:12px}';document.head.append(l01Compact);
document.title='l01正吸/侧吸 · 五箱失败原因';
document.querySelector('h1').textContent='l01 五箱失败 · 正吸 / 侧吸原始记录';
document.querySelector('header .badge').innerHTML='250 mm薄型剪叉 · 历史正吸/侧吸轮次<br>不是随后顶吸IK失败那一轮';
$('identity').textContent='实测源码 '+DATA.raw.source_commit.slice(0,12)+' · 拖动旋转 / 滚轮缩放 / 右键平移';
const l01Panel=document.createElement('section');l01Panel.id='l01Panel';
l01Panel.innerHTML=`<div class="case-grid">${DATA.inspection_cases.map((c,i)=>`<button data-l01="${i}">${c.target_id.replace('carton_','')}</button>`).join('')}</div>
<h2 id="l01Title"></h2><p id="l01Detail"></p><div id="l01Stages" class="stage-tags"></div>
<div id="l01Modes" class="l01-tabs"></div><div id="l01Gap" class="gap-card"></div>
<label id="l01SampleBox">本对象对的已记录拒绝构型<input id="l01Sample" type="range" min="0" value="0"><span id="l01SampleInfo" class="subtle"></span></label>
<p id="l01State" class="subtle"></p><button id="l01Focus">聚焦原因</button> <button id="l01Whole">全局视角</button>
<label><input id="l01Robot" type="checkbox" checked> 显示机械臂</label>
<label><input id="l01Ghost" type="checkbox" checked> 显示原因标注</label>
<label>背景透明度<input id="l01Opacity" type="range" min=".04" max=".7" step=".02" value=".18"></label>
<p class="subtle">红色：当前记录的相交/接触或净空不足对象对。紫色：侧面遮挡箱，不代表碰撞。青色：未执行的请求带面 / 业务面。</p>
<details><summary>当前构型与原始证据</summary><pre id="l01Evidence"></pre></details>
<p class="subtle">本页只显示原始状态，不连接失败构型形成动画。箱体附着使用该请求的真实相对变换；所有失败箱仍保留在任务世界中。物理执行 NOT_RUN。</p>`;
document.querySelector('aside').prepend(l01Panel);
const l01Annotations=new T.Group();scene.add(l01Annotations);
let l01Index=0,l01Mode='',l01Snapshot=0;
const shortL01=id=>id.replace('carton_','');
const currentCase=()=>DATA.inspection_cases[l01Index];
function labelL01(text,p,color='#e2eef8',width=1.){
  const canvas=document.createElement('canvas');canvas.width=1200;canvas.height=100;
  const ctx=canvas.getContext('2d');ctx.fillStyle='#0a1424ef';ctx.fillRect(0,0,1200,100);ctx.fillStyle=color;
  ctx.font='bold 35px "Microsoft YaHei",sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,600,50);
  const s=new T.Sprite(new T.SpriteMaterial({map:new T.CanvasTexture(canvas),depthTest:false}));s.position.copy(p);s.scale.set(width,width/12,1);s.renderOrder=20;l01Annotations.add(s);
}
function objectCenterL01(obj){obj.updateMatrixWorld(true);return new T.Box3().setFromObject(obj).getCenter(new T.Vector3());}
function pairObjectsL01(){const w=currentCase().snapshots[l01Snapshot].witness;if(!w)return[];return w.pair.map(id=>modelObjects.find(([o,l])=>l.link===id)?.[0]||obstacles.find(o=>o.userData.name===id));}
function paintL01(){
  const c=currentCase(),pair=c.snapshots[l01Snapshot].witness?.pair||[];
  for(const o of obstacles){const selected=o.userData.name===c.target_id,bad=pair.includes(o.userData.name),blocked=l01Mode==='face'&&c.blockers.includes(o.userData.name);
    o.material.color.set(bad?0xff4057:blocked?0xcc8bf1:selected?0xffac59:o.userData.baseColor);o.material.opacity=selected?.78:bad?.55:blocked?.72:Number($('l01Opacity').value);}
  for(const [o,l,,original] of modelObjects){const bad=pair.includes(l.link);o.material.color.copy(original);if(bad)o.material.color.set(0xff4057);o.material.transparent=true;o.material.opacity=bad?1:pair.length?.35:1;}
  robot.visible=$('l01Robot').checked;if(payload)payload.material.opacity=.65;
}
function focusL01(){
  const c=currentCase(),w=c.snapshots[l01Snapshot].witness;
  if(w){const p=objectCenterL01(pairObjectsL01()[0]);const wall=w.pair.includes('trailer_left_wall');camera.position.copy(p).add(new T.Vector3(wall?-2.2:-.85,wall?-2.5:-1.25,wall?1.25:.9));controls.target.copy(p).add(new T.Vector3(0,0,wall?0:-.04));}
  else if(l01Mode==='face'){camera.position.set(-2.1,2.8,1.8);controls.target.set(.3,-.35,.44);}
  else{camera.position.set(-3.7,-3.9,2.4);controls.target.set(-.55,-.15,.5);}
  controls.update();
}
function drawHeightL01(){
  const belt=active.request.world.find(b=>b.id==='conveyor_transverse'),p=position(matrix(belt.pose));
  const plane=new T.Mesh(new T.BoxGeometry(belt.size[0],belt.size[1],.004),new T.MeshBasicMaterial({color:0x6de2ef,transparent:true,opacity:.4,depthTest:false}));
  plane.position.set(p.x,p.y,-.05);plane.userData.meaning='REQUESTED_HMIN_SCHEMATIC_ONLY';l01Annotations.add(plane);
  const x=-1.15,y=-1.2;l01Annotations.add(line([new T.Vector3(x,y,-.05),new T.Vector3(x,y,.25)],0x6de2ef,1));
  labelL01('最低带面 +250 mm（实际）',new T.Vector3(-.6,1.3,.3),'#b7d6f4',1.3);
  labelL01('Hmin −50 mm · 越界300 mm',new T.Vector3(-.6,-1.3,-.07),'#7ce8f5',1.3);
}
function renderL01(snapshotIndex){
  const c=currentCase(),s=c.snapshots[snapshotIndex];l01Snapshot=snapshotIndex;
  attempts=DATA.raw.attempts.map(a=>({a,p:null}));$('attempt').innerHTML=attempts.map((_,i)=>`<option value="${i}">${i}</option>`).join('');$('attempt').value=s.display_index;
  chooseAttempt();stage=active.request.stage;pose(s.q_rad,s.state_label);clear(framesGroup);clear(graph);clear(l01Annotations);paintL01();
  const w=s.witness,center=position(matrix(c.target_box.pose));
  if(w){
    const pair=pairObjectsL01();for(let i=0;i<pair.length;i++){const p=objectCenterL01(pair[i]);if(i===1&&w.pair[i]==='trailer_left_wall')p.copy(objectCenterL01(pair[0])).setY(position(pair[i].matrix).y);if(i===1&&w.pair[i]==='chassis')p.copy(objectCenterL01(pair[0])).add(new T.Vector3(-.32,0,-.32));labelL01(w.pair[i]==='chassis'?'chassis · 安装底座':w.pair[i],p.add(new T.Vector3(0,0,i?.2:.17)),'#ffb2bc',w.pair[0]==='J2_link'?1.4:.7);}
    $('l01Gap').innerHTML=w.reason==='INTERSECTION_OR_CONTACT'?'<b>相交 / 接触</b><br>原生碰撞检查拒绝<small>未记录穿透深度和接触点。原始负距离标记不表示穿透了相应长度。</small>':`实测表面间隙 <b>${(w.surface_distance_m*1000).toFixed(3)} mm</b><br>要求 ≥ ${(w.required_pair_clearance_m*1000).toFixed(0)} mm<small>此构型为正间隙不足。未记录最近点，不虚构碰撞点。</small>`;
  }else if(c.kind==='clearance'){
    labelL01('已通过contact · 实际48杯有效',center.clone().add(new T.Vector3(.25,0,.28)),'#8ce7c1',1.4);
    $('l01Gap').innerHTML='contact末态已通过<small>48杯实际几何有效；此处尚未开始抽离。点击上方对象对查看随后被拒绝的LIN搜索构型。</small>';
  }else{
    labelL01(shortL01(c.target_id),center.clone().add(new T.Vector3(0,0,.25)),'#ffd19d',.7);
    if(l01Mode==='face'){
      for(const id of c.blockers){const o=obstacles.find(b=>b.userData.name===id);labelL01(shortL01(id),objectCenterL01(o).add(new T.Vector3(0,0,.24)),'#e7bcff',.75);}
      const face=new T.Mesh(new T.PlaneGeometry(c.target_box.size[0],c.target_box.size[2]),new T.MeshBasicMaterial({color:0x6de2ef,side:T.DoubleSide,transparent:true,opacity:.65}));face.rotation.x=Math.PI/2;face.position.copy(center);face.position.y+=c.target_box.size[1]/2+.002;l01Annotations.add(face);
      l01Annotations.add(new T.ArrowHelper(new T.Vector3(0,1,0),face.position,.35,0x6de2ef,.05,.025));
      $('l01Gap').innerHTML='指定 +Y 内侧面未暴露<small>遮挡箱：'+c.blockers.map(shortL01).join('、')+'。属于面暴露筛选失败，没有目标IK或碰撞见证。</small>';
    }else{drawHeightL01();$('l01Gap').innerHTML='Hmin −50 mm < 最低250 mm<small>差300 mm，在派发IK前拒绝。青色仅为请求带面示意；实际剪叉和带面保持250 mm。</small>';}
  }
  const found=c.kind==='clearance';
  $('l01Stages').innerHTML=(found?[['接近通过','ok'],['contact通过','ok'],['抽离未完成','bad']]:[['业务筛选拒绝','bad'],['IK未派发',''],['contact未到达','']]).map(([t,k])=>`<span class="${k}">${t}</span>`).join('');
  $('l01State').textContent=s.state_label+'。实际带高250 mm；'+(found?`区域端点尝试${c.region_samples.length}个，最终阶段预算耗尽；不代表全局不可行。`:'没有这条业务路线的目标q或运动轨迹。');
  $('l01Evidence').textContent=JSON.stringify({source_commit:DATA.raw.source_commit,target:c.target_id,task:c.task,display_state:s.state_label,q_rad:s.q_rad,request_id:active.response_identity.request_id,stage_id:active.request.stage_id,scene_fingerprint:active.request.scene_fingerprint,witness:w,options:c.options,recorded_pair_occurrences:c.pair_counts,final_failure:c.failure},null,2);
  $('hud').innerHTML=`<b>${shortL01(c.target_id)} · ${w?'被拒绝搜索构型':found?'contact末态':'业务筛选根状态'}</b><br>${found?'请求中已附着 · 实际contact 48杯':'未附着 · contact未到达'} · 物理执行 NOT_RUN`;
  $('notice').textContent=w?'机器人和附着箱由该次见证q及真实相对变换显示；这是被拒绝的静态搜索构型，不是已执行位置或合法路径。':'历史正吸/侧吸轮次的保留记录；未派发目标不补机器人姿态，未完成的抽离不补运动动画。';
  l01Annotations.visible=$('l01Ghost').checked;focusL01();
}
function setL01Mode(mode){
  l01Mode=mode;const c=currentCase(),indices=c.snapshots.map((s,i)=>[s,i]).filter(([s])=>s.witness&&s.witness.pair.join(' / ')===mode).map(([,i])=>i);
  $('l01SampleBox').hidden=!indices.length;$('l01Sample').max=Math.max(0,indices.length-1);$('l01Sample').value=0;
  const choose=index=>{$('l01SampleInfo').textContent=`${index+1} / ${indices.length} 个原始拒绝构型；逐帧切换，不是运动轨迹`;renderL01(indices[index]);};
  $('l01Sample').oninput=()=>choose(Number($('l01Sample').value));
  if(indices.length)choose(0);else renderL01(0);
  for(const b of $('l01Modes').querySelectorAll('button'))b.classList.toggle('active',b.dataset.mode===mode);
}
function showL01(index,mode=null,updateHash=true){
  l01Index=index;const c=currentCase(),clearance=c.kind==='clearance';$('l01Robot').checked=clearance;
  const modes=clearance?[['contact','已通过的contact末态'],...Object.keys(c.pair_counts).map(k=>[k,k])]:[['height','所需Hmin / 实际最低带高'],...(c.kind==='face'?[['face','+Y侧吸面被遮挡']]:[])];
  $('l01Modes').innerHTML=modes.map(([k,t])=>`<button data-mode="${k}">${t}</button>`).join('');
  for(const b of $('l01Modes').querySelectorAll('button'))b.onclick=()=>setL01Mode(b.dataset.mode);
  $('l01Title').textContent=clearance?'抽离搜索中的碰撞 / 净空不足':c.kind==='face'?'高度越界，内侧面也未暴露':'接收高度超出下限';
  $('l01Detail').textContent=clearance?`${shortL01(c.target_id)} 的接近与contact均通过。抽离LIN搜索记录了下列对象对的相交/接触及净空不足；可切换contact末态和各个拒绝构型。`:c.kind==='face'?'正吸要求Hmin=−50 mm，超出范围；允许的+Y内侧吸在250 mm高度可定义，但被邻箱遮挡。两种拒绝原因可分别查看。':'正吸要求Hmin=−50 mm，低于250 mm下限。该业务选项在原生IK和运动派发前被拒绝。';
  for(const b of l01Panel.querySelectorAll('[data-l01]'))b.classList.toggle('active',Number(b.dataset.l01)===index);
  setL01Mode(mode||modes[clearance?1:c.kind==='face'?1:0][0]);
  if(updateHash)history.replaceState(null,'','#'+shortL01(c.target_id));
}
for(const b of l01Panel.querySelectorAll('[data-l01]'))b.onclick=()=>showL01(Number(b.dataset.l01));
$('l01Focus').onclick=focusL01;$('l01Whole').onclick=reset;$('l01Opacity').oninput=paintL01;$('l01Robot').onchange=paintL01;$('l01Ghost').onchange=()=>l01Annotations.visible=$('l01Ghost').checked;
window.l01Inspection={show:showL01,mode:setL01Mode,get index(){return l01Index;},get snapshot(){return currentCase().snapshots[l01Snapshot];},get case(){return currentCase();}};
const requested=DATA.inspection_cases.findIndex(c=>c.target_id==='carton_'+location.hash.slice(1));showL01(requested>=0?requested:2,null,false);resize();
