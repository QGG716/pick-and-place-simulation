// Read-only inspection over the existing production-model FK renderer.
const minStyle=document.createElement('style');
minStyle.textContent=`main{grid-template-columns:360px 1fr}aside>section,aside>details{display:none}aside>#minPanel{display:block;border:0}#minPanel h2{font-size:21px;margin:14px 0 8px}#minPanel p{line-height:1.65}#minCases{display:grid;grid-template-columns:1fr 1fr;gap:5px}#minCases button{font-size:12px;text-align:left;padding:7px}#minPanel .tabs{display:flex;flex-wrap:wrap;gap:5px;margin:12px 0}#minPanel .subtle{font-size:12px;color:#aabed4;line-height:1.6}.pill{display:inline-block;padding:5px 8px;border-radius:5px;background:#26364d;color:#ffcb91;margin:0 4px 4px 0;font-size:12px}#hud{top:auto;bottom:68px;max-width:560px;font-size:12px}#pinCard{position:absolute;right:18px;top:18px;width:294px;background:#0c192bee;border:1px solid #3b6580;border-radius:8px;padding:15px;line-height:1.6;pointer-events:none}#pinCard b{color:#77e1ed}#pinCard small{display:block;margin-top:8px}#pinCard .warn{color:#ffa99a}#pinCard .eq{font-family:Consolas,monospace;font-size:13px}#minPanel details{font-size:12px}#minPanel button.active{border-color:#6cdce8;background:#20566a}#notice{font-size:12px}header h1{font-size:20px}@media(max-width:1000px){main{grid-template-columns:310px 1fr}#pinCard{width:238px;font-size:12px}}`;
document.head.append(minStyle);
document.title='250 mm 续跑 · 12 个剩余箱原因';
document.querySelector('h1').textContent='12 个剩余箱 · 失败原因三维检查';
document.querySelector('header .badge').innerHTML='本轮完成 8/20 · 累计 28/40<br>原生规划记录 · 物理执行 NOT_RUN';
$('identity').textContent='实测源码 '+DATA.raw.source_commit.slice(0,12)+' · 拖动旋转 / 滚轮缩放 / 右键平移';
const minPanel=document.createElement('section');minPanel.id='minPanel';
minPanel.innerHTML=`<div class="subtle">5 箱直接失败 · 7 箱因上层未卸而未搜索</div><div id="minCases">${DATA.inspection_cases.map((c,i)=>`<button data-min-case="${i}">${c.target_id.replace('carton_','')} · ${c.kind==='support'?'支撑依赖':'高度限制'}</button>`).join('')}</div>
<h2 id="minTitle"></h2><p id="minDetail"></p><div><span class="pill">未派发 IK / 运动目标</span><span class="pill">未附着</span></div>
<div class="tabs"><button id="minScene">现场与目标</button><button id="minPins">250 mm 几何</button><button id="minFace">侧抓筛选</button></div>
<p id="minContext" class="subtle"></p><p id="minLegend" class="subtle"></p>
<label><input id="minLabels" type="checkbox" checked> 显示三维标注</label><label><input id="minGhost" type="checkbox" checked> 显示请求带面 / 高度示意</label>
<label><input id="minRobot" type="checkbox" checked> 显示机械臂（局部视图默认隐藏）</label>
<label>背景透明度<input id="minOpacity" type="range" min=".03" max=".65" step=".02" value=".15"></label>
<button id="minWhole">全局视角</button> <button id="minFocus">聚焦原因</button>
<details><summary>本箱原始原因与证据</summary><pre id="minEvidence"></pre></details>
<p class="subtle">严格按“箱顶低于250 mm才顶吸”。最底层箱顶300 mm，本轮没有箱子转为顶吸。成功回放中的名义下落与理想清空不代表物理执行。</p>`;
document.querySelector('aside').prepend(minPanel);
const pinCard=document.createElement('div');pinCard.id='pinCard';view.append(pinCard);
const minAnnotations=new T.Group(),minLabels=new T.Group(),minGhosts=new T.Group();scene.add(minAnnotations,minLabels,minGhosts);
let minIndex=0,minMode='scene';
const short=id=>id.replace('carton_','');
function minLabel(text,p,color='#e1ebf5',width=1.05){
  const canvas=document.createElement('canvas');canvas.width=1000;canvas.height=100;const ctx=canvas.getContext('2d');
  ctx.fillStyle='#0b1425ee';ctx.fillRect(0,0,1000,100);ctx.fillStyle=color;ctx.font='bold 35px "Microsoft YaHei",sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,500,50);
  const sprite=new T.Sprite(new T.SpriteMaterial({map:new T.CanvasTexture(canvas),depthTest:false}));sprite.position.copy(p);sprite.scale.set(width,width*.1,1);sprite.renderOrder=20;minLabels.add(sprite);return sprite;
}
function boxCenter(id){return position(matrix(active.request.world.find(b=>b.id===id).pose));}
function evidenceBlockers(){const c=DATA.inspection_cases[minIndex];return minMode==='face'?c.options.flatMap(o=>(o.screening||[]).filter(f=>f.geometric_face_id==='y+'&&!f.available).flatMap(f=>f.blockers)):c.blockers;}
function paintMin(){const c=DATA.inspection_cases[minIndex],blockers=evidenceBlockers();for(const o of obstacles){const id=o.userData.name,selected=id===c.target_id,blocked=blockers.includes(id),mech=id.startsWith('lift_transverse');o.material.color.set(selected?0xffa24a:blocked?0xd586ee:minMode==='pins'&&mech?0x6cc6e0:o.userData.baseColor);o.material.opacity=selected?.8:blocked?.72:minMode==='pins'&&mech?.7:Number($('minOpacity').value);}robot.visible=$('minRobot').checked;}
function focusMin(){const p=boxCenter(DATA.inspection_cases[minIndex].target_id);if(minMode==='pins'){camera.position.set(-2.85,-3.5,1.65);controls.target.set(-.62,-.25,.30);}else if(minMode==='face'){camera.position.set(-2.8,2.6,1.95);controls.target.set(.2,-.25,.64);}else if(DATA.inspection_cases[minIndex].kind==='support'){camera.position.set(-2.3,p.y-2.3,1.85);controls.target.set(.25,p.y,.48);}else{camera.position.set(-4.1,-4.1,2.8);controls.target.set(-.55,-.1,.72);}controls.update();}
function drawPinSchematic(){
  const belt=active.request.world.find(b=>b.id==='conveyor_transverse'),p=position(matrix(belt.pose));
  const deck=new T.Mesh(new T.BoxGeometry(belt.size[0],belt.size[1],.004),new T.MeshBasicMaterial({color:0x67e4ee,transparent:true,opacity:.25,depthWrite:false}));deck.position.set(p.x,p.y,.25);deck.userData.meaning='REQUESTED_SURFACE_ONLY_NOT_VALID_MECHANISM';minGhosts.add(deck);
  const edge=new T.LineSegments(new T.EdgesGeometry(deck.geometry),new T.LineBasicMaterial({color:0x67e4ee}));edge.position.copy(deck.position);minGhosts.add(edge);
  // This ruler is deliberately outside the mechanism. X/Y are annotation placement,
  // not claimed upper-pin coordinates for the impossible configuration.
  const x=-1.0,y=-1.20;
  minGhosts.add(line([new T.Vector3(x,y,0),new T.Vector3(x,y,.6)],0x8baec7));
  for(const [z,color] of [[.55,0x98b8d4],[.36,0xffba72],[.25,0x67e4ee],[.10,0xe8cf73],[.06,0xff738b]]){
    minGhosts.add(line([new T.Vector3(x-.12,y,z),new T.Vector3(x+.12,y,z)],color));
    const dot=new T.Mesh(new T.SphereGeometry(.012,12,8),new T.MeshBasicMaterial({color}));dot.position.set(x,y,z);minGhosts.add(dot);
  }
  minLabel('请求带面 250 mm · 未到达',new T.Vector3(-.68,1.22,.25),'#7cebf3',1.1);
  if(minMode==='pins'){
    minLabel('实际横带 550 mm',new T.Vector3(-.5,.15,.65),'#bed7ef',.9);
    minLabel('360 mm · 首个拒绝采样高度',new T.Vector3(-1.12,-1.40,.43),'#ffbd7f',.86);
    minLabel('下铰点 100 mm',new T.Vector3(-1.12,-1.40,.19),'#f2d979',.72);
    minLabel('请求上铰点 60 mm',new T.Vector3(-1.12,-1.40,.005),'#ff8da1',.78);
  }
  pinCard.innerHTML='<b>250 mm 为什么不可构造</b><div class="eq">上铰点 = 250 − 120 − 70<br>　　　 = 60 mm<br>下铰点 = 100 mm</div><p class="warn">请求上铰点比下铰点低40 mm。</p><div class="eq">每级升高 = (60 − 100) / 2<br>　　　　 = −20 mm</div><small>现有两级剪叉要求每级升高 &gt;35 mm。下降采样在约360 mm即被几何构造拒绝。</small><small>青色只有请求带面；旁侧标尺只示意Z高度。未构造250 mm机构，未产生碰撞见证；高于360 mm也不代表已通过碰撞检查。</small>';
}
function showMin(index,mode='scene',updateHash=true){
  minIndex=index;minMode=mode;const c=DATA.inspection_cases[index];
  $('minRobot').checked=mode==='scene';
  attempts=DATA.raw.attempts.map(a=>({a,p:null}));$('attempt').innerHTML=attempts.map((_,i)=>`<option value="${i}">${i}</option>`).join('');$('attempt').value=index;chooseAttempt();pose(c.display_q,c.state_label);
  clear(framesGroup);clear(minAnnotations);clear(minLabels);clear(minGhosts);paintMin();
  const p=boxCenter(c.target_id),blockers=evidenceBlockers();
  if(minMode!=='pins')minLabel(short(c.target_id)+' · 当前查看箱体',p.clone().add(new T.Vector3(0,0,minMode==='face'?.4:.23)),'#ffc58b',.9);
  if(c.kind==='support'){
    const names=blockers.map(short).join('、');$('minTitle').textContent='上方箱体尚未卸走';$('minDetail').textContent=`${short(c.target_id)} 仍支撑 ${names}。为保留堆垛支撑关系，本箱未派发原生IK或抓放搜索。紫色箱体是当前直接支撑依赖。`;
    for(const id of blockers){const b=boxCenter(id);minAnnotations.add(new T.ArrowHelper(new T.Vector3(0,0,1),p.clone(),b.z-p.z,0xd586ee,.05,.025));minLabel(short(id)+' · 必须先卸',b.clone().add(new T.Vector3(.1,0,.24)),'#e7b3fb',.85);}
    pinCard.hidden=true;
  }else{
    const low=c.options.some(o=>o.receive_height_m<0);
    $('minTitle').textContent=minMode==='face'?'内侧抓取面未暴露':'接收高度被机构几何拒绝';
    $('minDetail').textContent=minMode==='face'?'l02_c00 的 +Y 内侧面被 l02_c01、l02_c04 遮挡。这里显示当时尚有13箱的筛选现场；l02_c04随后已成功卸走。该侧抓选项在派发IK前被拒绝，不是工具与箱体碰撞。':low?'正面 Hmax 需要250 mm，但剪叉无法构造；Hmin 为−50 mm，低于配置的250 mm下限。尚未进入机器人IK和运动规划。':'交汇区正面出料要求 Hmin=250 mm。下降采样在约360 mm就触发剪叉几何范围拒绝，尚未进入机器人IK和运动规划。';
    if(minMode==='face'){
      for(const id of blockers)minLabel(short(id)+' · 面暴露筛选遮挡',boxCenter(id).add(new T.Vector3(0,0,.25)),'#e7b3fb',1.0);
      const b=c.target_box,face=new T.Mesh(new T.PlaneGeometry(b.size[0],b.size[2]),new T.MeshBasicMaterial({color:0x67e4ee,side:T.DoubleSide,transparent:true,opacity:.6}));face.rotation.x=Math.PI/2;face.position.copy(p);face.position.y+=b.size[1]/2+.003;minAnnotations.add(face);
      minAnnotations.add(new T.ArrowHelper(new T.Vector3(0,1,0),face.position,.45,0x67e4ee,.06,.03));pinCard.hidden=true;
    }else{drawPinSchematic();pinCard.hidden=false;}
  }
  $('minPins').hidden=c.kind==='support';$('minFace').hidden=!c.options.some(o=>o.screening);
  for(const [id,v] of [['minScene','scene'],['minPins','pins'],['minFace','face']])$(id).classList.toggle('active',minMode===v);
  $('minContext').textContent=`${c.state_label} · ${c.world_carton_count}箱 · 实际横带 ${(c.height_m*1000).toFixed(0)} mm。机器人保持记录中的原生q；本页没有生成或播放机器人运动。`;
  $('minLegend').textContent='橙色：选中箱体。紫色：记录中的支撑依赖或面筛选遮挡，不表示碰撞。青色：未执行的请求带面或业务抓取面。';
  $('minEvidence').textContent=JSON.stringify({source_commit:DATA.raw.source_commit,target:c.target_id,q_rad:c.display_q,actual_height_m:c.height_m,source_archive_sha256:DATA.source_archive_sha256,options:c.options,supported_cartons:c.blockers,native_search_performed:false,collision_witness:null},null,2);
  $('hud').innerHTML=`<b>${short(c.target_id)} · ${c.kind==='support'?'支撑依赖未搜索':'业务/机构筛选被拒绝'}</b><br>未附着 · 无实际contact杯 · 物理执行 NOT_RUN`;
  $('notice').textContent='本页展示保留记录与静态原因示意。没有这12箱的目标q、接近或抽离轨迹，也没有本轮碰撞见证。';
  minLabels.visible=$('minLabels').checked;minGhosts.visible=$('minGhost').checked;
  for(const b of minPanel.querySelectorAll('[data-min-case]'))b.classList.toggle('active',Number(b.dataset.minCase)===index);
  if(updateHash)history.replaceState(null,'','#'+short(c.target_id));focusMin();
}
for(const b of minPanel.querySelectorAll('[data-min-case]'))b.onclick=()=>showMin(Number(b.dataset.minCase));
$('minScene').onclick=()=>showMin(minIndex,'scene');$('minPins').onclick=()=>showMin(minIndex,'pins');$('minFace').onclick=()=>showMin(minIndex,'face');
$('minOpacity').oninput=paintMin;$('minLabels').onchange=e=>minLabels.visible=e.target.checked;$('minGhost').onchange=e=>minGhosts.visible=e.target.checked;
$('minRobot').onchange=paintMin;
$('minWhole').onclick=reset;$('minFocus').onclick=focusMin;
window.min250Inspection={show:showMin,get index(){return minIndex;},get mode(){return minMode;},get case(){return DATA.inspection_cases[minIndex];}};
const minInitial=DATA.inspection_cases.findIndex(c=>c.target_id==='carton_'+location.hash.slice(1));showMin(minInitial>=0?minInitial:0,'scene',false);resize();
