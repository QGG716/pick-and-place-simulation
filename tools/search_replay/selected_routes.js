// Reuses the native-q renderer; no generated motion or collision inference.
const routeStyle=document.createElement('style');
routeStyle.textContent=`main{grid-template-columns:350px 1fr}aside>section,aside>details{display:none}aside>#routePanel{display:block;border:0}#routePanel p{line-height:1.65}#routePanel button{display:block;width:100%;text-align:left;margin:6px 0;padding:10px}#routePanel button.active{border-color:#69d9ea;background:#19485b}#routePanel pre{white-space:pre-wrap;font-size:11px}#hud{top:auto;bottom:58px;font-size:13px;max-width:500px}#notice{font-size:11px}`;
document.head.append(routeStyle);
document.title=document.querySelector('h1').textContent='l01 三箱指定路线 · 本次失败记录';
document.querySelector('header .badge').innerHTML='C01 → C00 → C04 · 本轮 0/3<br>Hmax 固定 250 mm · 物理执行 NOT_RUN';
$('identity').textContent='实测源码 '+DATA.raw.source_commit.slice(0,12)+' · 拖动旋转 / 滚轮缩放 / 右键平移';
const routePanel=document.createElement('section');routePanel.id='routePanel';
routePanel.innerHTML=`${DATA.inspection_cases.map((c,i)=>`<button data-case="${i}">${i+1}. ${c.target_id.replace('carton_','')} · ${c.kind==='face'?'侧面筛选未通过':'顶吸接近 IK 未求解'}</button>`).join('')}
<h2 id="routeTitle"></h2><p id="routeDetail"></p><p id="routeMetrics"></p>
<p>橙色：目标箱<br>紫色：本次面可见性筛选的遮挡箱<br>青色：指定方向或请求法兰位置<br>机器人保持本次真实起点 q；没有轨迹可播放。</p>
<button id="routeFocus">聚焦目标</button><button id="routeWhole">全局视角</button>
<label><input type="checkbox" id="routeGhosts" checked> 显示方向 / 请求位置</label>
<details><summary>本次原始记录</summary><pre id="routeEvidence"></pre></details>
<p>历史已完成 32 箱单列，本轮新增 0 箱，现场仍有 8 箱。l00 五箱未纳入本次重试。</p>`;
document.querySelector('aside').prepend(routePanel);
const routeAnnotations=new T.Group();scene.add(routeAnnotations);let routeIndex=0;
function routeFocus(){const b=DATA.inspection_cases[routeIndex].target_box,p=position(matrix(b.pose));camera.position.set(-2.9,p.y-3.1,2.8);controls.target.set(-.1,p.y,.65);controls.update();}
function showRoute(i,changeHash=true){
  routeIndex=i;const c=DATA.inspection_cases[i];
  attempts=DATA.raw.attempts.map(a=>({a,p:null}));$('attempt').innerHTML=attempts.map((_,j)=>`<option value="${j}">${j}</option>`).join('');$('attempt').value=i;
  stage='pregrasp';chooseAttempt();pose(c.display_q,'本次实际起点 q · 未生成运动');clear(framesGroup);clear(routeAnnotations);
  for(const o of obstacles){const target=o.userData.name===c.target_id,blocked=c.blockers.includes(o.userData.name);o.material.opacity=target?.85:blocked?.75:.16;o.material.color.set(target?0xf9a35b:blocked?0xcc8bf1:o.userData.baseColor);}
  if(c.kind==='face'){
    const p=position(matrix(c.target_box.pose));p.y+=c.target_box.size[1]/2;
    routeAnnotations.add(new T.ArrowHelper(new T.Vector3(0,1,0),p,.6,0x66e3ed,.10,.06));
  }else{
    const g=matrix(c.goal_pose),a=new T.AxesHelper(.22);a.matrixAutoUpdate=false;a.matrix.copy(g);routeAnnotations.add(a);
    const m=new T.Mesh(new T.SphereGeometry(.025,12,8),new T.MeshBasicMaterial({color:0x66e3ed}));m.position.copy(position(g));routeAnnotations.add(m);
  }
  routeAnnotations.visible=$('routeGhosts').checked;
  $('routeTitle').textContent=c.title;$('routeDetail').textContent=c.detail;
  $('routeMetrics').textContent=`用时 ${c.task_wall_s.toFixed(2)} s · IK 请求 ${c.ik_calls}\n带高 250 mm · 偏移 delta=0\n预测 / 命令杯：${c.commanded_cups===null?'未到计算阶段':c.predicted_cups+' / '+c.commanded_cups}；实际 contact 未到达\n目标碰撞、接近、contact、抽离、完整抓放均未通过`;
  $('routeMetrics').style.whiteSpace='pre-line';$('routeEvidence').textContent=JSON.stringify(c,null,2);
  $('hud').innerHTML=`<b>${c.target_id.replace('carton_','')} · 实际起点</b><br>未附着 · contact 未到达 · 本轮接收 / 出料 0`;
  $('notice').textContent='静态证据查看：本次没有碰撞见证；遮挡筛选与 IK 搜索失败分开记录。图示不代表实际机器人运动，也不证明全局不可行。';
  for(const b of routePanel.querySelectorAll('[data-case]'))b.classList.toggle('active',Number(b.dataset.case)===i);
  if(changeHash)history.replaceState(null,'','#'+c.target_id.replace('carton_',''));routeFocus();
}
for(const b of routePanel.querySelectorAll('[data-case]'))b.onclick=()=>showRoute(Number(b.dataset.case));
$('routeFocus').onclick=routeFocus;$('routeWhole').onclick=reset;$('routeGhosts').onchange=e=>routeAnnotations.visible=e.target.checked;
window.routeInspection={show:showRoute,get case(){return DATA.inspection_cases[routeIndex];}};
const initialRoute=DATA.inspection_cases.findIndex(c=>c.target_id==='carton_'+location.hash.slice(1));showRoute(initialRoute<0?0:initialRoute,false);resize();
