// Exact retained height samples and a tool-only pose preview. No generated robot q.
const prefix=DATA.ik_prefix,actualWorld=structuredClone(active.request.world),c=DATA.inspection_cases[0];
const prefixPanel=document.createElement('div');
prefixPanel.innerHTML=`<h3>本次受检降带</h3><input id="prefixHeight" type="range" min="0" max="${prefix.frames.length-1}" value="${prefix.frames.length-1}" step="1"><p id="prefixValue" class="counter"></p><p class="legend">滑条只显示原检查高度；全部机构同源更新，机器人 q 始终不变。失败前缀未提交到整批末态。</p><h3>请求位姿（静态预览）</h3><label><input id="toolPosePreview" type="checkbox" checked> 青色工具：期望接近位姿，无IK解</label><p class="legend">青色工具不与机器人连接；它不是已到达构型，也不是碰撞见证。箱顶接触面 Z=${prefix.contact_pose[2][3].toFixed(3)} m；吸盘接近面 Z=${prefix.approach_pose[2][3].toFixed(3)} m。求解器TCP位于 Z=${prefix.solver_goal_pose[2][3].toFixed(4)} m，沿用模型中的固定接触面偏置。</p>`;
$('failureMetrics').after(prefixPanel);
$('failureStages').insertAdjacentHTML('afterbegin','<span class="stage-chip ok">✓ 降带</span>');
const ghostTool=new T.Group();scene.add(ghostTool);
const actualFrames=fk(c.display_q),taskInverse=actualFrames.m710_virtual_task_tcp.clone().invert(),goal=matrix(prefix.solver_goal_pose);
for(const [obj,item,origin] of modelObjects){if(!item.tool)continue;const g=new T.Mesh(obj.geometry,new T.MeshBasicMaterial({color:0x60e3ee,wireframe:true,transparent:true,opacity:.55,depthWrite:false}));g.matrixAutoUpdate=false;g.matrix.copy(goal.clone().multiply(taskInverse).multiply(actualFrames[item.link]).multiply(origin));ghostTool.add(g);}
function prefixFrame(index){
  const frame=prefix.frames[index];active.request.world=frame.world;chooseAttempt();pose(c.display_q,c.state_label);clear(framesGroup);styleFailureWorld();
  $('prefixHeight').value=index;$('prefixValue').textContent=`${index+1}/${prefix.frames.length} 个受检高度 · H=${frame.height_m.toFixed(3)} m · 机器人保持实际起点`;
  $('hud').innerHTML=`<b>l03_c02 · 未附着，contact未到达</b><br>降带受检高度 ${frame.height_m.toFixed(3)} m · 机器人未运动<br>IK ${prefix.ik_calls}次 / 成功0 · 名义接收0 / 理想清空0`;
  $('notice').textContent='青色工具是未求解的接近请求；没有本轮目标 q、碰撞见证或机器人路径。预算内未求解不等于全局不可达。';
  window.lowIkState={index,height_m:frame.height_m,q:currentQ.slice(),ghost_pose:prefix.solver_goal_pose,robot_motion_points:0,attached:false};
}
$('prefixHeight').oninput=e=>prefixFrame(Number(e.target.value));
$('toolPosePreview').onchange=e=>ghostTool.visible=e.target.checked;
$('focusFailure').onclick=()=>{camera.position.set(-3.4,-3.4,2.8);controls.target.set(-.55,0,1.2);controls.update();};
$('failureOpacity').value=.32;
prefixFrame(prefix.frames.length-1);$('focusFailure').click();
window.lowIkInspection={setFrame:prefixFrame,ghostTool,get data(){return prefix;}};
failurePanel.querySelector('[data-case]').onclick=()=>{prefixFrame(prefix.frames.length-1);$('focusFailure').click();};
