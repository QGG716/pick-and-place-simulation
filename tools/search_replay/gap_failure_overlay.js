
// Displays only saved q and witnesses. No invented motion between rejected states.
const failureData=DATA.failure_review,reviewGroup=new T.Group();scene.add(reviewGroup);
document.title=`${(failureData.gap_m*1000).toFixed(0)} mm · 当前失败情况`;
document.querySelector('h1').textContent=`${(failureData.gap_m*1000).toFixed(0)} mm · ${failureData.target_id} 失败诊断`;
$('identity').textContent=`原始 40 箱 · 已卸 ${failureData.completed_count} 箱 / 现场 ${failureData.carton_count} 箱 · 尝试编号 ${failureData.trial_number} · ${failureData.wall_s.toFixed(1)} s 搜索`;
document.querySelector('.badge').innerHTML=`完整抓放 ${failureData.completed_count} / 40 · 未执行<br>有限搜索未完成 ≠ 已证明不可达`;
$('hud').style.top='auto';$('hud').style.bottom='62px';$('hud').style.maxWidth='600px';
const panel=document.createElement('section');panel.innerHTML=`<h3>查看已记录的失败位置</h3>
<button id="reviewInitial">本次起点</button><button id="reviewLift">升带受阻</button><button id="reviewSide">侧视</button><button id="reviewIso">三维</button>
<button id="reviewContact">已返回接触终点</button>
<button id="reviewExtraction">外移失败的起点与目标</button>
<p><select id="reviewWitness" style="width:100%"></select></p><button id="reviewShow">显示拒绝状态</button>
<p class="metric">红色：本次检查涉及的碰撞对<br>青色箭头：吸盘实际工作法向<br>白框：目标箱正面；本次失败目标保留在场</p><pre id="reviewSummary"></pre>`;
document.querySelector('aside').prepend(panel);
$('searchPlay').closest('section').hidden=true;
$('motionTitle').textContent='返回的原生候选路径（若有）';
$('notice').textContent='本页仅展示归档中的受检姿态、拒绝原因及原生候选路径。静态失败姿态不是机器人实际走到的位置，不在失败状态之间补动画。原始搜索树与日志保留在服务器。';
const reasons={CLEARANCE_INSUFFICIENT:'净空不足 5 mm',INTERSECTION_OR_CONTACT:'相交 / 接触'};
const pairs=failureData.witness_pair_counts;
const contactPaths=failureData.returned_contact_path_count??failureData.native_endpoints.filter(e=>e.stage==='contact').length;
const approachTimeouts=failureData.native_requests.filter(r=>r.stage==='pregrasp'&&r.solver==='TIMED_OUT').reduce((n,r)=>n+r.count,0);
$('reviewSummary').textContent=`返回接触路径 ${contactPaths} 条\n接近规划超时 ${approachTimeouts} 次\n累计完成抓放 ${failureData.completed_count} / 40\n已保存拒绝见证最多的碰撞对：\n`+pairs.slice(0,4).map(r=>r.pair.join(' / ')+`（${r.count} 条）`).join('\n')+'\n这是见证记录数，不是碰撞概率。';
if(failureData.receiver_lift_failures.length){const w=failureData.receiver_lift_failures[0];$('reviewSummary').textContent=`已记录升带拒绝 ${failureData.receiver_lift_failures.length} 次\n首个拒绝高度 ${w.height_m.toFixed(3)} m\n${w.pair.join(' ↔ ')}\n完成抓放 ${failureData.completed_count} / 40\n\n其他候选见下拉框；升带拒绝不代表所有路径的结果。`;}
const noIK=failureData.native_requests.filter(r=>r.stage==='extraction' && r.solver==='NO_IK_SOLUTION').reduce((n,r)=>n+r.count,0);
if(noIK)$('reviewSummary').textContent=`当前阻塞：带箱外移\n${noIK} 次原生外移请求返回无 IK 解\n已返回接触路径 ${contactPaths} 条 · 累计完整抓放 ${failureData.completed_count}/40\n这是有限数值尝试的结果，不是全局不可达证明。\n\n另有 ${failureData.witnesses.length} 条碰撞见证，见下拉框。`;
$('reviewWitness').innerHTML=failureData.witnesses.map((w,i)=>`<option value="${i}">${i+1}. ${w.pair.join(' / ')} · ${reasons[w.reason]||w.reason}</option>`).join('');
$('reviewWitness').disabled=$('reviewShow').disabled=!failureData.witnesses.length;
function reviewAnnotations(q){
 for(const [o] of modelObjects)o.material.emissive?.set(0x000000);
 clear(reviewGroup);const f=fk(q),contact=f.flange.clone().multiply(matrix(model.flange_from_contact));
 const center=position(contact),direction=new T.Vector3(0,0,1).transformDirection(contact);
 reviewGroup.add(new T.ArrowHelper(direction,center,.38,0x59e5ff,.085,.04));
 const points=failureData.target.face_boundary_m.map(p=>new T.Vector3(...p));reviewGroup.add(line([...points,points[0]],0xffffff));
 for(const o of obstacles){const carton=o.userData.name.startsWith('carton_');o.material.opacity=carton?.72:o.userData.name.includes('ceiling')?.10:o.userData.name.includes('wall')?.07:.55;}
}
function reviewSelectAttempt(index){
 const a=DATA.raw.attempts[index];setStage(a.request.pipeline_id==='external_axis'?'external_axis':a.request.stage);
 const selected=attempts.findIndex(x=>x.a===a);$('attempt').value=selected;chooseAttempt();
 motionPlaying=searchPlaying=false;clear(graph);clear(framesGroup);
}
function reviewInitial(){
 reviewSelectAttempt(0);pose(failureData.initial_q_rad,'本次请求的实际起点；承接已接受父状态');reviewAnnotations(failureData.initial_q_rad);
 const angle=(failureData.initial_orientation?.actual_angle_rad||0)*180/Math.PI;
 const shape=failureData.initial_arm_posture;
 $('hud').innerHTML=`<b>本次起点 · ${failureData.target_id}</b><br>吸盘法向与 +X 夹角 ${angle.toFixed(4)}°`+(shape?`<br>大臂仰角 ${shape.upper_arm.elevation_deg.toFixed(1)}° · 小臂仰角 ${shape.forearm_elevation_deg.toFixed(1)}°`:'')+`<br>未附着 · 完成抓放 ${failureData.completed_count} / 40`;
 $('motionText').textContent='初始状态；静态展示';
 window.failureReviewState={kind:'initial',q:currentQ,pair:[],expected:failureData.initial_expected_flange_flat};
}
function reviewWitness(index){
 const w=failureData.witnesses[index];reviewSelectAttempt(w.attempt_index);
 if(w.world_at_witness){clear(world);payload=null;obstacles=[];for(const b of w.world_at_witness){const color=b.id===failureData.target_id?0xf9a35b:b.category==='carton'?0xa98661:0x61758c;const o=new T.Mesh(solidGeometry(b),new T.MeshStandardMaterial({color,transparent:true,opacity:.5,depthWrite:false}));o.matrixAutoUpdate=false;o.matrix.copy(matrix(b.pose));o.userData={name:b.id,baseColor:color};world.add(o);obstacles.push(o);}}
 pose(w.q_rad,'原生碰撞检查拒绝的实际受检 q（非执行位置）\n'+JSON.stringify(w,null,2));reviewAnnotations(w.q_rad);
 for(const [o,l] of modelObjects)if(w.pair.includes(l.link)){o.material.color.set(0xff304d);o.material.emissive?.set(0x5f0715);}
 for(const o of obstacles)if(w.pair.includes(o.userData.name)){o.material.color.set(0xff304d);o.material.opacity=.43;}
 const distance=w.surface_distance_m>=0?`记录表面距离 ${(w.surface_distance_m*1000).toFixed(3)} mm`:'相交 / 接触；负值为哨兵，不是穿透深度';
 $('hud').innerHTML=`<b>${w.phase==='receiver_lift'?'升带受阻 · H='+w.height_m.toFixed(3)+' m':'拒绝状态'} · ${w.pair.join(' ↔ ')}</b><br>${reasons[w.reason]||w.reason}<br>${distance}<br>非执行位置 · ${active.request.attachment?'附着':'未附着'} · 完成抓放 ${failureData.completed_count} / 40`;
 $('motionText').textContent='静态受检姿态；不补造运动';$('reviewWitness').value=index;
 window.failureReviewState={kind:'witness',index,q:currentQ,pair:w.pair,expected:w.expected_flange_flat};
}
function reviewCamera(side){camera.position.set(...(side?[-.95,-6.7,2.65]:[-4.6,-5.1,3.7]));controls.target.set(-.75,0,1.4);controls.update();}
function reviewContact(){
 const e=failureData.native_endpoints?.find(e=>e.stage==='contact');if(!e)return;
 reviewSelectAttempt(e.attempt_index);setMotion(active.final_timed_path.length-1);reviewAnnotations(e.q_rad);
 const s=e.arm_posture;
 $('hud').innerHTML=`<b>原生接触路径终点 · ${failureData.target_id}</b><br>大臂仰角 ${s.upper_arm.elevation_deg.toFixed(1)}° · 小臂仰角 ${s.forearm_elevation_deg.toFixed(1)}°<br>规划状态 · 尚未完成卸箱 · 未执行`;
 window.failureReviewState={kind:'contact',q:currentQ,pair:[],expected:e.expected_flange_flat};
}
function reviewExtraction(){
 const i=DATA.raw.attempts.findIndex(a=>a.request.stage==='extraction' && a.response_identity.status!=='SUCCESS' && a.request.goal_pose);if(i<0)return;
 reviewSelectAttempt(i);const a=DATA.raw.attempts[i],q=a.request.q_start;pose(q);reviewAnnotations(q);
 const f=fk(q),fv=f.flange.clone().invert().multiply(f.m710_virtual_task_tcp);
 const goal=matrix(a.request.goal_pose).multiply(fv.invert()).multiply(matrix(model.flange_from_contact));
 const start=f.flange.clone().multiply(matrix(model.flange_from_contact)),delta=position(goal).sub(position(start));
 const axis=new T.AxesHelper(.3);axis.matrixAutoUpdate=false;axis.matrix.copy(goal);reviewGroup.add(axis);
 if(delta.length()>1e-8)reviewGroup.add(new T.ArrowHelper(delta.clone().normalize(),position(start),delta.length(),0xff5870,.12,.055));
 $('hud').innerHTML=`<b>带箱外移失败 · ${failureData.target_id}</b><br>红色箭头指向请求的吸具目标<br>${a.response_identity.solver_message||a.response_identity.status} · 未返回目标关节构型<br>机械臂显示实际请求起点 · 未执行`;
 $('motionText').textContent='只显示归档起点和笛卡尔目标；没有补造外移动画';
 window.failureReviewState={kind:'extraction_start',q:currentQ,pair:[],expected:a.start_expected_flange_flat};
}
$('reviewInitial').onclick=reviewInitial;$('reviewShow').onclick=()=>reviewWitness(Number($('reviewWitness').value));
const firstLiftWitness=failureData.witnesses.findIndex(w=>w.phase==='receiver_lift');
$('reviewLift').disabled=firstLiftWitness<0;$('reviewLift').onclick=()=>reviewWitness(firstLiftWitness);
$('reviewSide').onclick=()=>reviewCamera(true);$('reviewIso').onclick=()=>reviewCamera(false);
$('reviewContact').disabled=!failureData.native_endpoints?.some(e=>e.stage==='contact');$('reviewContact').onclick=reviewContact;
$('reviewExtraction').disabled=!DATA.raw.attempts.some(a=>a.request.stage==='extraction' && a.response_identity.status!=='SUCCESS' && a.request.goal_pose);$('reviewExtraction').onclick=reviewExtraction;
window.failureReview={initial:reviewInitial,witness:reviewWitness,camera:reviewCamera,contact:reviewContact,extraction:reviewExtraction,data:failureData};
if(firstLiftWitness>=0)reviewWitness(firstLiftWitness);else reviewInitial();reviewCamera(true);
