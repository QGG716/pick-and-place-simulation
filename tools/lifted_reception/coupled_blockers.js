// Reuse the existing offline viewer and exact model/FK exporter. No fabricated q,
// unattached animated tool, conveyor travel, fall or drain is rendered.
let selectedFailure=0;
function drawBlocker(){
 clear(content);
 const f=R.initial.failures[selectedFailure],ids=new Set(f.pair);
 for(const b of R.state.world){
   const selected=ids.has(b.id),carton=b.category==='carton',metal=b.category==='lift_mechanism';
   box(b,selected?0xf17872:carton?0xbb935e:metal?0xd5a448:0x6d96b0,selected?.7:carton?.13:metal?.7:.07);
   if(selected)box(b,0xffe2be,1,true);
 }
 const v=f.exact_axis_aligned_intersection;
 if(v){
   const center=v.min_world_m.map((x,i)=>(x+v.max_world_m[i])/2);
   const pose=[[1,0,0,center[0]],[0,1,0,center[1]],[0,0,1,center[2]],[0,0,0,1]];
   box({pose,size:v.size_m},0xff214f,1);box({pose,size:v.size_m},0xffffff,1,true);
 }
 $('heights').innerHTML='<dt>横带当前高度</dt><dd>0.600 m</dd><dt>目标 Hmax / Hmin</dt><dd>2.050 / 1.750 m</dd><dt>保留箱体</dt><dd>'+R.initial.carton_ids.length+'</dd><dt>原生运动阶段</dt><dd>0</dd>';
 $('region').textContent=f.pair.join(' ↔ ')+'\n'+f.classification+'\n'+(v?'精确盒体相交尺寸：'+v.size_m.map(x=>(x*1000).toFixed(0)).join(' × ')+' mm':'表面间距：'+(f.surface_distance_m*1000).toFixed(3)+' mm')+'\n所需非装配间隙：'+(f.required_clearance_m*1000).toFixed(1)+' mm';
 $('issues').textContent='这些是现有概念模型实体的检测结果，不是原始 CAD 穿透证明。\n允许的销轴/滑轨/支座连接按具名拓扑逐对记录；未添加整体机构豁免。';
 $('raw').textContent=JSON.stringify(f,null,2);
 $('hud').innerHTML='<b>初态拒绝 · 0/'+R.summary.target_count+'</b><br>目标：尚未选定 · 仿真时间：未开始<br>附着：否 · 吸盘：未命令<br>名义接收 0 · 理想清空 0 · 实测接收 0';
 window.previewState={kind:'INITIAL_REJECTION_NOT_BATCH_REPLAY',q_rad:R.reference_q_rad,
   initial_world_sha256:R.initial.world_sha256,selected_pair:f.pair,completed_count:0,
   carton_count:R.initial.carton_ids.length,accepted_motion_edges:0};
 window.displaySceneIdentity=R.state.scene_fingerprint;
}
function blockerCloseup(){
 const f=R.initial.failures[selectedFailure],v=f.exact_axis_aligned_intersection;
 const c=v?v.min_world_m.map((x,i)=>(x+v.max_world_m[i])/2):f.bodies[1].pose.map(r=>r[3]).slice(0,3);
 camera.position.set(c[0]-.65,c[1]-.85,c[2]+.35);controls.target.set(...c);controls.update();
}
function initCoupled(){
 document.title='M-710 · 顶层批次初态阻塞证据';
 document.querySelector('h1').textContent='顶层批次 · 初态几何拒绝';
 document.querySelector('header small').textContent='完整 40 箱 / 同一 coupled 世界 / 未启动抓放规划';
 document.querySelector('header .scope').innerHTML='BATCH_START_REJECTED · 0/'+R.summary.target_count+'<br>PHYSICAL_EXECUTION=NOT_RUN';
 const sections=document.querySelectorAll('aside section');
 sections[0].querySelector('h3').textContent='未声明接触与相交';
 sections[1].querySelector('h3').textContent='保持原始状态';
 sections[1].querySelector('.muted').textContent='机构没有移动；下方 35 箱及初始顶层全部保留。';
 sections[2].style.display='none';$('height').style.display='none';
 for(const [i,f] of R.initial.failures.entries())$('case').add(new Option(f.pair.join(' / '),i));
 selectedFailure=Math.max(0,R.initial.failures.findIndex(f=>f.exact_axis_aligned_intersection));
 $('case').value=selectedFailure;
 $('case').onchange=()=>{selectedFailure=Number($('case').value);drawBlocker();blockerCloseup();};
 $('caseInfo').textContent='两个预先固定种子均保留为批次启动拒绝，不是两次完整批次或成功率样本。';
 $('initial').textContent='q = '+R.reference_q_rad.map(x=>x.toFixed(6)).join(', ')+'\n'+R.protocol.frozen_top_layer.target_ids.join('\n');
 $('max').textContent='相交特写';$('min').textContent='整场景';$('recommended').textContent='后端侧视';
 $('max').onclick=blockerCloseup;
 $('min').onclick=()=>{camera.position.set(-5.2,-5.8,3.5);controls.target.set(-1.2,-.15,1.1);controls.update();};
 $('recommended').onclick=()=>{camera.position.set(-4.4,-.75,.6);controls.target.set(-2.8,-.75,.48);controls.update();};
 document.querySelector('.legend').innerHTML='红色：当前被拒绝实体对<br>亮红实体：精确轴对齐相交体积<br>半透明机器人：官方模型在本次原始 q<br>全部箱体保留；没有独立运动吸具。';
 $('bottom').textContent='诊断查看器，不是完整批次回放。没有生成接近、抓取、释放、名义触带或清空事件。';
 $('hud').style.top='auto';$('hud').style.bottom='75px';$('hud').style.fontSize='12px';
 $('download').textContent='保存阻塞与原始世界证据';
 drawRobot();resize();$('min').click();drawBlocker();
 window.showBlocker=n=>{$('case').value=n;selectedFailure=n;drawBlocker();blockerCloseup();};
 window.showWholeScene=()=>$('min').click();
}
