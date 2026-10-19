// All poses come from Python's authoritative OBB scene; no second linkage solver.
function drawCoupled(){
 clear(content);
 const state=R.states[Number($('height').value)],m=state.mechanism;
 for(const b of state.world){
   const mechanism=b.id.startsWith('lift_'),belt=b.category==='conveyor';
   let color=mechanism?0xf1ad4b:belt?0x54b9e7:0x71879b;
   let opacity=mechanism||belt?1:b.category==='carton'?.12:b.category==='trailer'?.035:.18;
   box(b,color,opacity);
   if(mechanism||belt)box(b,0x172636,.6,true);
 }
 const rear=m.rear_hinge_world,near=m.near_tip_world;
 line([rear,near],0xdafaf7);label('远端固定铰点',rear.map((x,i)=>x+(i===2?.18:0)));
 label('近端升降 + X 滑移',near.map((x,i)=>x+(i===2?.18:0)),'#95f0d2');
 const rows=[['横带带面',m.height_m.toFixed(3)+' m'],['纵带倾角',(m.tilt_rad*180/Math.PI).toFixed(2)+'°'],
 ['纵带固定长度',m.longitudinal_length_m.toFixed(3)+' m'],['近端 X 滑移',(1000*m.near_tip_slide_x_m).toFixed(1)+' mm'],
 ['远端高度',rear[2].toFixed(3)+' m'],['当前几何状态',Number($('height').value)+'/100']];
 $('heights').innerHTML=rows.map(([a,b])=>`<dt>${a}</dt><dd>${b}</dd>`).join('');
 $('region').textContent='近端 X 位于横带边缘范围：'+(m.near_tip_within_transverse_x?'是':'否')+'\n该判断不等于已验证箱体转接。';
 $('issues').textContent='剪叉与滑动/转动支座：项目概念几何\n尚未评估：机构碰撞扫掠、承载、驱动力、升降速度、倾斜输送与两带交接。';
 $('raw').textContent=JSON.stringify({source:R.source_commit,scene:state.scene_fingerprint,mechanism:m},null,2);
 $('hud').innerHTML='<b>横纵带剪叉机构与联动</b><br>纵带远端固定，近端随横带升降并自然滑移<br><span class="badge">横带 '+m.height_m.toFixed(3)+' m · 纵带 '+(m.tilt_rad*180/Math.PI).toFixed(2)+'°</span>';
 window.previewState=state;window.displaySceneIdentity=state.scene_fingerprint;
}
function initCoupled(){
 document.title='M-710 · 剪叉升降与纵带倾斜联动';
 document.querySelector('h1').textContent='剪叉升降 · 横纵带联动';
 document.querySelector('header small').textContent='同一几何源驱动显示与 USD 碰撞体 · 纵带保持 2.8 m 刚性长度';
 document.querySelector('header .scope').innerHTML='KINEMATIC_MODEL_NOT_EXECUTABLE<br>PHYSICAL_EXECUTION=NOT_RUN';
 const sections=document.querySelectorAll('aside section');
 sections[0].style.display='none';sections[2].style.display='none';
 sections[1].querySelector('h3').textContent='高度参数（不是运动时间）';
 sections[1].querySelector('.muted').textContent='0.60–2.05 m 为本布局展示区间，未认定硬件行程。';
 document.querySelector('.legend').innerHTML='<span class="swatch" style="background:#54b9e7"></span>横带与固定远端的纵带<br><span class="swatch" style="background:#f1ad4b"></span>剪叉杆、导轨、转轴与后端支架<br><span class="swatch" style="background:#71879b"></span>原场景保留，未移动其他箱体或底盘';
 $('bottom').textContent='几何联动可查看、可导出 USD；不是机械设计或运动许可。未启用旧原生纯 Z 扫掠；未运行机器人、物理或放箱。';
 Object.assign($('height'),{min:0,max:R.states.length-1,step:1,value:0});
 $('height').oninput=drawCoupled;
 $('max').textContent='升到 Hmax';$('min').textContent='降到初始';$('recommended').textContent='三维视角';
 $('max').onclick=()=>window.setLiftIndex(100);$('min').onclick=()=>window.setLiftIndex(0);
 $('recommended').onclick=()=>{camera.position.set(-5.2,-5.8,3.5);controls.target.set(-1.2,-.15,1.1);controls.update();};
 $('download').textContent='保存当前机构状态';
 window.setLiftIndex=n=>{$('height').value=n;drawCoupled();};
 if(DATA.model)drawRobot();
 resize();$('recommended').click();drawCoupled();
}
