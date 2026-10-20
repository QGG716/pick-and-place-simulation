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
 line([rear,near],0xdafaf7);label(m.rear_fixed?'历史固定低端':'低端：固定高度 / X 滑车',rear.map((x,i)=>x+(i===2?.18:0)));
 label(m.rear_fixed?'历史近端 X 滑移':'上端：保持横带接缝',near.map((x,i)=>x+(i===2?.18:0)),'#95f0d2');
 if(m.seam){const trans=state.world.find(b=>b.id==='conveyor_transverse'),long=state.world.find(b=>b.id==='conveyor_longitudinal');line([[rear[0],m.seam.ymin_world_m,rear[2]],[near[0],m.seam.ymin_world_m,near[2]],[trans.pose[0][3]+trans.size[0]/2,m.seam.ymin_world_m,m.height_m]],0xff6ca8);line([[near[0],m.seam.ymin_world_m,m.height_m],[near[0],m.seam.ymin_world_m+long.size[1],m.height_m]],0xefff77);}
 const rows=[['横带带面',m.height_m.toFixed(3)+' m'],['纵带倾角',(m.tilt_rad*180/Math.PI).toFixed(2)+'°'],
 ['纵带固定长度',m.longitudinal_length_m.toFixed(3)+' m'],['低端 X 滑移',(1000*(m.lower_slide_x_m||0)).toFixed(1)+' mm'],
 ['远端高度',rear[2].toFixed(3)+' m'],['当前几何状态',Number($('height').value)+'/'+(R.states.length-1)]];
 $('heights').innerHTML=rows.map(([a,b])=>`<dt>${a}</dt><dd>${b}</dd>`).join('');
 $('region').textContent=m.seam?'顶面接缝 0 mm；固定底面斜切避空，完整顶面不缩小。\n粉色：两带 Ymin；黄色：有效端边接缝。':'历史模型：近端 X 滑移。';
 $('issues').textContent='剪叉与滑动/转动支座：项目概念几何\n尚未评估：机构碰撞扫掠、承载、驱动力、升降速度、倾斜输送与两带交接。';
 $('raw').textContent=JSON.stringify({source:R.source_commit,scene:state.scene_fingerprint,mechanism:m},null,2);
 $('hud').innerHTML='<b>横纵带几何预览</b><br>同源实体；仅参数预览，无物理接收<br><span class="badge">横带 '+m.height_m.toFixed(3)+' m · 纵带 '+(m.tilt_rad*180/Math.PI).toFixed(2)+'°</span>';
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
 document.querySelector('.legend').innerHTML='<span class="swatch" style="background:#54b9e7"></span>完整带面与实际斜切端部<br><span class="swatch" style="background:#f1ad4b"></span>剪叉、固定导轨、滑车、轴承<br><span class="swatch" style="background:#71879b"></span>固定底盘与原始箱堆';
 $('bottom').textContent='几何联动可查看、可导出 USD；不是机械设计或运动许可。未启用旧原生纯 Z 扫掠；未运行机器人、物理或放箱。';
 Object.assign($('height'),{min:0,max:R.states.length-1,step:1,value:0});
 $('height').oninput=drawCoupled;
 $('max').textContent='升到 Hmax';$('min').textContent='降到初始';$('recommended').textContent='三维视角';
 $('max').onclick=()=>window.setLiftIndex(R.states.length-1);$('min').onclick=()=>window.setLiftIndex(0);
 const middle=document.createElement('button');middle.textContent='Hmin';middle.onclick=()=>window.setLiftIndex(R.states.reduce((best,s,i)=>Math.abs(s.mechanism.height_m-R.working_heights.Hmin)<Math.abs(R.states[best].mechanism.height_m-R.working_heights.Hmin)?i:best,0));$('min').after(middle);
 $('recommended').onclick=()=>{camera.position.set(-5.2,-5.8,3.5);controls.target.set(-1.2,-.15,1.1);controls.update();};
 $('download').textContent='保存当前机构状态';
 window.setLiftIndex=n=>{$('height').value=n;drawCoupled();};
 if(DATA.model)drawRobot();
 resize();$('recommended').click();drawCoupled();
}
