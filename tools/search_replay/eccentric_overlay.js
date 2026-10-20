
// This extension only annotates the existing actual-q replay.
const eccentricGroup=new T.Group();scene.add(eccentricGroup);
const eccentricPanel=document.createElement('section');
eccentricPanel.innerHTML='<h3>面中心 / 偏心参考 / 实际上臂</h3><pre id="eccentricInfo"></pre><div>白：真实面中心　紫：本次工具参考点<br>吸盘内圈：几何有效（绿）<br>中圈：命令启用（黄）<br>外圈：实际 contact 末态几何（青）；灰＝尚未到达</div><p>偏心静态预览（未派发，不改变机器人 q）</p><select id="offsetPreview"></select><pre id="previewInfo"></pre>';
document.querySelector('aside').prepend(eccentricPanel);
const toolToggle=document.createElement('label');
toolToggle.innerHTML='<input type="checkbox" id="cupXray" checked> 工具半透明（显示全部杯与参考点）';
eccentricPanel.append(toolToggle);
let eccentricState=null;
const originalPose=pose;
function eccentricAnnotations(q){
  clear(eccentricGroup);
  const frames=fk(q),a=position(frames.J2_link),b=position(frames.J3_link),u=b.clone().sub(a);
  const alpha=Math.atan2(u.z,Math.hypot(u.x,u.y));
  eccentricGroup.add(line([a,b],0xffffff));
  eccentricGroup.add(new T.ArrowHelper(u.clone().normalize(),a,u.length(),0xffffff,.07,.035));
  const row=DATA.eccentric.targets[active.request.task_id],target=row?.target;
  const atContact=!!row?.contact_q&&row.contact_q.every((v,i)=>Math.abs(v-q[i])<1e-9);
  const physical=frames.flange.clone().multiply(matrix(model.flange_from_contact));
  const eligible=row?.expected?.geometrically_eligible_mask||Array(72).fill(false);
  const commanded=row?.expected?.commanded_active_mask||Array(72).fill(false);
  const actual=atContact?row?.actual?.actual_contact_mask:null;
  for(const cup of DATA.eccentric.cups.cups){
    const center=new T.Vector3(...cup.center_contact_frame_m).applyMatrix4(physical);
    for(const [radius,color] of [
      [.014,eligible[cup.index]?0x4de5a6:0x667085],
      [.018,commanded[cup.index]?0xffd15c:0x667085],
      [.022,actual?.[cup.index]?0x59e5ff:0x667085]]){
      const points=[];for(let i=0;i<=32;i++){
        const angle=i*2*Math.PI/32,local=new T.Vector3(...cup.center_contact_frame_m);
        local.x+=radius*Math.cos(angle);local.y+=radius*Math.sin(angle);
        points.push(local.applyMatrix4(physical));
      }
      eccentricGroup.add(line(points,color));
    }
  }
  if(target){
    const center=new T.Vector3(...target.face_center_m),reference=position(matrix(target.contact_pose));
    const sphere=(p,c)=>{const o=new T.Mesh(new T.SphereGeometry(.018,14,10),new T.MeshBasicMaterial({color:c}));o.position.copy(p);eccentricGroup.add(o);};
    sphere(center,0xffffff);sphere(reference,0xbf8cff);
    const boundary=target.face_boundary_m.map(p=>new T.Vector3(...p));boundary.push(boundary[0]);
    eccentricGroup.add(line(boundary,0xffffff,.7));
    if(target.world_y_offset_m>0)eccentricGroup.add(new T.ArrowHelper(new T.Vector3(0,1,0),center,target.world_y_offset_m,0x65baff,.04,.025));
    const pending=row.pending_target_geometry||[];
    if($('offsetPreview').dataset.task!==active.request.task_id){
      $('offsetPreview').dataset.task=active.request.task_id;
      $('offsetPreview').innerHTML='<option value="-1">仅看实际派发目标</option>'+pending.map((p,i)=>`<option value="${i}">δ=${p.target.world_y_offset_m.toFixed(5)} m · ${p.eligible_count} 杯有效</option>`).join('');
    }
    const preview=pending[Number($('offsetPreview').value)];
    $('previewInfo').textContent='';
    if(preview){
      const pt=preview.target,pm=matrix(pt.contact_pose),ref=position(pm);
      sphere(ref,0x65baff);
      if(pt.world_y_offset_m>0)eccentricGroup.add(new T.ArrowHelper(new T.Vector3(0,1,0),center,pt.world_y_offset_m,0x65baff,.04,.025));
      for(const cup of DATA.eccentric.cups.cups){
        const ring=[];for(let i=0;i<=32;i++){const angle=i*2*Math.PI/32;
          ring.push(new T.Vector3(cup.center_contact_frame_m[0]+cup.seal_radius_m*Math.cos(angle),
            cup.center_contact_frame_m[1]+cup.seal_radius_m*Math.sin(angle),0).applyMatrix4(pm));}
        eccentricGroup.add(line(ring,pt.predicted_eligible_mask[cup.index]?0x65baff:0x667085,.6));
      }
      $('previewInfo').textContent=`未派发 δ=${pt.world_y_offset_m.toFixed(5)} m\n预测有效杯 ${preview.eligible_count}/72；命令/实际接触未产生\n只显示目标几何；没有这个偏心目标的 IK 或轨迹`;
    }
  }
  eccentricState={q:q.slice(),q2_rad:q[1],alpha_rad:alpha,shoulder:a.toArray(),elbow:b.toArray(),
    candidate_id:target?.candidate_id,delta_m:target?.world_y_offset_m,
    eligible:eligible.filter(Boolean).length,commanded:commanded.filter(Boolean).length,
    actual:actual?actual.filter(Boolean).length:null,physical_cups:72,
    face_center:target?.face_center_m,reference:target?.contact_pose.map(r=>r[3]).slice(0,3)};
  // Annotations are visible through the solid; this changes display only.
  eccentricGroup.traverse(o=>{if(o.material){o.material.depthTest=false;o.renderOrder=10;}});
  for(const [o,l] of modelObjects)if(l.tool){o.material.transparent=$('cupXray').checked;o.material.opacity=$('cupXray').checked ? .25 : 1;}
  $('eccentricInfo').textContent=`J2 = ${(q[1]*180/Math.PI).toFixed(3)}°\n上臂仰角 α = ${(alpha*180/Math.PI).toFixed(3)}°\n结构长度 ${u.length().toFixed(3)} m\n`+
    `δ = ${target?target.world_y_offset_m.toFixed(5):'未保存目标详情'} m\n临时高度 ${row?.option.H_temporary_m} m\nHmin ${row?.option.H_receive_m} m\n同步：${row?.receive_sync||'NOT_REACHED'}\n`+
    `全杯 72 / 有效 ${eccentricState.eligible} / 命令 ${eccentricState.commanded}\n实际 contact 几何：${eccentricState.actual??'NOT_REACHED'}\n`+
    `箱体原 Y：${target?.face_center_m[1]??'UNKNOWN'} m\n完整任务 ${DATA.eccentric.completed_count}/1\n`+
    '所选有效杯承载足够：理想假设\n静态目标检查不是运动轨迹';
}
pose=function(q,detail){originalPose(q,detail);eccentricAnnotations(q);};
$('offsetPreview').onchange=()=>eccentricAnnotations(currentQ);
$('cupXray').onchange=()=>eccentricAnnotations(currentQ);
window.eccentricReplay={
  inspect(index){setStage('pregrasp');$('attempt').value=index;chooseAttempt();$('goalInspect').click();return eccentricState;},
  get state(){return eccentricState;},
  closeup(){camera.position.set(-3.6,-3.4,3.1);controls.target.set(-.4,-.35,1.65);controls.update();},
  targetView(){camera.position.set(-2.3,-2.5,2.9);controls.target.set(.1,-.55,2.25);controls.update();}
};
document.querySelector('h1').textContent='c00 · 上臂方向与偏心候选';
$('notice').textContent='仅显示本次原生 q 和原始碰撞见证。白线是官方 J2→J3 结构方向；面中心保持不变。未完成阶段不补动画；接收与清空为理想规划假设。';
if(currentQ)eccentricAnnotations(currentQ);
window.ECCENTRIC_READY=true;
