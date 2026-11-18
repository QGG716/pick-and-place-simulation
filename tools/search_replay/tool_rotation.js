// Optional static before/after view. No interpolation, motion or old pass reuse.
if(DATA.rotation_preview){
  $('heading').textContent='吸具在当前姿态上 · 再逆时针旋转 180°';
  document.title='ECO65 · 吸具新增 180° 前后对比';
  $('subtitle').textContent='保持 J1–J5、箱体位置与上移量 · 新姿态独立核验 · 未规划转动路径 / 无实机动作';
  $('posture-label').textContent='同一姿态 · 吸具旋转前后对比';
  $('posture-a').textContent='旋转前';$('posture-b').textContent='逆时针 +180° 后';
  $('posture-original').textContent='脱垛末点 · 旋转后';$('retry-preferred').style.display='none';
  $('posture-note').textContent='灰色箭头：旋转前吸具 +Y；粉色箭头：当前吸具 +Y。逆时针按从 J6 正轴端看向轴心定义。仅显示端点，未验证转动路径。绿色为接触区预计密封杯。';
  // A symmetric cup array can look similar after a half-turn. Show one
  // tool-fixed reference direction, independent of the unchanged J6 axis.
  const rollBefore=new T.ArrowHelper(new T.Vector3(0,1,0),new T.Vector3(),.13,0x9aaebc,.02,.007);
  const rollCurrent=new T.ArrowHelper(new T.Vector3(0,1,0),new T.Vector3(),.13,0xfa7fd3,.02,.007);
  scene.add(rollBefore,rollCurrent);
  const drawOriginal=draw;
  draw=function(){
    const v=modes[Number($('mode').value)]?.rotation;
    rollBefore.visible=rollCurrent.visible=!!v;
    if(v){
      const before=fk(v.original_q)[model.tcp_link],after=currentFrames[model.tcp_link];
      for(const [arrow,frame] of [[rollBefore,before],[rollCurrent,after]]){
        arrow.position.setFromMatrixPosition(frame);
        arrow.setDirection(new T.Vector3(0,1,0).transformDirection(frame));
      }
    }
    drawOriginal();
  };
  const stageName={approach:'接近',contact:'接触',detach:'原脱垛输出末点'};
  function staticNote(v){
    const gate=v.wrist_rejection;
    return `本轮新增 J6：${v.j6_delta_deg.toFixed(0)}°；J1–J5 保持原值\n`+
      `静态碰撞检查：${v.static_valid?'通过':'未通过'}；${gate?gate.reason+(Number.isFinite(gate.travel_deg)?` ${gate.travel_deg.toFixed(2)}° > ${gate.limit_deg.toFixed(2)}°`:''):'腕部规则通过'}\n`+
      '未规划旋转路径或完整抓放；无执行许可。\n来源：'+v.native_evidence;
  }
  retryDetails=function(){
    const m=modes[Number($('mode').value)],v=m.rotation;
    const details=$('posture-details').textContent.split('\n\nJ2/J3/J4')[0];
    if(v){
      const q6Before=v.original_q[5]*180/Math.PI,q6After=v.q[5]*180/Math.PI;
      $('posture-details').textContent=`本轮新增旋转：${v.j6_delta_deg.toFixed(0)}°\nJ6：${q6Before.toFixed(2)}° → ${q6After.toFixed(2)}°\n沿面上移：${(chosen.contact.contact_search.lift_along_face_m*1000).toFixed(0)} mm\n预计密封：${v.anticipated_seals} / 12 杯\n\n`+details;
      for(const r of robot){
        const id=model.cup_ids.find(id=>{const b=id.replace(/_seal$/,'');return r.item.id===b||r.item.id.startsWith(b+'_');});
        if(id&&!pair.includes(r.item.id))r.o.material.color.set(v.sealing_ids.includes(id)?0x52d69b:0x82919f);
      }
      $('reason').textContent=staticNote(v);
    }else $('reason').textContent='实测起点仅作参考；本轮没有修改 q_start。';
    $('posture-a').setAttribute('aria-pressed',!!v&&!v.after);
    $('posture-b').setAttribute('aria-pressed',!!v&&v.after);
    $('posture-original').setAttribute('aria-pressed',!!v&&v.stage==='detach'&&v.after);
    $('posture-original').disabled=!chosen.rotation_variants.some(v=>v.stage==='detach');
    draw();
  };
  candidate=function(){
    chosen=active.candidates[Number($('candidate').value)];
    modes=[{label:'实测 q_start',q:[active.q_start],note:'原始实测起点，不是旋转命令'}];
    for(const v of chosen.rotation_variants){
      modes.push({label:stageName[v.stage]+' · '+(v.after?'新增 +180° 后':'旋转前'),q:[v.q],
        attachment:v.attachment,pair:v.failure?.pair,stage:v.stage,rotation:v,note:staticNote(v)});
    }
    $('mode').replaceChildren();modes.forEach((m,i)=>option($('mode'),m.label,i));
    $('mode').value=modes.findIndex(m=>m.rotation?.after&&m.stage==='contact');
    $('posture-panel').style.display='block';
    $('stats').textContent=`箱位 ${Number($('case').value)+1} · 原候选 ${chosen.index} · 端点对比 / 非完整计划`;
    mode();
  };
  function selectRotation(after,stage){
    const current=modes[Number($('mode').value)];
    const target=stage||current.stage||'contact';
    const i=modes.findIndex(m=>m.rotation?.stage===target&&m.rotation.after===after);
    if(i>=0){$('mode').value=i;mode();}
  }
  $('posture-a').onclick=()=>selectRotation(false);
  $('posture-b').onclick=()=>selectRotation(true);
  $('posture-original').onclick=()=>selectRotation(true,'detach');
  $('candidate').onchange=candidate;
  // The most recently discussed field pose was case 2, contact, +60 mm.
  $('case').value=Math.min(1,DATA.cases.length-1);selectCase();
}
