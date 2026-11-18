// A separate static start preview; never replace the archived planning start.
if(DATA.start_rotation_preview){
  document.title='ECO65 · 新现场重规划与起点半周旋转预览';
  $('heading').textContent='新现场重规划 · 实测起点与 J6 顺时针 180° 预览';
  $('subtitle').textContent='规划始终使用原始实测起点 · 预览仅改 J6 · 无实机动作 / 无执行许可';
  $('case').previousElementSibling.textContent='本帧恢复的目标';
  $('candidate').previousElementSibling.textContent='规划候选或目标检查结论';
  const style=document.createElement('style');style.textContent='body{height:100vh;overflow:hidden}aside{min-width:0}#view{min-height:0}#start-details{overflow-wrap:anywhere;min-height:125px}#reason{max-height:190px;overflow:auto}#start-rotation-panel button[aria-pressed=true]{background:#325d87;border-color:#8ed9ec}';document.head.append(style);
  const panel=document.createElement('section');panel.id='start-rotation-panel';
  panel.innerHTML='<label>起点预览（独立于规划）</label><div class="buttons"><button id="start-measured">实测起点</button><button id="start-rotated">顺时针 180°</button></div><button id="start-rejection">查看本次规划拒绝位置</button><div id="start-details" style="white-space:pre-wrap;font-variant-numeric:tabular-nums"></div><small>从 J6 正轴端朝轴心看定义顺时针。灰：原吸具 +Y；粉：当前 +Y。青点为实际观测的平面支持点；箱体包络含推断与不确定性。</small>';
  $('stats').after(panel);
  panel.after($('reason'));
  view.querySelector('.legend').textContent='黄/红：含不确定性的箱体包络　青点：观测平面支持点　蓝灰：现场障碍';
  const beforeArrow=new T.ArrowHelper(new T.Vector3(0,1,0),new T.Vector3(),.13,0x9aaebc,.02,.007);
  const afterArrow=new T.ArrowHelper(new T.Vector3(0,1,0),new T.Vector3(),.13,0xfa7fd3,.02,.007);
  scene.add(beforeArrow,afterArrow);
  const observed=[];
  for(const c of DATA.cases){
    for(const f of c.box.faces.filter(f=>f.observed&&f.support_points_uv_m)){
      const vertices=f.support_points_uv_m.flatMap(p=>f.center_m.map((x,i)=>x+p[0]*f.u[i]+p[1]*f.v[i]));
      const g=new T.BufferGeometry();g.setAttribute('position',new T.Float32BufferAttribute(vertices,3));
      const points=new T.Points(g,new T.PointsMaterial({color:0x55e8df,size:.0025}));scene.add(points);observed.push(points);
    }
  }
  const originalDraw=draw;
  draw=function(){
    const v=modes[Number($('mode').value)]?.startRotation;
    beforeArrow.visible=afterArrow.visible=!!v;
    if(v){
      for(const [a,f] of [[beforeArrow,fk(active.q_start)[model.tcp_link]],[afterArrow,currentFrames[model.tcp_link]]]){
        a.position.setFromMatrixPosition(f);a.setDirection(new T.Vector3(0,1,0).transformDirection(f));
      }
    }
    for(const r of world){if(DATA.cases.some(c=>c.box.id===r.id))r.o.material.opacity=.24;}
    for(const p of observed)p.visible=$('environment').checked;
    originalDraw();
  };
  function note(v){
    const f=v.failure;
    return `J6 增量 ${v.j6_delta_deg.toFixed(0)}°；J1–J5 不变\n`+
      `静态机器人/现场碰撞：${v.static_valid?'通过':'拒绝'}`+
      (f?.pair?' · '+f.pair.join(' ↔ '):'')+'\n'+
      (v.wrist_rejection?'腕部规则：'+v.wrist_rejection.reason+'\n':'')+
      '仅端点；未验证原位转动路径。规划 q_start 未改变。';
  }
  const originalDetails=retryDetails;
  retryDetails=function(){
    originalDetails();
    const m=modes[Number($('mode').value)],v=m.startRotation;
    $('start-measured').setAttribute('aria-pressed',v?.name==='measured');
    $('start-rotated').setAttribute('aria-pressed',v?.name==='rotated-preview');
    $('start-rejection').disabled=!active.target_failures.length;
    $('start-details').textContent=v?`实测 J6 ${active.q_start[5]*180/Math.PI < 0?'':'+'}${(active.q_start[5]*180/Math.PI).toFixed(3)}° → 预览 ${(v.q[5]*180/Math.PI).toFixed(3)}°\n`+note(v):'当前显示本次规划保存的状态。';
    $('reason').textContent=chosen.reason;$('reason').title=chosen.reason;
    $('hud').textContent=`目标 ${Number($('case').value)+1} · base_link · ${world.length} 个环境与箱体 · ${model.cup_ids.length} 杯\n${m.label}`;
    $('candidate').style.display=chosen.target_level?'none':'block';
    $('candidate').previousElementSibling.style.display=chosen.target_level?'none':'block';
    if(chosen.target_level){
      $('posture-panel').style.display='none';
      const hit=active.target_failures.flatMap(f=>f.evidence?.intersections||[])[0];
      if(hit)$('reason').textContent=`规划未通过：箱体不确定性包络相交 ${(hit.overlap_depth_m*1000).toFixed(3)} mm。\n未进入候选生成、逆解与路径搜索。\n此值描述模型包络，不等同于实物穿入深度。\n完整原因可悬停查看，或查阅 planning-failures.json。`;
    }
    draw();
  };
  const originalCandidate=candidate;
  candidate=function(){
    originalCandidate();
    const original=active.start_rotation[0],rotated=active.start_rotation[1];
    Object.assign(modes[0],{startRotation:original,pair:original.failure?.pair,note:note(original)});
    modes.push({label:'起点 · J6 顺时针 180° 预览',q:[rotated.q],startRotation:rotated,pair:rotated.failure?.pair,note:note(rotated)});
    if(active.target_failures.length){
      const ids=[active.box.id,...active.target_failures.flatMap(f=>(f.evidence?.intersections||[]).map(i=>i.id))];
      modes.push({label:'本次规划拒绝 · 箱体包络',q:[active.q_start],pair:ids,targetVerdict:true,
        note:'红色为规划拒绝涉及的保守箱体包络；机器人保持实测起点。没有对应的抓取逆解或路径。'});
    }
    $('mode').replaceChildren();modes.forEach((m,i)=>option($('mode'),m.label,i));
    $('mode').value=modes.findIndex(m=>m.startRotation?.name==='rotated-preview');
    if(chosen.target_level)$('candidate').selectedOptions[0].textContent='目标检查 · 未进入候选 / 逆解';
    mode();
  };
  $('candidate').onchange=candidate;
  $('start-measured').onclick=()=>chooseMode(m=>m.startRotation?.name==='measured');
  $('start-rotated').onclick=()=>chooseMode(m=>m.startRotation?.name==='rotated-preview');
  $('start-rejection').onclick=()=>chooseMode(m=>m.targetVerdict);
  [...$('case').options].forEach((o,i)=>{o.textContent=`目标 ${i+1} · ${DATA.cases[i].box.id.split('-carton-').pop()}`;});
  selectCase();
}
