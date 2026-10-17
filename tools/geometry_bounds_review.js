/* Optional finite-bound inspection on the existing metric viewer. */
(() => {
  if (!D.instances.some(r => r.record.boundary_constraints || r.boundary_comparison)) return;
  const view = $('view');
  for (const [value, label] of [['bounds','物理边与两侧原始支持'], ['before_edges','修改前候选边（诊断）']]) {
    const option = document.createElement('option'); option.value=value; option.textContent=label; view.append(option);
  }
  const chooser=document.createElement('select'); chooser.id='edge'; view.after(chooser);
  const note=document.createElement('p'); chooser.after(note);
  const focusButton=document.createElement('button');focusButton.textContent='定位当前实例（原生像素）';note.after(focusButton);
  function focusInstance(){
    $('native').checked=true;draw();const p=current().proposals[0];if(!p)return;
    const b=p.actual_prompt?.base_bbox||p.bbox,container=$('image').parentElement;
    container.scrollLeft=(b[0]+b[2])/2-container.clientWidth/2;
    container.scrollTop=(b[1]+b[3])/2-container.clientHeight/2;
  }
  focusButton.onclick=focusInstance;
  const before=document.createElement('label'); before.innerHTML='<input type="checkbox" id="beforebox"> 橙色修改前候选（诊断）'; $('reset').before(before);
  const section=document.createElement('section'); section.className='panel';
  section.innerHTML='<h3>六边界约束与有符号误差</h3><p>行轴坐标 q = camera_xyz × axesᵀ；误差为预测 − 实际网格参考。单位 mm。空缺表示未确定，不使用薄盒占位。</p><div class="scroll"><table id="boundsTable"></table></div><p id="boundSummary"></p>';
  document.querySelector('.grid').after(section);
  const oldDraw=draw, oldSelect=select;
  const mm=v=>v==null?'未确定':(v*1000).toFixed(2);
  const esc=v=>String(v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const line=(ctx,a,b,color,width=2)=>{ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke()};
  const dot=(ctx,p,color,r=2)=>{ctx.fillStyle=color;ctx.beginPath();ctx.arc(...p,r,0,2*Math.PI);ctx.fill()};
  function choices(){
    chooser.replaceChildren(); const r=current(), evidence=r.record.boundary_constraints;
    const options=[['all','全部边']];
    (evidence?.edges||[]).forEach((e,i)=>options.push(['outer:'+i,`面 ${e.support_label} / 边 ${e.edge_index} → 轴 ${e.bound_axis} ${e.bound_side} · ${e.status}`]));
    (evidence?.shared_edges||[]).forEach((e,i)=>options.push(['joint:'+i,`共同边 ${e.support_labels.join(' ↔ ')} · ${e.supported_samples}/15` ]));
    options.forEach(([v,t])=>{const o=document.createElement('option');o.value=v;o.textContent=t;chooser.append(o)});
  }
  draw=function(){
    oldDraw(); const r=current(), mode=view.value, ctx=$('image').getContext('2d'), evidence=r.record.boundary_constraints;
    chooser.style.display=mode==='bounds'?'inline-block':'none';
    if(mode==='before_edges'){
      note.textContent='修改前的候选位置；勾选“红色拒绝候选（诊断）”才显示。橙色为旧规则支持，红色为旧规则拒绝。';
      if($('rejected').checked)for(const f of r.before_record?.completion_diagnostics?.candidate_faces||[])for(let i=0;i<4;i++){
        const b=f.boundary_evidence[i];line(ctx,f.corners_2d[i],f.corners_2d[(i+1)%4],b.kind==='PHYSICAL_EDGE_SUPPORTED'?'#ff9c4b':'#ff5365',3);
        const p=f.corners_2d[i];ctx.fillStyle='white';ctx.font='20px system-ui';ctx.fillText(`面${f.support_label}/边${i} ${b.kind}`,p[0],p[1]);
      }
      return;
    }
    if(mode!=='bounds'){note.textContent='';return}
    const selected=chooser.value; const reasons=[];
    for(const [i,e] of (evidence?.edges||[]).entries()){
      if(selected!=='all'&&selected!=='outer:'+i)continue;
      // The unaccepted candidate segment is diagnostic, independently toggled.
      if($('rejected').checked)line(ctx,...e.candidate_segment_pixels,'#ff5365',2);
      const counts={};
      for(const s of e.samples){
        counts[s.kind]=(counts[s.kind]||0)+1;
        const color=s.kind==='PHYSICAL_OUTLINE'?'#33ff88':s.kind==='OCCLUSION_BOUNDARY'?'#ff5365':'#ffba55';
        if(s.search_end_pixel)line(ctx,s.candidate_pixel,s.search_end_pixel,'#bb82ff',1);
        if(s.inner_pixel)dot(ctx,s.inner_pixel,'#fff',2);
        if(s.outer_pixel)dot(ctx,s.outer_pixel,color,2);
        if(s.pixel)dot(ctx,s.pixel,'#33ff88',3);
      }
      reasons.push(`面 ${e.support_label}/边 ${e.edge_index}：${JSON.stringify(counts)}`);
    }
    for(const [i,e] of (evidence?.shared_edges||[]).entries()){
      if(selected!=='all'&&selected!=='joint:'+i)continue;
      if(e.supported||$('rejected').checked)line(ctx,...e.corners_2d,e.supported?'#33ff88':'#ff5365',2);
      for(const s of e.samples){
        ctx.strokeStyle='#bb82ff';ctx.lineWidth=1;ctx.beginPath();ctx.arc(...s.pixel,e.search_px,0,2*Math.PI);ctx.stroke();
        s.surfaces.forEach((f,j)=>(f.point_pixels||[]).forEach(p=>dot(ctx,p,j?'#ffba55':'#fff',2)));
      }
      reasons.push(`共同边 ${e.support_labels.join(' ↔ ')}：邻近两侧 ${e.supported_samples}/15；交线实测 ${e.crease_depth_check?.status||'未记录'}，${e.reason}`);
    }
    note.textContent='绿点：定位的物理过渡；白/橙点：实际支持；紫线/圆：搜索范围。'+reasons.join('；');
  };
  select=function(){
    oldSelect();choices(); const r=current(), c=r.boundary_comparison;
    if(c){
      $('boundsTable').innerHTML='<tr><th>轴 / 端</th><th>初始分位数</th><th>旧优化后</th><th>正式约束</th><th>轮廓投影（仅诊断）</th><th>网格参考</th><th>旧误差</th><th>正式误差</th><th>诊断投影误差</th><th>来源 / 状态</th></tr>'+c.bounds.map(b=>`<tr><td>${b.axis} (${b.world_direction}) / ${b.side}</td><td>${mm(b.quantile_initial_m)}</td><td>${mm(b.before_optimized_m)}</td><td>${mm(b.final_m)}</td><td>${mm(b.diagnostic_outline_projection_m)}</td><td>${mm(b.reference_m)}</td><td>${mm(b.before_signed_error_m)}</td><td>${mm(b.final_signed_error_m)}</td><td>${mm(b.diagnostic_outline_signed_error_m)}</td><td>${esc(b.constraint_sources.map(s=>s.source+' / 面 '+s.support_label).join('; '))}<br>${esc(b.constraint_status)}</td></tr>`).join('');
      const describe=b=>b?`中心 ${mm(b.center_error_m)} mm，最大尺寸 ${mm(Math.max(...b.sorted_dimension_error_m))} mm，最大角点 ${mm(b.corner_max_m)} mm，精度 ${b.external_accuracy_pass?'通过':'未通过'}`:'未接受完整体；只输出可靠表面及已确定边界';
      $('boundSummary').textContent='修改前：'+describe(c.before_box)+'。本轮：'+describe(c.current_box)+'。'+(c.rejected_diagnostic_box?'拒绝候选（仅诊断）：'+describe(c.rejected_diagnostic_box)+'。':'');
      $('details').textContent=JSON.stringify({dimensions_in_corresponding_axes:c.dimensions,missing_constraints:r.record.boundary_constraints?.missing_bounds,planes:r.evaluation.planes},null,2);
    }else{$('boundsTable').textContent='该实例没有完整体求解，或没有本次实际 mesh 参考。';$('boundSummary').textContent=''}
    draw();
  };
  chooser.onchange=draw;$('beforebox').onchange=draw;
  for(const id of ['view','gt','native','initial3d','rejected'])$(id).onchange=draw;
  rgb.onload=draw;depth.onload=draw;
  choice.onchange=select;
  const wanted=new URLSearchParams(location.search).get('mask');
  const index=D.instances.findIndex(r=>String(r.mask_id)===wanted);if(index>=0)choice.value=index;
  select();
  if(index>=0){if(rgb.complete&&rgb.naturalWidth)focusInstance();else rgb.onload=focusInstance;}
})();
