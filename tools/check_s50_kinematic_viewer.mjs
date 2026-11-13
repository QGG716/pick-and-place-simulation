// Check this repository's offline viewer in a fresh, hidden Edge/Chrome profile.
// Never connect to an existing browser or alter a user's desktop session.
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import http from 'node:http';
import net from 'node:net';
import {spawn} from 'node:child_process';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';

const [html,output,browser]=process.argv.slice(2);
if(!html||!output||!browser)throw Error('Usage: node check_s50_kinematic_viewer.mjs HTML OUTPUT BROWSER');
await fs.mkdir(output,{recursive:true});
const body=await fs.readFile(html),server=http.createServer((req,res)=>{
  res.setHeader('Content-Type','text/html; charset=utf-8');res.end(body);
});
await new Promise(r=>server.listen(0,'127.0.0.1',r));
const probe=net.createServer();await new Promise(r=>probe.listen(0,'127.0.0.1',r));
const debugPort=probe.address().port;await new Promise(r=>probe.close(r));
const tempRoot=path.resolve(os.tmpdir()),profile=await fs.mkdtemp(path.join(tempRoot,'s50-viewer-'));
const child=spawn(browser,['--headless=new','--no-first-run','--no-default-browser-check',
  '--disable-background-networking','--disable-sync','--disable-extensions',
  '--disable-renderer-backgrounding','--disable-background-timer-throttling',
  '--enable-unsafe-swiftshader','--use-angle=swiftshader','--hide-scrollbars',
  '--remote-debugging-address=127.0.0.1',`--remote-debugging-port=${debugPort}`,
  `--user-data-dir=${profile}`,'about:blank'],{windowsHide:true,stdio:'ignore'});
const delay=ms=>new Promise(r=>setTimeout(r,ms));
let ws,serial=0;const pending=new Map(),errors=[];
const call=(method,params={})=>new Promise((resolve,reject)=>{
  const id=++serial,timer=setTimeout(()=>{pending.delete(id);reject(Error('CDP timeout: '+method));},30000);
  pending.set(id,{resolve:v=>{clearTimeout(timer);resolve(v);},reject:e=>{clearTimeout(timer);reject(e);}});
  ws.send(JSON.stringify({id,method,params}));
});
try{
  let ready=false;
  for(let i=0;i<100;i++){
    try{await fetch(`http://127.0.0.1:${debugPort}/json/version`);ready=true;break;}catch{}
    await delay(100);
  }
  if(!ready)throw Error('Fresh headless browser unavailable');
  const tab=await (await fetch(`http://127.0.0.1:${debugPort}/json/new?about:blank`,{method:'PUT'})).json();
  ws=new WebSocket(tab.webSocketDebuggerUrl);
  await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
  ws.onmessage=e=>{
    const value=JSON.parse(e.data);
    if(value.id){const p=pending.get(value.id);if(p){pending.delete(value.id);value.error?p.reject(Error(JSON.stringify(value.error))):p.resolve(value.result);}}
    else if(value.method==='Runtime.exceptionThrown')errors.push(value.params.exceptionDetails);
  };
  const evaluate=async expression=>{
    const r=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});
    if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;
  };
  await call('Page.enable');await call('Runtime.enable');
  await call('Emulation.setDeviceMetricsOverride',{width:1440,height:900,deviceScaleFactor:1,mobile:false});
  await call('Page.navigate',{url:`http://127.0.0.1:${server.address().port}/`});
  await call('Page.bringToFront');
  for(let i=0;i<150;i++){
    const state=await evaluate('({ready:window.S50_VIEWER_READY||window.S50_DIAGNOSTIC_READY,error:window.S50_VIEWER_ERROR||window.S50_DIAGNOSTIC_ERROR})');
    if(state.error)throw Error(state.error);
    if(state.ready){ready=true;break;}ready=false;await delay(100);
  }
  assert.ok(ready,'Viewer did not initialize');
  if(await evaluate('Boolean(window.S50_DIAGNOSTIC_READY)')){
    const checks=await evaluate(`(()=>{
      const D=S50_DIAGNOSTIC_DATA,S=D.diagnostic,K=S50Kinematics,checks=[];
      const flat=m=>[0,1,2,3].flatMap(c=>[0,1,2,3].map(r=>m[r][c]));
      const err=(a,b)=>Math.max(...a.map((v,i)=>Math.abs(v-b[i])));
      for(const [bi,b]of S.branches.entries()){
        s50Diagnostic.choose(bi,false);let fk=0,box=0,connector=0,states=0;
        for(const [i,knot]of b.knots.entries()){
          s50Diagnostic.show(i,false);const state=s50Diagnostic.state;
          fk=Math.max(fk,err(state.physical_contact.flat(),knot.world_from_physical_contact.flat()));
          box=Math.max(box,err(state.carton_matrix,flat(knot.world_from_carton)));
          if(state.rejected||state.cartons_visible!==40||err(state.q_rad,knot.q_rad)>1e-12)throw Error('Invalid diagnostic knot');
          const F=K.forward(D.model_definition,knot.q_rad,S.world_from_model_base,S.settings.joint_coordinate_convention),tcp=K.multiply(F.s50_flange,S.flange_from_virtual_task_tcp);
          s50Diagnostic.connectors.forEach((part,j)=>{
            const r=D.environment.tool_intrinsic.rigid_local_boxes_virtual_tcp_m[[11,12,13][j]],local=[[1,0,0,r[0]],[0,1,0,r[1]],[0,0,1,r[2]],[0,0,0,1]];
            if(!part.visible||!part.opaque)throw Error('Missing opaque original connector');
            connector=Math.max(connector,err(part.world_matrix,flat(K.multiply(tcp,local))));
          });states++;
        }
        s50Diagnostic.show(b.knots.length-1,true);
        if(!s50Diagnostic.state.rejected||err(s50Diagnostic.state.q_rad,b.rejected.q_rad)>1e-12)throw Error('Rejected candidate not distinguished');
        checks.push({branch:b.id,reason:b.reason,states,max_fk_error:fk,max_carton_error:box,max_connector_error:connector});
      }
      return {checks,source_sha:S.source_report_sha256};
    })()`);
    for(const c of checks.checks){assert.ok(c.max_fk_error<1e-9);assert.ok(c.max_carton_error<1e-9);assert.ok(c.max_connector_error<1e-9);}
    await evaluate("document.getElementById('collision').click();document.getElementById('play').click()");
    let playingStep=0;
    for(let i=0;i<20&&playingStep===0;i++){await delay(250);playingStep=await evaluate('s50Diagnostic.state.step');}
    assert.ok(playingStep>0,'Diagnostic playback advances');
    await evaluate("document.getElementById('pause').click()");
    const paused=await evaluate('s50Diagnostic.state.step');await delay(250);
    assert.equal(await evaluate('s50Diagnostic.state.step'),paused);
    await evaluate("document.getElementById('seek').value=8.5;document.getElementById('seek').dispatchEvent(new Event('input'))");
    assert.equal(await evaluate('s50Diagnostic.state.step'),8.5);
    const capture=async(name,expression)=>{
      await evaluate(expression);await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
      const shot=await call('Page.captureScreenshot',{format:'png'});await fs.writeFile(path.join(output,name),Buffer.from(shot.data,'base64'));
    };
    await capture('collision-close.png',"document.getElementById('collision').click()");
    await capture('collision-overview.png',"s50Diagnostic.cameraView('overview')");
    await capture('ik-boundary.png',"document.getElementById('ik').click()");
    await capture('connector.png',"document.getElementById('collision').click();s50Diagnostic.cameraView('tcp')");
    assert.equal(errors.length,0);
    const result={status:'PASS',scope:'ACTUAL_REJECTED_WITHDRAWAL_DIAGNOSTIC',
      html_sha256:createHash('sha256').update(body).digest('hex'),source_report_sha256:checks.source_sha,
      branches:checks.checks,play_pause_seek:'PASS',initial_cartons_visible:40,opaque_fk_connected_parts:3,errors};
    await fs.writeFile(path.join(output,'browser-checks.json'),JSON.stringify(result,null,2)+'\n');
    console.log(JSON.stringify({status:'PASS',diagnostic_branches:checks.checks.length,output}));
  }else{
  const source=await evaluate(`(()=>{const S=S50_VIEWER_DATA.sim;return {sha:S50_VIEWER_DATA.sim_report_sha256,stages:[...S.stages,...S.samples.flatMap((s,i)=>s.phase==='task_start'?[{phase:s.phase,target:s.target,last_sample:i}]:[])],samples:S.samples.length,end:S.samples.at(-1).time_s,status:S.status,completed:S.completed_cartons,remaining:S.remaining_cartons,failure:S.failure?JSON.stringify(S.failure,null,2):''}})()`);
  const renderer=await evaluate(`(()=>{const g=document.querySelector('canvas').getContext('webgl2')||document.querySelector('canvas').getContext('webgl'),e=g.getExtension('WEBGL_debug_renderer_info');return e?g.getParameter(e.UNMASKED_RENDERER_WEBGL):g.getParameter(g.RENDERER)})()`);
  const checks=[];
  for(const stage of source.stages){
    const result=await evaluate(`(()=>{
      const D=S50_VIEWER_DATA,S=D.sim,s=S.samples[${stage.last_sample}],K=S50Kinematics;
      s50Viewer.pause();s50Viewer.seek(s.time_s);
      const F=K.forward(D.model_definition,s.q_rad,S.world_from_model_base,S.settings.joint_coordinate_convention),tcp=K.multiply(F.s50_flange,S.flange_from_virtual_task_tcp);
      const intrinsic=D.environment.tool_intrinsic,ids=intrinsic.rigid_body_ids||intrinsic.rigid_local_boxes_virtual_tcp_m.map((_,i)=>'tool_rigid_'+i);
      const expected=s50Viewer.connectors.map(part=>{const r=intrinsic.rigid_local_boxes_virtual_tcp_m[ids.indexOf(part.id)],mesh=(D.tool_mesh_parts||[]).some(m=>m.name===part.id),local=mesh?[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]:[[1,0,0,r[0]],[0,1,0,r[1]],[0,0,1,r[2]],[0,0,0,1]],world=K.multiply(tcp,local);return {matrix:[0,1,2,3].flatMap(c=>[0,1,2,3].map(r=>world[r][c])),size:r.slice(3),mesh};});
      const target=s.target||S.target,retired=new Set(S.stages.filter(r=>r.phase==='outfeed'&&S.samples[r.last_sample].time_s<=s.time_s).map(r=>r.target||S.target));
      const expectedCartons=D.environment.height_samples.find(r=>Math.abs(r.height_m-.6)<1e-9).entities.filter(r=>r.category==='carton').map(r=>({id:r.id,visible:!retired.has(r.id),matrix:(r.id===target?s.world_from_carton:r.world_from_entity)[0].map((_,c)=>(r.id===target?s.world_from_carton:r.world_from_entity).map(row=>row[c])).flat()}));
      return {state:s50Viewer.state,connectors:s50Viewer.connectors,expected,source:s,mount:s50Viewer.directMount,bodyIds:s50Viewer.toolBodyIds,standoff:intrinsic.mount_standoff_m,expectedMountCount:intrinsic.added_mount_body_ids?.length||3,expectedRigidCount:intrinsic.rigid_body_ids?.length||130,cartons:s50Viewer.cartons,expectedCartons,error:document.getElementById('failure').textContent};
    })()`);
    assert.equal(result.error,source.failure);assert.equal(result.connectors.length,result.expectedMountCount);
    assert.deepEqual(result.state.q_rad,result.source.q_rad);
    assert.equal(result.state.attached,result.source.attached);
    assert.equal(result.state.belt_height_m,result.source.belt_height_m);
    assert.equal(result.state.target,result.source.target||stage.target);
    assert.equal(result.cartons.length,40);
    result.cartons.forEach((part,i)=>{
      const expected=result.expectedCartons[i];assert.equal(part.id,expected.id);assert.equal(part.visible,expected.visible);
      assert.ok(part.world_matrix.every((v,j)=>Math.abs(v-expected.matrix[j])<1e-9),'Every carton retains its own pose');
    });
    const column=result.source.world_from_carton[0].map((_,c)=>result.source.world_from_carton.map(row=>row[c])).flat();
    assert.ok(result.state.carton_matrix.every((v,i)=>Math.abs(v-column[i])<1e-9));
    assert.ok(Math.abs(result.state.posture.forearm_elevation_deg-result.source.posture.forearm_elevation_deg)<1e-7);
    if(result.source.posture.upper_arm_forward_tilt_deg!==undefined)
      assert.ok(Math.abs(result.state.posture.upper_arm_forward_tilt_deg-result.source.posture.upper_arm_forward_tilt_deg)<1e-7);
    result.connectors.forEach((c,i)=>{
      assert.ok(c.visible&&c.opaque,'Connector must retain opaque geometry');
      assert.ok(c.world_matrix.every((v,j)=>Math.abs(v-result.expected[i].matrix[j])<1e-9),'Connector must follow actual FK');
      if(!result.expected[i].mesh)assert.deepEqual([c.dimensions.width,c.dimensions.height,c.dimensions.depth],result.expected[i].size);
    });
    if(result.mount){
      assert.ok(Math.abs(result.mount.gap_m-result.standoff)<1e-9,'Physical backplate spacing follows flange FK');
      for(const id of ['tool_rigid_11','tool_rigid_12','tool_rigid_13'])assert.ok(!result.bodyIds.includes(id),'Old long connector absent');
      assert.equal(result.mount.rigid_count,result.expectedRigidCount);assert.equal(result.mount.cup_count,72);
    }
    checks.push({target:result.state.target,phase:stage.phase,connectors:result.connectors.length,attached:result.state.attached,visible_cartons:result.state.visible_cartons,passed:true});
  }
  await evaluate("s50Viewer.seek(0);document.getElementById('play').click()");
  let playingTime=0;
  for(let i=0;i<20&&playingTime===0;i++){await delay(250);playingTime=await evaluate('s50Viewer.state.time_s');}
  assert.ok(playingTime>0,'Play advances time');
  await evaluate("document.getElementById('pause').click()");
  const pausedTime=await evaluate('s50Viewer.state.time_s');await delay(250);
  assert.equal(await evaluate('s50Viewer.state.time_s'),pausedTime,'Pause keeps time fixed');
  await evaluate(`document.getElementById('seek').value=${source.end/2};document.getElementById('seek').dispatchEvent(new Event('input'))`);
  assert.ok(Math.abs(await evaluate('s50Viewer.state.time_s')-source.end/2)<.011,'Timeline seeks recorded task');
  await evaluate("document.getElementById('target').selectedIndex=document.getElementById('target').options.length-1;document.getElementById('target').dispatchEvent(new Event('change'))");
  assert.equal(await evaluate('s50Viewer.state.target'),await evaluate("document.getElementById('target').value"));
  await evaluate('s50Viewer.seek(0)');assert.equal(await evaluate('s50Viewer.state.visible_cartons'),40,'Scrubbing back restores the full initial stack');
  const screenshot=async(name,phase,view,target=null)=>{
    await evaluate(`s50Viewer.pause();s50Viewer.seek(S50_VIEWER_DATA.sim.samples[S50_VIEWER_DATA.sim.stages.find(s=>s.phase===${JSON.stringify(phase)}&&(!${JSON.stringify(target)}||s.target===${JSON.stringify(target)})).last_sample].time_s);s50Viewer.cameraView(${JSON.stringify(view)})`);
    await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
    const r=await call('Page.captureScreenshot',{format:'png'});
    await fs.writeFile(path.join(output,name),Buffer.from(r.data,'base64'));
  };
  await screenshot('home.png','home','all');
  await screenshot('withdraw.png','withdraw','close');
  await screenshot('release.png','release','close');
  await screenshot('connector.png','release','tcp');
  if(source.stages.some(s=>s.target==='carton_l07_c00'&&s.phase==='release')){
    await screenshot('c00-grasp.png','grasp','side','carton_l07_c00');
    await screenshot('c00-withdraw.png','withdraw','close','carton_l07_c00');
    await screenshot('c00-connector.png','withdraw','tcp','carton_l07_c00');
    await screenshot('c00-release.png','release','close','carton_l07_c00');
  }
  await evaluate("s50Viewer.seek(S50_VIEWER_DATA.sim.samples[S50_VIEWER_DATA.sim.stages.filter(s=>s.phase==='release').at(-1).last_sample].time_s);s50Viewer.cameraView('close')");
  await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
  let lastShot=await call('Page.captureScreenshot',{format:'png'});await fs.writeFile(path.join(output,'last-release.png'),Buffer.from(lastShot.data,'base64'));
  await evaluate("s50Viewer.seek(S50_VIEWER_DATA.sim.samples.at(-1).time_s);s50Viewer.cameraView('all')");
  await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
  lastShot=await call('Page.captureScreenshot',{format:'png'});await fs.writeFile(path.join(output,'last-state.png'),Buffer.from(lastShot.data,'base64'));
  if(await evaluate('Boolean(S50_VIEWER_DATA.sim_closest_belt_pose)')){
    await evaluate("document.getElementById('nearest').click()");
    await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
    const r=await call('Page.captureScreenshot',{format:'png'});
    await fs.writeFile(path.join(output,'closest-belt.png'),Buffer.from(r.data,'base64'));
  }
  const rejectedCount=await evaluate('S50_VIEWER_DATA.sim_failure_poses?.poses.length||0');
  for(let i=0;i<rejectedCount;i++){
    const rejected=await evaluate(`(()=>{s50Viewer.showRejected(${i});return {state:s50Viewer.state,source:S50_VIEWER_DATA.sim_failure_poses.poses[${i}],cartons:s50Viewer.cartons}})()`);
    assert.equal(rejected.state.diagnostic,true);assert.equal(rejected.state.phase,'REJECTED_CANDIDATE');
    assert.deepEqual(rejected.state.q_rad,rejected.source.q_rad);
    assert.ok(rejected.state.physical_contact.flat().every((v,j)=>Math.abs(v-rejected.source.world_from_physical_contact.flat()[j])<1e-9));
    assert.equal(rejected.cartons.filter(c=>c.visible).length,source.remaining,'Rejected candidate cannot remove a carton');
    await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
    const shot=await call('Page.captureScreenshot',{format:'png'});await fs.writeFile(path.join(output,`rejected-${i+1}.png`),Buffer.from(shot.data,'base64'));
  }
  await evaluate("document.getElementById('back-to-record').click()");assert.equal(await evaluate('s50Viewer.state.diagnostic'),false);
  assert.equal(errors.length,0);
  const report={status:'PASS',html_sha256:createHash('sha256').update(body).digest('hex'),source_report_sha256:source.sha,
    renderer,source_status:source.status,completed_cartons:source.completed,remaining_cartons:source.remaining,recorded_samples:source.samples,stage_checks:checks,opaque_fk_connected_parts:checks[0]?.connectors||0,play_pause_timeline:'PASS',carton_identity_and_reverse_scrub:'PASS',rejected_candidates_separately_checked:rejectedCount,errors};
  await fs.writeFile(path.join(output,'browser-checks.json'),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({status:report.status,renderer,stages:checks.length,output}));
  }
}catch(error){
  await fs.writeFile(path.join(output,`browser-failed-${Date.now()}.json`),JSON.stringify({status:'FAIL',
    html_sha256:createHash('sha256').update(body).digest('hex'),error:String(error.stack||error),errors},null,2)+'\n');
  throw error;
}finally{
  if(ws?.readyState===WebSocket.OPEN){try{await Promise.race([call('Browser.close'),delay(2000)]);}catch{}ws.close();}
  child.kill();server.closeAllConnections();await new Promise(r=>server.close(r));
  // Delete only this generated profile beneath the already resolved OS temp root.
  const resolved=path.resolve(profile);
  if(path.dirname(resolved)===tempRoot&&path.basename(resolved).startsWith('s50-viewer-')){
    await fs.rm(resolved,{recursive:true,force:true,maxRetries:3,retryDelay:200}).catch(()=>{});
  }
}
