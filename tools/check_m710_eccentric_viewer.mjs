// Headless checks of saved J2 failure comparisons and independent FK witnesses. Uses a fresh Chrome
// profile, serves one local artifact, never connects to a user's browser session.
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import net from 'node:net';
import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';

const [html,output,chrome]=process.argv.slice(2);
if(!html||!output||!chrome)throw Error('Usage: node check_m710_eccentric_viewer.mjs HTML OUTPUT CHROME');
await fs.mkdir(output,{recursive:true});
const body=await fs.readFile(html),server=http.createServer((req,res)=>{
  res.setHeader('Content-Type','text/html; charset=utf-8');res.end(body);
});
await new Promise(r=>server.listen(0,'127.0.0.1',r));
const port=server.address().port;
const probe=net.createServer();await new Promise(r=>probe.listen(0,'127.0.0.1',r));
const debugPort=probe.address().port;await new Promise(r=>probe.close(r));
const profile=await fs.mkdtemp(path.join(path.resolve(output),'chrome-profile-'));
const graphics=process.argv.includes('--hardware-rendering')?['--enable-gpu']:
  ['--enable-unsafe-swiftshader','--use-angle=swiftshader'];
const child=spawn(chrome,['--headless=new','--no-first-run','--no-default-browser-check',
  '--disable-background-networking','--disable-sync','--disable-extensions',
  ...graphics,'--hide-scrollbars',
  '--remote-debugging-address=127.0.0.1',`--remote-debugging-port=${debugPort}`,
  `--user-data-dir=${profile}`,'about:blank'],{windowsHide:true,stdio:'ignore'});
const delay=ms=>new Promise(r=>setTimeout(r,ms));
let ws,serial=0;const pending=new Map(),errors=[];
const call=(method,params={})=>new Promise((resolve,reject)=>{
  const id=++serial,timer=setTimeout(()=>{pending.delete(id);reject(Error('CDP timeout: '+method));},20000);
  pending.set(id,{resolve:v=>{clearTimeout(timer);resolve(v);},reject});
  ws.send(JSON.stringify({id,method,params}));
});
try{
  let ready=false;
  for(let i=0;i<100;i++){
    try{await fetch(`http://127.0.0.1:${debugPort}/json/version`);ready=true;break;}catch{}
    await delay(100);
  }
  if(!ready)throw Error('Headless Chrome unavailable');
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
  await call('Emulation.setDeviceMetricsOverride',{width:1280,height:720,deviceScaleFactor:1,mobile:false});
  await call('Page.navigate',{url:`http://127.0.0.1:${port}/`});
  let state;
  for(let i=0;i<100;i++){state=await evaluate('window.ECCENTRIC_READY || null');if(state)break;await delay(100);}
  assert.ok(state,'Viewer did not initialize');
  const rendererInfo=await evaluate(`(()=>{const g=renderer.getContext(),e=g.getExtension('WEBGL_debug_renderer_info');return e?g.getParameter(e.UNMASKED_RENDERER_WEBGL):g.getParameter(g.RENDERER)})()`);
  console.log(JSON.stringify({renderer:rendererInfo}));


  const screenshot=async name=>{
    await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
    const r=await call('Page.captureScreenshot',{format:'png'});
    await fs.writeFile(path.join(output,name),Buffer.from(r.data,'base64'));
  };

  const actual=await evaluate('eccentricReplay.inspect(0)');
  const expected=await evaluate('REPLAY_DATA.raw.attempts[0].fk_upper_arm');
  assert.ok(Math.abs(actual.alpha_rad-expected.elevation_rad)<1e-10);
  assert.ok(Math.abs(actual.q2_rad-expected.q2_rad)<1e-10);
  assert.equal(actual.physical_cups,72);
  assert.equal(actual.commanded,48);
  assert.equal(actual.actual,null);
  assert.equal(actual.delta_m,0);
  await evaluate('eccentricReplay.closeup()');
  await screenshot('actual-native-ik.png');
  const count=await evaluate('document.getElementById("offsetPreview").options.length-1');
  let outside=0;
  for(let i=0;i<count;i++){
    const state=await evaluate(`(()=>{document.getElementById('offsetPreview').value=${i};document.getElementById('offsetPreview').onchange();return {q:eccentricReplay.state.q,text:document.getElementById('previewInfo').textContent,preview:DATA.eccentric.targets[active.request.task_id].pending_target_geometry[${i}]};})()`);
    assert.deepEqual(state.q,actual.q,'Static preview must never move robot');
    assert.ok(state.text.includes('\u672a\u6d3e\u53d1'));
    assert.ok(state.preview.target.world_y_offset_m>0);
    assert.ok(state.preview.eligible_count>0);
    assert.deepEqual(state.preview.target.face_center_m,actual.face_center);
    if(state.preview.target.world_y_offset_m>.2)outside++;
    if(i===0){await evaluate('eccentricReplay.targetView()');await screenshot('pending-eccentric-geometry.png');}
  }
  assert.ok(outside>0);
  assert.equal(errors.length,0,JSON.stringify(errors));
  const evidence={passed:true,actual_native_ik_cases:1,static_pending_targets:count,
    outside_reference_valid_ring_cases:outside,upper_arm_fk_error_rad:actual.alpha_rad-expected.elevation_rad,
    all_72_cups_preserved:true,actual_contact_not_claimed:true,static_preview_q_unchanged:true,renderer:rendererInfo};
  await fs.writeFile(path.join(output,'viewer-check.json'),JSON.stringify(evidence,null,2));
  console.log(JSON.stringify(evidence));
}finally{
  if(ws)ws.close();child.kill();await new Promise(r=>server.close(r));
}
