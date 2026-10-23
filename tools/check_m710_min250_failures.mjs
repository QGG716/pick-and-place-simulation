// Headless checks of same-source layout and recorded approach viewers. Uses a fresh Chrome
// profile, serves one local artifact, never connects to a user's browser session.
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import net from 'node:net';
import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';

const [html,output,chrome]=process.argv.slice(2);
if(!html||!output||!chrome)throw Error('Usage: node check_m710_min250_failures.mjs HTML OUTPUT CHROME');
await fs.mkdir(output,{recursive:true});
const body=await fs.readFile(html),server=http.createServer((req,res)=>{
  res.setHeader('Content-Type','text/html; charset=utf-8');res.end(body);
});
await new Promise(r=>server.listen(0,'127.0.0.1',r));
const port=server.address().port;
const probe=net.createServer();await new Promise(r=>probe.listen(0,'127.0.0.1',r));
const debugPort=probe.address().port;await new Promise(r=>probe.close(r));
const profileRoot=path.resolve(output);
await fs.mkdir(profileRoot,{recursive:true});
const profile=await fs.mkdtemp(path.join(profileRoot,'chrome-profile-'));
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
  await call('Emulation.setDeviceMetricsOverride',{width:1440,height:900,deviceScaleFactor:1,mobile:false});
  await call('Page.navigate',{url:`http://127.0.0.1:${port}/`});
  let state;
  for(let i=0;i<100;i++){state=await evaluate('(window.previewState || window.REPLAY_READY) || null');if(state)break;await delay(100);}
  assert.ok(state,'Viewer did not initialize');
  const rendererInfo=await evaluate(`(()=>{const g=renderer.getContext(),e=g.getExtension('WEBGL_debug_renderer_info');return e?g.getParameter(e.UNMASKED_RENDERER_WEBGL):g.getParameter(g.RENDERER)})()`);
  console.log(JSON.stringify({renderer:rendererInfo}));


  const screenshot=async name=>{
    await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
    const r=await call('Page.captureScreenshot',{format:'png'});
    await fs.writeFile(path.join(output,name),Buffer.from(r.data,'base64'));
  };
  const checks=[];
  const cases=await evaluate('DATA.inspection_cases');
  assert.equal(cases.length,12);
  for(let i=0;i<cases.length;i++){
    const c=cases[i];
    const r=await evaluate(`min250Inspection.show(${i});({q:currentQ,flange:fk(currentQ).flange.toArray(),points:active.final_timed_path.length,error:document.getElementById('error').textContent,worldMatches:obstacles.every(o=>JSON.stringify(o.matrix.toArray())===JSON.stringify(matrix(active.request.world.find(b=>b.id===o.userData.name).pose).toArray())),cartons:obstacles.filter(o=>o.userData.name.startsWith('carton_')).length,red:obstacles.filter(o=>o.material.color.getHex()===0xff304d).length,attached:!!payload})`);
    assert.equal(r.error,'');assert.equal(r.points,0);assert.deepEqual(r.q,c.display_q);
    assert.equal(r.worldMatches,true);assert.equal(r.cartons,c.world_carton_count);assert.equal(r.red,0);assert.equal(r.attached,false);
    assert.ok(r.flange.every((x,k)=>Math.abs(x-c.production_flange_column_major[k])<1e-7));
    checks.push({target:c.target_id,world_cartons:r.cartons,production_fk_matches:true,recorded_q_preserved:true,robot_motion_points:0});
    if(i===0)await screenshot('geometry-scene.png');
    if(c.target_id==='carton_l00_c02')await screenshot('support-dependency.png');
  }
  await evaluate("min250Inspection.show(0,'pins')");
  assert.equal(await evaluate('robot.visible'),false);
  assert.equal(await evaluate('minGhosts.children[0].position.z'),.25);
  await screenshot('pin-height-schematic.png');
  await evaluate("document.getElementById('minGhost').click()");
  assert.equal(await evaluate('minGhosts.visible'),false);
  await evaluate("document.getElementById('minGhost').click()");
  await evaluate("min250Inspection.show(1,'face')");
  assert.equal(await evaluate('min250Inspection.case.target_id'),'carton_l02_c00');
  assert.deepEqual(await evaluate('evidenceBlockers()'),['carton_l02_c01','carton_l02_c04']);
  assert.equal(await evaluate('obstacles.filter(o=>o.material.color.getHex()===0xd586ee).length'),2);
  assert.equal(await evaluate('obstacles.filter(o=>o.userData.name.startsWith("carton_")).length'),13);
  await screenshot('side-face-screening.png');
  assert.equal(errors.length,0);
  await fs.writeFile(path.join(output,'browser-checks.json'),JSON.stringify({status:'PASS',checks,errors,renderer:rendererInfo,schematic_only:true,solver_calls_added:0},null,2));
  console.log(JSON.stringify({status:'PASS',output,checks:checks.length,errors:0}));
}finally{
  if(ws?.readyState===WebSocket.OPEN){try{await call('Browser.close');}catch{}ws.close();}
  child.kill();await new Promise(r=>server.close(r));
}
