// Headless UI regression and presentation-only clip frames. Uses a fresh Chrome
// profile, serves one local artifact, never connects to a user's browser session.
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import net from 'node:net';
import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';

const [html,output,chrome]=process.argv.slice(2);
if(!html||!output||!chrome)throw Error('Usage: node record_m710_blocker_inspection.mjs HTML OUTPUT CHROME');
await fs.mkdir(output,{recursive:true});
const body=await fs.readFile(html),server=http.createServer((req,res)=>{
  res.setHeader('Content-Type','text/html; charset=utf-8');res.end(body);
});
await new Promise(r=>server.listen(0,'127.0.0.1',r));
const port=server.address().port;
const probe=net.createServer();await new Promise(r=>probe.listen(0,'127.0.0.1',r));
const debugPort=probe.address().port;await new Promise(r=>probe.close(r));
const profile=await fs.mkdtemp(path.join(path.resolve(output),'chrome-profile-'));
const child=spawn(chrome,['--headless=new','--no-first-run','--no-default-browser-check',
  '--disable-background-networking','--disable-sync','--disable-extensions',
  '--enable-unsafe-swiftshader','--use-angle=swiftshader','--hide-scrollbars',
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
  for(let i=0;i<100;i++){state=await evaluate('window.previewState || null');if(state)break;await delay(100);}
  assert.ok(state,'Viewer did not initialize');
  assert.equal(state.carton_count,40);assert.equal(state.completed_count,0);assert.equal(state.accepted_motion_edges,0);
  const screenshot=async name=>{
    await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
    const r=await call('Page.captureScreenshot',{format:'png'});
    await fs.writeFile(path.join(output,name),Buffer.from(r.data,'base64'));
  };
  await screenshot('whole-scene.png');
  const indices=await evaluate('R.initial.failures.flatMap((f,i)=>f.exact_axis_aligned_intersection?[i]:[])');
  assert.equal(indices.length,2);
  for(const [i,index] of indices.entries()){
    await evaluate(`window.showBlocker(${index})`);
    await screenshot(`rear-stand-${i}.png`);
  }
  await call('Emulation.setDeviceMetricsOverride',{width:640,height:360,deviceScaleFactor:1,mobile:false});
  await evaluate(`(()=>{
    document.querySelector('header').style.display='none';document.querySelector('aside').style.display='none';
    document.querySelector('main').style.cssText='display:block;height:100vh';
    document.querySelector('#view').style.cssText='width:100vw;height:100vh';
    document.querySelector('#bottom').style.display='none';
    document.querySelector('#hud').style.cssText='position:absolute;left:10px;top:auto;right:auto;bottom:10px;height:auto;padding:8px;font-size:10px;line-height:1.4';
    const label=document.createElement('div');label.textContent='初态阻塞诊断 · 不是抓放视频';
    label.style.cssText='position:absolute;top:10px;left:12px;color:#ffd09a;font:13px system-ui;z-index:5';
    document.querySelector('#view').append(label);resize();window.showWholeScene();
  })()`);
  await fs.mkdir(path.join(output,'frames'),{recursive:true});
  for(let frame=0;frame<40;frame++){
    if(frame===10)await evaluate(`window.showBlocker(${indices[0]})`);
    if(frame===20)await evaluate(`window.showBlocker(${indices[1]})`);
    if(frame===30)await evaluate('window.showWholeScene()');
    await screenshot(`frames/frame-${String(frame).padStart(3,'0')}.png`);
  }
  assert.equal(errors.length,0);
  const final=await evaluate('window.previewState');
  assert.deepEqual(final.q_rad,state.q_rad);
  assert.equal(final.initial_world_sha256,state.initial_world_sha256);
  await fs.writeFile(path.join(output,'browser-checks.json'),JSON.stringify({status:'PASS',errors,
    initial:state,final,frames:40,width:640,height:360,fps:5,presentation_duration_s:8,
    simulated_time_s:null,classification:'STATIC_INITIAL_REJECTION_INSPECTION_NOT_BATCH_VIDEO'},null,2));
  console.log(JSON.stringify({status:'PASS',output,frames:40,errors:0}));
}finally{
  if(ws?.readyState===WebSocket.OPEN){try{await call('Browser.close');}catch{}ws.close();}
  child.kill();await new Promise(r=>server.close(r));
}
