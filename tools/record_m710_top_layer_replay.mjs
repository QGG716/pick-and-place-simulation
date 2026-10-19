// Headless UI regression and presentation-only clip frames. Uses a fresh Chrome
// profile, serves one local artifact, never connects to a user's browser session.
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import net from 'node:net';
import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';

const [html,output,chrome]=process.argv.slice(2);
if(!html||!output||!chrome)throw Error('Usage: node record_m710_top_layer_replay.mjs HTML OUTPUT CHROME');
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
  for(let i=0;i<100;i++){state=await evaluate('window.previewState || null');if(state)break;await delay(100);}
  assert.ok(state,'Viewer did not initialize');
  const rendererInfo=await evaluate(`(()=>{const g=renderer.getContext(),e=g.getExtension('WEBGL_debug_renderer_info');return e?g.getParameter(e.UNMASKED_RENDERER_WEBGL):g.getParameter(g.RENDERER)})()`);
  console.log(JSON.stringify({renderer:rendererInfo}));

  assert.equal(state.carton_count,40);assert.equal(state.completed_count,0);
  const screenshot=async name=>{
    await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
    const r=await call('Page.captureScreenshot',{format:'png'});
    await fs.writeFile(path.join(output,name),Buffer.from(r.data,'base64'));
  };
  await screenshot('initial.png');
  const timeline=await evaluate('({duration:batchData.duration,segments:batchData.segments.map(s=>({start:s.start,duration:s.duration,kind:s.kind,completed:s.completed,q:s.q||s.points[0].q})),fk:batchData.fk_witnesses,count:batchData.result.completed_count})');
  const checks=[];
  for(const [i,s] of timeline.segments.entries()){
    const actual=await evaluate(`setBatchTime(${s.start});window.previewState`);
    assert.equal(actual.completed_count,s.completed);
    assert.ok(actual.flange_matrix_column_major.every((x,k)=>Math.abs(x-timeline.fk[i][k])<1e-7),'Rendered FK mismatch');
    assert.ok(actual.q_rad.every((x,k)=>Math.abs(x-s.q[k])<1e-9));
    assert.equal(actual.carton_count,40-s.completed);
    checks.push({segment:i,kind:s.kind,time:s.start,completed:actual.completed_count,cartons:actual.carton_count});
  }
  await evaluate(`setBatchTime(${timeline.duration})`);await screenshot('final.png');
  await call('Emulation.setDeviceMetricsOverride',{width:640,height:360,deviceScaleFactor:1,mobile:false});
  await evaluate(`(()=>{
    document.querySelector('header').style.display='none';document.querySelector('aside').style.display='none';
    document.querySelector('main').style.cssText='display:block;height:100vh';
    document.querySelector('#view').style.cssText='width:100vw;height:100vh';
    document.querySelector('#hud').style.cssText='position:absolute;left:10px;top:auto;right:auto;bottom:10px;padding:7px;font-size:10px;line-height:1.4';
    document.querySelector('#source').style.fontSize='10px';resizeBatch();
  })()`);
  await fs.mkdir(path.join(output,'frames'),{recursive:true});
  const frames=process.argv.includes('--check-only')?0:Math.ceil(timeline.duration*5)+1;
  for(let frame=0;frame<frames;frame++){
    await evaluate(`setBatchTime(${Math.min(timeline.duration,frame/5)})`);
    await screenshot(`frames/frame-${String(frame).padStart(5,'0')}.png`);
    if(frame%50===0)console.log(JSON.stringify({frame,frames}));
  }
  assert.equal(errors.length,0);
  const final=await evaluate('window.previewState');assert.equal(final.completed_count,timeline.count);
  assert.equal(final.carton_count,40-timeline.count);
  await fs.writeFile(path.join(output,'browser-checks.json'),JSON.stringify({status:'PASS',errors,
    initial:state,final,checks,frames,width:640,height:360,fps:5,playback_speed:1,renderer:rendererInfo,
    classification:'NATIVE_PLANNING_COUPLED_GEOMETRY_NOMINAL_DROP_NOT_PHYSICAL'},null,2));
  console.log(JSON.stringify({status:'PASS',output,frames,errors:0}));
}finally{
  if(ws?.readyState===WebSocket.OPEN){try{await call('Browser.close');}catch{}ws.close();}
  child.kill();await new Promise(r=>server.close(r));
}
