// Headless checks of same-source layout and recorded approach viewers. Uses a fresh Chrome
// profile, serves one local artifact, never connects to a user's browser session.
import fs from 'node:fs/promises';
import path from 'node:path';
import http from 'node:http';
import net from 'node:net';
import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';

const [html,output,chrome]=process.argv.slice(2);
if(!html||!output||!chrome)throw Error('Usage: node check_m710_l01_failures.mjs HTML OUTPUT CHROME');
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
  const cases=await evaluate('DATA.inspection_cases');assert.equal(cases.length,5);
  const fkAll=await evaluate(`DATA.inspection_cases.every(c=>c.snapshots.every(s=>fk(s.q_rad).flange.toArray().every((v,k)=>Math.abs(v-s.production_flange[k])<1e-7)))`);
  assert.ok(fkAll,'Every saved witness must match production FK');
  const checks=[];
  async function inspect(i,mode=null){
    await evaluate(`l01Inspection.show(${i},${JSON.stringify(mode)})`);
    const r=await evaluate(`(()=>{const c=l01Inspection.case,s=l01Inspection.snapshot,pair=s.witness?.pair||[];return {
      q:currentQ,expected:s.q_rad,flange:fk(currentQ).flange.toArray(),expectedFlange:s.production_flange,
      modelPair:pair.filter(id=>modelObjects.some(([o,l])=>l.link===id&&o.material.color.getHex()===0xff4057)),
      worldPair:pair.filter(id=>obstacles.some(o=>o.userData.name===id&&o.material.color.getHex()===0xff4057)),
      pair,points:active.final_timed_path.length,error:document.getElementById('error').textContent,
      worldMatches:obstacles.every(o=>JSON.stringify(o.matrix.toArray())===JSON.stringify(matrix(active.request.world.find(b=>b.id===o.userData.name).pose).toArray())),
      cartonCount:obstacles.filter(o=>o.userData.name.startsWith('carton_')).length+(payload?1:0),
      attached:!!payload,payloadMatches:!payload||payload.matrix.toArray().every((v,k)=>Math.abs(v-fk(currentQ).flange.clone().multiply(matrix(active.request.attachment.pose)).toArray()[k])<1e-10),
      gapText:document.getElementById('l01Gap').textContent,purple:obstacles.filter(o=>o.material.color.getHex()===0xcc8bf1).map(o=>o.userData.name),
      source:DATA.raw.source_commit,physical:DATA.physical_execution};})()`);
    assert.deepEqual(r.q,r.expected);assert.equal(r.points,0);assert.equal(r.error,'');
    assert.ok(r.flange.every((x,k)=>Math.abs(x-r.expectedFlange[k])<1e-7));
    assert.ok(r.worldMatches);assert.ok(r.payloadMatches);assert.equal(r.cartonCount,10);
    assert.equal(r.attached,cases[i].kind==='clearance');
    assert.deepEqual([...r.modelPair,...r.worldPair].sort(),[...r.pair].sort());
    assert.equal(r.source,'f17d636ed547a86389150f268445d44bae6f58c4');assert.equal(r.physical,'NOT_RUN');
    assert.ok(!r.gapText.includes('-1000')&&!r.gapText.includes('−1000'));
    checks.push({target:cases[i].target_id,mode,pair:r.pair,native_q_preserved:true,production_fk_matches:true,
      world_matches:true,attachment_matches:true,carton_count:r.cartonCount,gap_label:r.gapText});
    return r;
  }
  for(let i=0;i<cases.length;i++){
    const c=cases[i];await inspect(i);await screenshot(c.target_id.replace('carton_','')+'.png');
    if(c.kind==='clearance'){
      await inspect(i,'contact');
      for(const pair of Object.keys(c.pair_counts)){
        await inspect(i,pair);
        if(pair.includes('J2_link'))await screenshot('l01_c04-J2-wall.png');
        await evaluate("document.getElementById('l01Sample').value=document.getElementById('l01Sample').max;document.getElementById('l01Sample').dispatchEvent(new Event('input'))");
        assert.ok(await evaluate("l01Inspection.snapshot.witness.reason==='CLEARANCE_INSUFFICIENT' && document.getElementById('l01Gap').textContent.includes('mm')"));
      }
    }else if(c.kind==='face'){
      const r=await inspect(i,'face');assert.deepEqual(r.purple.sort(),c.blockers.slice().sort());
      await inspect(i,'height');await screenshot('l01_c00-height.png');
    }else assert.ok((await inspect(i,'height')).gapText.includes('−50'));
  }
  await evaluate("document.getElementById('l01Ghost').click()");assert.equal(await evaluate('l01Annotations.visible'),false);
  assert.equal(errors.length,0);
  const result={status:'PASS',cases:cases.length,snapshots:cases.reduce((n,c)=>n+c.snapshots.length,0),
    all_snapshots_production_fk_match:fkAll,checks,errors,renderer:rendererInfo};
  await fs.writeFile(path.join(output,'browser-checks.json'),JSON.stringify(result,null,2));
  console.log(JSON.stringify({status:'PASS',cases:cases.length,display_checks:checks.length,errors:0}));
}finally{
  if(ws?.readyState===WebSocket.OPEN){try{await call('Browser.close');}catch{}ws.close();}
  child.kill();await new Promise(r=>server.close(r));
}
