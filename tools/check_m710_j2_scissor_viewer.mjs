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
if(!html||!output||!chrome)throw Error('Usage: node check_m710_j2_scissor_viewer.mjs HTML OUTPUT CHROME');
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
  for(let i=0;i<100;i++){state=await evaluate('window.COMPARISON_READY || null');if(state)break;await delay(100);}
  assert.ok(state,'Viewer did not initialize');
  const rendererInfo=await evaluate(`(()=>{const g=renderer.getContext(),e=g.getExtension('WEBGL_debug_renderer_info');return e?g.getParameter(e.UNMASKED_RENDERER_WEBGL):g.getParameter(g.RENDERER)})()`);
  console.log(JSON.stringify({renderer:rendererInfo}));


  const screenshot=async name=>{
    await evaluate('new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))');
    const r=await call('Page.captureScreenshot',{format:'png'});
    await fs.writeFile(path.join(output,name),Buffer.from(r.data,'base64'));
  };
  const initial=await evaluate('({state:comparisonState,source:COMPARISON_DATA.cases[comparisonState.case_index]})');
  assert.equal(initial.state.selected_part,'lift_transverse_left_pin_1');
  assert.equal(initial.state.selected_bodies[0].pair.passed,true);
  assert.equal(initial.state.selected_bodies[1].pair.collision,true);
  assert.equal(initial.state.selected_bodies[2].pair.collision,true);
  await screenshot('pin-contact-comparison.png');
  const count=await evaluate('COMPARISON_DATA.cases.length');assert.equal(count,40);
  const ids=await evaluate("[...document.getElementById('part').options].map(o=>o.value)");
  const checks=[];
  const column=m=>Array.from({length:4},(_,c)=>Array.from({length:4},(_,r)=>m[r][c])).flat();
  const near=(a,b)=>{assert.equal(a.length,b.length);assert.ok(a.every((x,k)=>Math.abs(x-b[k])<1e-7),'Rendered transform differs from recorded geometry / independent FK');};
  for(let k=0;k<count;k++){
    const r=await evaluate(`setComparisonCase(${k});({state:comparisonState,source:COMPARISON_DATA.cases[${k}]})`);
    assert.deepEqual(r.state.q_rad,r.source.q_rad);
    near(r.state.J2_matrix,column(r.source.frame_witnesses.J2_link));
    near(r.state.flange_matrix,column(r.source.frame_witnesses.flange));
    for(const id of ids){
      const s=await evaluate(`setComparisonPart(${JSON.stringify(id)});({state:comparisonState,source:COMPARISON_DATA.cases[${k}],worlds:COMPARISON_DATA.worlds[COMPARISON_DATA.cases[${k}].world_key],error:document.getElementById('error').textContent})`);
      assert.equal(s.error,'');assert.equal(s.state.physical_execution,'NOT_RUN');
      assert.ok(s.state.section_segment_count>0);
      assert.ok(s.state.section_points.every(p=>Math.abs(p[2]-s.state.section_z)<1e-9),'Section vertices must lie on the selected actual Z plane');
      s.state.selected_bodies.forEach((b,i)=>{
        near(b.matrix,column(s.worlds[i].find(x=>x.id===id).pose));
        assert.deepEqual(b.pair,s.source.variants[i].pairs.find(x=>x.pair[1]===id));
      });
      if(id==='lift_transverse_left_pin_1')near(s.state.selected_bodies.map(b=>b.matrix[12]),[-.78,-.70,-.68]);
      checks.push({case:k,part:id,passed:s.state.selected_bodies.map(b=>b.pair.passed),collision:s.state.selected_bodies.map(b=>b.pair.collision)});
    }
  }
  await evaluate(`setComparisonCase(${initial.state.case_index});setComparisonPart('lift_transverse_left_arm_1_0')`);
  await screenshot('arm-a-comparison.png');
  await evaluate("setComparisonPart('lift_transverse_left_arm_1_1')");await screenshot('arm-b-comparison.png');
  await evaluate("setComparisonPart('lift_transverse_left_pin_1');comparisonFocus('assembly')");await screenshot('assembly-comparison.png');
  await evaluate("document.getElementById('pinFilter').click()");
  assert.equal(await evaluate("document.getElementById('case').options.length"),30);
  await evaluate("document.getElementById('pinFilter').click();document.getElementById('mode').value='2';document.getElementById('mode').dispatchEvent(new Event('change'))");
  assert.deepEqual(await evaluate('comparisonState.variants'),[2]);
  await evaluate("document.getElementById('mode').value='all';document.getElementById('mode').dispatchEvent(new Event('change'));document.getElementById('context').checked=false;document.getElementById('context').dispatchEvent(new Event('change'));comparisonFocus('local')");
  assert.deepEqual(await evaluate('comparisonState.variants'),[0,1,2]);
  assert.equal(errors.length,0);
  await fs.writeFile(path.join(output,'browser-checks.json'),JSON.stringify({status:'PASS',
    html_sha256:createHash('sha256').update(body).digest('hex'),cases:count,parts:ids,checks,errors,renderer:rendererInfo},null,2));
  console.log(JSON.stringify({status:'PASS',output,checks:checks.length,errors:0}));
}finally{
  if(ws?.readyState===WebSocket.OPEN){try{await call('Browser.close');}catch{}ws.close();}
  child.kill();await new Promise(r=>server.close(r));
}
