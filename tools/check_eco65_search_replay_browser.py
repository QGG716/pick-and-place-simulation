"""Run directed tests in installed Edge/Chrome against a self-contained file:// viewer."""
import argparse
import json
from pathlib import Path
import re
import subprocess
from time import perf_counter

CHECK=r'''
<script>
setTimeout(()=>{const result={checks:[],status:'RUNNING'};
const check=(name,value)=>{if(!value)throw Error(name);result.checks.push(name);};
try{
check('VIEWER_READY',window.REPLAY_READY===true);const runSelect=document.getElementById('run');if(runSelect.options.length>1){runSelect.value=runSelect.options[runSelect.options.length-1].value;runSelect.dispatchEvent(new Event('change'));check('CASCADING_RUN_REQUEST_SELECTION',active.run_id===runSelect.value&&active.request.stage==='transit');}
let maxError=0;
for(const fixture of DATA.fk_fixtures){const actual=window.replay.fk(fixture.q);
for(const [name,reference] of Object.entries(fixture.frames)){const col=reference[0].map((_,i)=>reference.map(row=>row[i])).flat();
maxError=Math.max(maxError,...col.map((v,i)=>Math.abs(v-actual[name][i])));}
const col=fixture.tcp[0].map((_,i)=>fixture.tcp.map(row=>row[i])).flat();
maxError=Math.max(maxError,...col.map((v,i)=>Math.abs(v-actual[model.tcp_link][i])));}
result.maximum_fk_matrix_error=maxError;check('BROWSER_FK_MATCHES_CURRENT_PYTHON_MODEL',maxError<1e-8);
check('TWELVE_CUPS',model.cup_ids.length===12);if(!active.native_path?.length&&planner?.raw_solution?.length>1){renderChart();check('RAW_SOLUTION_JOINT_CHART',document.getElementById('angles').getContext('2d').getImageData(0,0,312,130).data.some((v,i)=>i%4===3&&v>0));}
const mounted=model.joints.filter(j=>j.child.startsWith('eco65_mast_')||j.child.includes('camera'));
check('J1_ACCESSORY_PARENTAGE',mounted.length>0&&mounted.every(j=>{let parent=j.parent;while(parent!=='link_1'&&parent!=='world'){const joint=model.joints.find(k=>k.child===parent);if(!joint)return false;parent=joint.parent;}return parent==='link_1';}));
const toggle=document.querySelector('input[data-layer="collision"]');toggle.click();check('COLLISION_LAYER_TOGGLE',robotObjects.filter(r=>r.item.layer==='collision').every(r=>r.o.visible));toggle.click();
document.getElementById('top').click();check('TOP_VIEW',Math.abs(camera.position.x-controls.target.x)<1e-9);document.getElementById('side').click();document.getElementById('reset').click();
if(planner?.nodes.length){document.getElementById('event').value=planner.events.length;drawGraph();renderer.render(scene,camera);
let clicked=false;
for(const n of drawnNodes){const v=tcp(n.q).clone().project(camera);if(Math.abs(v.x)>.95||Math.abs(v.y)>.95)continue;const r=renderer.domElement.getBoundingClientRect();renderer.domElement.dispatchEvent(new MouseEvent('click',{clientX:r.left+(v.x+1)*r.width/2,clientY:r.top+(1-v.y)*r.height/2,bubbles:true}));
if(document.getElementById('selection').textContent.startsWith('真实 PlannerData 节点')){clicked=true;break;}}
check('REAL_NODE_CANVAS_CLICK',clicked);check('CLICK_POSE_IS_RECORDED_NODE',planner.nodes.some(n=>n.q.every((v,i)=>Math.abs(v-currentQ[i])<1e-12)));
if(failures.length){document.getElementById('failure').value='0';document.getElementById('failureInspect').click();check('FAILED_SAMPLE_FULL_POSE',currentQ.every((v,i)=>v===failures[0].q[i]));}
document.getElementById('event').value=0;document.getElementById('step').click();check('SEARCH_EVENT_STEP',document.getElementById('event').value==='1');
document.getElementById('searchPlay').click();check('SEARCH_PLAY',searchPlaying);document.getElementById('searchPlay').click();check('SEARCH_PAUSE',!searchPlaying);}
if(payload){const expected=fk(currentQ)[model.flange_link].clone().multiply(matrix(active.request.attachment.pose));check('ATTACHED_COLLISION_LAYER_PRESENT',payloadCollision&&payloadCollision.userData.layer==='collision');check('LOCAL_ATTACHMENT_TRANSFORM',payload.matrix.elements.every((v,i)=>Math.abs(v-expected.elements[i])<1e-12));check('NO_DUPLICATE_TARGET',!active.request.world.some(b=>b.id===active.request.attachment.id));}
if(DATA.plan){document.getElementById('mode').value='complete';document.getElementById('mode').dispatchEvent(new Event('change'));for(const fixture of DATA.plan_fixtures||[]){setMotion(fixture.time_s);check('PLAN_TIME_'+fixture.time_s,currentQ.every((v,i)=>Math.abs(v-fixture.q[i])<1e-9));const target=payload?.matrix||worldObjects.find(o=>o.userData.id===DATA.scene_target&&o.userData.layer==='visual')?.matrix;check('PLAN_PAYLOAD_'+fixture.time_s,!!target&&target.elements.every((v,i)=>Math.abs(v-matrix(fixture.target_pose).elements[i])<1e-8));check('PLAN_ATTACHMENT_'+fixture.time_s,!!payload===fixture.attached);}setMotion(DATA.plan.simulated_duration_s);check('COMPLETE_END_IS_HOME',currentQ.every((v,i)=>Math.abs(v-DATA.plan.home_q[i])<1e-9));setMotion(0);document.getElementById('motionPlay').click();check('FINAL_PLAY',motionPlaying);document.getElementById('motionPlay').click();check('FINAL_PAUSE',!motionPlaying);result.complete_plan='TESTED';}else result.complete_plan='NOT_RUN_NO_COMPLETE_PLAN';
if(failures.length)failureInspect(0);result.status='PASS';
}catch(error){result.status='FAIL';result.error=error.stack;}
const pre=document.createElement('pre');pre.id='browser-check-result';pre.textContent=JSON.stringify(result);document.body.append(pre);window.BROWSER_CHECK_RESULT=result;
},500);
</script>
'''


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--html',type=Path,required=True)
    p.add_argument('--browser',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);start=perf_counter()
    page=a.output/'browser-test.html';page.write_text(a.html.read_text(encoding='utf-8')+CHECK,encoding='utf-8')
    proc=subprocess.run([str(a.browser),'--headless','--no-first-run','--user-data-dir='+str((a.output/'profile').resolve()),
        '--window-size=1440,1000','--virtual-time-budget=4000','--dump-dom',
        '--screenshot='+str((a.output/'checked.png').resolve()),page.resolve().as_uri()],capture_output=True,timeout=90)
    dom=proc.stdout.decode('utf-8','replace');match=re.search(r'<pre id="browser-check-result">(.*?)</pre>',dom,re.S)
    if not match:
        (a.output/'browser-stderr.txt').write_bytes(proc.stderr)
        raise RuntimeError('BROWSER_TEST_RESULT_MISSING:'+str(proc.returncode))
    import html
    result=json.loads(html.unescape(match.group(1)));result['wall_s']=perf_counter()-start
    (a.output/'checks.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    if result['status']!='PASS':raise SystemExit(1)


if __name__=='__main__':main()
