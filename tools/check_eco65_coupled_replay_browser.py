"""Directed browser checks for the self-contained coupled ECO65 viewer."""
import argparse
import html
import json
from pathlib import Path
import re
import subprocess
from time import perf_counter

CHECK = r'''
<script>
setTimeout(()=>{const result={checks:[],status:'RUNNING'};
const check=(name,value)=>{if(!value)throw Error(name);result.checks.push(name);};
const close=(a,b,eps=1e-8)=>a.length===b.length&&a.every((v,i)=>Math.abs(v-b[i])<=eps);
try{
check('VIEWER_READY',window.REPLAY_READY===true);
check('TWELVE_PHYSICAL_CUPS',model.cup_ids.length===12);
let maximum=0;
for(const fixture of DATA.fk_fixtures){const frames=window.replay.fk(fixture.q);
for(const [name,reference]of Object.entries(fixture.frames)){const columns=reference[0].map((_,i)=>reference.map(row=>row[i])).flat();
maximum=Math.max(maximum,...columns.map((v,i)=>Math.abs(v-frames[name][i])));}}
result.maximum_fk_matrix_error=maximum;check('PYTHON_FK_MATCH',maximum<1e-8);
check('THREE_REAL_HEIGHT_PREVIEWS',DATA.previews.length===3);
for(let i=0;i<DATA.previews.length;i++){
preview(i);const expected=DATA.previews[i].state;
check('PREVIEW_HEIGHT_'+i,state.height_m===expected.height_m);
check('PREVIEW_Q_'+i,close(currentQ,expected.q));
let error=0;
for(const b of expected.world){const objects=worldObjects.filter(o=>o.userData.id===b.id);
check('PREVIEW_OBJECT_PRESENT_'+i+'_'+b.id,objects.length===2);
for(const object of objects)error=Math.max(error,...matrix(b.pose).elements.map((v,j)=>Math.abs(v-object.matrix.elements[j])));}
check('SAME_NATIVE_WORLD_POSES_'+i,error<1e-12);}
const collision=document.querySelector('input[data-layer="collision"]');collision.click();
check('COLLISION_LAYER_VISIBLE',robotObjects.filter(r=>r.item.layer==='collision').every(r=>r.object.visible));collision.click();
const accessory=model.joints.filter(j=>j.child.startsWith('eco65_mast_')||j.child.includes('camera'));
check('J1_ACCESSORY_PARENTAGE',accessory.length>0&&accessory.every(j=>{let p=j.parent;while(p!=='link_1'&&p!=='world'){const edge=model.joints.find(k=>k.child===p);if(!edge)return false;p=edge.parent;}return p==='link_1';}));
document.getElementById('top').click();check('TOP_VIEW',Math.abs(camera.position.x-controls.target.x)<1e-9);document.getElementById('reset').click();
if(DATA.segments.length){
document.getElementById('mode').value='cycle';modeChanged();
for(const segment of DATA.segments){
setTime(segment.start_s+segment.duration_s*.37);
if(segment.kind==='external_axis')check('EXTERNAL_AXIS_HOLDS_Q_'+segment.record_id,close(currentQ,segment.state_before.q,1e-12));
if(segment.kind==='robot_motion')check('ROBOT_HOLDS_HEIGHT_'+segment.record_id,state.height_m===segment.state_before.height_m);
if(state.attachment){check('NO_DUPLICATE_ATTACHMENT_'+segment.record_id,!state.world.some(b=>b.id===state.attachment.id));const actual=window.replay.payload.matrix.elements,expected=fk(currentQ)[model.flange_link].clone().multiply(matrix(state.attachment.pose)).elements;check('LOCAL_ATTACHMENT_'+segment.record_id,close(actual,expected,1e-12));}
}
setTime(DATA.duration_s);check('TERMINAL_Q',close(currentQ,DATA.terminal_state.q,1e-12));
check('TERMINAL_OWNERSHIP',state.world.length===DATA.terminal_state.world.length&&state.attachment?.id===DATA.terminal_state.attachment?.id);
result.complete_task=DATA.plan_summary.complete?'TESTED':'PARTIAL_ONLY';
}else result.complete_task='NOT_RUN_NO_PLAN';
if(DATA.attempts.length){
document.getElementById('mode').value='search';modeChanged();
const index=DATA.attempts.findIndex(a=>a.trace?.planners?.some(p=>p.nodes.length));
document.getElementById('request').value=Math.max(0,index);chooseRequest();
check('REQUEST_OWNS_WORLD',state.world===active.request.world);
check('REQUEST_OWNS_ATTACHMENT',state.attachment===active.request.attachment);
check('REQUEST_HEIGHT',state.height_m===active.request.mechanism_state?.height_m||state.height_m===DATA.mechanisms[active.mechanism_ref]?.height_m);
if(planner?.nodes.length){
document.getElementById('event').value=planner.events.length;drawGraph();renderer.render(scene,camera);
let clicked=false;
for(const n of drawnNodes){const v=tcp(n.q).clone().project(camera);if(Math.abs(v.x)>.95||Math.abs(v.y)>.95)continue;const r=renderer.domElement.getBoundingClientRect();
renderer.domElement.dispatchEvent(new MouseEvent('click',{clientX:r.left+(v.x+1)*r.width/2,clientY:r.top+(1-v.y)*r.height/2,bubbles:true}));
if(document.getElementById('selection').textContent.startsWith('真实 PlannerData 节点')){clicked=true;break;}}
check('REAL_NODE_CANVAS_CLICK',clicked);
if(failures.length){inspectFailure(0);check('FAILED_SAMPLE_ACTUAL_Q',close(currentQ,failures[0].q,1e-12));}
if(planner.events.length){document.getElementById('event').value=0;document.getElementById('step').click();check('SEARCH_EVENT_STEP',document.getElementById('event').value==='1');}
}
}else result.search='NOT_RUN_NO_NATIVE_REQUESTS';
document.getElementById('mode').value=DATA.segments.length?'cycle':'preview';modeChanged();
if(DATA.segments.length)setTime(DATA.duration_s);else preview(DATA.previews.length-1);
result.status='PASS';
}catch(error){result.status='FAIL';result.error=error.stack;}
const pre=document.createElement('pre');pre.id='browser-check-result';pre.textContent=JSON.stringify(result);document.body.append(pre);
window.BROWSER_CHECK_RESULT=result;
},500);
</script>
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--html', type=Path, required=True)
    parser.add_argument('--browser', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    start = perf_counter()
    page = args.output/'browser-test.html'
    page.write_text(args.html.read_text(encoding='utf-8')+CHECK, encoding='utf-8')
    process = subprocess.run([str(args.browser), '--headless', '--no-first-run',
        '--user-data-dir='+str((args.output/'profile').resolve()),
        '--window-size=1440,1000', '--virtual-time-budget=4000', '--dump-dom',
        '--screenshot='+str((args.output/'checked.png').resolve()), page.resolve().as_uri()],
        capture_output=True, timeout=120)
    match = re.search(r'<pre id="browser-check-result">(.*?)</pre>',
                      process.stdout.decode('utf-8', 'replace'), re.S)
    if not match:
        (args.output/'browser-stderr.txt').write_bytes(process.stderr)
        raise RuntimeError('BROWSER_TEST_RESULT_MISSING:'+str(process.returncode))
    result = json.loads(html.unescape(match.group(1)))
    result['wall_s'] = perf_counter()-start
    (args.output/'checks.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({**{k:v for k,v in result.items() if k!='checks'},
                      'check_count':len(result['checks'])}))
    if result['status'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
