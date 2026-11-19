"""Check archived-state identity, independent FK and display-only controls in Edge."""
import argparse
import html
import json
from pathlib import Path
import re
import subprocess

CHECK = r'''
<script>setTimeout(()=>{const result={checks:[],status:'RUNNING'};
const check=(key,value)=>{if(!value)throw Error(key);result.checks.push(key);};
const same=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
try{
const d=window.BLOCKER_DATA,v=window.blockerViewer,id=x=>document.getElementById(x);
check('READY',window.BLOCKER_VIEWER_READY===true);
check('INITIAL_SEGMENTATION_DECODED',id('image').complete&&id('image').naturalWidth===640&&id('image').naturalHeight===480);
check('READ_ONLY',d.execution_interface===false&&d.hardware_execution==='NOT_RUN'&&d.full_path_search==='NOT_RUN');
check('CURRENT_TASK',d.task_id==='6e13d059fc1443f2ae1c8c56962d21fe');
check('FULL_SOURCE_MODEL',v.robot.filter(r=>r.item.layer==='collision').length===715&&v.rings.length===12&&d.model.static_visual.length===0);
let maxError=0;
for(const fixture of d.fk_fixtures){const actual=v.fk(fixture.q);
for(const [name,reference] of Object.entries(fixture.frames)){
const expected=reference[0].map((_,i)=>reference.map(row=>row[i])).flat();
maxError=Math.max(maxError,...expected.map((x,i)=>Math.abs(x-actual[name].elements[i])));}}
check('ALL_ARCHIVED_FK_MATCH_INDEPENDENT_MODEL',maxError<1e-8);result.maximum_fk_matrix_error=maxError;
const fingerprint=JSON.stringify(d);v.selectTab('bounds');
check('FROZEN_START',same(v.q,d.q_start));
check('CONFIGURED_BOUNDARY_IDENTIFIED',d.summary.primary.category==='workspace_boundary'&&Math.abs(d.summary.primary.overlap_depth_m-.202490556183)<1e-10);
const viewport=()=>{const r=id('view').getBoundingClientRect();return [r.x,r.y,r.width,r.height];},layout=viewport();
id('envelope').click();check('DISPLAY_SWITCH_ONLY',!v.envelopeBox.surface.visible&&v.fitBox.surface.visible&&same(d,JSON.parse(fingerprint)));id('envelope').click();
id('environment').click();check('ALL_SCENE_OBJECTS_RETAINED',v.world.length===d.world.length&&v.world.every(r=>r.o.visible));id('environment').click();
id('collision').click();check('ALL_COLLISION_SOLIDS_VISIBLE',v.robot.filter(r=>r.item.layer==='collision').every(r=>r.o.visible));id('collision').click();
v.selectTab('contact');
for(let i=0;i<d.candidates.length;i++){
id('candidate').value=String(i);id('candidate').dispatchEvent(new Event('change'));
const row=d.candidates[i],s=row.stages.find(s=>s.stage==='contact').checked[0],w=s.gap_witness;
check('EXACT_CONTACT_POSE_'+row.index,same(v.q,s.q)&&same(v.pair,s.native.failure.pair));
check('VERIFIED_VERTEX_WITNESS_'+row.index,!!w&&Math.abs(w.independent_gap_m-w.native_gap_m)<1e-6&&Math.abs(w.rigid_from_observed_plane_m-w.envelope_front_from_observed_plane_m-w.independent_gap_m)<1e-10);
check('REAL_HIGHLIGHT_'+row.index,v.robot.some(r=>r.item.id==='part_002_rigid'&&r.o.visible&&r.o.material.color.getHex()===0xff526b));
check('ALL_CUPS_RETAINED_'+row.index,v.rings.every(r=>r.line.visible)&&row.active_ids.length===6);
id('stage').value='approach';id('stage').dispatchEvent(new Event('change'));
check('APPROACH_IS_SEPARATE_STATIC_STATE_'+row.index,same(v.q,row.stages.find(s=>s.stage==='approach').checked[0].q)&&v.pair.length===0);
id('stage').value='contact';id('stage').dispatchEvent(new Event('change'));
}
id('candidate').value='0';v.selectTab('branch');
check('SECOND_BRANCH_COLLISION',same(v.pair,['link_3','eco65_mast_mast_part_019'])&&same(v.q,d.candidates[0].stages.find(s=>s.stage==='contact').checked[1].q));
check('NEGATIVE_DISTANCE_NOT_PENETRATION_DEPTH',id('branch-panel').textContent.includes('不是 1 m'));
v.selectTab('holding');check('HOLDING_COMPARISON_FROM_SOURCE',id('full-margin').textContent.startsWith(d.candidates[0].nominal_estimate.full_envelope.toFixed(3))&&id('nominal-margin').textContent.startsWith(d.candidates[0].nominal_estimate.unverified_nominal_only.toFixed(3)));
check('NO_LAYOUT_SHIFT_ON_TEXT_CHANGE',same(viewport(),layout));
for(const source of ['rgb','segmentation']){id(source).click();const png=atob(d.images[source].split(',')[1]);const size=offset=>[0,1,2,3].reduce((n,i)=>n*256+png.charCodeAt(offset+i),0);check('SAME_FRAME_IMAGE_'+source,id('image').src===d.images[source]&&size(16)===640&&size(20)===480);}
check('NO_SOURCE_MUTATION',JSON.stringify(d)===fingerprint);
v.selectTab('contact');id('isolate').click();check('ISOLATION_SHOWS_EXACT_RIGID_SOLID_NOT_WHOLE_CAD_PART',v.robot.some(r=>r.o.visible)&&v.robot.filter(r=>r.o.visible).every(r=>r.item.layer==='collision'&&r.item.id==='part_002_rigid'));
v.selectTab('__VIEW__');if('__ISOLATE__'==='true'&&v.tab==='contact')id('isolate').click();
document.querySelector('aside').scrollTop=0;
check('NO_JS_ERROR',id('error').textContent==='');result.status='PASS';
}catch(error){result.status='FAIL';result.error=error.stack;}
const pre=document.createElement('pre');pre.id='browser-check-result';pre.textContent=JSON.stringify(result);document.body.append(pre);
},500);</script>
'''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--html',type=Path,required=True)
    p.add_argument('--browser',type=Path,default=Path('/opt/microsoft/msedge/microsoft-edge'))
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--view',choices=['bounds','contact','branch','holding'],default='bounds')
    p.add_argument('--isolate',action='store_true');a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    page=a.output/'browser-test.html'
    page.write_text(a.html.read_text()+CHECK.replace('__VIEW__',a.view).replace('__ISOLATE__',str(a.isolate).lower()))
    proc=subprocess.run([str(a.browser),'--headless','--no-first-run',
        '--user-data-dir='+str((a.output/'profile').resolve()),'--window-size=1440,1000',
        '--virtual-time-budget=4500','--dump-dom','--screenshot='+str((a.output/'checked.png').resolve()),
        page.resolve().as_uri()],capture_output=True,timeout=90)
    (a.output/'browser-stderr.txt').write_bytes(proc.stderr)
    match=re.search(r'<pre id="browser-check-result">(.*?)</pre>',proc.stdout.decode('utf-8','replace'),re.S)
    result=json.loads(html.unescape(match.group(1))) if match else dict(status='FAIL',error='BROWSER_TEST_RESULT_MISSING',exit_code=proc.returncode)
    (a.output/'checks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    if result['status']!='PASS':raise SystemExit(1)


if __name__=='__main__':main()
