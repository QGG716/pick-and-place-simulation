// Focused numerical replay check; this does not claim browser UI acceptance.
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const crypto=require('node:crypto'),K=require('./s50_kinematics.js');
const file=process.argv[2],output=process.argv[3];
if(!file||!output)throw Error('Usage: node tools/check_s50_kinematic_replay.cjs cycle.json output.json');
const root=path.resolve(__dirname,'..'),raw=fs.readFileSync(file),S=JSON.parse(raw);
const modelRaw=fs.readFileSync(path.join(root,S.settings.model_definition)),model=JSON.parse(modelRaw);
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
assert.equal(hash(modelRaw),S.source_model_sha256);
assert.equal(hash(fs.readFileSync(path.join(root,S.source_step_path))),S.source_step_sha256);
const complete=['COMPLETED_KINEMATIC_SINGLE_CARTON','COMPLETED_KINEMATIC_FULL_FACE'].includes(S.status);
assert.ok(complete||process.argv.includes('--allow-partial'),'Partial evidence needs --allow-partial');
const targetOf=s=>s.target||S.target,initial=new Set(S.initial_carton_ids);
const error=(a,b)=>Math.max(...a.flat().map((v,i)=>Math.abs(v-b.flat()[i])));
let fkError=0,boxError=0,maxSpeed=0,maxDelta=0,lastCompleted=[];
for(let i=0;i<S.samples.length;i++){
  const s=S.samples[i],previous=S.samples[i-1];
  const F=K.forward(model,s.q_rad,S.world_from_model_base,S.settings.joint_coordinate_convention);
  const contact=K.multiply(F.s50_flange,S.flange_from_physical_contact);
  fkError=Math.max(fkError,error(contact,s.world_from_physical_contact));
  assert.ok(initial.has(targetOf(s)),'Known target identity');
  if(s.attached)boxError=Math.max(boxError,error(K.multiply(contact,s.physical_contact_from_attached_box||S.physical_contact_from_attached_box),s.world_from_carton));
  if(s.completed_carton_ids){
    assert.ok(lastCompleted.every((id,j)=>s.completed_carton_ids[j]===id),'Completed prefix cannot reset');
    assert.equal(new Set(s.completed_carton_ids).size,s.completed_carton_ids.length);
    for(const id of s.completed_carton_ids)assert.ok(initial.has(id));
    if(s.attached)assert.ok(!s.completed_carton_ids.includes(targetOf(s)),'Retired carton cannot be reattached');
    lastCompleted=s.completed_carton_ids;
  }
  if(previous&&targetOf(s)!==targetOf(previous)){
    assert.equal(s.phase,'task_start');assert.equal(s.attached,false);
    assert.ok(error(s.q_rad,previous.q_rad)<1e-12,'No robot teleport between cartons');
  }
  s.q_rad.forEach((v,j)=>{
    const [lo,hi]=S.settings.joint_ranges_deg[j].map(x=>x*Math.PI/180);
    assert.ok(v>=lo-1e-10&&v<=hi+1e-10,'temporary joint range');
    if(previous){const dt=s.time_s-previous.time_s;assert.ok(dt>0);const dq=Math.abs(v-previous.q_rad[j]);maxDelta=Math.max(maxDelta,dq);maxSpeed=Math.max(maxSpeed,dq/dt);}
  });
}
assert.ok(fkError<1e-9&&boxError<1e-9,'independent viewer FK / attached transform');
assert.ok(maxSpeed<=S.settings.timing.joint_velocity_rad_s+1e-9);
const first=S.samples[0],last=S.samples.at(-1);
const returnError=Math.max(...first.q_rad.map((v,j)=>Math.abs(v-last.q_rad[j])));
if(complete){
  assert.ok(returnError<1e-9);assert.equal(last.belt_height_m,.6);assert.equal(last.attached,false);
  for(const phase of ['home','belt_raise','belt_lower','return'])assert.ok(S.stages.some(s=>s.phase===phase&&s.status==='PASS'),phase);
}
assert.equal(S.initial_carton_ids.length,40);assert.equal(new Set(S.initial_carton_ids).size,40);
assert.equal(S.completed_cartons+S.remaining_cartons,40);
if(S.status==='COMPLETED_KINEMATIC_FULL_FACE')assert.equal(S.completed_cartons,40);
if(S.status==='COMPLETED_KINEMATIC_SINGLE_CARTON')assert.equal(S.completed_cartons,1);
if(S.completed_carton_ids){
  assert.equal(S.completed_carton_ids.length,S.completed_cartons);
  assert.deepEqual(S.completed_carton_ids,S.preferred_order.slice(0,S.completed_cartons));
  assert.deepEqual(new Set(S.remaining_carton_ids),new Set(S.initial_carton_ids.filter(id=>!S.completed_carton_ids.includes(id))));
}
for(const s of S.samples.filter(s=>['release','drop','exit','park','outfeed','belt_lower','return','complete'].includes(s.phase)))assert.equal(s.attached,false);
const tasks=S.tasks||[{target:S.target,cups:S.cups}];
assert.equal(tasks.length,S.completed_cartons);
for(const task of tasks){
  for(const phase of ['pregrasp','approach','grasp','separate','withdraw','transfer','release','drop','exit','park','outfeed'])
    assert.ok(S.stages.some(s=>(s.target||S.target)===task.target&&s.phase===phase&&s.status==='PASS'),task.target+':'+phase);
  const release=S.samples[S.stages.find(s=>(s.target||S.target)===task.target&&s.phase==='release').last_sample];
  const outfeed=S.samples[S.stages.find(s=>(s.target||S.target)===task.target&&s.phase==='outfeed').last_sample];
  assert.ok(error(release.world_from_carton,outfeed.world_from_carton)>.1,'Released carton travels independently');
  if(task.release_drop_m!==undefined)assert.ok(task.release_drop_m>=0&&task.release_drop_m<=S.maximum_release_drop_m+1e-6);
  for(const k of ['eligible','commanded','actual_geometric_contact'])assert.equal(task.cups[k].length,72);
  assert.ok(task.cups.commanded.some(Boolean));
  task.cups.commanded.forEach((v,j)=>{if(v)assert.ok(task.cups.eligible[j]&&task.cups.actual_geometric_contact[j]);});
}
const result={status:'PASS',source_report_sha256:hash(raw),source_step_sha256:S.source_step_sha256,
  samples:S.samples.length,core_to_viewer_fk_max_matrix_error:fkError,attached_box_max_matrix_error:boxError,
  home_return_error_rad:returnError,maximum_sampled_joint_speed_rad_s:maxSpeed,maximum_joint_sample_delta_rad:maxDelta,
  source_status:S.status,initial_carton_count:40,completed_cartons:S.completed_cartons,remaining_cartons:S.remaining_cartons,final_belt_height_m:last.belt_height_m,
  scope:'INDEPENDENT_JAVASCRIPT_FK_AND_RECORDED_TASK_STATE',browser_acceptance:'SEPARATE_BROWSER_CHECK_REQUIRED'};
fs.writeFileSync(output,JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
