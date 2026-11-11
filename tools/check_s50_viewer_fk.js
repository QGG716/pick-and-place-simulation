/* Run with: node tools/check_s50_viewer_fk.js input.json output.json */
(function(root){
 'use strict';
 function verify(input,kinematics){
  if(input.schema!=='s50_viewer_independent_fk_input_v1')throw Error('UNKNOWN_PROBE_SCHEMA');
  let maximum=0,frameCount=0;const rows=[];
  for(const probe of input.probes){
   const actual=kinematics.forward(input.model,probe.q_rad,probe.world_from_mount);let error=0;
   for(const [name,expected]of Object.entries(probe.expected_frames)){
    if(!actual[name])throw Error('VIEWER_FRAME_MISSING:'+name);
    for(let i=0;i<4;i++)for(let j=0;j<4;j++)error=Math.max(error,Math.abs(actual[name][i][j]-expected[i][j]));
    frameCount++;
   }
   if(!Number.isFinite(error)||error>1e-9)throw Error('VIEWER_CORE_FK_MISMATCH:'+error);
   maximum=Math.max(maximum,error);rows.push({q_rad:probe.q_rad,coordinate_scope:probe.coordinate_scope,max_matrix_error:error});
  }
  return {schema:'s50_viewer_independent_fk_validation_v1',status:'PASS',
   model_definition_sha256:input.model.definition_sha256,javascript_sha256:input.javascript_sha256,
   runtime:'JavaScript Number, independent Rodrigues and row-major multiplication',
   core_reference:'URDFRobot named_link_frames from real S50 link-local URDF',
   probes:rows,probe_count:rows.length,compared_frame_count:frameCount,max_matrix_error:maximum,
   controller_limits_verified:false,installation_qualified:false,collision_qualified:false,task_reachability_claimed:false};
 }
 root.verifyS50ViewerFK=verify;
 if(typeof module==='object'&&module.exports){
  module.exports=verify;
  if(typeof require==='function'&&require.main===module){const fs=require('fs'),input=JSON.parse(fs.readFileSync(process.argv[2],'utf8')),report=verify(input,require('./s50_kinematics.js'));fs.writeFileSync(process.argv[3],JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({status:report.status,probes:report.probe_count,max_matrix_error:report.max_matrix_error}));}
 }
})(typeof globalThis==='object'?globalThis:this);
