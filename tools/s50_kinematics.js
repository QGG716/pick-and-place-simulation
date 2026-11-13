/* Independent row-major matrix FK for the CAD S50 preview and replay viewer. */
(function (root) {
  'use strict';
  const identity = () => [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
  function multiply(a,b) {
    return a.map((row,i) => row.map((_,j) => row.reduce((sum,x,k) => sum+x*b[k][j],0)));
  }
  function rotation(axis,angle) {
    const norm=Math.hypot(...axis);
    if(!Number.isFinite(norm)||norm<1e-12||!Number.isFinite(angle))throw Error('INVALID_JOINT_ROTATION');
    const [x,y,z]=axis.map(v=>v/norm),c=Math.cos(angle),s=Math.sin(angle),v=1-c;
    return [[x*x*v+c,x*y*v-z*s,x*z*v+y*s,0],
            [y*x*v+z*s,y*y*v+c,y*z*v-x*s,0],
            [z*x*v-y*s,z*y*v+x*s,z*z*v+c,0],[0,0,0,1]];
  }
  function forward(model,q,worldFromMount,convention) {
    if(!Array.isArray(q)||q.length!==6||!q.every(Number.isFinite))throw Error('S50_EXPECTS_SIX_FINITE_RADIANS');
    if(!model||!Array.isArray(model.joints)||model.joints.length!==6)throw Error('S50_EXPECTS_SIX_MEASURED_JOINTS');
    if(convention)q=q.map((v,i)=>convention.sign[i]*v+convention.cad_offset_deg[i]*Math.PI/180);
    const frames={},base=model.base_link||model.links[0].name;
    frames[base]=worldFromMount||identity();
    model.joints.forEach((joint,i)=>{
      if(!frames[joint.parent_link]||frames[joint.child_link])throw Error('INVALID_JOINT_CHAIN');
      frames[joint.child_link]=multiply(multiply(frames[joint.parent_link],joint.origin),rotation(joint.axis,q[i]));
    });
    for(const joint of model.fixed_joints||[]){
      if(!frames[joint.parent_link]||frames[joint.child_link])throw Error('INVALID_FIXED_FRAME_CHAIN');
      frames[joint.child_link]=multiply(frames[joint.parent_link],joint.origin);
    }
    return frames;
  }
  const api={identity,multiply,rotation,forward};
  root.S50Kinematics=api;
  if(typeof module==='object'&&module.exports)module.exports=api;
})(typeof globalThis==='object'?globalThis:this);
