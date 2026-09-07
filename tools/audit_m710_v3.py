"""Independent numeric and mechanism evidence for V3, no success-rate targets."""
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT))
import numpy as np
from unloading_sim.geometry import make_transform,rotation_matrix_from_rpy,rotation_vector_from_matrix
from unloading_sim.fanuc_m710id70 import target_pose
from unloading_sim.depalletizing import minimum_clearance_extraction_distance
from unloading_sim.validation_config import load_validation_config
from unloading_sim.validation_motion import Cell
from unloading_sim.validation_physics import RigidAttachment,external_load
from unloading_sim.validation_scenes import box
from unloading_sim.timing import JointMotionLimits,time_parameterize_joint_path
from tools.run_m710id70_v3 import write_json,write_csv


def main():
    cfg=load_validation_config();cell=Cell(cfg);r=cell.robot;out=ROOT/'outputs/m710id70_v3/v3';out.mkdir(parents=True,exist_ok=True)
    q=np.asarray(cfg.data['robot']['home_joints']);size=cfg.data['scene']['box_sizes_m']['A_600x400x300'];load_rows=[]
    target=box('reference_box',[1.3,0,.9],size)
    for face in ['front','left','right','top']:
        for roll in cfg.data['planning']['roll_candidates_deg']:
            contact,_=target_pose(1,0,.9,size,face,roll)
            attachment=RigidAttachment.capture(contact,target)
            load=external_load(r,cfg.tool,q,attachment,cfg.data['scene']['box_mass_kg'],cfg.data['scene']['box_com_fraction'],cfg.model)
            load_rows.append({'face':face,'roll_deg':roll,'tcp_from_box':attachment.tcp_from_box.tolist(),
                              'reference_q':q.tolist(),'load':load,'scope':'rigid_attached_load_case_at_reference_q_not_a_completed_pick'})
    write_json(out/'load_face_roll_audit.json',load_rows)
    # A genuinely nonzero, collision-checked empty trajectory in the same cell.
    delta=np.array([.02,.015,-.01,.02,.01,-.015]);path=[q,q+delta,q]
    obs=[*cell.fixtures(),*cell.decks((0,.2))]
    failure=cell.path_failure(path,obs)
    timed=time_parameterize_joint_path(path,cfg.motion_limits())
    samples=[]
    for t in np.linspace(0,timed.duration_seconds,201):
        qi,qd,qdd,j=timed.sample(t)
        load=external_load(r,cfg.tool,qi,None,0,[0,0,0],cfg.model,qd,qdd)
        samples.append({'t_s':float(t),'q':qi.tolist(),'qd':qd.tolist(),'qdd':qdd.tolist(),'jerk':j.tolist(),'load':load})
    one=time_parameterize_joint_path([[0],[1]],JointMotionLimits(np.array([100.]),np.array([1.]),np.array([1e6])))
    write_json(out/'trajectory_dynamics_audit.json',{'path_collision_failure':failure,'trajectory':timed.audit(cfg.motion_limits()),
        'q_knots':np.asarray(path).tolist(),'t_knots':timed.time_from_start.tolist(),'samples':samples,
        'single_joint_rest_to_rest':{'distance_rad':1,'acceleration_limit_rad_s2':1,'independent_lower_bound_s':2,'actual_duration_s':one.duration_seconds},
        'full_robot_inverse_dynamics':'NOT_EVALUATED','external_load_model':'tool_only_Newton_Euler_on_same_analytic_trajectory'})
    # Independent central differences use matrix derivatives, not the SO(3)
    # log implementation under test. Exercise rotated base + 6D TCP offsets.
    r.base_transform=make_transform(rotation_matrix_from_rpy(.2,-.4,.7),[-.5,.1,.8])
    r.tip_from_tcp=make_transform(rotation_matrix_from_rpy(.3,.2,-.6),[.12,-.08,.29]);rng=np.random.default_rng(71070)
    errors=[]
    for _ in range(20):
        qi=rng.uniform(r.joint_limits[:,0]+.1,r.joint_limits[:,1]-.1);jac=r.geometric_jacobian(qi);numeric=np.zeros_like(jac);R=r.fk(qi)[:3,:3];h=1e-6
        for j in range(6):
            dq=np.eye(6)[j]*h;plus=r.fk(qi+dq);minus=r.fk(qi-dq)
            numeric[:3,j]=(plus[:3,3]-minus[:3,3])/(2*h)
            W=((plus[:3,:3]-minus[:3,:3])/(2*h))@R.T
            numeric[3:,j]=[W[2,1],W[0,2],W[1,0]]
        errors.append({'q':qi.tolist(),'translation_max_error':float(np.max(abs(jac[:3]-numeric[:3]))),'rotation_max_error':float(np.max(abs(jac[3:]-numeric[3:])))})
    write_json(out/'independent_fk_jacobian_audit.json',{'seed':71070,'step_rad':1e-6,'samples':errors,
        'base_transform':r.base_transform.tolist(),'tip_from_tcp':r.tip_from_tcp.tolist(),'pass':all(max(e['translation_max_error'],e['rotation_max_error'])<2e-8 for e in errors)})
    # Explain distance per constraint; do not require a wider distribution.
    rows=[]
    for depth in [.4,.6]:
        target=box('target',[1+depth/2,0,.9],[depth,.4,.3])
        for count in [0,1,2]:
            neighbors=[box(f'neighbor_{i}',target.center+[0,sign*.41,0],[depth,.4,.3]) for i,sign in enumerate([-1,1][:count])]
            distance=minimum_clearance_extraction_distance(target,[-1,0,0],neighbors,free_space_clearance_m=.02,scan_step_m=.01)
            rows.append({'depth_m':depth,'neighbor_count':count,'neighbor_names':[b.name for b in neighbors],'fixed_orientation':True,
                         'clearance_m':.02,'distance_m':distance,'explanation':'no local constraints' if not count else 'box depth + clearance; initial lateral gap .01 < .02'})
    write_csv(out/'extraction_independent_cases.csv',rows)
    print('Wrote load, trajectory, Jacobian and extraction evidence to',out)


if __name__=='__main__':main()
