"""Explain a retained fixed-target failure using its actual CAD chain.

This is a diagnostic, never a replacement trajectory or a feasibility gate.
The wrist radial bound is necessary, not sufficient, for a fixed TCP normal.
"""
from pathlib import Path
import sys,argparse,json,math
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,save


def diagnose(report,output,plot=False,mesh_check=False):
    data=json.loads(Path(report).read_text(encoding='utf8'))
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    c=make_context(ROOT,output/'model',data['settings'])
    rows=[]
    for failure in data['failures']:
        if failure.get('kind')!='EXTRACTION_CANDIDATE_FAILED':continue
        detail=failure['detail'];q=np.asarray(detail['q_rad'])
        axes=c.robot.joint_axis_frames(q);j1=axes['s50_joint_1'][0];j5=axes['s50_joint_5'][0]
        shoulder_axis=axes['s50_joint_2'][1];actual=c.robot.fk(q)
        local=actual[:3,:3].T@(actual[:3,3]-j5)
        desired=np.asarray(detail['desired_physical_contact'])
        desired_j5=desired[:3,3]-desired[:3,:3]@local
        offset=abs(float((j5-j1)@shoulder_axis))
        radius=float(np.linalg.norm(desired_j5[:2]-j1[:2]))
        rows.append(dict(branch=failure['branch'],reason=detail['reason'],point_index=detail['point_index'],
            first_rejected_tcp_xyz_m=desired[:3,3].tolist(),
            last_valid_tcp_xyz_m=np.asarray(detail['last_valid_physical_contact'])[:3,3].tolist(),
            ik_success=detail['ik_success'],ik_position_error_m=detail['position_error_m'],
            ik_orientation_error_rad=detail['orientation_error_rad'],q_deg=np.rad2deg(q).tolist(),
            collision_pair=None if detail['collision'] is None else detail['collision']['pair'],
            j1_axis_origin_m=j1.tolist(),j5_from_tcp_in_tcp_axes_m=(-local).tolist(),
            necessary_minimum_j5_xy_radius_m=offset,requested_j5_xy_radius_m=radius,
            necessary_radius_deficit_m=max(0.,offset-radius),desired_j5_xyz_m=desired_j5.tolist()))
    offset=rows[0]['necessary_minimum_j5_xy_radius_m']
    axial=-rows[0]['j5_from_tcp_in_tcp_axes_m'][2]
    # CAD J2/J3/J4 axes are parallel and horizontal. Their summed axial
    # translation fixes projection(J1->J5, unit J2 axis) at this offset.
    # Hence horizontal norm(J1->J5) >= |offset|, by Cauchy-Schwarz.
    rng=np.random.default_rng(0);errors=[]
    for q in rng.uniform(c.robot.joint_limits[:,0],c.robot.joint_limits[:,1],size=(24,6)):
        axes=c.robot.joint_axis_frames(q)
        d=abs(float((axes['s50_joint_5'][0]-axes['s50_joint_1'][0])@axes['s50_joint_2'][1]))
        errors.append(abs(d-offset))
    layouts=[]
    for gap in [.1,.2,.3]:
        base_x=-1.15-gap;base_y=.15
        limit=-(base_x+math.sqrt(offset**2-base_y**2)+axial)
        radius_full=math.hypot(-.6-axial-base_x,base_y)
        layouts.append(dict(gap_m=gap,base_xyz_m=[base_x,base_y,.9],
            ideal_normal_positive_x_first_radial_boundary_withdrawal_m=limit,
            box_depth_m=.6,j5_radius_at_600mm_withdrawal_m=radius_full,
            radial_necessary_condition_at_600mm_passed=radius_full>=offset,
            does_not_prove_collision_free_path=True))
    result=dict(scope='RETAINED_FAILURE_DIAGNOSIS_NOT_EXECUTED_MOTION',target='carton_l07_c02',
        source_report=str(report),model_sha256=data['source_model_sha256'],
        belt_surface_height_m=1.5,position_tolerance_m=.00004,orientation_tolerance_rad=.0001,
        rows=rows,radial_bound=dict(necessary_not_sufficient=True,minimum_radius_m=offset,
            j5_to_physical_contact_axial_distance_m=axial,random_configuration_identity_max_error_m=max(errors)),
        same_target_layout_comparison=layouts)
    if mesh_check:
        from unloading_sim.s50_sim_collision import SimCollision,bounds
        from unloading_sim.s50_collision_diagnostics import _world_geometry
        checker=SimCollision(c);belt=next(e for e in checker.environment(1.5) if e['name']=='conveyor_transverse')
        for row in rows:
            if row['collision_pair']!=['s50_link_2','conveyor_transverse']:continue
            q=np.deg2rad(row['q_deg']);hits=[]
            for i,g in enumerate(_world_geometry(checker.backend,q)):
                if g['link']!='s50_link_2':continue
                entity=dict(name=g['link'],shape=g['shape'],T=g['world'],bounds=bounds(checker.robot_local[i],g['world']))
                hit=checker.query(entity,belt)
                hits.append(dict(geometry=g['geometry'],intersection=hit is not None,
                    contacts=None if hit is None else hit['contact_points']))
            row['independent_link2_mesh_pair_recheck']=hits
    save(output/'diagnosis.json',result)
    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle,Rectangle
        plt.rcParams['font.sans-serif']=['Microsoft YaHei','DejaVu Sans']
        plt.rcParams['axes.unicode_minus']=False
        fig,(ax,bx)=plt.subplots(1,2,figsize=(13,5.5),layout='constrained',facecolor='white')
        ax.add_patch(Rectangle((-.8,-1.1),.7,2.2,color='#6cabb8',alpha=.22,label='横带投影（高 1.5 m）'))
        ax.add_patch(Rectangle((0,-1.04),.6,2.08,color='#d6bc91',alpha=.4,label='未移走的箱堆'))
        ax.add_patch(Circle((-1.25,.15),offset,color='#d96a5e',alpha=.25,label='J5 必要不可达区'))
        x=np.linspace(0,-.69055,300)
        ax.plot(x,np.zeros_like(x),'--',color='#398da4',label='吸盘接触点直线')
        ax.plot(x-axial,np.zeros_like(x),color='#263b68',label='固定朝向要求的 J5 轨迹')
        ax.scatter([-1.25],[.15],c='#222222',s=38);ax.annotate('J1',(-1.25,.15),xytext=(6,8),textcoords='offset points')
        ax.scatter([-.6-axial],[0],c='#b3272f',s=55,zorder=5)
        ax.annotate('箱子刚完全抽出时的 J5',(-.6-axial,0),xytext=(-1.52,-.48),
                    arrowprops=dict(arrowstyle='->',color='#b3272f'),fontsize=9)
        ax.set(xlim=(-1.6,.65),ylim=(-1.15,1.15),xlabel='世界 X / m',ylabel='世界 Y / m',title='100 mm 布局：俯视几何')
        ax.set_aspect('equal');ax.legend(loc='upper left',fontsize=8);ax.grid(alpha=.15)
        withdrawal=np.linspace(0,.72,500)
        for row,color in zip(layouts,['#d44d48','#dc963e','#298f87']):
            radius=np.hypot(-withdrawal-axial-row['base_xyz_m'][0],.15)
            bx.plot(1000*withdrawal,1000*radius,color=color,label=f"间距 {row['gap_m']*1000:.0f} mm")
        bx.axhline(offset*1000,color='#8a3232',ls='--',label='J5 必要下限 ≈ 254 mm')
        bx.axvline(600,color='#444444',ls=':',label='箱深 600 mm：刚完全抽出')
        bx.fill_between([0,720],0,offset*1000,color='#d96a5e',alpha=.10)
        bx.set(xlim=(0,720),ylim=(100,950),xlabel='从抓取点直线抽离 / mm',ylabel='要求的 J5 到 J1 水平距离 / mm',
               title='只检验运动学必要条件，不代表路径无碰撞')
        bx.legend(fontsize=8);bx.grid(alpha=.18)
        fig.suptitle('固定中间箱、固定正吸方向：100 mm 抽离失败诊断',fontsize=15)
        fig.savefig(output/'wrist-radius-diagnostic.png',dpi=160);plt.close(fig)
    print(json.dumps(dict(minimum_radius_m=offset,j5_tcp_axial_m=axial,layouts=layouts,branches=len(rows)),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('report',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--plot',action='store_true');p.add_argument('--mesh-check',action='store_true')
    a=p.parse_args();diagnose(a.report,a.output,a.plot,a.mesh_check)
