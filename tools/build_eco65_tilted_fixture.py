"""Create a private, source-bound 30-degree installation and supported fixture."""
import argparse,copy,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from unloading_sim.eco65.model import ROOT,load_json,save_json,transform
from unloading_sim.eco65.native_io import file_hash
from eco65_touch_sources import verify_scene_sources,derived_scene_identity
from eco65_camera_mount import pivot_transform,mount_check


def build(source,out):
    start=time.perf_counter();out=out.resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    old=load_json(source);verify_scene_sources(old);c=copy.deepcopy(old);s=load_json(ROOT/c['snapshot'])
    camera=c['camera'];M=np.asarray(s['scene']['mast']['T_link1_mast'])
    # Existing pocket floor z=.410, wall top=.450 (original mast CAD planar faces).
    # A new saddle raises the rear-cover seating plane to .460. The pitch axle
    # runs across that saddle in mast +X, not through the optical centre.
    pivot=np.array([.113,-.00405,.460]);lift=transform([0,0,.050])
    A=pivot_transform(pivot,[1,0,0],30.)@lift
    old_cad=np.array(camera['T_mast_CAD']);old_opt=np.array(camera['T_mast_optical'])
    camera['T_mast_CAD']=(A@old_cad).tolist();camera['T_mast_optical']=(A@old_opt).tolist()
    camera['T_link1_optical']=(M@A@old_opt).tolist()
    adapter=[]
    def box(name,size,T):adapter.append(dict(name=name,size_m=size,T_mast=np.asarray(T).tolist()))
    # Two fixed risers and axle sit on the existing pocket. The tilted saddle
    # touches the unchanged camera rear cover; all are project-derived pieces.
    for side,x in [('left',.0705),('right',.1555)]:
        box('pitch_riser_'+side,[.002,.024,.011],transform([x,-.00405,.4555]))
        # Two side journals define one mechanical axis. A solid cross-shaft
        # through the unmodified rear cover would invent a CAD mounting hole.
        box('pitch_journal_'+side,[.002,.006,.006],transform([x,pivot[1],pivot[2]]))
    saddle=transform([.113,-.00405,.4585]);saddle=pivot_transform(pivot,[1,0,0],30.)@saddle
    box('pitch_saddle',[.083,.049,.003],saddle)
    camera['pitch_adapter']=adapter
    camera['mount']=dict(schema='eco65_camera_mechanical_pitch_v1',downward_pitch_deg=30.,pivot_mast_m=pivot.tolist(),
        axis_mast=[1,0,0],pre_pitch_rear_cover_lift_m=.050,T_new_from_old_mast=A.tolist(),
        pivot_basis='Project-derived axle at top of risers seated on original mast pocket walls; not the entrance pupil',
        classification='PROJECT_DERIVED_SIMULATION_DESIGN',original_CAD_read_only=True,
        original_CAD_to_optical=(np.linalg.inv(old_cad)@old_opt).tolist(),
        connections=[['mast_pocket_left','pitch_riser_left'],['mast_pocket_right','pitch_riser_right'],
                     ['pitch_riser_left','pitch_journal_left'],['pitch_riser_right','pitch_journal_right'],
                     ['pitch_journal_left','pitch_saddle'],['pitch_journal_right','pitch_saddle'],['pitch_saddle','camera_rear_cover']])
    assert np.max(abs(np.linalg.inv(A@old_cad)@(A@old_opt)-np.linalg.inv(old_cad)@old_opt))<1e-12
    assert mount_check(M@A@old_opt,np.eye(4),camera)['accepted']
    c['probe']['bracket_size_m']=[.016,.005,.012]
    c['probe']['revision']=dict(reason='Shorten plate-normal bracket thickness only: top F.z .085 -> .0835 m; original part_001 bottom .086 m',
                              previous_size_m=old['probe']['bracket_size_m'],tip_and_axis_unchanged=True,expected_part001_gap_m=.0025)
    # A board looking upward 30 degrees faces the downward-looking camera.
    R=np.array([[0,.5,-np.sqrt(3)/2],[1,0,0],[0,-np.sqrt(3)/2,-.5]])
    center=np.array([-.76,.20,.52]);P=transform(center-R@np.array([.075,.0625,0]),R)
    # Connected base -> vertical column -> horizontal back arm -> rear plate.
    back_P=np.array([.075,.0625,.012]);back_D=R@back_P+P[:3,3]
    column_xy=back_D[:2]+[-.065,0];foot_h=.012;column_top=back_D[2]
    support=[]
    def support_D(name,size,pos):
        support.append(dict(name=name,size_m=size,T_P=(np.linalg.inv(P)@transform(pos)).tolist()))
    support_D('support_base',[.16,.14,foot_h],[*column_xy,foot_h/2])
    support_D('support_column',[.025,.025,column_top-foot_h],[*column_xy,(column_top+foot_h)/2])
    support_D('support_back_arm',[.065,.025,.020],[(column_xy[0]+back_D[0])/2,column_xy[1],column_top-.010])
    support.append(dict(name='support_rear_mount',size_m=[.08,.08,.008],T_P=transform(back_P).tolist()))
    c['board']['support']=dict(schema='eco65_table_board_support_v1',parts=support,table_id='trailer_floor',table_top_D_m=0.,
        classification='PROJECT_DERIVED_FIXED_SIMULATION_FIXTURE',locked_during_group=True,
        connections=[['trailer_floor','support_base'],['support_base','support_column'],['support_column','support_back_arm'],
                     ['support_back_arm','support_rear_mount'],['support_rear_mount','board_body']],
        fixed_to_table=True,fasteners_and_strength='NOT_EVALUATED')
    c['fixture_preparation']=dict(board_rotation_D=R.tolist(),candidate_centers_D_m=[center.tolist(),[-.79,.20,.52],[-.73,.20,.52]],
        heldout_translation_D_m=[.015,.025,0.],view_offsets_deg=[-8,0,8],old_placements=old['placements'])
    c['placements']=[dict(board_placement_id='train_01',split='train',T_D_P=P.tolist(),center_D_m=center.tolist()),
                     dict(board_placement_id='heldout_01',split='validation',T_D_P=(transform([.015,.025,0])@P).tolist(),center_D_m=(center+[.015,.025,0]).tolist())]
    c['touches']=[];c['photos']=[];c['static_recheck_status']='NOT_RUN'
    c['statuses']=['NEW_MECHANICAL_INSTALLATION','CALIBRATION_NOT_RUN','PHYSICAL_TOUCH_NOT_RUN']
    c['derived_scene_identity']=derived_scene_identity(c)
    save_json(out/'configuration.json',c)
    save_json(out/'installation_change.json',dict(source_configuration=dict(path=source.relative_to(ROOT).as_posix(),sha256=file_hash(source)),
        camera_CAD_optical_relation_max_delta=float(np.max(abs(np.linalg.inv(A@old_cad)@(A@old_opt)-np.linalg.inv(old_cad)@old_opt))),
        old_optical_link1=old['camera']['T_link1_optical'],new_optical_link1=camera['T_link1_optical'],
        mount_check=mount_check(M@A@old_opt,np.eye(4),camera),probe_tip_preserved=c['probe']['p_F_tip_m']==old['probe']['p_F_tip_m'],
        probe_axis_preserved=c['probe']['rod_start_F_m']==old['probe']['rod_start_F_m'],seconds=time.perf_counter()-start))
    # Reuse the exact local pattern bitmap, not historical camera photographs.
    import cv2
    b=c['board'];board=cv2.aruco.CharucoBoard(tuple(b['squares']),b['square_m'],b['marker_m'],cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,b['dictionary'])))
    cv2.imwrite(str(out/'board_pattern.png'),board.generateImage((1200,1000),marginSize=0,borderBits=1))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();build(a.source.resolve(),a.output)
