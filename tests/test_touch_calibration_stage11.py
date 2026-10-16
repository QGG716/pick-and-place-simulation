"""Small acceptance/control regressions. Mock vision is NOT rendered-image evidence."""
import copy,hashlib,json,os,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from unloading_sim.touch_calibration import (RESULT_CONTRACT,STAGE1_PROFILE,TRAINING_CHECKS,fingerprint,
    observation_binding,definition_fingerprints,sample_groups,validate_observations,validate_result,coverage,
    fk,points,inv,intrinsics,rigid)

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from eco65_touch_geometry import pair_rule,distance_evidence
from eco65_touch_sources import validate_manifest_identity,bind_scene_sources


def fixture():
    joints=[]
    for i in range(6):
        T=np.eye(4);T[0,3]=0 if i==0 else .1
        joints.append(dict(name=f'joint_{i+1}',parent='base_link' if i==0 else f'link_{i}',child=f'link_{i+1}',
            type='revolute',origin=T.tolist(),axis=[0,0,1]))
    kin=dict(active_joint_names=[f'joint_{i+1}' for i in range(6)],base_link='base_link',T_D_base=np.eye(4).tolist(),
        limits_rad=[[-3.14,3.14]]*6,urdf_sha256='c'*64,joints=joints)
    probe=dict(p_F_tip_m=[.1,0,0]);qs=[[a,b,0,0,0,0] for a,b in [(-.6,-.3),(-.3,.4),(0,.7),(.3,-.4),(.6,.2),(.8,-.8)]]
    board=dict(touch_points=[dict(point_id=f'p{i}',p_P_m=points(fk(kin,q)['link_6'],probe['p_F_tip_m']).tolist()) for i,q in enumerate(qs)])
    ids=dict(camera='a'*64,probe=fingerprint(probe),board=fingerprint(board))
    camera=dict(K=intrinsics().tolist(),resolution=[640,480],distortion=[0.]*5,pixel_convention='Integer pixel centres; edges -0.5,N-0.5')
    photos=[];touches=[]
    for placement,split,indices in [('train_board','train',range(6)),('heldout_board','validation',[0,2,4])]:
        for i in indices:touches.append(dict(sample_id=f'{placement}_touch_{i}',board_placement_id=placement,split=split,point_id=f'p{i}',q_rad=qs[i],probe_id=ids['probe']))
        for i,angle in enumerate([-.2,0,.2]):
            name=f'{placement}_photo_{i}'
            photos.append(dict(sample_id=name,board_placement_id=placement,split=split,q_rad=[angle,0,0,0,0,0],
                camera_configuration_id=ids['camera'],K=camera['K'],image=name+'.png',sha256='b'*64))
    th=dict(touch_residual_m=.0001,tip_target_m=.0001,minimum_charuco_corners=8,reprojection_rms_px=.8,pnp_ambiguity_gap_px=.03,
        extrinsic_consistency_m=.005,extrinsic_consistency_deg=1.,truth_translation_m=.005,truth_rotation_deg=1.,validation_point_m=.005)
    obs=dict(schema='known_tip_touch_observations_v1',kinematics=kin,camera=camera,board=board,probe=probe,
        photos=photos,touches=touches,thresholds=th,source='SIMULATION',identifiers=ids)
    evidence=dict(T_parent_child=np.eye(4).tolist(),training_checks={k:dict(status='PASS') for k in TRAINING_CHECKS})
    result=dict(**RESULT_CONTRACT,**evidence,identifiers=ids,definition_fingerprints=definition_fingerprints(obs),
        observation_binding=observation_binding(obs),acceptance_profile=STAGE1_PROFILE,**sample_groups(obs),status='SOLVED_INDEPENDENT_VALIDATION_REQUIRED')
    return obs,result,evidence


def mocked_cli():
    """A subprocess exercises real CLI exit/report logic with explicitly mocked vision."""
    import evaluate_touch_calibration as entry
    obs,result,evidence=fixture()
    with patch.object(entry,'training_evidence',return_value=evidence), \
         patch.object(entry,'read_observation_image',side_effect=lambda root,photo,camera:photo), \
         patch.object(entry,'camera_from_board',side_effect=lambda photo,*a:(inv(fk(obs['kinematics'],photo['q_rad'])['link_1']),dict(corner_count=20,reprojection_rms_px=0.))):
        raise SystemExit(entry.main(sys.argv[1:]))


class AcceptanceControlTests(unittest.TestCase):
    def test_result_identity_frames_units_and_se3(self):
        obs,result,_=fixture();validate_observations(obs);validate_result(result,obs)
        for field,value in [('parent_frame','world'),('child_frame','link_6'),('translation_unit','mm'),
            ('direction','inverse'),('optical_axes','USD'),('schema','known_tip_fixed_extrinsics_v1')]:
            with self.subTest(field=field):
                bad=copy.deepcopy(result);bad[field]=value
                with self.assertRaises(ValueError):validate_result(bad,obs)
        for key in ('camera','probe','board'):
            bad=copy.deepcopy(result);bad['identifiers'][key]='d'*64
            with self.assertRaisesRegex(ValueError,'IDENTITY'):validate_result(bad,obs)
        for T in [np.diag([-1,1,1,1]),np.full((4,4),float('nan'))]:
            bad=copy.deepcopy(result);bad['T_parent_child']=T.tolist()
            with self.assertRaisesRegex(ValueError,'RIGID'):validate_result(bad,obs)

    def test_complete_content_binding_and_kinematic_identity(self):
        obs,result,_=fixture()
        for modify in [lambda o:o['photos'][0]['q_rad'].__setitem__(1,.02),
            lambda o:o['thresholds'].__setitem__('truth_translation_m',.02),
            lambda o:o['photos'][0].__setitem__('sha256','e'*64),
            lambda o:o['kinematics'].__setitem__('urdf_sha256','e'*64)]:
            bad=copy.deepcopy(obs);modify(bad);validate_observations(bad)
            with self.assertRaisesRegex(ValueError,'BINDING|IDENTITY'):validate_result(result,bad)
        bad=copy.deepcopy(result);bad.pop('observation_binding')
        with self.assertRaisesRegex(ValueError,'BINDING'):validate_result(bad,obs)

    def test_group_change_invalidates_bound_result(self):
        obs,result,_=fixture()
        for item in obs['photos']+obs['touches']:
            if item['split']=='validation':item['split']='train'
        validate_observations(obs)
        with self.assertRaisesRegex(ValueError,'BINDING'):validate_result(result,obs)

    def test_camera_probe_record_identity(self):
        for collection,key in [('photos','camera_configuration_id'),('touches','probe_id')]:
            obs,_,_=fixture();obs[collection][0][key]='d'*64
            with self.assertRaisesRegex(ValueError,'IDENTITY'):validate_observations(obs)

    def test_one_view_and_duplicate_j1_do_not_prove_consistency(self):
        obs,_,_=fixture();coverage(obs,'train');coverage(obs,'validation')
        obs['photos']=[p for p in obs['photos'] if p['split']=='train' or p['sample_id'].endswith('_0')]
        with self.assertRaisesRegex(ValueError,'COVERAGE'):coverage(obs,'validation')
        obs,_,_=fixture()
        for p in obs['photos']:
            if p['split']=='validation':p['q_rad'][0]=0
        with self.assertRaisesRegex(ValueError,'DISTINCT_J1'):coverage(obs,'validation')

    def test_duplicate_ids_or_touch_points_cannot_inflate_coverage(self):
        for duplicate_point in [False,True]:
            obs,_,_=fixture();row=copy.deepcopy(obs['touches'][0])
            if duplicate_point:row['sample_id']='different_id_same_point'
            obs['touches'].append(row)
            with self.assertRaisesRegex(ValueError,'DUPLICATE'):validate_observations(obs)
        obs,_,_=fixture();obs['photos'].append(copy.deepcopy(obs['photos'][0]))
        with self.assertRaisesRegex(ValueError,'DUPLICATE_SAMPLE'):validate_observations(obs)

    def test_collinear_heldout_datums_rejected(self):
        obs,_,_=fixture()
        for i,p in enumerate(obs['board']['touch_points']):p['p_P_m']=[i*.1,0,0]
        with self.assertRaisesRegex(ValueError,'COLLINEAR'):coverage(obs,'validation')

    def test_missing_training_evidence_and_failed_result_rejected(self):
        obs,result,_=fixture()
        for key in TRAINING_CHECKS:
            bad=copy.deepcopy(result);bad['training_checks'].pop(key)
            with self.assertRaisesRegex(ValueError,'TRAINING_EVIDENCE'):validate_result(bad,obs)
        for status in ['SIMULATION_VALIDATION_FAILED','SOLVE_FAILED','FAIL']:
            bad=copy.deepcopy(result);bad['status']=status
            with self.assertRaisesRegex(ValueError,'NOT_ELIGIBLE'):validate_result(bad,obs)

    def test_cli_numeric_failure_has_nonzero_exit_and_separate_checks(self):
        obs,result,_=fixture()
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);(p/'observations.json').write_text(json.dumps(obs));(p/'result.json').write_text(json.dumps(result))
            truth=np.eye(4);truth[0,3]=.02
            (p/'truth.json').write_text(json.dumps(dict(T_L_C=truth.tolist(),perturbation=np.eye(4).tolist())))
            env=dict(os.environ,PYTHONPATH=os.pathsep.join(str(ROOT/d) for d in ['src','tools','tests']))
            run=subprocess.run([sys.executable,'-c','from test_touch_calibration_stage11 import mocked_cli; mocked_cli()',
                '--observations',str(p),'--result',str(p/'result.json'),'--truth',str(p/'truth.json'),'--output',str(p/'output')],env=env,capture_output=True,text=True)
            self.assertEqual(run.returncode,1,run.stdout+run.stderr)
            r=json.loads((p/'output/validation.json').read_text());self.assertEqual(r['status'],'FAIL')
            self.assertEqual(r['checks']['simulation_truth_error']['status'],'FAIL')
            self.assertEqual(r['checks']['held_out_point_error']['status'],'PASS')
            self.assertEqual(r['checks']['training_touch_registration']['status'],'PASS')


class DistanceControlTests(unittest.TestCase):
    def test_nonfinite_and_backend_exceptions_fail_closed(self):
        for value in [float('nan'),float('inf'),-float('inf')]:
            r=distance_evidence(lambda:value,.0024,.0026);self.assertFalse(r['accepted']);self.assertEqual(r['error'],'NONFINITE_DISTANCE')
        def broken():raise RuntimeError('query failed')
        self.assertEqual(distance_evidence(broken,.0024,.0026)['error'],'DISTANCE_BACKEND_EXCEPTION')

    def test_native_clearance_rejects_old_point_one_mm_pass(self):
        a=dict(owner='link_3',kind='robot');b=dict(owner='board_body',kind='fixture')
        rule=pair_rule(a,b,set());self.assertEqual(rule['threshold_m'],.0024)
        self.assertTrue(distance_evidence(lambda:.001,.0001,.0026)['accepted'])
        self.assertFalse(distance_evidence(lambda:.001,rule['threshold_m'],.0026)['accepted'])
        self.assertEqual(pair_rule(a,dict(owner='link_6',kind='robot'),set())['threshold_m'],.0024)

    def test_designated_contact_is_not_rescued_by_large_distance(self):
        self.assertFalse(distance_evidence(lambda:.0026,-.0001,.0026,dict(accepted=False))['accepted'])
        self.assertFalse(distance_evidence(lambda:float('nan'),-.0001,.0026,dict(accepted=True))['accepted'])
        self.assertTrue(distance_evidence(lambda:-.0049,-.0001,.0026,dict(accepted=True))['accepted'])

    def test_truncated_query_is_explicit(self):
        result=distance_evidence(lambda:.0026,.0024,.0026)
        self.assertTrue(result['possibly_truncated']);self.assertTrue(result['accepted'])


class SceneSourceControlTests(unittest.TestCase):
    def test_new_snapshot_old_manifest_rejected(self):
        s=dict(fingerprint='new',config_sha256={},scene=dict(T_W_D=np.eye(4).tolist(),base_position_m=[0,0,0]))
        m=dict(source_snapshot_fingerprint='old',source_snapshot_sha256='abc')
        with self.assertRaisesRegex(ValueError,'SNAPSHOT_GEOMETRY'):validate_manifest_identity(s,'abc',m)

    def test_actual_usd_reference_hash_mismatch_rejected(self):
        try:from pxr import Usd,UsdGeom
        except ImportError:self.skipTest('USD unavailable; exercised in existing Linux environment')
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);stage=Usd.Stage.CreateNew(str(p/'old.usda'));UsdGeom.Xform.Define(stage,'/Workstation');stage.GetRootLayer().Save()
            scene=Usd.Stage.CreateNew(str(p/'composed.usda'));UsdGeom.SetStageUpAxis(scene,'Z');UsdGeom.SetStageMetersPerUnit(scene,1)
            UsdGeom.Xform.Define(scene,'/World/Workstation').GetPrim().GetReferences().AddReference('old.usda','/Workstation');scene.GetRootLayer().Save()
            s=dict(fingerprint='new',config_sha256={},scene=dict(T_W_D=np.eye(4).tolist(),base_position_m=[0,0,0]));(p/'snapshot.json').write_text(json.dumps(s))
            m=dict(source_snapshot_fingerprint='new',source_snapshot_sha256=hashlib.sha256((p/'snapshot.json').read_bytes()).hexdigest(),
                source_config_sha256={},T_W_D=np.eye(4).tolist(),base_position_D_m=[0,0,0],workstation_sha256='different')
            (p/'manifest.json').write_text(json.dumps(m))
            with self.assertRaisesRegex(ValueError,'USD_GEOMETRY_MANIFEST'):
                bind_scene_sources(p/'snapshot.json',p/'composed.usda',p/'manifest.json',root=p)


if __name__=='__main__':unittest.main()
