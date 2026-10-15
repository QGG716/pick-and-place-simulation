"""CPU contract substitutes only. No model accuracy claims, network or CUDA."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
from unloading_perception.proposals import DetectorConfig, filter_candidates, fingerprint, validate_detector_document

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))


def test_non_square_official_api_coordinates():
    from grounding_dino_runtime import official_postprocess
    class Processor:
        def post_process_grounded_object_detection(self, outputs, ids, *, threshold, text_threshold, target_sizes):
            assert target_sizes == [(200, 400)] and threshold == 0 and text_threshold == .25
            h,w=target_sizes[0]
            return [dict(boxes=np.array([[.1*w,.2*h,.9*w,.8*h]]),text_labels=['cardboard box'],scores=[.8])]
    result=official_postprocess(Processor(),None,None,height=200,width=400,text_threshold=.25)
    assert result['boxes'].tolist()==[[40,40,360,160]]


def test_clipping_invalid_empty_and_conservative_duplicates():
    boxes=[[-2,-3,40,30],[0,0,40,30],[30,0,70,30],[0,0,0,3],[np.nan,1,2,3],[0,1,np.inf,4],[101,0,120,20]]
    rows,kept=filter_candidates(boxes,[.9,.8,.7,.8,.8,.8,.8],['box']*7,100,50,DetectorConfig())
    assert [r['proposal_id'] for r in kept]==[1,3]
    assert kept[0]['clipped'] and kept[0]['bbox_xyxy']==[0,0,40,30]
    assert rows[1]['reason']=='NEAR_IDENTICAL_BOX'
    assert rows[-1]['reason']=='DEGENERATE_AFTER_CLIP'
    assert filter_candidates([],[],[],100,50,DetectorConfig())==([],[])
    json.dumps(rows,allow_nan=False)


def test_explicit_candidate_limit_is_not_an_object_count_prior():
    rows,kept=filter_candidates([[0,0,10,10],[10,0,20,10]],[.9,.8],['box']*2,100,50,DetectorConfig(max_candidates=1))
    assert len(kept)==1 and rows[1]['reason']=='CANDIDATE_LIMIT'


def test_one_to_one_matching_no_duplicate_gt_credit():
    from evaluate_detector_sam import match_boxes
    p=[dict(id=i,bbox=[0,0,10,10],detector_score=.9-i*.1) for i in (1,2)]
    truth=[dict(simulation_object_id='g',bbox_xyxy=[0,0,10,10])]
    assert match_boxes(p,truth)==[dict(proposal_id=1,gt_id='g',bbox_iou=1.)]


@pytest.fixture
def rgb_folder(tmp_path):
    from workcell_once_fakes import capture_fixture
    from unloading_perception.rgb_input import load_rgb_input,snapshot_rgb_input
    capture,_,modules=capture_fixture(tmp_path,('normal','normal'))
    source=capture/'FULL_STACK_NOMINAL/modules'/modules[0]
    payload=load_rgb_input(source)
    folder=tmp_path/'rgb-only';snapshot_rgb_input(payload,folder)
    assert {p.name for p in folder.iterdir()}=={'sensor_rgb.png','camera_info.json','capture_metadata.json','capture_binding.json'}
    return folder


class CpuRuntime:
    """Explicit synthetic boundary; production proposal/SAM orchestration is real."""
    def __init__(self, args):
        self.args=args; self.calls=0
        self.moge_model=None
        config=DetectorConfig(**json.loads(args.detector_config))
        self.detector=SimpleNamespace(identity=dict(repository='CPU_TEST_SUBSTITUTE',revision='test',config=config.to_dict(),
            config_identity=fingerprint(config.to_dict())),cold_start_seconds=0.)
    def detect(self,rgb):
        self.calls+=1
        rows,kept=filter_candidates([[0.,0.,3.,2.]],[.9],['cardboard box'],3,2,DetectorConfig())
        return dict(candidates=rows,final=kept,raw=dict(normalized_cxcywh=np.zeros((1,4))),model_input_shape=[1,3,2,3],timings_seconds={})
    def infer(self,request):
        from vision_resident_worker import ResidentRuntime
        # Actual worker request validation, path ownership and artifact assembly.
        runtime=ResidentRuntime.__new__(ResidentRuntime)
        runtime.args=self.args;runtime.proposal=None;runtime.output_root=self.args.output_root
        runtime.output_root.mkdir(parents=True,exist_ok=True)
        runtime.upstream_commit='1d208f2ed380a207e6e46b4a62d2ac640edfe477'
        runtime.torch=SimpleNamespace(cuda=SimpleNamespace(reset_peak_memory_stats=lambda:None))
        def sam(source,proposal,image,masks,instances,log,timings,samples):
            from PIL import Image
            p=json.loads(proposal.read_text())['instances'][0]
            # Proposal 1 -> SAM instance/mask 9, intentionally different.
            row=dict(**{k:v for k,v in p.items() if k!='instance_id'},instance_id=9,mask_area=6,
                sam_iou_score=.88,sam_prompt_stability=.95,validation_score=.8,boundary_source='CPU_TEST_SUBSTITUTE')
            np.savez_compressed(masks,masks=np.ones((1,2,3),bool),mask_ids=[9],boxes=[p['bbox']],scores=[.8],labels=['box'],sources=['CPU_TEST_SUBSTITUTE'])
            instances.write_text(json.dumps(dict(source=str(source),box_source=str(proposal),instances=[row],rejected=[],
                proposal_audit=[dict(proposal_id=p['id'],status='accepted',instance_id=9)])))
            Image.open(source).save(image);log.write_text('CPU_TEST_SUBSTITUTE');timings['sam']=0.
        runtime._sam=sam
        runtime._stage=lambda *a,**k:pytest.fail('geometry must not run')
        return runtime.infer(request)


def test_actual_rgb_only_entry_never_opens_gt_and_builds_review(rgb_folder,tmp_path,monkeypatch):
    from run_workcell_perception_once import main
    from run_detector_sam_review import forbid_truth_reads
    import unloading_perception.finite_sequence as finite
    import build_sam_review
    monkeypatch.setattr(finite,'verify_sam_files',lambda models:None)
    monkeypatch.setattr(build_sam_review,'contours',lambda mask:[[[0,0],[2,0],[2,1],[0,1]]])
    models=tmp_path/'models.json';models.write_text(json.dumps(dict(sam=dict(snapshot_path='CPU_TEST_SUBSTITUTE'))))
    output=tmp_path/'out'
    with forbid_truth_reads():
        code=main(['--capture',str(rgb_folder),'--models',str(models),'--vision',str(tmp_path),'--output-directory',str(output),'--stop-after','sam'],runtime_factory=CpuRuntime)
    summary=json.loads((output/'summary.json').read_text())
    assert code==0,summary
    row=summary['runs'][0];folder=Path(row['artifact_directory'])
    doc=json.loads((folder/'detector_proposals.json').read_text())
    assert doc['source']=='GROUNDING_DINO' and 'simulation_object_id' not in doc['instances'][0]
    review=json.loads((folder/'sam_review.json').read_text())
    assert review['proposals'][0]['proposal_id']==1 and review['proposals'][0]['mask_id']==9
    assert review['evaluation'] is None and all(s=='NOT_RUN' for s in summary['stages'].values())
    assert not (folder/'oracle_proposals.json').exists()


@pytest.mark.parametrize('field,value',[('frame_sequence',999),('rgb_sha256','f'*64),('source_size',[2,3]),('capture_id','other'),('module_id','other')])
def test_detector_provenance_rejects_cross_bindings(rgb_folder,field,value):
    from perception_sam_pipeline import prepare_proposals
    from unloading_perception.rgb_input import load_rgb_input
    rgb=load_rgb_input(rgb_folder)
    runtime=CpuRuntime(SimpleNamespace(detector_config='{}'))
    doc,_=prepare_proposals(runtime,rgb_folder,mode='grounding-dino')
    doc[field]=value
    with pytest.raises(ValueError,match='BINDING'):
        validate_detector_document(doc,rgb_sha256=rgb.rgb_sha256,width=3,height=2,capture_id=rgb.metadata.capture_id,
            module_id=rgb.camera['module_id'],sensor_epoch=rgb.metadata.sensor_epoch,frame_sequence=rgb.metadata.frame_sequence)


def test_detector_config_binding_and_failure_no_oracle(rgb_folder):
    from perception_sam_pipeline import prepare_proposals
    from unloading_perception.rgb_input import load_rgb_input
    rgb=load_rgb_input(rgb_folder);runtime=CpuRuntime(SimpleNamespace(detector_config='{}'))
    doc,_=prepare_proposals(runtime,rgb_folder,mode='grounding-dino')
    doc['detector']['config']['text']='another box.'
    with pytest.raises(ValueError,match='CONFIG'):
        validate_detector_document(doc,rgb_sha256=rgb.rgb_sha256,width=3,height=2,capture_id=rgb.metadata.capture_id,
            module_id=rgb.camera['module_id'],sensor_epoch=rgb.metadata.sensor_epoch,frame_sequence=rgb.metadata.frame_sequence)
    def fail(_):raise RuntimeError('MODEL_MISSING')
    runtime.detect=fail
    with pytest.raises(RuntimeError,match='MODEL_MISSING'):prepare_proposals(runtime,rgb_folder,mode='grounding-dino')
    assert not (rgb_folder/'oracle_proposals.json').exists()


def test_no_simulation_id_rejected_and_merged_lineage(tmp_path):
    from test_instance_lineage import fixture,load
    p,g=fixture(tmp_path,proposal_ids=(10,30,70,80),kept=(30,70),supporting={30:[10]})
    for row in p['instances']:row.pop('simulation_object_id')
    result=load(tmp_path,p,g)
    assert result[1].sam['proposal_id']==30 and result[1].audit['supporting_proposal_ids']==[10]
    assert result[1].audit['evaluation_entity_ids']==[] and 80 not in [r.proposal['id'] for r in result.values()]


def test_resident_detector_loaded_once(monkeypatch):
    from vision_resident_worker import ResidentRuntime
    import grounding_dino_runtime
    loads=[]
    class Detector:
        def __init__(self,m,c):loads.append(1)
        def detect(self,rgb):return id(self)
    monkeypatch.setattr(grounding_dino_runtime,'GroundingDinoRuntime',Detector)
    monkeypatch.setattr(Path,'read_text',lambda *a,**k:'{}')
    runtime=ResidentRuntime.__new__(ResidentRuntime);runtime.detector=None
    runtime.args=SimpleNamespace(detector_manifest=Path('fake'),detector_config='{}')
    assert runtime.detect(None)==runtime.detect(None) and loads==[1]


def test_empty_detection_is_legal_and_skips_sam(rgb_folder,tmp_path):
    from perception_sam_pipeline import prepare_proposals,infer_sam
    from vision_resident_worker import parser,ResidentRuntime
    args=parser().parse_args(['--upstream-root',str(tmp_path),'--output-root',str(tmp_path/'runs'),
        '--input-root',str(tmp_path),'--sam-model','CPU_TEST_SUBSTITUTE'])
    runtime=CpuRuntime(args)
    runtime.detect=lambda rgb:dict(candidates=[],final=[],raw={},model_input_shape=[1,3,2,3],timings_seconds={})
    prepare_proposals(runtime,rgb_folder,mode='grounding-dino')
    worker=ResidentRuntime.__new__(ResidentRuntime);worker.args=args;worker.proposal=None
    worker.output_root=args.output_root;worker.output_root.mkdir()
    worker.upstream_commit='1d208f2ed380a207e6e46b4a62d2ac640edfe477'
    worker.torch=SimpleNamespace(cuda=SimpleNamespace(reset_peak_memory_stats=lambda:None))
    worker._sam=lambda *a,**k:pytest.fail('SAM called for empty detection')
    worker._stage=lambda *a,**k:pytest.fail('geometry called after stop sam')
    response=infer_sam(worker,rgb_folder,'empty',mode='grounding-dino',stop_after='sam')
    metrics=json.loads(Path(response['metrics_reference']['path']).read_text())
    with np.load(metrics['artifacts']['cargo_masks.npz']['path']) as data:
        assert data['masks'].shape==(0,2,3) and data['boxes'].shape==(0,4)
    assert metrics['config']['geometry_2d']=='NOT_RUN'


@pytest.mark.parametrize('evaluate_gt', [False, True])
def test_default_full_entry_uses_real_detector_sam_stages_before_geometry(tmp_path,monkeypatch,evaluate_gt):
    from workcell_once_fakes import capture_fixture
    from run_workcell_perception_once import main
    import workcell_geometry_process,build_sam_review
    capture,models,modules=capture_fixture(tmp_path,('normal','normal'))
    import unloading_perception.finite_sequence as finite
    monkeypatch.setattr(finite,'verify_sam_files',lambda models:None)
    monkeypatch.setattr(build_sam_review,'contours',lambda mask:[])
    reached=[];evaluated=[]
    import evaluate_detector_sam
    def evaluate(source, folder, artifacts):
        assert (folder/'detector_proposals.json').exists()
        assert Path(artifacts['cargo_masks.npz']['path']).exists()
        evaluated.append(folder.name)
        return {'test_evaluation':'prediction artifacts already frozen'}
    monkeypatch.setattr(evaluate_detector_sam,'evaluate',evaluate)
    class FullRuntime(CpuRuntime):
        def infer(self,request):
            # CPU substitute still exercises actual request/proposal validation.
            response=super().infer({**request,'stop_after':'sam'})
            ref=response['metrics_reference'];path=Path(ref['path']);metrics=json.loads(path.read_text())
            face=path.parent/'box_geometry_2d.json';face.write_text('{"instances":[]}')
            metrics['artifacts'][face.name]=dict(path=str(face),sha256=hashlib.sha256(face.read_bytes()).hexdigest())
            path.write_text(json.dumps(metrics));ref['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
            return response
    def geometry(**kwargs):
        folder=kwargs['module_dir'];p=json.loads((folder/'detector_proposals.json').read_text())
        assert p['source']=='GROUNDING_DINO' and 'simulation_object_id' not in p['instances'][0]
        reached.append(folder.name)
        raise RuntimeError('CPU_TEST_STOP_AT_UNCHANGED_GEOMETRY_BOUNDARY')
    monkeypatch.setattr(workcell_geometry_process,'dispatch_geometry',geometry)
    output=tmp_path/'out'
    assert main(['--capture',str(capture),'--models',str(models),'--vision',str(tmp_path),
        '--output-directory',str(output)] + (['--evaluate-gt'] if evaluate_gt else []),runtime_factory=FullRuntime)==1
    assert reached==modules
    assert evaluated==(modules if evaluate_gt else [])
    summary=json.loads((output/'summary.json').read_text())
    assert all(r['failure_stage']=='metric_geometry' for r in summary['runs'])


@pytest.mark.parametrize('damage',['rgb','dimensions','frame'])
def test_rgb_input_rejects_bad_sensor_binding(rgb_folder,damage):
    from unloading_perception.rgb_input import load_rgb_input
    if damage=='rgb':(rgb_folder/'sensor_rgb.png').write_bytes(b'corrupt')
    if damage=='dimensions':
        path=rgb_folder/'camera_info.json';d=json.loads(path.read_text());d['width']=7;path.write_text(json.dumps(d))
    if damage=='frame':
        path=rgb_folder/'capture_binding.json';d=json.loads(path.read_text());d['frame_sequence']+=1;path.write_text(json.dumps(d))
    with pytest.raises(ValueError):load_rgb_input(rgb_folder)


def test_review_keeps_merged_rejected_and_no_mask_proposals(rgb_folder,monkeypatch):
    import build_sam_review
    monkeypatch.setattr(build_sam_review,'contours',lambda mask:[])
    proposal=rgb_folder/'detector_proposals.json'
    document=dict(source='GROUNDING_DINO',source_size=[3,2],instances=[
        dict(id=i,proposal_id=i,bbox=[0.,0.,3.,2.],label='box',detector_score=.8) for i in (10,30,70,80)])
    proposal.write_text(json.dumps(document))
    (rgb_folder/'proposal_binding.json').write_text(json.dumps(dict(filename=proposal.name,sha256=hashlib.sha256(proposal.read_bytes()).hexdigest())))
    row=dict(instance_id=9,proposal_id=30,supporting_proposal_ids=[10],bbox=[0.,0.,3.,2.],label='box',
        mask_area=6,boundary_source='CPU_TEST_SUBSTITUTE',sam_iou_score=.9,sam_prompt_stability=.95,validation_score=.8)
    record=dict(source=str(rgb_folder/'sensor_rgb.png'),box_source=str(proposal),instances=[row],
        rejected=[dict(proposal_id=10,rejection_reason='duplicate',sam_iou_score=.7),dict(proposal_id=80,rejection_reason='person')],
        proposal_audit=[dict(proposal_id=10,status='merged_duplicate',accepted_proposal_id=30),
            dict(proposal_id=30,status='accepted',instance_id=9),dict(proposal_id=70,status='no_mask'),dict(proposal_id=80,status='rejected_person')])
    instances=rgb_folder/'cargo_instances.json';instances.write_text(json.dumps(record))
    masks=rgb_folder/'cargo_masks.npz'
    np.savez(masks,masks=np.ones((1,2,3),bool),mask_ids=[9],boxes=[row['bbox']],labels=['box'],scores=[.8],sources=['CPU_TEST_SUBSTITUTE'])
    build_sam_review.build_review(rgb_folder,{'cargo_instances.json':dict(path=str(instances)),'cargo_masks.npz':dict(path=str(masks))})
    review=json.loads((rgb_folder/'sam_review.json').read_text())
    assert [r['mask_id'] for r in review['proposals']]==[9,9,None,None]
    assert [r['status'] for r in review['proposals']]==['merged_duplicate','accepted','no_mask','rejected_person']
    assert review['proposals'][0]['original_candidate_metrics']['sam_iou_score']==.7
    assert review['proposals'][0]['sam_iou_score']==.9 and review['evaluation'] is None
