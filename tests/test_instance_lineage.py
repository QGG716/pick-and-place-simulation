import json

import numpy as np
import pytest

from unloading_perception.lineage import load_instance_lineage


def fixture(tmp_path, proposal_ids=(10, 30, 70), kept=(30, 70), supporting=None):
    proposals = {"instances": [dict(id=i, bbox=[0, 0, 4, 4], label="box", simulation_object_id=f"box-{i}") for i in reversed(proposal_ids)]}
    records, audit = [], []
    for instance, proposal in enumerate(kept, 1):
        record = dict(instance_id=instance, proposal_id=proposal, bbox=[0, 0, 4, 4], label="box", boundary_source="stable_multi_box_prompt_sam", validation_score=.8, sam_iou_score=.9, sam_prompt_stability=.99, mask_area=16)
        audit.append(dict(status="accepted", proposal_id=proposal, instance_id=instance))
        if supporting and proposal in supporting:
            record["supporting_proposal_ids"] = supporting[proposal]
            audit.extend(dict(status="merged_duplicate", proposal_id=i, accepted_proposal_id=proposal) for i in supporting[proposal])
        records.append(record)
    sam = dict(source=str(tmp_path/'sensor_rgb.png'), box_source=str(tmp_path/'oracle_proposals.json'), instances=records, proposal_audit=audit)
    (tmp_path/'cargo_instances.json').write_text(json.dumps(sam))
    # Deliberately shuffled NPZ rows: consumers must use mask_ids.
    np.savez(tmp_path/'cargo_masks.npz', mask_ids=np.arange(len(kept), 0, -1), masks=np.ones((len(kept),4,4), bool), boxes=np.tile([0,0,4,4],(len(kept),1)), scores=np.full(len(kept),.8), labels=np.full(len(kept),'box'), sources=np.full(len(kept),'stable_multi_box_prompt_sam'))
    return proposals, dict(instances=[dict(mask_id=i) for i in range(1,len(kept)+1)])


def load(tmp_path, proposals, geometry, module="upper", capture="capture"):
    return load_instance_lineage(tmp_path/'cargo_masks.npz', tmp_path/'cargo_instances.json', proposals, geometry, sensor_epoch="epoch", module_id=module, capture_id=capture, source_path=tmp_path/'sensor_rgb.png', proposal_path=tmp_path/'oracle_proposals.json')


@pytest.mark.parametrize("kept", [(10,70),(30,70),(70,)])
def test_reindexed_deleted_first_middle_and_noncontiguous(tmp_path, kept):
    p,g=fixture(tmp_path,kept=kept)
    result=load(tmp_path,p,g)
    assert [result[i].proposal['id'] for i in sorted(result)] == list(kept)
    assert result[1].audit['sam_iou_score'] == .9


def test_merge_retains_conflicting_entities_without_cloning(tmp_path):
    p,g=fixture(tmp_path,kept=(30,70),supporting={30:[10]})
    result=load(tmp_path,p,g)
    assert len(result)==2
    assert result[1].audit['identity_status']=='AMBIGUOUS_MERGED_ENTITIES'
    assert result[1].audit['supporting_proposal_ids']==[10]


def test_missing_geometry_and_same_mask_across_modules(tmp_path):
    p,g=fixture(tmp_path); g['instances'].pop()
    a=load(tmp_path,p,g); b=load(tmp_path,p,g,module='lower')
    assert a[1].identity != b[1].identity
    assert a[2].audit['geometry_status']=='MISSING'


@pytest.mark.parametrize('damage',['duplicate','missing','box','score','capture','source','audit','geometry','mask_area'])
def test_corrupt_mapping_fails_closed(tmp_path,damage):
    p,g=fixture(tmp_path)
    path=tmp_path/'cargo_instances.json'; sam=json.loads(path.read_text())
    if damage=='duplicate': sam['instances'].append(sam['instances'][0])
    elif damage=='missing': sam['instances'][0]['proposal_id']=99
    elif damage=='box': sam['instances'][0]['bbox']=[0,0,3,3]
    elif damage=='score': sam['instances'][0]['validation_score']=.2
    elif damage=='capture': sam['instances'][0]['capture_id']='another'
    elif damage=='source': sam['source']=str(tmp_path/'other_capture.png')
    elif damage=='audit': sam['proposal_audit']=[]
    elif damage=='geometry': g['instances'].append(dict(mask_id=999))
    elif damage=='mask_area': sam['instances'][0]['mask_area']=12
    path.write_text(json.dumps(sam))
    with pytest.raises(ValueError): load(tmp_path,p,g)
