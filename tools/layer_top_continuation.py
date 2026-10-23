"""Restore the checked 28-box prefix, clearing all historical native parents."""
from copy import deepcopy
import hashlib
import json
import tarfile
import numpy as np
from unloading_sim.moveit2_backend import box_message

SCOPE='XY_TOP_SELECTED_LAYER_CONTINUATION'
RECORDS={
    'summary.json':'3b00015dcb2c8dc99520b074f50c3ec53f7d0746f8faa1529786733915c96ef8',
    'protocol.json':'4e6fb5f87942cffdea803d3fb90a653c9ab23a29f1522fbd71f4992eb235a6f7',
    'verification.json':'122b40e44ea34448d1b7b51f9f57d25c545db1d4373faad29b19f123025a0f82',
}


def restore_layer_top_terminal(run,state):
    for name,sha in RECORDS.items():
        if hashlib.sha256((run/name).read_bytes()).hexdigest()!=sha:
            raise ValueError('LAYER_TOP_PREDECESSOR_RECORD_CHANGED')
    read=lambda n:json.loads((run/n).read_text(encoding='utf-8'))
    summary,protocol,verification=(read(n) for n in RECORDS)
    if (summary['source_commit']!='3f449e89da518ac89b3c112b4a83eb9a83000b11'
            or protocol['scope']!='XY_TOP_MIN250_CONTINUATION'
            or protocol['mounting_variant']!='x_minus_500_y_minus_100'
            or verification['completed_count']!=5 or not verification['status'].startswith('PASS_')
            or summary['historical_completed_count']!=23 or summary['completed_count']!=5
            or protocol['original_cartons']!=[box_message(b) for b in state.cartons]):
        raise ValueError('LAYER_TOP_PREDECESSOR_SCOPE_CHANGED')
    original={b.name:b for b in state.cartons};initial=summary['initial']
    state.q=np.asarray(initial['q_rad']);state.height=initial['height_m']
    state.cartons=tuple(original[b['id']] for b in initial['world_cartons'])
    state.ledger=deepcopy(summary['historical_ledger']);state.cycles=[]
    if state.root()!=initial:raise ValueError('LAYER_TOP_PREDECESSOR_INITIAL_CHANGED')
    receipts=[]
    for outcome in summary['outcomes']:
        receipt=outcome['evidence'];archive=run/receipt['archive'];prefix=archive.name.removesuffix('.tar.gz')
        if archive.exists():
            if hashlib.sha256(archive.read_bytes()).hexdigest()!=receipt['sha256']:
                raise ValueError('LAYER_TOP_PREDECESSOR_ARCHIVE_CHANGED')
            with tarfile.open(archive) as t:raw=t.extractfile(prefix+'/result.json').read()
        else:
            path=(run/prefix/'result.json').resolve()
            if not path.is_relative_to(run.resolve()):raise ValueError('LAYER_TOP_PREDECESSOR_PATH_CHANGED')
            raw=path.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=receipt['file_sha256']['result.json']:
            raise ValueError('LAYER_TOP_PREDECESSOR_RESULT_CHANGED')
        result=json.loads(raw)
        if result['initial']!=state.root() or result['completed_count']!=len(result['cycles']):
            raise ValueError('LAYER_TOP_PREDECESSOR_LINEAGE_CHANGED')
        for cycle in result['cycles']:state.commit(cycle)
        if state.root()!=result['terminal'] or state.root()!=outcome['terminal']:
            raise ValueError('LAYER_TOP_PREDECESSOR_TERMINAL_CHANGED')
        receipts.append(dict(target_id=outcome['target_id'],completed_count=result['completed_count'],
            result_sha256=receipt['file_sha256']['result.json']))
        state.cycles=state.cycles[-1:]
    if state.root()!=summary['terminal'] or state.ledger!=summary['ledger'] or len(state.cartons)!=12:
        raise ValueError('LAYER_TOP_PREDECESSOR_FINAL_CHANGED')
    prior=deepcopy(state.root());state.cycles=[]
    return dict(source_commit=summary['source_commit'],records_sha256=RECORDS,outcomes=receipts,
        archived_root=prior,new_native_root=state.root(),historical_completed_count=28,
        first_target_id='carton_l02_c01',tree_retained=False,no_historical_solution_hints=True,
        search_operation='NEW_NATIVE_ROOT_WITH_SELECTED_LAYER_TOP_PERMISSION')
