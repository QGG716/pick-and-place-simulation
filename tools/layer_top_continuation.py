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
LOWPROFILE_TOP_SCOPE='XY_TOP_LOWPROFILE250_BOTTOM_TWO_TOP'
LOWPROFILE_RECORDS={
    'summary.json':'21dc616dd94d31365d963e383822af7737cd3293b322d3feba6b701ace2fa6c3',
    'protocol.json':'7cdeab4bfdd3122e2283f71deaf4d39781160256d3a66ec206423aec9d1ab78c',
    'verification.json':'17bc81fb8818186c348b0f0e2695b1c8249417c4ad079b747b05810f71bf3652',
}
L01_TOOL_RECORDS={
    'summary.json':'235718f12be39fe7490acebc2b90a083b9a0e1961440187f462124b73c7f98ab',
    'protocol.json':'dbf902f9b56e31e7c4c51e6c4851759aa7ee5e773e3acfd0e64a17e1a52a45bc',
    'verification.json':'21f2dd959a1c21ff9770d0daf011b45af1ef561731e2d3ef691b7d7424982a38',
}
L01_HMAX_RECORDS={
    'summary.json':'84af5576e3cd1beef022026372ef45431dcc2d58e66fc2c9ed8f134c5480a3e4',
    'protocol.json':'8cac21080c964f887dd02f66411d783a6c00dc8319627e032b27a9f1c0296b9f',
    'verification.json':'59a2985e348a76cdc11fe89386bf29da01033993ee89d306ec3280c6122ed73a',
}
BOTTOM_CURRENT_RECORDS={
    'summary.json':'0b4cfa27d578176f1f776a19fa91d672db0931ea72fcdfec141fd0d5c1b553cb',
    'protocol.json':'4eb15000c07ccf00231833a9fd6a473cde6c80550f9fa776a20c07b8859b2eeb',
    'verification.json':'7a1177c3ac3e1a25f651292ecfa9fbab46e56effb769f5cbf5eb6ebb00c233a9',
}
BOTTOM_ROTATED_RECORDS={
    'summary.json':'26efbf6ee825b6c0033781d51a0b4fbe012f8b1157e2ad6e581243722ba86a7b',
    'protocol.json':'fa4b0dbd1b9a1d35ab5f573fa2e08cbba2277217d73b4d0c946b08a273ab0084',
    'verification.json':'3e8f3b80cc19dc06c24985970a9e89ec506521e27738c9b42f8e9820b4070903',
}


def restore_c03_prefix(run,state):
    """Same checked 37-box root for c04 face comparisons; read only c03 result."""
    return _restore_terminal(run,state,lowprofile_top=True,bottom_prefix=True)


def restore_bottom_current_terminal(run,state):
    """Restore only the checked c01/c02 commits, with a fresh native root."""
    return _restore_terminal(run,state,lowprofile_top=True,bottom_current=True)


def restore_l01_hmax_terminal(run,state):
    """Retain c01/c00 completions; no failed c04 q, path or native tree."""
    return _restore_terminal(run,state,lowprofile_top=True,l01_hmax=True)


def restore_l01_tool_terminal(run,state):
    return _restore_terminal(run,state,lowprofile_top=True,l01_tool=True)


def restore_layer_top_terminal(run,state):
    return _restore_terminal(run,state,lowprofile_top=False)


def restore_lowprofile_top_terminal(run,state):
    """Use the actual 30-box terminal, never an old top-grasp solution hint."""
    return _restore_terminal(run,state,lowprofile_top=True)


def _restore_terminal(run,state,*,lowprofile_top,l01_tool=False,l01_hmax=False,bottom_current=False,bottom_prefix=False):
    records=BOTTOM_ROTATED_RECORDS if bottom_prefix else BOTTOM_CURRENT_RECORDS if bottom_current else L01_HMAX_RECORDS if l01_hmax else L01_TOOL_RECORDS if l01_tool else LOWPROFILE_RECORDS if lowprofile_top else RECORDS
    completed=1 if bottom_prefix else 2 if lowprofile_top else 5
    verified_completed=3 if bottom_prefix else completed
    history=36 if bottom_prefix else 34 if bottom_current else 32 if l01_hmax else 30 if l01_tool else 28 if lowprofile_top else 23
    for name,sha in records.items():
        if hashlib.sha256((run/name).read_bytes()).hexdigest()!=sha:
            raise ValueError('LAYER_TOP_PREDECESSOR_RECORD_CHANGED')
    read=lambda n:json.loads((run/n).read_text(encoding='utf-8'))
    summary,protocol,verification=(read(n) for n in records)
    if (summary['source_commit']!=('a4552c363c0e2cce56be4d4b28958dd44ebbe3e8' if bottom_prefix else 'b05652ea3267863acd12a87b50b3f21e8bf20293' if bottom_current else '163eb5ec25f348263957e9e0764b43c8eb93ee5d' if l01_hmax else 'e9dfef3215b55bb9905ed0671d43ca4b9716a71a' if l01_tool else 'f17d636ed547a86389150f268445d44bae6f58c4' if lowprofile_top else '3f449e89da518ac89b3c112b4a83eb9a83000b11')
            or protocol['scope']!=('XY_TOP_CURRENT_BOTTOM_ROTATED_C03_C04' if bottom_prefix else 'XY_TOP_L00_FOUR_TOP_EXTENSION200' if bottom_current else 'XY_TOP_L01_ALL_HMAX_THREE_TARGETS' if l01_hmax else 'XY_TOP_LOWPROFILE250_L01_TOOL_CHASSIS' if l01_tool else 'XY_TOP_LOWPROFILE250_CONTINUATION' if lowprofile_top else 'XY_TOP_MIN250_CONTINUATION')
            or protocol['mounting_variant']!=('x_minus_300_y_minus_100' if bottom_current or bottom_prefix else 'x_minus_500_y_minus_100')
            or verification['completed_count']!=verified_completed or not verification['status'].startswith('PASS_')
            or summary['historical_completed_count']!=history or summary['completed_count']!=verified_completed
            or protocol['original_cartons']!=[box_message(b) for b in state.cartons]):
        raise ValueError('LAYER_TOP_PREDECESSOR_SCOPE_CHANGED')
    original={b.name:b for b in state.cartons};initial=summary['initial']
    state.q=np.asarray(initial['q_rad']);state.height=initial['height_m']
    state.cartons=tuple(original[b['id']] for b in initial['world_cartons'])
    state.ledger=deepcopy(summary['historical_ledger']);state.cycles=[]
    if state.root()!=initial:raise ValueError('LAYER_TOP_PREDECESSOR_INITIAL_CHANGED')
    receipts=[]
    outcomes=summary['outcomes'][:1] if bottom_prefix else summary['outcomes']
    if bottom_prefix and (outcomes[0]['target_id']!='carton_l00_c03' or outcomes[0]['completed_count']!=1):
        raise ValueError('C03_COMPARISON_PREFIX_CHANGED')
    for outcome in outcomes:
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
    terminal=outcomes[-1]['terminal'] if bottom_prefix else summary['terminal']
    if state.root()!=terminal or state.ledger!=summary['ledger'][:history+completed] or len(state.cartons)!=40-history-completed:
        raise ValueError('LAYER_TOP_PREDECESSOR_FINAL_CHANGED')
    prior=deepcopy(state.root());state.cycles=[]
    return dict(source_commit=summary['source_commit'],records_sha256=records,outcomes=receipts,
        archived_root=prior,new_native_root=state.root(),historical_completed_count=history+completed,
        first_target_id='carton_l01_c04' if l01_hmax else 'carton_l01_c01' if l01_tool else 'carton_l01_c02' if lowprofile_top else 'carton_l02_c01',tree_retained=False,no_historical_solution_hints=True,
        search_operation='NEW_NATIVE_ROOT_C04_SIDE_NEG_Y_HMAX' if l01_hmax else 'NEW_NATIVE_ROOT_WITH_SELECTED_SIDE_TOP_ROUTES' if l01_tool else 'NEW_NATIVE_ROOT_WITH_SELECTED_LAYER_TOP_PERMISSION')
