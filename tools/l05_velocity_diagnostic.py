"""Narrow, explicitly requested l05_c03 J4 velocity-only planning experiment."""
from copy import deepcopy
import hashlib
import json
import tarfile

TARGET = 'carton_l05_c03'
SCOPE = 'XY_TOP_L05_J4_VELOCITY_RESTART'


def restore_before_l05(run, state, mounting_variant):
    read = lambda name: json.loads((run/name).read_text())
    protocol, summary, verification = (read(n) for n in ('protocol.json','summary.json','verification.json'))
    if (mounting_variant != 'x_minus_400_y_minus_100'
            or protocol['mounting_variant'] != mounting_variant
            or protocol['scope'] != 'XY_TOP_NEW_WORLD_40'
            or summary['status'] != 'INCOMPLETE_RETAINED'
            or not verification['status'].startswith('PASS_')
            or summary['initial'] != state.root()):
        raise ValueError('L05_DIAGNOSTIC_PREDECESSOR_MISMATCH')
    receipts=[]
    for outcome in summary['outcomes']:
        if outcome['target_id'] == TARGET:
            if outcome['completed_count'] != 0 or outcome['root'] != state.root():
                raise ValueError('L05_DIAGNOSTIC_FAILURE_ROOT_MISMATCH')
            break
        archive=run/outcome['evidence']['archive']
        if hashlib.sha256(archive.read_bytes()).hexdigest()!=outcome['evidence']['sha256']:
            raise ValueError('L05_PREDECESSOR_ARCHIVE_CHANGED')
        with tarfile.open(archive) as tar:
            raw=tar.extractfile(archive.name.removesuffix('.tar.gz')+'/result.json').read()
        if hashlib.sha256(raw).hexdigest()!=outcome['evidence']['file_sha256']['result.json']:
            raise ValueError('L05_PREDECESSOR_RESULT_CHANGED')
        result=json.loads(raw)
        if result['initial']!=state.root() or result['completed_count']!=1 or len(result['cycles'])!=1:
            raise ValueError('L05_PREDECESSOR_PREFIX_NOT_COMPLETE')
        state.commit(result['cycles'][0])
        if state.root()!=result['terminal'] or state.root()!=outcome['terminal']:
            raise ValueError('L05_PREDECESSOR_TERMINAL_CHANGED')
        receipts.append(dict(target_id=outcome['target_id'],archive_sha256=outcome['evidence']['sha256'],
                             result_sha256=outcome['evidence']['file_sha256']['result.json']))
        state.cycles=state.cycles[-1:]
    else:
        raise ValueError('L05_PREDECESSOR_FAILURE_MISSING')
    if len(state.cartons)!=29 or len(state.ledger)!=11:
        raise ValueError('L05_PREDECESSOR_PREFIX_COUNT_CHANGED')
    prior_root=deepcopy(state.root())
    state.cycles=[]  # Legal scene/q only. No retained native tree or solver hints.
    return dict(source_commit=summary['source_commit'],prefix=receipts,
        summary_sha256=hashlib.sha256((run/'summary.json').read_bytes()).hexdigest(),
        protocol_sha256=hashlib.sha256((run/'protocol.json').read_bytes()).hexdigest(),
        verification_sha256=hashlib.sha256((run/'verification.json').read_bytes()).hexdigest(),
        archived_root=prior_root,new_native_root=state.root(),historical_completed_count=11,
        search_operation='RESTART_AT_LEGAL_COMMITTED_STATE_NOT_NATIVE_TREE_CONTINUATION',
        previous_failure_retained=True,no_historical_solution_hints=True)
