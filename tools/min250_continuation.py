"""Restore the verified x500 population terminal as a new native root."""
from copy import deepcopy
import hashlib
import json
import tarfile

SCOPE = 'XY_TOP_MIN250_CONTINUATION'
PREVIOUS_SOURCE = '66604f4fd4a346be8d1947f3d3d44c544a5ad407'
PREVIOUS_RECORDS = {
    'summary.json':'1f09a5814fc672891c8adfde325db44fb9c355f9a4668e9bbd618d808e569597',
    'protocol.json':'8860c78b1edc8a5ce58b211949e6ea36ea8ae85bb25175a78fa8b5b0961b0556',
    'verification.json':'85b42869624fe4f8523b7830a07f094d928295642261a0001d248964605569e4',
}
REPAIR_RECORDS = {
    'summary.json':'49bce0c472db34152b06739bb82f7fe60c8c440e1a67771aeab56e2b7e14000f',
    'protocol.json':'3f80a1c68cc5bd2cdfcdf322df20aee2e6a2d7f20d9244e6b3d5955ba7d103a9',
    'prefix-verification.json':'92428217b65851cdc6d3bec8c632572bc5eb6c59e19681455ccbf57d99229fe1',
}


def restore_population_terminal(run, state, mounting_variant):
    if any(hashlib.sha256((run/n).read_bytes()).hexdigest()!=sha for n,sha in PREVIOUS_RECORDS.items()):
        raise ValueError('MIN250_VERIFIED_PREDECESSOR_RECORD_CHANGED')
    read = lambda name: json.loads((run/name).read_text(encoding='utf-8'))
    protocol, summary, verification = (read(n) for n in ('protocol.json','summary.json','verification.json'))
    if (mounting_variant != 'x_minus_500_y_minus_100'
            or protocol['mounting_variant'] != mounting_variant
            or protocol['scope'] != 'XY_TOP_NEW_WORLD_40'
            or protocol['source_commit'] != summary['source_commit'] or summary['source_commit'] != PREVIOUS_SOURCE
            or summary['status'] != 'INCOMPLETE_RETAINED' or summary['historical_completed_count'] != 0
            or summary['completed_count'] != 20 or summary['target_count'] != 40
            or verification['source_commit'] != PREVIOUS_SOURCE or verification['completed_count'] != 20
            or not verification['status'].startswith('PASS_') or summary['initial'] != state.root()):
        raise ValueError('MIN250_PREDECESSOR_MISMATCH')
    receipts=[]
    for outcome in summary['outcomes']:
        archive=run/outcome['evidence']['archive']
        prefix=archive.name.removesuffix('.tar.gz')
        if archive.exists():
            if hashlib.sha256(archive.read_bytes()).hexdigest()!=outcome['evidence']['sha256']:
                raise ValueError('MIN250_PREDECESSOR_ARCHIVE_CHANGED')
            with tarfile.open(archive) as tar:
                raw=tar.extractfile(prefix+'/result.json').read()
        else:
            path=(run/prefix/'result.json').resolve()
            if not path.is_relative_to(run.resolve()):raise ValueError('MIN250_PREDECESSOR_PATH_CHANGED')
            raw=path.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=outcome['evidence']['file_sha256']['result.json']:
            raise ValueError('MIN250_PREDECESSOR_RESULT_CHANGED')
        result=json.loads(raw)
        if (result['initial']!=state.root() or outcome['root']!=state.root()
                or result['completed_count']!=len(result['cycles'])
                or result['completed_count']!=outcome['completed_count']):
            raise ValueError('MIN250_PREDECESSOR_LINEAGE_CHANGED')
        for cycle in result['cycles']:state.commit(cycle)
        if state.root()!=result['terminal'] or state.root()!=outcome['terminal']:
            raise ValueError('MIN250_PREDECESSOR_TERMINAL_CHANGED')
        receipts.append(dict(target_id=outcome['target_id'],completed_count=result['completed_count'],
            archive_sha256=outcome['evidence']['sha256'],result_sha256=outcome['evidence']['file_sha256']['result.json']))
        state.cycles=state.cycles[-1:]
    if state.root()!=summary['terminal'] or state.ledger!=summary['ledger'] or len(state.cartons)!=20:
        raise ValueError('MIN250_PREDECESSOR_FINAL_STATE_CHANGED')
    prior_root=deepcopy(state.root())
    state.cycles=[]  # No historical trajectory, q candidate, attachment or parent enters the new worker.
    return dict(source_commit=PREVIOUS_SOURCE, outcomes=receipts,
        **{n.removesuffix('.json')+'_sha256':hashlib.sha256((run/n).read_bytes()).hexdigest()
           for n in ('summary.json','protocol.json','verification.json')},
        archived_root=prior_root,new_native_root=state.root(),historical_completed_count=20,
        first_target_id='carton_l03_c02',previous_failures_retained=True,
        search_operation='NEW_NATIVE_ROOT_WITH_CHANGED_MINIMUM_AND_FACE_POLICY',
        tree_retained=False,no_historical_solution_hints=True)


def restore_before_world_sync_repair(run,state):
    """Keep this round's three accepted cartons when restarting after the bug."""
    if any(hashlib.sha256((run/n).read_bytes()).hexdigest()!=sha for n,sha in REPAIR_RECORDS.items()):
        raise ValueError('MIN250_REPAIR_PREDECESSOR_CHANGED')
    summary=json.loads((run/'summary.json').read_text())
    verified=json.loads((run/'prefix-verification.json').read_text())
    if (summary['initial']!=state.root() or summary['completed_count']!=3
            or verified['status']!='PASS_SAVED_NATIVE_PREFIX_INTEGRATION_ERROR_RETAINED'
            or verified['summary_sha256']!=REPAIR_RECORDS['summary.json']):
        raise ValueError('MIN250_REPAIR_PREFIX_MISMATCH')
    receipts=[]
    for outcome in summary['outcomes']:
        if 'evidence' not in outcome:
            partial=run/outcome['folder']/'result.json'
            want=verified['outcomes'][-1]['uncommitted_failure_files_sha256']['result.json']
            if hashlib.sha256(partial.read_bytes()).hexdigest()!=want:
                raise ValueError('MIN250_REPAIR_ERROR_RECORD_CHANGED')
            result=json.loads(partial.read_text())
            if result['initial']!=state.root() or result['terminal']!=state.root() or result['cycles']:
                raise ValueError('MIN250_REPAIR_UNCOMMITTED_RESULT_CHANGED')
            continue
        archive=run/outcome['evidence']['archive']
        if archive.exists():
            if hashlib.sha256(archive.read_bytes()).hexdigest()!=outcome['evidence']['sha256']:
                raise ValueError('MIN250_REPAIR_ARCHIVE_CHANGED')
            with tarfile.open(archive) as tar:
                raw=tar.extractfile(archive.name.removesuffix('.tar.gz')+'/result.json').read()
        else:
            path=(run/archive.name.removesuffix('.tar.gz')/'result.json').resolve()
            if not path.is_relative_to(run.resolve()):raise ValueError('MIN250_REPAIR_PATH_CHANGED')
            raw=path.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=outcome['evidence']['file_sha256']['result.json']:
            raise ValueError('MIN250_REPAIR_RESULT_CHANGED')
        result=json.loads(raw)
        if result['initial']!=state.root() or outcome['root']!=state.root():
            raise ValueError('MIN250_REPAIR_ROOT_CHANGED')
        for cycle in result['cycles']:state.commit(cycle)
        if state.root()!=result['terminal'] or state.root()!=outcome['terminal']:
            raise ValueError('MIN250_REPAIR_TERMINAL_CHANGED')
        receipts.append(dict(target_id=outcome['target_id'],completed_count=result['completed_count'],
            result_sha256=outcome['evidence']['file_sha256']['result.json']))
        state.cycles=state.cycles[-1:]
    if state.root()!=summary['terminal'] or state.ledger!=summary['ledger'] or len(state.cartons)!=17:
        raise ValueError('MIN250_REPAIR_FINAL_STATE_CHANGED')
    prior_root=deepcopy(state.root());state.cycles=[]
    return dict(source_commit=summary['source_commit'],records_sha256=REPAIR_RECORDS,outcomes=receipts,
        archived_root=prior_root,new_native_root=state.root(),historical_completed_count=23,
        this_round_predecessor_completed_count=3,tree_retained=False,no_historical_solution_hints=True,
        search_operation='RESTART_AFTER_WORLD_SYNC_REPAIR_AT_CHECKED_COMMITTED_STATE')
