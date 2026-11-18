"""Pure deadline classification; no IO, waiting or permit mutation."""
import time

def snapshot(self,boxes,failures,selected,error,deadline=None):
    """Per-target searched / unsearched candidate counts under the unified deadline.

    A candidate is searched only when it was concluded: rejected (it has a
    recorded failure) or accepted. Candidates screened but not planned, in
    flight at expiry, or never reached are unsearched and are never counted
    as infeasible. A definite target-level verdict concludes every candidate
    of that target. Overlapping uncertainty envelopes do not establish that
    every underlying geometry hypothesis is infeasible; keep those unsearched.
    """
    from .planner import DEADLINE_REPLY_RESERVE_S
    targets=[]
    for box in boxes:
        if selected is not None and box['detection_index']!=selected:continue
        ident=box['id'];counts=self.search_counts.get(ident)
        level=[f for f in failures if f.get('target')==ident and 'candidate' not in f]
        unresolved=any(f['reason'].startswith('GEOMETRY_UNCERTAINTY_') for f in level)
        failed={f['candidate'] for f in failures if f.get('target')==ident and 'candidate' in f}
        if counts is None or counts['considered'] is None:
            # Candidates never generated: a target-level verdict, or not reached.
            status='GEOMETRY_UNRESOLVED' if unresolved else 'TARGET_REJECTED' if level else 'NOT_STARTED' if counts is None else (
                'BUDGET_EXHAUSTED_IN_CANDIDATE_GENERATION' if error is not None else 'CANDIDATE_GENERATION')
            targets.append(dict(target=ident,status=status,candidates=None,considered=None,screened=0,
                searched='ALL_BY_TARGET_VERDICT' if level and not unresolved else 0,
                unsearched=None if level and not unresolved else 'ALL_NOT_GENERATED',
                infeasible_candidates=0,target_reason=level[0]['reason'][:300] if level else None))
            continue
        considered=counts['considered']
        accepted=counts.get('accepted')
        if unresolved:
            status,searched,infeasible='GEOMETRY_UNRESOLVED',len(failed&set(considered)),len(failed&set(considered))
        elif level:
            status,searched,infeasible='TARGET_BLOCKED',len(considered),len(considered)
        else:
            infeasible=len(failed&set(considered));searched=len((failed|({accepted} if accepted is not None else set()))&set(considered))
            status=(('SEARCH_SUCCEEDED_FINALIZATION_BUDGET_EXHAUSTED' if error is not None else 'SEARCH_SUCCEEDED') if accepted is not None else
                    'SEARCHED_NO_PLAN' if counts['phase']=='DONE' else
                    'BUDGET_EXHAUSTED_IN_'+counts['phase'] if error is not None else counts['phase'])
        targets.append(dict(target=ident,status=status,candidates=counts['candidates'],considered=len(considered),
                            screened=counts['screened'],searched=searched,unsearched=len(considered)-searched,
                            infeasible_candidates=infeasible,accepted_candidate=accepted,
                            target_reason=level[0]['reason'][:300] if level else None))
    return dict(status='PLANNING_BUDGET_EXHAUSTED' if error is not None else 'SEARCH_FINISHED_WITHIN_DEADLINE',
                where=getattr(error,'where',None),detail=getattr(error,'detail',None),
                deadline_monotonic_s=self.deadline if deadline is None else deadline,classified_monotonic_s=time.monotonic(),
                reply_reserve_s=DEADLINE_REPLY_RESERVE_S,targets=targets,
                rule='UNSEARCHED_CANDIDATES_ARE_NOT_INFEASIBLE; BUDGET_EXHAUSTION_IS_NOT_IPC_FAILURE_OR_INFEASIBILITY')
