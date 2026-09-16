"""Deterministic contact-pose queues: unseen family variants before retries."""
from collections import deque
from copy import deepcopy

from .workcell_layout import canonical_digest


SCHEDULE_MODE = "UNVISITED_FAMILY_VARIANTS_THEN_BOUNDED_RETRIES"
SLICE_SECONDS = (20., 50., 100.)


class ContactCandidateScheduler:
    def __init__(self, poses, *, target, context, request_seed, batch_size,
                 attempt_limit, complete_connection_limit, max_retries=2):
        for name, value, minimum in (("batch_size", batch_size, 1),
                ("attempt_limit", attempt_limit, 0),
                ("complete_connection_limit", complete_connection_limit, 0),
                ("max_retries", max_retries, 0)):
            if isinstance(value, bool) or int(value) != value or value < minimum:
                raise ValueError(f"{name} must be an integer >= {minimum}")
        self.batch_size = batch_size
        self.attempt_limit = attempt_limit
        self.complete_connection_limit = complete_connection_limit
        self.max_retries = max_retries
        self.request_seed = int(request_seed)
        self.generated_count = 0
        self.candidates = {}
        self.families = {}
        for face, (roll, pose, variant) in poses:
            self.generated_count += 1
            family_id = canonical_digest({"target": target.name, "face": face, "roll": int(roll)})
            identity = {"family_id": family_id, "target_pose": target.world_from_local.tolist(),
                "target_half_extents": target.half_extents.tolist(), "pose": pose.tolist(),
                "variant": dict(variant), "context": context}
            candidate_id = canonical_digest(identity)
            family = self.families.setdefault(family_id, {"face": face, "roll_deg": int(roll),
                "generated_count": 0, "candidate_ids": []})
            family["generated_count"] += 1
            if candidate_id in self.candidates:
                continue
            family["candidate_ids"].append(candidate_id)
            self.candidates[candidate_id] = dict(family_id=family_id, candidate_id=candidate_id,
                face=face, roll=int(roll), pose=pose.copy(), variant=deepcopy(dict(variant)))
        # All family cursors advance; a short/exhausted family never blocks a tail.
        self.unvisited = deque(candidate_id
            for i in range(max((len(f["candidate_ids"]) for f in self.families.values()), default=0))
            for f in self.families.values() if i < len(f["candidate_ids"])
            for candidate_id in [f["candidate_ids"][i]])
        self.retries = deque()
        self.records = []
        self.visited = set()
        self.complete_attempts = 0
        self.termination = None

    def _seed(self, candidate_id, retry_index, purpose):
        base = int(canonical_digest([self.request_seed, candidate_id, purpose])[:8], 16)
        return (base + retry_index * 104729) % (2 ** 32)

    def next_attempt(self, *, deadline_reached=False):
        if self.termination is not None:
            return None
        if deadline_reached:
            self.termination = "PLANNING_WALL_CLOCK_DEADLINE"
        elif not self.unvisited and not self.retries:
            self.termination = "CANDIDATE_POOL_EXHAUSTED"
        elif len(self.records) >= self.attempt_limit:
            self.termination = "CANDIDATE_ATTEMPT_LIMIT_REACHED"
        elif self.complete_attempts >= self.complete_connection_limit:
            self.termination = "COMPLETE_CONNECTION_ATTEMPT_LIMIT_REACHED"
        if self.termination is not None:
            return None
        if self.unvisited:
            candidate_id, retry_index, reason = self.unvisited.popleft(), 0, None
        else:
            candidate_id, retry_index, reason = self.retries.popleft()
        candidate = self.candidates[candidate_id]
        ordinal = len(self.records)
        batch = ordinal // self.batch_size
        record = dict(candidate_id=candidate_id, family_id=candidate["family_id"],
            attempt_id=f"{candidate_id}:attempt_{retry_index}", retry_index=retry_index,
            retry_reason=reason, variant=deepcopy(candidate["variant"]),
            face=candidate["face"], roll_deg=candidate["roll"],
            pose_ordinal=ordinal, batch_index=batch, mode=SCHEDULE_MODE,
            ik_seed=self._seed(candidate_id, retry_index, "grasp_ik"),
            path_seed=self._seed(candidate_id, retry_index, "path_connection"),
            candidate_wall_budget_s=SLICE_SECONDS[min(batch, len(SLICE_SECONDS)-1)],
            actual_complete_connection_attempted=False)
        self.records.append(record)
        self.visited.add(candidate_id)
        return candidate, record

    def finish(self, record, result, *, complete_connection_attempted, elapsed_s):
        record.update(elapsed_s=float(elapsed_s), failure_stage=result.get("failure_stage"),
            failure_reason=result.get("failure_reason"), search_status=result.get("search_status"),
            actual_complete_connection_attempted=bool(complete_connection_attempted))
        self.complete_attempts += int(complete_connection_attempted)
        if result.get("complete_trajectory") is True:
            self.termination = "COMPLETE_TRAJECTORY_FOUND"
            return
        if record["retry_index"] >= self.max_retries:
            return
        # A fixed requested pose with no valid seal ring cannot improve through
        # another random search. Numerical IK failures remain finite-search results.
        reason = result.get("failure_reason")
        if result.get("failure_stage") == "coverage" or reason in {
                "FINAL_CONTACT_GEOMETRY_FAILED", "GRASP_FK_RESIDUAL_FAILED",
                "CONTACT_CUP_GEOMETRY_INVALID", "PLANNING_WALL_CLOCK_DEADLINE"}:
            return
        retry_reason = None
        if reason in {"NO_IK", "GRASP_IK_DEADLINE"}:
            retry_reason = "FINITE_IK_SEARCH_NEW_IK_AND_PATH_SEEDS"
        elif complete_connection_attempted:
            retry_reason = "INCOMPLETE_PATH_NEW_IK_AND_PATH_SEEDS"
        if retry_reason:
            self.retries.append((record["candidate_id"], record["retry_index"] + 1, retry_reason))

    def summary(self):
        per_family = {}
        for family_id, family in self.families.items():
            ids = set(family["candidate_ids"])
            per_family[family_id] = {"face": family["face"], "roll_deg": family["roll_deg"],
                "generated_candidate_count": family["generated_count"],
                "unique_candidate_count": len(ids), "unique_candidates_evaluated": len(ids & self.visited),
                "total_attempt_count": sum(r["family_id"] == family_id for r in self.records),
                "remaining_unsearched_count": len(ids - self.visited)}
        return dict(mode=SCHEDULE_MODE, generated_candidate_count=self.generated_count,
            unique_generated_candidate_count=len(self.candidates),
            duplicate_generated_candidate_count=self.generated_count-len(self.candidates),
            unique_candidates_evaluated=len(self.visited), total_attempt_count=len(self.records),
            retry_count=sum(r["retry_index"] > 0 for r in self.records),
            actual_complete_connection_attempt_count=self.complete_attempts,
            batch_size=self.batch_size, attempt_limit=self.attempt_limit,
            complete_connection_attempt_limit=self.complete_connection_limit,
            remaining_unsearched_count=len(self.candidates)-len(self.visited),
            remaining_unsearched_candidate_ids=list(self.unvisited),
            pending_retry_count=len(self.retries), per_family=per_family,
            termination=self.termination, attempts=deepcopy(self.records),
            unsearched_candidates_are_infeasible=False)
