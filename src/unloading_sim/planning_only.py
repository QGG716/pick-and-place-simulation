"""Isolated experiment adapter. Generated states carry lineage, never permission.

The normal connector, dense validators and execution entry never select this
class. MoveIt's scene predicates, collision geometry and planners are unchanged.
"""
from copy import deepcopy
from time import perf_counter
from types import SimpleNamespace

import numpy as np

from .moveit2_backend import MoveItLayoutConnector, MoveItUnavailable, digest

MARKER = "PLANNING_ONLY_NOT_EXECUTABLE"
SKIPPED = "SKIPPED_PLANNING_ONLY"
SKIPPED_CHECKS = (
    "python_independent_state_and_path_validation", "native_dense_output_recheck",
    "full_path_recheck", "independent_seal_compression_and_contact_lifecycle_audit",
    "sampled_tcp_lin_audit", "full_path_provenance_scan", "preflight",
    "execution_export", "execution_bundle_load", "dynamics_torque_jerk_qualification",
    "execution_time_law_conversion", "isaac_and_physical_reception",
)


class PlanningInputGuard:
    """Only shape/finite/bounds and context checks for existing search entry gates.

This deliberately is not a MotionValidator or a collision validity certificate.
No edge, template, or execution validation API is provided.
"""
    def __init__(self, connector, obstacles, options):
        self.connector, self.obstacles, self.options = connector, obstacles, options
        self.context = SimpleNamespace(context_id=self.context_current(), guarantee=MARKER)

    def context_current(self):
        return self.connector._context_identity(self.obstacles, **self.options)

    def check_states(self, states, request):
        result = []
        for q in states:
            failure = self.connector._state_failure(q, self.obstacles, **self.options)
            evidence = dict(status=SKIPPED, input_well_formed=failure is None,
                            collision_validated=False, execution_ready=False)
            result.append(SimpleNamespace(valid=failure is None, failure=failure,
                status=SimpleNamespace(value=SKIPPED), evidence=lambda row=evidence: dict(row)))
        return result


class PlanningOnlyConnector(MoveItLayoutConnector):
    planning_only = True

    @classmethod
    def from_existing(cls, *args, **kwargs):
        kwargs.update(native_cold=True, require_native_motion=True)
        self = super().from_existing(*args, **kwargs)
        if self.native_startup.get("planning_only") is not True:
            self.native.close()
            raise MoveItUnavailable("WORKER_PLANNING_ONLY_CAPABILITY_MISSING")
        # This is a model-loaded connector, not an execution-qualified result.
        self.execution_qualified = False
        self.skipped_calls = {}
        self._experiment_complete_segment = None
        return self

    def _skip(self, name):
        self.skipped_calls[name] = self.skipped_calls.get(name, 0) + 1

    def _state_failure(self, q, obstacles, **options):
        self._skip("independent_state_validation")
        if self._native_cancelled():
            return dict(reason="PLANNING_WALL_CLOCK_DEADLINE", stage=options.get("stage"))
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all() or not self.robot.within_limits(q):
            return dict(reason="INVALID_EXPERIMENT_JOINT_STATE", stage=options.get("stage"))
        return None

    def _path_failure(self, path, obstacles, **options):
        self._skip("independent_path_validation")
        if not path:
            return dict(reason="EMPTY_EXPERIMENT_PATH", stage=options.get("stage"))
        # No interpolation, collision, TCP audit or validation cache.
        for q in path:
            failure = self._state_failure(q, obstacles, **options)
            if failure:
                return failure
        return None

    def _motion_validator(self, obstacles, **options):
        return PlanningInputGuard(self, obstacles, options)

    def _optional_quality(self, path, deadline):
        return None

    def _remember_path(self, *args, **kwargs):
        return None

    def _accept_generated_stage(self, path, attempt, attempts, submitted, *,
                                attachment, initial_proximity, started):
        if attempt.get("native_output_status") != SKIPPED:
            raise MoveItUnavailable("PLANNING_ONLY_OUTPUT_CHECK_NOT_SKIPPED")
        if any(self._state_failure(q, ()) for q in path):
            raise MoveItUnavailable("INVALID_EXPERIMENT_NATIVE_STATE")
        if initial_proximity is not None:
            # State propagation only at the actual generated endpoint. The
            # ordered dense contact-lifecycle audit is explicitly not performed.
            if attachment is None or not hasattr(initial_proximity, "_update_released"):
                raise MoveItUnavailable("PLANNING_ONLY_UNSUPPORTED_PROXIMITY_POLICY")
            initial_proximity._update_released(attachment.box_at(path[-1]))
        attempt.update(authoritative_status=SKIPPED, authoritative_s=None,
            lin_constraint_audit=SKIPPED, failure=None, end_to_end_s=perf_counter()-started)
        self.native_evidence.extend(attempts)
        self.native_generated.append(deepcopy(attempt))
        context = self._native_state_context(submitted, endpoint=path[-1], attachment=attachment)
        path[-1] = self._native_issue_state(path[-1], submitted["stage_id"], context)
        return path, None, dict(backend="moveit2", success=True,
            validation_level=MARKER, validation_completed=False, attempts=attempts)

    def _departure(self, *args, **kwargs):
        path, failure, evidence = super()._departure(*args, **kwargs)
        # The normal wait fallback needs physical separation feedback. This
        # offline experiment must actually plan departure with the box present.
        if failure is None and len(path) == 1:
            return [], dict(reason="PLANNING_ONLY_REQUIRES_GENERATED_WITHDRAWAL",
                            stage="withdrawal"), evidence
        return path, failure, evidence

    def _finalize_task_checked(self, segment, obstacles, target):
        selection = self._native_final_selection
        if (selection is None or selection[1] != digest(segment["path"])
                or segment["target"] != target.name):
            raise MoveItUnavailable("PLANNING_ONLY_SELECTED_STATE_MISSING")
        self._native_state_receipt(selection[0])
        # Discard the normal result's execution-oriented schema/metadata.
        # Retain experiment geometry and events, not an executable segment.
        keep = {name: segment[name] for name in (
            "target", "face", "path", "stage_ranges", "events", "grasp_index",
            "release_index", "release_retreat_index", "contact")}
        keep["place"] = {name: segment["place"][name] for name in (
            "actual_box_pose_world", "support_names", "placement_family", "place_surface")}
        selected = self._native_selected_records(selection[0])
        keep.update(schema="m710_planning_only_trajectory_v1", status=MARKER,
            task_id=self.native_task_id, execution_ready=False, execution_qualified=False,
            qualification_status="NOT_EVALUATED", skipped_checks={k: SKIPPED for k in SKIPPED_CHECKS},
            native_stages=[{key: r[key] for key in (
                "stage", "stage_id", "parent_stage_id", "pipeline_id", "planner_id",
                "points", "joint_names", "time_parameterization", "authoritative_status",
                "native_output_status", "planning_only_status")} for r in selected],
            final_state_receipt=selection[0].native_receipt)
        segment.clear()
        segment.update(keep)
        self._experiment_complete_segment = segment
        return None

    def _task_completed(self, segment, obstacles, target):
        return (segment is not None and segment is self._experiment_complete_segment
            and segment.get("status") == MARKER and segment.get("task_id") == self.native_task_id
            and segment.get("target") == target.name)
