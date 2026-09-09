"""Standard-library integrity gates for layout-bound M-710 replay artifacts.

The hashes in this module are content and staleness checks, not signatures and
not an authorization mechanism.  A caller that can replace an artifact can
also recompute its hashes.  Replay therefore has to bind a bundle to a
verified preflight *and* compare the preflight's source identities with the
current workspace before starting a heavyweight simulator.

This file intentionally imports only the Python standard library so the
fail-closed gate can execute before importing Isaac Sim, NumPy, or the core
``unloading_sim`` package.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


PREFLIGHT_SCHEMA = "m710id70_dynamic_execution_preflight_v1"
REPLAY_INPUT_BINDING_SCHEMA = "m710id70_replay_input_binding_v1"
REPLAY_CONTRACT_SCHEMA = "m710id70_replay_contract_v1"
REPLAY_BUNDLE_FORMAT = "isaacsim_fanuc_replay_v1"
INTEGRITY_SCOPE = "CONTENT_AND_STALENESS_ONLY_NOT_AUTHORIZATION_OR_SIGNATURE"


class M710ReplayContractError(ValueError):
    """Raised when an M-710 preflight or replay bundle fails closed."""


def canonical_sha256(value: Any) -> str:
    """Return the repository's canonical JSON SHA-256 without third parties."""
    try:
        encoded = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise M710ReplayContractError("content is not canonical finite JSON") from exc
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise M710ReplayContractError(f"{name} must be a mapping")
    return value


def _string_list(value: Any, name: str) -> list[str]:
    if not isinstance(value, list) or any(
        not isinstance(item, str) or not item.strip() for item in value
    ):
        raise M710ReplayContractError(f"{name} must be a list of non-empty strings")
    return list(value)


def _sha256(value: Any, name: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise M710ReplayContractError(f"{name} must be a lowercase SHA-256")
    return value


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise M710ReplayContractError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise M710ReplayContractError(f"{name} must be a finite number")
    return result


def _validate_input_identity(identity: Mapping[str, Any]) -> None:
    for name in (
        "execution_config_fingerprint",
        "layout_fingerprint",
        "scene_fingerprint",
        "motion_evidence_fingerprint",
        "dynamics_fingerprint",
        "robot_manifest_sha256",
        "tool_manifest_sha256",
    ):
        _sha256(identity.get(name), f"input_identity.{name}")
    sources = _mapping(
        identity.get("execution_implementation_source_sha256"),
        "input_identity.execution_implementation_source_sha256",
    )
    if not sources:
        raise M710ReplayContractError("execution implementation identity may not be empty")
    for relative, digest in sources.items():
        if (
            not isinstance(relative, str)
            or not relative.strip()
            or Path(relative).is_absolute()
            or ".." in Path(relative).parts
        ):
            raise M710ReplayContractError(
                "execution implementation source paths must be repository-relative"
            )
        _sha256(digest, f"input_identity source {relative}")
    missing = identity.get("robot_missing_mesh_outputs")
    if not isinstance(missing, list) or any(
        not isinstance(relative, str) or not relative.strip() for relative in missing
    ):
        raise M710ReplayContractError(
            "input_identity.robot_missing_mesh_outputs must be a list of paths"
        )


def validate_trajectory_segment(segment: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the minimum complete-cycle segment consumed by the adapter."""
    item = _mapping(segment, "trajectory segment")
    target = item.get("target")
    if not isinstance(target, str) or not target.strip():
        raise M710ReplayContractError("trajectory segment target must be non-empty")
    path = item.get("path")
    if not isinstance(path, list) or len(path) < 2:
        raise M710ReplayContractError("trajectory segment path must have at least two waypoints")
    for row_index, row in enumerate(path):
        if not isinstance(row, list) or len(row) != 6:
            raise M710ReplayContractError(
                f"trajectory segment path[{row_index}] must contain six joints"
            )
        for column_index, value in enumerate(row):
            _finite_number(value, f"trajectory segment path[{row_index}][{column_index}]")

    indices: dict[str, int] = {}
    for name in ("grasp_index", "release_index", "release_retreat_index"):
        value = item.get(name)
        if isinstance(value, bool) or not isinstance(value, int):
            raise M710ReplayContractError(f"trajectory segment {name} must be an integer")
        if not 0 <= value < len(path):
            raise M710ReplayContractError(f"trajectory segment {name} is outside the path")
        indices[name] = value
    if not (
        indices["grasp_index"]
        <= indices["release_index"]
        <= indices["release_retreat_index"]
    ):
        raise M710ReplayContractError(
            "trajectory event order must be grasp <= release <= release_retreat"
        )
    return copy.deepcopy(dict(item))


def build_replay_input_binding(
    *,
    plan_common: Mapping[str, Any],
    configuration: Mapping[str, Any],
    scene_primitives: Sequence[Mapping[str, Any]],
    trajectory_segment: Mapping[str, Any] | None,
    trajectory_segment_status: str,
    input_identity: Mapping[str, Any],
    execution_asset_fingerprint_sha256: str,
) -> dict[str, Any]:
    """Build the self-hashed input binding embedded in an execution preflight."""
    identity = _mapping(input_identity, "input_identity")
    _validate_input_identity(identity)
    execution_fingerprint = _sha256(
        execution_asset_fingerprint_sha256, "execution asset fingerprint"
    )
    if canonical_sha256(identity) != execution_fingerprint:
        raise M710ReplayContractError("execution asset fingerprint does not bind input_identity")
    if trajectory_segment_status == "NOT_AVAILABLE":
        if trajectory_segment is not None:
            raise M710ReplayContractError(
                "NOT_AVAILABLE trajectory status requires a null trajectory segment"
            )
        trajectory_sha256 = None
    elif trajectory_segment_status == "VERIFIED":
        if trajectory_segment is None:
            raise M710ReplayContractError("VERIFIED trajectory status requires a trajectory segment")
        validate_trajectory_segment(trajectory_segment)
        trajectory_sha256 = canonical_sha256(trajectory_segment)
    else:
        raise M710ReplayContractError("unsupported trajectory segment status")

    plan = _mapping(plan_common, "plan_common")
    cfg = _mapping(configuration, "configuration")
    primitives = list(scene_primitives)
    if plan.get("scene_primitives") != primitives:
        raise M710ReplayContractError(
            "plan_common scene primitives do not match the frozen preflight scene"
        )
    result: dict[str, Any] = {
        "schema": REPLAY_INPUT_BINDING_SCHEMA,
        "integrity_scope": INTEGRITY_SCOPE,
        "execution_asset_fingerprint_sha256": execution_fingerprint,
        "execution_implementation_identity_sha256": canonical_sha256(
            identity.get("execution_implementation_source_sha256")
        ),
        "asset_audit_sha256": canonical_sha256(
            {
                "robot_manifest_sha256": identity.get("robot_manifest_sha256"),
                "tool_manifest_sha256": identity.get("tool_manifest_sha256"),
                "robot_missing_mesh_outputs": identity.get("robot_missing_mesh_outputs"),
            }
        ),
        "plan_common_sha256": canonical_sha256(plan),
        "configuration_sha256": canonical_sha256(cfg),
        "scene_primitives_sha256": canonical_sha256(primitives),
        "trajectory_segment_status": trajectory_segment_status,
        "trajectory_segment_sha256": trajectory_sha256,
    }
    result["binding_sha256"] = canonical_sha256(result)
    return result


def _verify_binding_self_hash(binding: Mapping[str, Any]) -> None:
    if binding.get("schema") != REPLAY_INPUT_BINDING_SCHEMA:
        raise M710ReplayContractError("unsupported M-710 replay input binding schema")
    content = copy.deepcopy(dict(binding))
    recorded = _sha256(content.pop("binding_sha256", None), "replay input binding hash")
    if canonical_sha256(content) != recorded:
        raise M710ReplayContractError("M-710 replay input binding fingerprint mismatch")
    if binding.get("integrity_scope") != INTEGRITY_SCOPE:
        raise M710ReplayContractError("M-710 replay input binding has an invalid integrity scope")


def verify_m710_preflight_contract(
    preflight: Mapping[str, Any], *, require_ready: bool = False
) -> dict[str, Any]:
    """Verify fingerprint, readiness semantics, and every bound adapter input."""
    item = _mapping(preflight, "M-710 execution preflight")
    if item.get("schema") != PREFLIGHT_SCHEMA:
        raise M710ReplayContractError("unsupported M-710 execution preflight schema")
    content = copy.deepcopy(dict(item))
    recorded = _sha256(content.pop("preflight_fingerprint", None), "preflight fingerprint")
    if canonical_sha256(content) != recorded:
        raise M710ReplayContractError("M-710 execution preflight fingerprint mismatch")

    identity = _mapping(item.get("input_identity"), "preflight input_identity")
    _validate_input_identity(identity)
    execution_fingerprint = _sha256(
        item.get("execution_asset_fingerprint_sha256"), "execution asset fingerprint"
    )
    if canonical_sha256(identity) != execution_fingerprint:
        raise M710ReplayContractError("execution asset fingerprint does not bind input_identity")

    ready = item.get("simulation_execution_ready")
    machine_qualified = item.get("machine_qualified")
    overall_qualified = item.get("execution_qualified")
    if not isinstance(ready, bool) or not isinstance(machine_qualified, bool):
        raise M710ReplayContractError("preflight readiness fields must be booleans")
    if not isinstance(overall_qualified, bool) or overall_qualified != (
        ready and machine_qualified
    ):
        raise M710ReplayContractError(
            "execution_qualified must equal simulation readiness AND machine qualification"
        )
    blockers = _string_list(item.get("simulation_readiness_blockers"), "readiness blockers")
    alias = _string_list(item.get("blockers"), "blocker compatibility alias")
    if blockers != alias:
        raise M710ReplayContractError("preflight blocker fields disagree")
    if ready == bool(blockers):
        raise M710ReplayContractError("preflight readiness and blockers are inconsistent")
    if (not ready) != (item.get("status") == "BLOCKED"):
        raise M710ReplayContractError("preflight status and readiness are inconsistent")
    machine_warnings = _string_list(
        item.get("machine_qualification_warnings"), "machine qualification warnings"
    )
    audit = _mapping(item.get("asset_audit"), "preflight asset audit")
    for asset_name in ("robot", "tool"):
        asset = _mapping(audit.get(asset_name), f"asset_audit.{asset_name}")
        manifest_sha = _sha256(
            asset.get("manifest_sha256"), f"asset_audit.{asset_name}.manifest_sha256"
        )
        if manifest_sha != identity.get(f"{asset_name}_manifest_sha256"):
            raise M710ReplayContractError(f"{asset_name} manifest identity fields disagree")
    if ready:
        if identity.get("robot_missing_mesh_outputs") != []:
            raise M710ReplayContractError("ready preflight may not have missing robot meshes")
        if any(
            _mapping(audit[asset_name], f"asset_audit.{asset_name}").get(
                "source_integrity"
            )
            is not True
            or _mapping(audit[asset_name], f"asset_audit.{asset_name}").get(
                "execution_qualified"
            )
            is not True
            for asset_name in ("robot", "tool")
        ):
            raise M710ReplayContractError(
                "ready preflight requires execution-qualified robot and tool assets"
            )
        motion = _mapping(item.get("motion"), "preflight motion")
        if motion.get("complete_trajectory_status") != "PASS":
            raise M710ReplayContractError("ready preflight requires a complete motion trajectory")

    adapter = _mapping(item.get("replay_adapter_inputs"), "replay_adapter_inputs")
    plan_common = _mapping(adapter.get("plan_common"), "replay plan_common")
    configuration = _mapping(adapter.get("configuration"), "replay configuration")
    scene = _mapping(item.get("scene"), "preflight scene")
    primitives = scene.get("primitives")
    if not isinstance(primitives, list):
        raise M710ReplayContractError("preflight scene primitives must be a list")
    if plan_common.get("scene_primitives") != primitives:
        raise M710ReplayContractError("replay plan and preflight scene disagree")
    cartons = [
        primitive for primitive in primitives
        if isinstance(primitive, Mapping) and primitive.get("category") == "carton"
    ]
    if len(cartons) != 40 or any(primitive.get("dynamic") is not True for primitive in cartons):
        raise M710ReplayContractError("M-710 replay requires all 40 cartons to remain dynamic")
    carton_names = [primitive.get("name") for primitive in cartons]
    if any(not isinstance(name, str) or not name for name in carton_names) or len(
        set(carton_names)
    ) != 40:
        raise M710ReplayContractError("M-710 dynamic carton identities must be unique")

    expected_plan_fields = {
        "execution_asset_fingerprint_sha256": execution_fingerprint,
        "simulation_execution_ready": ready,
        "execution_qualified": overall_qualified,
        "execution_blockers": blockers,
        "machine_qualified": machine_qualified,
        "machine_qualification_warnings": machine_warnings,
    }
    for name, expected in expected_plan_fields.items():
        if plan_common.get(name) != expected:
            raise M710ReplayContractError(f"replay plan {name} disagrees with preflight")

    trajectory_status = adapter.get("trajectory_segment_status")
    trajectory = adapter.get("trajectory_segment")
    if not ready:
        if trajectory_status != "NOT_AVAILABLE" or trajectory is not None:
            raise M710ReplayContractError(
                "blocked preflight must expose trajectory status NOT_AVAILABLE and no segment"
            )
    else:
        if trajectory_status != "VERIFIED" or not isinstance(trajectory, Mapping):
            raise M710ReplayContractError(
                "ready preflight requires one VERIFIED trajectory segment"
            )
        validated_segment = validate_trajectory_segment(trajectory)
        if carton_names.count(validated_segment["target"]) != 1:
            raise M710ReplayContractError(
                "verified trajectory target must identify exactly one dynamic carton"
            )

    binding = _mapping(adapter.get("input_binding"), "replay input binding")
    _verify_binding_self_hash(binding)
    expected_binding = build_replay_input_binding(
        plan_common=plan_common,
        configuration=configuration,
        scene_primitives=primitives,
        trajectory_segment=trajectory,
        trajectory_segment_status=str(trajectory_status),
        input_identity=identity,
        execution_asset_fingerprint_sha256=execution_fingerprint,
    )
    if dict(binding) != expected_binding:
        raise M710ReplayContractError("preflight replay input binding does not match its inputs")
    if require_ready and not ready:
        raise M710ReplayContractError(
            "M-710 dynamic replay is fail-closed until simulation readiness passes: "
            + ", ".join(blockers)
        )
    return {
        "status": "PASS",
        "preflight_fingerprint": recorded,
        "execution_asset_fingerprint_sha256": execution_fingerprint,
        "simulation_execution_ready": ready,
        "input_binding_sha256": binding["binding_sha256"],
    }


def build_m710_replay_contract(
    preflight: Mapping[str, Any],
    plan: Mapping[str, Any],
    configuration: Mapping[str, Any],
    trajectory_segment: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind the exact builder inputs to a ready, fingerprint-verified preflight."""
    verified = verify_m710_preflight_contract(preflight, require_ready=True)
    adapter = _mapping(preflight["replay_adapter_inputs"], "replay_adapter_inputs")
    expected_plan = copy.deepcopy(dict(adapter["plan_common"]))
    supplied_plan = copy.deepcopy(dict(_mapping(plan, "replay plan")))
    segments = supplied_plan.pop("segments", None)
    if supplied_plan != expected_plan:
        raise M710ReplayContractError("replay plan does not exactly match verified preflight")
    if not isinstance(segments, list) or len(segments) != 1 or segments[0] != trajectory_segment:
        raise M710ReplayContractError(
            "replay plan must contain exactly the selected verified trajectory segment"
        )
    if dict(configuration) != dict(adapter["configuration"]):
        raise M710ReplayContractError(
            "replay configuration does not exactly match verified preflight"
        )
    if dict(trajectory_segment) != dict(adapter["trajectory_segment"]):
        raise M710ReplayContractError(
            "trajectory segment does not exactly match verified preflight"
        )

    binding = adapter["input_binding"]
    result: dict[str, Any] = {
        "schema": REPLAY_CONTRACT_SCHEMA,
        "integrity_scope": INTEGRITY_SCOPE,
        "source_preflight_fingerprint": verified["preflight_fingerprint"],
        "execution_asset_fingerprint_sha256": verified[
            "execution_asset_fingerprint_sha256"
        ],
        "input_binding_sha256": binding["binding_sha256"],
        "plan_common_sha256": binding["plan_common_sha256"],
        "configuration_sha256": binding["configuration_sha256"],
        "scene_primitives_sha256": binding["scene_primitives_sha256"],
        "trajectory_segment_sha256": binding["trajectory_segment_sha256"],
        "execution_implementation_identity_sha256": binding[
            "execution_implementation_identity_sha256"
        ],
        "asset_audit_sha256": binding["asset_audit_sha256"],
    }
    result["contract_sha256"] = canonical_sha256(result)
    return result


def verify_m710_replay_contract(
    contract: Mapping[str, Any], preflight: Mapping[str, Any]
) -> dict[str, Any]:
    """Recompute a serialized replay contract from its embedded preflight."""
    item = _mapping(contract, "M-710 replay contract")
    if item.get("schema") != REPLAY_CONTRACT_SCHEMA:
        raise M710ReplayContractError("unsupported M-710 replay contract schema")
    content = copy.deepcopy(dict(item))
    recorded = _sha256(content.pop("contract_sha256", None), "replay contract hash")
    if canonical_sha256(content) != recorded:
        raise M710ReplayContractError("M-710 replay contract fingerprint mismatch")
    adapter = _mapping(preflight.get("replay_adapter_inputs"), "replay_adapter_inputs")
    plan = copy.deepcopy(dict(adapter["plan_common"]))
    segment = copy.deepcopy(dict(adapter["trajectory_segment"]))
    plan["segments"] = [segment]
    expected = build_m710_replay_contract(
        preflight,
        plan,
        copy.deepcopy(dict(adapter["configuration"])),
        segment,
    )
    if dict(item) != expected:
        raise M710ReplayContractError("M-710 replay contract does not match embedded preflight")
    return {"status": "PASS", "contract_sha256": recorded}


def add_bundle_payload_sha256(bundle_payload: Mapping[str, Any]) -> dict[str, Any]:
    """Return a copy with a digest over every other bundle field."""
    result = copy.deepcopy(dict(bundle_payload))
    result.pop("bundle_payload_sha256", None)
    if result.get("format") != REPLAY_BUNDLE_FORMAT:
        raise M710ReplayContractError("unsupported replay bundle format")
    result["bundle_payload_sha256"] = canonical_sha256(result)
    return result


def verify_bundle_payload_sha256(bundle_payload: Mapping[str, Any]) -> str:
    item = copy.deepcopy(dict(_mapping(bundle_payload, "replay bundle")))
    if item.get("format") != REPLAY_BUNDLE_FORMAT:
        raise M710ReplayContractError("unsupported replay bundle format")
    recorded = _sha256(item.pop("bundle_payload_sha256", None), "bundle payload hash")
    if canonical_sha256(item) != recorded:
        raise M710ReplayContractError("replay bundle payload fingerprint mismatch")
    return recorded


def verify_workspace_preflight_identity(
    preflight: Mapping[str, Any],
    project_root: str | Path,
    *,
    current_asset_audit: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Compare bound implementation/manifests (and optional audit) to a workspace."""
    verify_m710_preflight_contract(preflight, require_ready=True)
    root = Path(project_root).resolve()
    identity = _mapping(preflight["input_identity"], "preflight input_identity")
    sources = _mapping(
        identity.get("execution_implementation_source_sha256"),
        "execution implementation source identities",
    )
    checked: dict[str, str] = {}
    for relative, expected in sources.items():
        if not isinstance(relative, str) or Path(relative).is_absolute():
            raise M710ReplayContractError("implementation source path must be repository-relative")
        expected_sha = _sha256(expected, f"source SHA-256 for {relative}")
        path = (root / relative).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise M710ReplayContractError("implementation source escapes project root") from exc
        if not path.is_file() or sha256_file(path) != expected_sha:
            raise M710ReplayContractError(f"workspace implementation source mismatch: {relative}")
        checked[relative] = expected_sha

    audit = _mapping(preflight.get("asset_audit"), "preflight asset audit")
    for asset_name in ("robot", "tool"):
        asset = _mapping(audit.get(asset_name), f"asset_audit.{asset_name}")
        relative = asset.get("manifest_path")
        if not isinstance(relative, str) or Path(relative).is_absolute():
            raise M710ReplayContractError("asset manifest path must be repository-relative")
        expected = _sha256(asset.get("manifest_sha256"), f"{asset_name} manifest SHA-256")
        if expected != identity.get(f"{asset_name}_manifest_sha256"):
            raise M710ReplayContractError(f"{asset_name} manifest identity fields disagree")
        path = (root / relative).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise M710ReplayContractError("asset manifest escapes project root") from exc
        if not path.is_file() or sha256_file(path) != expected:
            raise M710ReplayContractError(f"workspace {asset_name} manifest mismatch")
        checked[relative] = expected

    if current_asset_audit is not None and dict(current_asset_audit) != dict(audit):
        raise M710ReplayContractError("current workspace asset audit does not match preflight")
    return {"status": "PASS", "checked_sha256": checked}


def verify_m710_replay_bundle(
    bundle_payload: Mapping[str, Any],
    *,
    project_root: str | Path | None = None,
    current_asset_audit: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate an executable M-710 bundle before constructing SimulationApp."""
    bundle_hash = verify_bundle_payload_sha256(bundle_payload)
    metadata = _mapping(bundle_payload.get("metadata"), "replay bundle metadata")
    if metadata.get("robot_model") != "fanuc_m710id_70":
        raise M710ReplayContractError("M-710 replay contract used for a different robot")
    preflight = _mapping(
        metadata.get("m710_execution_preflight"), "embedded M-710 execution preflight"
    )
    contract = _mapping(metadata.get("m710_replay_contract"), "embedded M-710 replay contract")
    verify_m710_replay_contract(contract, preflight)
    if metadata.get("execution_asset_fingerprint_sha256") != preflight.get(
        "execution_asset_fingerprint_sha256"
    ):
        raise M710ReplayContractError("bundle execution asset identity disagrees with preflight")
    if metadata.get("simulation_execution_ready") is not True or metadata.get(
        "execution_blockers"
    ) != []:
        raise M710ReplayContractError("M-710 bundle is not simulation-execution ready")
    if metadata.get("scene_primitives") != preflight["scene"]["primitives"]:
        raise M710ReplayContractError("bundle scene primitives disagree with preflight")
    if project_root is not None:
        verify_workspace_preflight_identity(
            preflight, project_root, current_asset_audit=current_asset_audit
        )
    return {
        "status": "PASS",
        "bundle_payload_sha256": bundle_hash,
        "source_preflight_fingerprint": preflight["preflight_fingerprint"],
        "integrity_scope": INTEGRITY_SCOPE,
    }


__all__ = [
    "INTEGRITY_SCOPE",
    "M710ReplayContractError",
    "PREFLIGHT_SCHEMA",
    "REPLAY_BUNDLE_FORMAT",
    "REPLAY_CONTRACT_SCHEMA",
    "REPLAY_INPUT_BINDING_SCHEMA",
    "add_bundle_payload_sha256",
    "build_m710_replay_contract",
    "build_replay_input_binding",
    "canonical_sha256",
    "sha256_file",
    "validate_trajectory_segment",
    "verify_bundle_payload_sha256",
    "verify_m710_preflight_contract",
    "verify_m710_replay_bundle",
    "verify_m710_replay_contract",
    "verify_workspace_preflight_identity",
]
