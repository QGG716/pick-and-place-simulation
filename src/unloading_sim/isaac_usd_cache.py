"""Reuse an evidenced official robot USD package without converting its meshes."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from .m710_initialization_diagnostic import generated_usd_tree_identity
from .workcell_layout import canonical_digest


def verify_recorded_official_usd(entrypoint, run_evidence, source_contract, metadata):
    entry = Path(entrypoint).resolve()
    evidence = json.loads(Path(run_evidence).read_text(encoding="utf-8"))
    source = json.loads(Path(source_contract).read_text(encoding="utf-8"))
    original = dict(source)
    fingerprint = original.pop("contract_fingerprint", None)
    if fingerprint != canonical_digest(original) or fingerprint != evidence.get("contract_fingerprint"):
        raise ValueError("cached USD run and source contract identities differ")
    recorded = evidence.get("generated_usd")
    if not isinstance(recorded, dict) or not recorded.get("entrypoint"):
        raise ValueError("cached official USD requires its original generated-tree evidence")
    relative = Path(recorded["entrypoint"])
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("cached USD evidence entrypoint must remain inside its package")
    root = entry
    for _ in relative.parts:
        root = root.parent
    if root / relative != entry:
        raise ValueError("selected cached USD does not match the evidenced entrypoint")
    current = generated_usd_tree_identity(entry, root)
    if current != recorded:
        raise ValueError("cached official USD tree bytes differ from the archived run")
    official = source.get("official_model_audit", {})
    if (metadata.get("robot_model") != "fanuc_m710id_70"
            or official.get("upstream_commit") != "fb40c9803a826ba68c7c8e28ba904a25efa7fcd2"
            or official.get("manifest_sha256") != metadata.get("official_model_manifest_sha256")):
        raise ValueError("cached USD source is not the same fixed official FANUC model")
    urdf_sha = source["robot"]["urdf"]["sha256"]
    expanded = metadata["official_model_manifest"]["integration"]["expanded_urdf"]
    if urdf_sha != expanded.get("sha256"):
        raise ValueError("cached USD was generated from different official URDF bytes")
    # The referenced package contains robot assets only. Current tool CAD,
    # inertials, drives, self-collision and the approved pair policy are all
    # checked/reauthored by the actual replay, never inherited as readiness.
    return {"schema": "official_usd_reuse_v1", "usd_path": str(entry),
            "source_urdf_sha256": urdf_sha, "tree_identity": current,
            "source_run_evidence_sha256": hashlib.sha256(Path(run_evidence).read_bytes()).hexdigest(),
            "source_contract_fingerprint": fingerprint,
            "official_model_manifest_sha256": official["manifest_sha256"],
            "reuse_scope": "OFFICIAL_ROBOT_MESHES_AND_JOINT_TOPOLOGY_ONLY",
            "runtime_properties_reauthored": ["official_link_inertials", "finite_drives", "initial_q",
                                              "self_collision", "pair_filters", "current_verified_tool_CAD"],
            "mesh_conversion_performed": False}
