from __future__ import annotations

import copy
import os
from pathlib import Path

import pytest
import yaml

from unloading_sim.isaac_usd_cache import verify_recorded_official_usd


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs/validation/evidence/m710id70_official_dynamics_20260910"


def test_existing_official_usd_matches_recorded_model_and_rejects_changed_source():
    entry = os.environ.get("M710_REUSE_USD_ENTRYPOINT")
    if entry is None:
        pytest.skip("requires the existing official server USD cache")
    manifest_path = ROOT / "assets/robots/fanuc_m710id_70/official/provenance.yaml"
    import hashlib
    metadata = {"robot_model": "fanuc_m710id_70",
                "official_model_manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                "official_model_manifest": yaml.safe_load(manifest_path.read_text(encoding="utf-8"))}
    args = (entry,
            os.environ.get("M710_REUSE_RUN_EVIDENCE", EVIDENCE / "isaac_initialization/run_status.json"),
            os.environ.get("M710_REUSE_SOURCE_CONTRACT", EVIDENCE / "initialization_contract.json"))
    audit = verify_recorded_official_usd(*args, metadata)
    assert audit["tree_identity"]["aggregate_sha256"] == "c7001b60ad6e2208a57a2c4d1244c52003702fe531263636542c95984ded3a7f"
    assert len(audit["tree_identity"]["files"]) == 9
    assert not audit["mesh_conversion_performed"]
    wrong = copy.deepcopy(metadata)
    wrong["official_model_manifest"]["integration"]["expanded_urdf"]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="different official URDF"):
        verify_recorded_official_usd(*args, wrong)
