"""Bind v0.2 path evidence to unchanged checkers without replaying that path."""
import hashlib
import json
from pathlib import Path
import tarfile


def digest(data):
    return hashlib.sha256(data).hexdigest()


def verify_checkers(root):
    root = Path(root)
    evidence = root / "docs/validation/evidence/backend_tesseract_ompl_v0_2"
    audit_file = evidence / "final-audit-validated/audit.json"
    audit = json.loads(audit_file.read_text())
    with tarfile.open(evidence / "validated-source.tar.gz") as archive:
        old = archive.extractfile("native/tesseract_ompl/worker.cpp").read()
    assert digest(old) == audit["sources"]["native/tesseract_ompl/worker.cpp"]
    current = (root / "native/tesseract_ompl/worker.cpp").read_bytes()
    def checker(data):
        return data[data.index(b"struct Context {"):data.index(b"struct SeededSampler")]
    def grid(data):
        return data[data.index(b"  // One supported geometric contract."):
                    data.index(b"  const std::string operation=")]
    assert checker(current) == checker(old), "state/motion checker changed; new proof required"
    assert grid(current) == grid(old), "subdivision validation changed; new proof required"
    unchanged = {}
    for name, expected in audit["sources"].items():
        if name.startswith("src/") and name != "src/unloading_sim/tesseract_ompl_backend.py":
            actual = digest((root / name).read_bytes())
            assert actual == expected, name
            unchanged[name] = actual
    fixtures = {
        "historical_state.json": "c2478fbd880806d8499cdc26a4f5613548709e7e76aab4af7731c67d430b2b1a",
        "historical_segment.json": "cf306dd532c1d20a2a5132dabe0af7ae7bf1ba1de2e2b14fc9a1d3bda1ee81dd",
    }
    for name, expected in fixtures.items():
        assert digest((root / "tests/fixtures/tesseract_ompl" / name).read_bytes()) == expected
    assert audit["native_path"]["status"] == "AUDIT_VALID"
    assert audit["native_path"]["audit_complete"]
    proof = audit["authority_evidence_reuse"]
    assert proof["identical_actual_q_grid"] and proof["authority_path_accepted"]
    assert proof["same_q_mismatches"] == 0
    return dict(schema="v3_checker_identity_reuse", audit_sha256=digest(audit_file.read_bytes()),
        previous_worker_sha256=audit["worker_sha256"],
        previous_worker_source_sha256=digest(old), current_worker_source_sha256=digest(current),
        context_and_dense_motion_sha256=digest(checker(current)),
        subdivision_guard_sha256=digest(grid(current)), unchanged_sources=unchanged,
        fixture_sha256=fixtures, scene_fingerprint=audit["scene_fingerprint"],
        constraints=audit["constraints"], previous_path_points=29, previous_same_q_mismatches=0,
        new_full_audit_executed=False, historical_path_injected=False)
