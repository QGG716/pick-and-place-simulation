"""Bind v0.2 path evidence to unchanged checkers without replaying that path."""
import hashlib
import json
from pathlib import Path
import tarfile
import ast
import gzip


def digest(data):
    return hashlib.sha256(data).hexdigest()


def without_evidence_sink(data):
    """Compare the entire authority module after removing only v6 observation hooks.

    State/edge checks, cache keys and process decisions remain in the comparison.
    This is not permission to change an unlisted checker or planner expression.
    """
    class StripObserver(ast.NodeTransformer):
        def visit_FunctionDef(self, node):
            if node.name == "_record_process_stage":
                return None
            node = self.generic_visit(node)
            for i in reversed(range(len(node.args.kwonlyargs))):
                if node.args.kwonlyargs[i].arg == "stage_evidence_callback":
                    del node.args.kwonlyargs[i]; del node.args.kw_defaults[i]
            return node
        def visit_Assign(self, node):
            if (len(node.targets) == 1 and ast.unparse(node.targets[0]) == "self.stage_evidence_callback"
                    and ast.unparse(node.value) == "stage_evidence_callback"):
                return None
            return node
        def visit_If(self, node):
            if ast.unparse(node.test) == "self.stage_evidence_callback is not None":
                assert len(node.body) == 1 and not node.orelse
                assert ast.unparse(node.body[0]) == "self.stage_evidence_callback(deepcopy(snapshot))"
                return None
            return self.generic_visit(node)
        def visit_Expr(self, node):
            if isinstance(node.value, ast.Call) and ast.unparse(node.value.func) == "self._record_process_stage":
                return None
            return node
    return ast.dump(StripObserver().visit(ast.parse(data)), include_attributes=False)


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
    observation_only = {}
    for name, expected in audit["sources"].items():
        if name.startswith("src/") and name != "src/unloading_sim/tesseract_ompl_backend.py":
            actual = digest((root / name).read_bytes())
            if name == "src/unloading_sim/layout_trajectory.py" and actual != expected:
                previous = gzip.decompress((root / "docs/validation/evidence/backend_tesseract_ompl_v0_6/authority-before.py.gz").read_bytes())
                assert digest(previous) == expected
                normalized = without_evidence_sink((root / name).read_bytes())
                assert normalized == without_evidence_sink(previous), "authority code changed beyond evidence sink"
                observation_only[name] = dict(previous_sha256=expected, current_sha256=actual,
                    authority_ast_sha256=digest(normalized.encode()), changes="OPTIONAL_EVIDENCE_SINK_ONLY")
            else:
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
        observation_only_sources=observation_only,
        fixture_sha256=fixtures, scene_fingerprint=audit["scene_fingerprint"],
        constraints=audit["constraints"], previous_path_points=29, previous_same_q_mismatches=0,
        new_full_audit_executed=False, historical_path_injected=False)
