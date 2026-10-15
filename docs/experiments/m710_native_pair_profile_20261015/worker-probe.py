"""Small real-worker option propagation check; no planning or execution."""
import copy
import gzip
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

root = Path("/work-native-pair-profile-20261015")
out = root / "docs/experiments/m710_native_pair_profile_20261015"
spec = importlib.util.spec_from_file_location("checks", root / "tools/benchmark_m710_native_checks.py")
checks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checks)
init = json.loads((out / "model.json").read_text())
case = json.loads((out / "fixed-input.json").read_text())["cases"][0]
# The frozen model init predates this fragment's scene/policy scope labels.
# Bind the new resident context to the checked request identity; no geometry changes.
differences = {k for k,v in case["request"]["identity"].items() if init["identity"].get(k)!=v}
assert differences <= {"scene_fingerprint", "policy_scope"}
init["identity"] = copy.deepcopy(case["request"]["identity"])
requests = [dict(init, request_id="profile-probe-init")]
for enabled in (True, False):
    req = copy.deepcopy(case["request"])
    req.update(op="validate", request_id="profile-on" if enabled else "default-off",
               probe_states=[req["q_start"], req["q_start"]], native_check_reuse=True)
    if enabled:
        req["pair_profile_enabled"] = True
    else:
        req.pop("pair_profile_enabled", None)  # exercise the production default
    requests.append(req)
run = subprocess.run(["/tmp/native-pair-profile-20261015/build/m710_moveit_worker"],
                     input="".join(json.dumps(r)+"\n" for r in requests),
                     capture_output=True, text=True, timeout=60)
(out / "worker-probe.log").write_text(run.stderr)
run.check_returncode()
results = [json.loads(line) for line in run.stdout.splitlines()]
with gzip.open(out / "worker-probe.json.gz", "wt") as f:
    json.dump(results, f)
assert len(results) == 3, results
assert results[0]["status"] == "READY" and all(r["status"] == "SUCCESS" for r in results[1:]), [r.get("detail",r["status"]) for r in results]
cases = [dict(clearance=r["clearance"], process=r["process_policy_evidence"],
              visited_states=len(r["states"])) for r in results[1:]]
for c, enabled in zip(cases, (True, False)):
    checks.check_pair_profile(c, enabled)
assert checks.pair_geometry_counts(cases[0]) == checks.pair_geometry_counts(cases[1])
a, b = [[{k:v for k,v in s.items() if k!="check_s"} for s in r["states"]] for r in results[1:]]
assert a == b
assert all(s["valid"] for s in a)
summary = dict(status="PASS", scope="real worker validate only; two repeated archived transit states",
               omitted_option_disables_pair_profile=True, explicit_true_enables_pair_profile=True,
               geometry_counts_identical=True, verdicts_and_witnesses_identical=True,
               planning_requests=0, execution_requests=0, init_identity_labels_rebound=sorted(differences), geometry=checks.pair_geometry_counts(cases[0]))
(out / "worker-probe-summary.json").write_text(json.dumps(summary, indent=2)+"\n")
print(json.dumps({k:v for k,v in summary.items() if k!="geometry"}))
