from __future__ import annotations

from dataclasses import replace
import json

from tests.online_acceptance_profiles import default_profiles
from tools.online_backend_acceptance import (
    AcceptanceStatus,
    main,
    run_acceptance,
)


def test_unified_acceptance_entry_runs_real_profiles_and_writes_json(tmp_path):
    output = tmp_path / "online-backend-acceptance.json"

    assert main(["--output", str(output)]) == 0
    report = json.loads(output.read_text(encoding="utf-8"))

    assert report["required_contracts_passed"]
    assert report["package_version"] != "unknown"
    assert report["code_revision"] != "unknown"
    assert report["latency_scope"]["production_throughput"] is None
    assert len(report["profiles"]) == 2
    assert all("planner_identity" in profile for profile in report["profiles"])
    assert all("execution_identity" in profile for profile in report["profiles"])
    categories = {case["category"] for case in report["cases"]}
    assert categories == {"backend_compliance", "runtime_tolerance", "geometry_evidence"}
    assert {
        case["profile_id"]
        for case in report["cases"]
        if case["case_id"] == "runtime.configured_backend_handoff"
    } == {profile.profile_id for profile in default_profiles()}
    geometry = next(
        case
        for case in report["cases"]
        if case["case_id"] == "v3.bottom_alternative.geometry_rerun"
    )
    assert geometry["status"] == AcceptanceStatus.NOT_EVALUATED.value
    assert "does not rerun" in geometry["reason"]


def test_undeclared_optional_capability_is_not_evaluated_with_reason():
    cold_profile = default_profiles()[1]

    report = run_acceptance((cold_profile,), include_runtime_tolerance=False)

    cache = next(case for case in report["cases"] if case["case_id"] == "planner.operational.CACHE_MISS")
    assert cache["status"] == AcceptanceStatus.NOT_EVALUATED.value
    assert "does not provide" in cache["reason"]
    assert report["required_contracts_passed"]


def test_declared_mandatory_contract_violation_is_reported_as_fail():
    good = default_profiles()[0]
    violating = replace(
        good,
        profile_id="intentional-invalid-planner-factory",
        planner_factory=lambda scenario, seed: object(),
        replay_parameters={"fault": "planner factory returned wrong type"},
    )

    report = run_acceptance((violating,), include_runtime_tolerance=False)

    assert not report["required_contracts_passed"]
    failures = [case for case in report["cases"] if case["status"] == AcceptanceStatus.FAIL.value]
    assert failures
    assert any(case["required"] for case in failures)
    assert all(case["replay"]["fault"] == "planner factory returned wrong type" for case in failures)


def test_unified_entry_returns_nonzero_for_intentional_violation(tmp_path):
    output = tmp_path / "invalid-online-backend.json"

    exit_code = main(
        [
            "--profiles",
            "tests.online_acceptance_profiles:intentional_invalid_profiles",
            "--output",
            str(output),
        ]
    )

    assert exit_code != 0
    report = json.loads(output.read_text(encoding="utf-8"))
    assert not report["required_contracts_passed"]
