"""Explicit, content-bound simulation assumptions; no machine qualification."""
from math import isfinite

POC = "proof_of_concept"
LEGACY = "physical_reception"
DEFAULT_MOTION = "configs/validation/m710id70_proof_of_concept.yaml"
DEFAULT_EXECUTION = "configs/simulation/m710id70_proof_of_concept.yaml"


def optional_seconds(value, name="wall time"):
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value) or value < 0:
        raise ValueError(f"{name} must be null or finite and nonnegative")
    return float(value)


def deadline_after(now, seconds):
    seconds = optional_seconds(seconds)
    return None if seconds is None else now + seconds


def profile_evidence(data):
    strategy = data.get("search_strategy", {})
    name = strategy.get("profile", LEGACY)
    if name not in {POC, LEGACY}:
        raise ValueError("unknown simulation profile")
    transport = strategy.get("post_landing_transport", {})
    reception = transport.get("reception_mode", "physical")
    if reception not in {"physical", "ideal"}:
        raise ValueError("unknown reception mode")
    if (name == POC) != (reception == "ideal"):
        raise ValueError("profile and reception mode disagree")
    if name == POC and (transport.get("mode") != "ideal_outfeed" or data["suction"]["mode"] != "ideal_independent_cups"):
        raise ValueError("proof_of_concept requires ideal cups, reception and outfeed")
    return {"name": name, "ideal_suction": data["suction"]["mode"] == "ideal_independent_cups",
            "ideal_reception": reception == "ideal", "ideal_outfeed": transport.get("mode") == "ideal_outfeed",
            "physical_execution_scope": "actual_grasp_transport_constraint_removal_and_robot_departure",
            "machine_qualification_claimed": False}
