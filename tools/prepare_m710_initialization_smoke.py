"""Freeze a diagnostic-only scene without manufacturing an executable plan."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.m710_initialization_diagnostic import build_initialization_diagnostic_contract

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validation-config", type=Path, default=ROOT / "configs/validation/m710id70_layout_v1.yaml")
    parser.add_argument("--dynamics-config", type=Path, default=ROOT / "configs/simulation/m710id70_official_dynamics_v2.yaml")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    contract = build_initialization_diagnostic_contract(args.validation_config, args.dynamics_config, ROOT)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(contract, indent=2), encoding="utf-8")
    print(json.dumps({"scope": contract["scope"], "initial_state_audit": contract["initial_state_audit"],
                      "contract_fingerprint": contract["contract_fingerprint"]}, indent=2))
