"""Create an initialization-only contract from current model/configuration inputs."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.m710_bootstrap import build_bootstrap_contract


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execution-config", type=Path,
                        default=ROOT / "configs/simulation/m710id70_proof_of_concept.yaml")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    contract = build_bootstrap_contract(args.execution_config, project_root=ROOT)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(contract, stream, indent=2, allow_nan=False)
    print(json.dumps(dict(output=str(args.output), contract_sha256=contract["contract_sha256"],
                         motion_execution_permitted=False)))


if __name__ == "__main__":
    main()
