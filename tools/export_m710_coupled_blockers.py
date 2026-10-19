"""Self-contained inspection of a rejected coupled batch start, not a replay."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from export_m710_search_replay import export_model


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    protocol = json.loads((args.run/"protocol.json").read_text())
    summary = json.loads((args.run/"summary.json").read_text())
    initial = json.loads((args.run/"initial-world-check.json").read_text())
    if initial["passed"] or summary["completed_count"] != 0:
        raise ValueError("REJECTION_INSPECTOR_REQUIRES_REJECTED_ZERO_COMPLETION")
    model_file = args.run/f"seed-{protocol['seeds'][0]}"/"model-init.json"
    model = export_model(json.loads(model_file.read_text()))
    result = dict(summary=summary, protocol=protocol, initial=initial,
                  reference_q_rad=protocol["q_rad"], state=protocol["geometry"])
    template = (ROOT/"tools/lifted_reception/viewer.html").read_text(encoding="utf-8")
    extension = (ROOT/"tools/lifted_reception/coupled_blockers.js").read_text(encoding="utf-8")
    vendor = ROOT/"tools/search_replay/vendor"
    scripts = (vendor/"three.min.js").read_text()+"\n"+(vendor/"OrbitControls.js").read_text()
    html = template.replace("/*__VENDOR__*/", scripts).replace("/*__COUPLED__*/", extension).replace(
        "/*__DATA__*/", json.dumps(dict(lift=True, result=result, model=model),
                                  separators=(",", ":"), allow_nan=False).replace("</", "<\\/"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(html, encoding="utf-8")
    print(json.dumps(dict(html=str(args.output), bytes=args.output.stat().st_size,
                         status="INITIAL_REJECTION_INSPECTION_NOT_BATCH_REPLAY")))


if __name__ == "__main__":
    main()
