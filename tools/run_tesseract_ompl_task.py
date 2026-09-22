"""One real connector.plan call using a frozen previously solved contact candidate.

All contact/extraction/place/release logic is the existing connector. A returned
full segment still requires the independent execution preflight before Isaac.
"""
import argparse
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))
from tools.run_tesseract_ompl_comparison import fixture_context
from unloading_sim.tesseract_ompl_backend import create_free_motion_backend


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--state", type=Path, required=True)
    ap.add_argument("--segment", type=Path, required=True)
    ap.add_argument("--worker", required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    scene, historical, connector, _ = fixture_context(args.state, args.segment)
    backend = create_free_motion_backend("tesseract_ompl", executable=args.worker)
    connector.free_motion_backend = backend
    connector.start_planning_request()
    connector.progress_callback = lambda event: print(json.dumps(event), flush=True)
    target = next(b for b in scene.cartons if b.name == historical["target"])
    started = perf_counter()
    try:
        outcome = connector.plan(target=target, face=historical["face"],
            requested_virtual_contact=np.asarray(historical["contact"]["requested_virtual_task_tcp_pose_world"]),
            grasp_candidates=[dict(candidate_id="frozen_previously_solved_contact",
                                   q_rad=historical["contact"]["actual_q_rad"])],
            home_q=np.asarray(historical["path"][0]), all_obstacles=scene.all_obstacles,
            receiver=scene.receiver, support_names=sorted(scene.support_graph.supported_by[target.name]),
            suction=scene.policy.data["suction"], seed=71070)
        result = dict(schema="tesseract_real_single_candidate_task_v1", task_entry="LayoutTrajectoryConnector.plan",
            target=target.name, scene_fingerprint=scene.snapshot["scene_fingerprint"],
            fixed_contact_candidate=historical["contact"], success=outcome.success,
            selected_trajectory_segment=outcome.segment, failure=outcome.failure,
            attempts=outcome.attempts, statistics=outcome.statistics,
            free_motion_records=getattr(connector, "free_motion_records", []),
            elapsed_s=perf_counter()-started, execution_preflight="NOT_RUN", isaac_run=False)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2))
        print(json.dumps({key: result[key] for key in ["task_entry", "success", "failure", "elapsed_s", "isaac_run"]}), flush=True)
    finally:
        backend.worker.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
