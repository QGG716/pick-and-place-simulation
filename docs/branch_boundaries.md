# Branch boundaries and source audit

The immutable references are recorded in `integration/manifest.json`. The feature branch is based directly on `baseline/v0.4-v3validated` at `c5fcf4d`; neither consumer branch is merged.

## Existing interfaces reviewed

- Baseline `perception.detect_carton_obbs` emits deterministic simulation-world OBB truth. It is reusable only behind the explicitly synthetic backend.
- Baseline `calibration.py` calibrates controller motion limits and process delays from logs; it is not a camera calibration implementation and is not reused for intrinsics/extrinsics.
- Baseline `TrailerScene`, `OBB`, FANUC configuration, grasp generation, validation, and time parameterization remain owned by the geometric core.
- Online commit `2c33709` already defines revisions, immutable world snapshots, motion boundaries, candidates, requests/results/envelopes, backend identity/capabilities, `PlannerBackend.plan`, and `PlanValidator.validate`. Version 1.0.0 of the shared package preserves those public constructor shapes and adds perception, timed-trajectory, and execution fields.
- Feasibility commit `c6fa455` contains the real V3 geometric/physics gates and time parameterizer but no online backend implementation. The supplied adapter maps results without changing IK, collision, payload, or timing mathematics.
- Visual commit `1d208f2` publishes `runs/current/cargo_7/06_final/final_instance_aware.json`. Its result assembly retains boxes and bags, with geometry optional.

## Required adapters and isolation

The visual repository requires Torch, Transformers, Ultralytics, SAM/MoGe-related inputs, NumPy 1.26.4 and OpenCV 4.10.0.84. It has no declared `requires-python` and no single real-time pipeline entry. It therefore runs in a separate worker environment; the ROS Python 3.10 process exchanges schema-1.0 JSON, never pickle, and never imports those packages.

The visual top-level `coordinate_space=source_image` is a 2D declaration. Its 3D points are generated from a MoGe monocular point map, `orthogonal_axes_3d` is emitted as row vectors, and hidden faces can be constraint-completed. The adapter records these as camera-optical model estimates and rejects execution eligibility until independent scale, calibration, capture-time TF, and uncertainty evidence exist.

Unverified capabilities: real model inference, physical planning through the new adapters, Humble communication on this Windows host, and hardware execution. They are not represented as passed.
