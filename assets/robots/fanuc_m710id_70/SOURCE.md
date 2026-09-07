# FANUC M-710iD/70 model evidence

This is a deterministic first-layer engineering URDF, not a manufacturer-supplied URDF.

- Joint axis lines, zero pose and mechanism identity: supplied `ROBCAD_M-710iD_70_v01.zip`, model v1.0, created by FANUC Robot Mechanical Research & Development Division on 2024-11-29, SHA-256 `8b1074538a993c53ce3e9f006434b03cc9c3f50c63f9609152c6f9c4fab0e09c`.
- Limits, maximum speeds, reach, wrist moments/inertias and mass: supplied `M-710iD_70_v1.pptx`, SHA-256 `2410b105e3a09ebc927eaa85e8800067cde8b734c5b5b56f872fb18539d901a5`.
- Exterior geometry cross-check: supplied `2D_M-710iD_70_v01.dxf` and `3D_M-710iD_70_v01.x_t`, hashes recorded in the qualification manifest.
- Primitive collision shapes and core capsule radii are conservative CAD-envelope engineering proxies (`CAD_DERIVED`), because no converted collision mesh is shipped here.
- Link masses, centres of mass, inertias and certified joint torque limits are unavailable. They are deliberately omitted. Precision inverse dynamics is therefore `BLOCKED_BY_MISSING_INERTIAL_DATA`.

Coordinate convention: the URDF `base_link` origin is the robot mounting-flange plane. At the ROBCAD zero pose, the flange origin is `[1.370, 0, 1.630]` m in the base frame. The fixed `tool0` adapter maps the FANUC flange +X direction to the simulator tool +Z direction.
