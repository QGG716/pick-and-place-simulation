# FANUC M-20iD/35 trailer cross-section study

This directory is intentionally separate from `src/unloading_sim`.  It reads
the existing FANUC URDF and supplied gripper STEP audit without adding a heavy
dependency to the core simulator.

The plotted cross-section coordinate `x` maps to the repository world `+Y`
(left); repository world `+X` remains the trailer longitudinal direction.
The configured mounting height is the physical mounting surface.  The J1
rotation centre is 0.425 m above it, as encoded by the official URDF.

Run the deterministic study from the repository root:

```bash
PYTHONPATH=src python studies/fanuc_m20id35_cross_section/run_study.py \
  --config studies/fanuc_m20id35_cross_section/study_config.json \
  --backend pybullet --workers 20 --audit-best
```

`pybullet` is optional and is imported only by this study.  `--backend
geometric` is provided for fast tests, but release results must use
`pybullet`, which checks the FANUC URDF collision meshes.  The tool collision
model uses every rigid-solid bounding box extracted from the supplied STEP,
plus an explicit active bottom-support rail/blade.  Cartesian extraction is
subdivided at the configured translation and joint-motion bounds; every
subdivision checks robot, tool, support and carried-carton collision.
This is deterministic dense swept-path sampling, not an analytic or FCL
continuous-collision proof; the sampling bounds and collision margin are
recorded in `simulation_parameters.json`.

The box dimensions are interpreted as cross-section width, cross-section
height and longitudinal depth.  A cross-section cell is the carton centre.
Cells where the requested carton cannot physically fit inside the trailer are
reported as unreachable, not silently clipped.
