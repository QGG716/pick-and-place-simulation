# Four-layer static layout review

`configs/workcells/eco65_desktop_4layer_v1.yaml` defines a separate 12-carton
static scene (one depth row, three columns, four layers). The original
four-carton native task and nine-layer static outputs remain unchanged.
No motion-planning code is changed.

The new roof is absent from both display and collision geometry, explicitly
`NOT_EVALUATED_ROOF_DISABLED`. Side/rear panels remain. Hash-pinned A06 source
bytes are unchanged. Connected-component roles identify rails, end plates,
uprights and caps for actual derived-geometry edits; no global upper cut or
hidden support is used. Their maximum height is 177 mm, below the 180 mm belt.
The project transition plate uses matching visual/collision geometry and
`simplified_static_connection`; transfer physics remains `NOT_EVALUATED`.

The updated private mast STEP imports through an independent XCAF converter.
Source solids, head/windows/interfaces and assembly transforms are retained;
units are normalized once. The fixed mast uses per-solid convex colliders,
not one enclosing obstacle. Its measured geometry, source hashes and explicit
`T_D_mast` are kept in ignored private manifests/configuration. Placement behind
the robot toward +Y is `SIMULATION_PLACEMENT_ASSUMPTION`; the window face points
toward the cartons, without claiming optical calibration or hardware mounting.
Earlier import errors and diagnostic outputs are preserved separately.

Two bounded static candidates were checked with MuJoCo signed convex distances
including the full warehouse, mast and new layout. The old HOME had ten violating
pairs; changing only J1 by -pi produced no violating pairs under this static
representation. The old state and failures are retained. Conveyor checks use
220 conservative connected-component hulls; Isaac retains exact triangles.
No old-to-new motion, IK search, grasp event or dynamics was evaluated.

Isaac Sim 6.0.1.0 produced six 1920 x 1080 raw images and annotated copies.
Views 01/06 share a camera; view 04 shows the actual mast. Readback matched 394
checks plus 782 warehouse leaf bounds. The actual gap is 100 mm, the box wall
650 x 400 mm, and the four layer centres 230/330/430/530 mm. The timeline stayed
at zero with no Play events. No triangle intersections with the transition
plate interior or mast/environment surface intersections were found. These
checks do not establish trajectory feasibility or transport mechanics.

Private outputs: `outputs/eco65_native/desktop_4layer_20261015_mast_run03/`.
`build/` contains snapshots, inherited/display poses and candidate checks;
`isaac/` contains images, composed USD and readback. Commands and measured
stage timings are in `timings.json`; download hashes are in
`server_delivery_manifest.json`. Two focused clipping/UV tests passed in the
existing USD runtime; both historical scene identities remain valid and the
native entry rejects the new static snapshot.

Status: `STATIC_LAYOUT_REVIEW`, `NO_MOTION_EXECUTED`,
`ROOF_CONSTRAINT_DISABLED`, `TRANSFER_PHYSICS_NOT_EVALUATED`,
`Dynamics: NOT_EVALUATED`. Stop after image delivery for user confirmation.
