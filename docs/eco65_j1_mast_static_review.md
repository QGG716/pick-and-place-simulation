# J1 mast static orientation review

The current user request replaces the previous table-fixed mast with a rigid
link_1 accessory. Planning, transit optimization, camera acquisition and physics
remain stopped. The original mast CAD, connector and camera are retained.

The arm heading is **link_1 local -X**, the horizontal direction from J1 toward
the official J2 origin. It is independent of J2–J6 and of the TCP position.
The explicitly defined camera frame uses local +Z as its optical axis, aligned
with the original CAD front-window normal. The selected fixed assembly gives
**+90 degrees** around world up, from arm heading to camera heading. The user
specified the mast on the left when looking into the trailer; the initial
upright lies on the +D-Y side of J1. This side description applies to the initial
display pose, while the signed relative angle remains fixed as J1 turns.

USD hierarchy is `Robot/link_1/Mast/CameraFrame`. Robot visual and collision
meshes are identity children of their named link transforms. Mast visual and
collision meshes share the same fixed local assembly. Only the external review
camera uses look-at; it does not change the mast or physical camera frame.

`prepare_eco65_j1_mast_review.py` verifies the existing static snapshot and
display-pose identities, binds a private attachment configuration, and emits
four independent poses: A unchanged, B J1 +15°, C J1 -15°, D J2 +10°. No search
or motion between these poses is performed. The exporter remains compatible
with existing table-fixed mast configurations.

Use the existing exporter followed by the existing Isaac renderer with
`--layout-review --j1-mast-review`. Run
`verify_eco65_j1_mast_review.py --folder <new-run>` after all four renders. This
reopens the final composed USDs, checks nested transforms, J1 covariance, J2
invariance, signed angles, image hashes, and stopped timeline evidence.

Actual 2026-10-15 evidence is private in
`outputs/eco65_native/j1_mast_20261015_run02/`. All four final USDs read back
+90°; additional official-URDF checks show invariance to each J2–J6 change.
Isaac 6.0.1.0 produced ten 1920×1080 originals with annotated copies. No Play
events or simulation time advancement occurred.

Orientation verification is separate from mounting fit and collision acceptance.
The connector is placed against the measured link_1 upper envelope; this remains
a simulation assembly assumption, without verified fastener/hole correspondence
or optical calibration. A directed original-triangle probe found a mast/link_3
surface crossing in case D. Its diagnostic images retain and mark the issue.
No transverse crossing was found in the checked A/B/C pairs, which is not a
complete clearance proof. No new HOME or trajectory acceptance is granted.

Evidence labels: `CAMERA_AXIS_90DEG_CONSTRAINT_APPLIED`,
`J1_CO_ROTATION_VERIFIED`, `NO_MOTION_EXECUTED`, `Dynamics: NOT_EVALUATED`.
All CAD, detailed geometry, attachment configuration and images remain ignored.

## October 16 horizontal reversal

The user corrected the silver base orientation: rotate it 180° in the horizontal
plane about the vertical installation axis. The rigid CAD assembly follows the
base without changing individual part relationships or mounting height. Relative
to the previous attachment, `T_link1_mast_new = Rz(pi) @ T_link1_mast_old`.
The camera heading is now **-90°** from the same link_1 local -X reference.
This replaces the previous initial-side assumption. Diagnostic labels read the
signed angle from the final USD instead of assuming a positive sign.

New private evidence: `outputs/eco65_native/j1_mast_20261016_yaw180_run01/`.
The four joint poses, other scene fields, CAD bytes and mesh asset hashes remain
unchanged. The directed original-mesh probe no longer finds the previous case-D
mast/link_3 crossing; this does not qualify full clearance or hardware mounting.
The new interface close-up shows a gap beneath the connector at the unchanged
height. The previous envelope-contact assumption is not carried forward as a
verified fit. This is retained as a visible installation issue for user review.

## October 16 mounting-gap closure

The user then requested a visible connection between the silver base and J1
shell. `eco65_mast_seating.vertical_seating` intersects projected original
triangles and solves their linear height difference, with rotation and XY fixed.
The base can move down 5.9882304 mm before first contact. The resulting close-up
still showed a gap because the shell is curved and the base is flat; this
intermediate result is preserved in the private `j1_mast_20261016_seated_run01`.

A separately identified project-derived conformal seat fills the remaining
interface. `conformal_seat` constructs closed cells between original surface
planes, retaining source openings and omitting ambiguous or over-15-mm patches.
The derived mesh is bound by SHA-256 into the new static snapshot. It is not
represented as an original CAD part or a verified manufactured mounting kit.
Original CAD/robot meshes, yaw, four joint poses and the rest of the layout stay
unchanged. The seat is a child of `Robot/link_1/MountingSeat`; its visual and
exact-triangle collision representation share geometry and transform. It is
only included in this static USD, with no planner acceptance implied.

Private evidence resides in `outputs/eco65_native/j1_mast_20261016_connected_run01/`:
`build/seating_geometry.json`, `build/seat_manifest.json`,
`build/connection_change_check.json`, `build/mast_surface_probe.json`, the four
composed USDs, Isaac PNGs, readback checks and timing records. The derived seat's
surface-plane residual is below 3e-14 m at construction samples; actual USD
points/transforms are checked independently during rendering. The original
base/J1 directed mesh probe finds no transverse crossing in A/B/C/D, excluding
coplanar contact; this is not a full clearance proof. Fasteners, load capacity
and manufacturing fit remain unqualified. The camera stays -90° and the time
axis stays stopped. No motion, planning, physics settling or camera algorithm
is introduced; all private assets and images remain ignored.
