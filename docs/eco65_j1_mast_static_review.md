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
