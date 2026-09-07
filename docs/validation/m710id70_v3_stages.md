# M-710iD/70 V3 validation work log

Reference: `1a9e7be084903b3d4d2471659b645e52a367ef8e` (`release/v0.4`).
Working branch: `fix/m710id70-v3-validation`. Initial worktree was clean.

## Stage 1 — frozen V2

Run the original acceptance entry point before changing imported modules:

```powershell
.venv\Scripts\python.exe tools/run_m710id70_acceptance.py --output-dir outputs/m710id70_v3/v2_baseline --grid-step 0.30
.venv\Scripts\python.exe tools/capture_m710_evidence.py --output-dir outputs/m710id70_v3/v2_baseline --command "python tools/run_m710id70_acceptance.py --output-dir outputs/m710id70_v3/v2_baseline --grid-step 0.30"
```

The baseline script's effective settings differ from the YAML: trailer 2.3 ×
2.7 m, walls X [-1.5, 2.2] m, carton front X 1 m, box A [0.6, 0.4,
0.3] m and B [0.4, 0.6, 0.3] m, box mass 42.5 kg, 20 kg tool,
collision margin 0.001 m (YAML says 0.01 m). Grid seed 71070 + row
index; random scenes 71071, 71072, 71073; continuous planning seed
74000 + scenario_index*1000 + sequence*50 + candidate_index. Base is
selected by the original coarse scan, not YAML base_position.

V2 grasp/extraction IK tolerances: 0.012 m / 0.09 rad; handoff:
0.015 m / 0.10 rad. The inherited common scene is not read by V2.
Only successful continuous boxes are exported by V2; this logging limitation
is part of the baseline, not evidence that other cartons were attempted.

The source manifest records input file hashes and raw configuration documents.
The V3 report will include the reproduced counts, effective V3 settings and
each change in acceptance meaning. Historical V2 successes are not safety
certificates and will not be forced into the corrected result.

V2 reproduction completed: 82/104 grasp, 66/104 extraction, 52/104 full
geometry; regular 0/40, random 71071 2/27, 71072 0/32, 71073 0/30.
No count discrepancy. Selected mount X=-0.5 m, Z=0.7 m.

## Stage 2 — parameter and rigid-frame contract

Added standalone `configs/validation/m710id70_v3.yaml` with strict inherited
overrides and leaf-level parameter provenance. Robot limits and physical tool
properties are loaded from their source files. All TCP translations and
rotations are composed with the URDF flange/tool0 relation. Corrected tool
geometry from [long, short, depth] storage to [short, long, normal] task axes.
Captured `T_TCP_box` is used for both collision and load calculations.

The corrected tool-vs-arm check rejects V2 home (`tool_envelope`, `J4_link`).
A separately recorded offline initialization search (3000 draws, seed 71070)
found 233 valid states with the regular stack; draw 1507 is the nearest to
V2 home. V3 explicitly uses this configuration; no motion from the invalid V2
home is claimed. `outputs/m710id70_v3/home_probe.json` records the witness.

The V3 collision margin is the configured 0.01 m, not V2's hidden 0.001 m.
OBB SAT expands both shapes by this margin, so it requires 0.02 m separation
on a shared face axis. This existing primitive meaning is preserved, not
silently changed to improve results. Only designated nonpenetrating contact
planes can waive the clearance margin; penetration beyond 0.0002 m fails.

## Stage 3 — rotation, IK and complete motion checks

SO(3) near-pi reconstruction uses a symmetric pivot to recover mixed axis
signs and atan2 for the angle. Joint centering is projected into the true
null space; convergence is decided by actual FK residual. Invalid converged
branches immediately yield to the next deterministic IK seed.

The V3 planner validates current state, pregrasp transit, contact, extraction,
carry, actual-FK support and empty withdrawal. It checks interpolated edge
interiors at at most 0.01 rad joint spacing, reads all URDF primitives and
adds tool/arm and payload/arm checks. It retains primitive approximation and
sampling limitations explicitly; it is not a mesh/continuous collision
certificate. Original capsule self-pair exclusions remain engineering
approximations requiring CAD review.

Targeted numerical/timing regression: 54 passed. Additional motion-contract
and numeric-contract run: 46 passed, including independent angular and linear
Jacobian differences, frozen scene population equality, missing capsule
corners, interior collision and invalid-home detection.
