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
