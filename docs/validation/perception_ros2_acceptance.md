# Perception / ROS 2 acceptance record

Date: 2026-09-08 (Asia/Shanghai)

## Frozen references

- Branch: `feat/v0.5-perception-ros2`
- Baseline: `c5fcf4da58183b50ac79f37ae88d918389becf70`
- Visual upstream: `1d208f2ed380a207e6e46b4a62d2ac640edfe477`
- Feasibility consumer: `c6fa45595833088319e342c7ef61cc94afef3f58`
- Online consumer: `2c337098ecc5e70363cb966750c5b4f336a8d9cd`
- Shared schema/package: `1.0.0`

## Commands actually run

The available host is Windows 10 build 26200, AMD64. An isolated uv-managed CPython 3.10.21 environment was used; no system Python or ROS installation was modified.

```text
python -m pytest -q  # first baseline attempt
234 passed, 1 deselected, 9 setup errors: sandbox denied the default user temp directory

python -m pytest -q -p no:cacheprovider --basetemp <workspace>/baseline-clean
243 passed, 1 deselected in 22.64s (frozen baseline worktree)

python -m pytest -q --basetemp <workspace>/full
267 passed, 1 deselected in 22.28s

python -m unloading_perception.demo --mode replay --input tests/fixtures/vision_upstream/cargo7_minimal.json
completed; four real-format records retained and correctly non-admissible

python -m unloading_perception.demo --mode synthetic
completed; snapshot, mock authorization, separate cancel acceptance and measured stop confirmation emitted

standalone Python 3.10 venv: pip install --no-deps ./packages/unloading_contracts
passed; import did not load numpy, rclpy, or torch

online fixed-SHA detached worktree + compatibility patch:
type identity assertion passed; 68 online backend/session tests passed in 1.43s

feasibility fixed-SHA detached worktree + adapter patch:
adapter called the branch's real generate_suction_candidates entry and returned 36 candidates

git diff --check; python -m compileall ...; XML parse for all package.xml files
passed
```

Installed host test versions were NumPy 2.2.6, PyYAML 6.0.3, Matplotlib 3.10.9, and pytest 9.1.1. These are host test facts, not the formal Humble image dependency set.

## Independent status

| Capability | Status | Evidence / limit |
|---|---|---|
| Real upstream JSON replay | PASS | Fixed-SHA extracted fixture; null/missing/rejected/completed/bag/unknown semantics exercised |
| Real visual model inference | NOT RUN | No GPU/model environment; upstream also has staged/generated prerequisites rather than one verified real-time entry |
| Evidence admission and immutable world snapshot | PASS (CPU contract) | Python 3.10 tests and both demos |
| Cross-branch contract compatibility | PARTIAL PASS | 68 online tests/type identity pass; feasibility adapter calls the real candidate generator (36 candidates); no physical plan executed through adapter |
| Ubuntu 22.04/Humble build and communication | NOT RUN LOCALLY | Host has no Docker, usable WSL, ROS, or `rclpy`; pinned CI/reproduction path supplied |
| Actual physical planning validation | NOT RUN | Remains owned by feasibility branch; contract checks never claim collision/load/dynamics success |
| Mock execution | PASS (CPU contract); NOT RUN (ROS) | CPU gate/stop tests pass; Humble launch test supplied but unavailable locally |
| Real robot execution | NOT IMPLEMENTED / REFUSED | `enable_hardware=true` fails at startup |

The formal Humble commands are in `docs/setup_ubuntu2204_ros2_humble.md`. They must be executed on Ubuntu 22.04 amd64 with ROS 2 Humble before marking ROS build, TF/topic/action/cancel/stop communication as passed.

## Remaining evidence

Real execution eligibility still requires independently validated scale, camera calibration identity, capture-time world TF, defensible pose/size uncertainty, current robot/mechanism state, physical planner success, time parameterization, independent validation, and an unchanged authorization context. The real replay intentionally demonstrates rejection because those facts are absent.

The online patch must be imported during migration before session construction; a later consumer commit should replace the compatibility shim with direct shared imports. The feasibility patch still needs a project-specific wrapper around its real `Cell/evaluate_task` setup and a real timed-trajectory mapping. Neither named consumer branch was modified or pushed in this work.
