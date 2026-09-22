# Validation-kernel evidence, 2026-09-22

Start commit: `4f6e3037aceb47a525711e58360d98c8751fec6b`.
Planning and Isaac source: `9b90c6bf6b4528041df2bfd13ef07079a23d1d81`.
`source_manifest.json` binds the exact uploaded source bytes. Subsequent commits
only correct report counters and retain direct-rejection evidence; they were
tested separately and did not replace the running source.

The baseline/current benchmark uses the same benchmark script, model, policy,
inputs and CPU environment on the configured GPU server. Run from each tree:

```sh
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python \
  tools/benchmark_validation_contract.py --output ../current_benchmark.json
```

`final_tests.log` is the seven-file, 93-test directed suite. Reporting corrections
are covered separately by `reporting_tests.log` (25 tests). This is not a full
pytest run. Profile records accompany the machine-readable benchmark reports.

The current benchmark was captured before the reporting-only cache counter fix:
cached `state_samples` in an edge result represents reused proof coverage, not
fresh computations. Actual execution work is recorded in instrumented counters
and validator totals. The final branch reports reused coverage separately.

Full input, planning, physics and video artifacts are retained under
`outputs/delivery/20260922_validation_kernel/delivery/` in the working tree and
`/root/autodl-tmp/m710-validation-kernel-20260922/delivery/` on the GPU server.
The source planning delivery's `isaac_executed: false` describes that planning
stage only; it is retained unchanged. The separate physical result is authoritative
for the one subsequently executed new-world trial.

See [the technical report](../../validation_kernel_motion_validator_recovery.md)
for interpretation, measured regressions and limitations.
