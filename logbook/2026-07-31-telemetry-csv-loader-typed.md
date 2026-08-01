---
title: Telemetry CSV loaders crashed on current-schema CSVs (throw_phase) and silently mistyped int fields — replaced by one typed load_records()
type: bugfix
date: 2026-07-31
status: resolved
phase: "Refactor programme Phase 0 — analysis-layer bugfix"
files_changed:
  - controller/telemetry.py
  - sim/analysis/compare.py
  - sim/viz/telemetry.py
  - tests/sim/test_telemetry_roundtrip.py
subsystem:
  - sim
  - controller
---

# Telemetry CSV loaders: typed load_records()

**What**: `sim/analysis/compare.py` carried a duplicate CSV loader that
hard-coded `solve_status` as the only string column; `StepRecord` gained
`throw_phase: str` (default `""`), so the next current-schema CSV crashed
every `/diagnose` / compare / interactive-plot entry point with
`ValueError: could not convert string to float: ''` (masked only because the
newest CSV in `temp/logs` is old 65-column schema). A second, quieter defect:
`TelemetryLogger.load()` derived only `float` fields from annotations, so
int-annotated fields (`ball0_held`, `catches_total`, `fk_iterations`, …)
round-tripped as `str`.

**Fix**: one canonical `load_records()` in `controller/telemetry.py` restoring
every column from the dataclass annotation (float/int/str; unknown annotation
hard-fails so a new field type can never silently round-trip as str; int cells
tolerate historical `'1.0'` formatting). `TelemetryLogger.load()` and
`analysis.compare.load_csv` (all four consumers keep their import path)
delegate to it. Load-side only: `controller.telemetry` IS imported by the
40 Hz loop (`runner.py`, `run_mpc.py`), but the hot-loop append/flush path
is byte-unchanged — the new code runs only in post-hoc analysis.

**Verification**: `pytest tests/sim/test_telemetry_roundtrip.py
tests/sim/test_compare_sessions.py -q`, run 2026-07-31: **26 passed in
1.91 s** (7 new round-trip tests pin types, not just values). Full gate:
`./run_tests.sh`, run 2026-08-01: **parallel 4292 passed + 3 xfailed in
434.35 s; serial 9 passed in 39.08 s; total 478 s — RESULT: PASS**.

Found by the 2026-07-31 codebase review (`plans/active/refactor-2026-07.md`
Phase 0); empirically reproduced in the venv before fixing.
