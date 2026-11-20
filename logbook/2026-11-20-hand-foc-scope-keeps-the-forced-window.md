---
title: "hand_foc_scope saves the window when the scope has to be force-stopped, and survives a failed capture (FEAT-49)"
type: bugfix
date: 2026-11-20
status: complete
phase: "BUG-57 scope tooling"
subsystem: tooling
files_changed:
  - tools/probes/hand_foc_scope.py
  - tools/probes/README.md
  - tests/motion/test_hand_foc_scope.py
---

# hand_foc_scope saves the window when the scope has to be force-stopped, and survives a failed capture (FEAT-49)

## What

On the robot on 2026-11-20 the fault trigger fired on a real SPINOUT, then `scope_download` found `oscilloscope.recording` still true 3.0 s later, stopped it with `ram_osci_trigger(1.0)` and raised, so the tool exited and the ring was thrown away. Hypothesis (not established): after the axis disarms, the drive's scope does not reach its stop condition for a post-trigger fill.

## Change

- `Drive.scope_download(..., keep_forced=True)` (used by `capture_once`) still force-stops after the unchanged 3.0 s, waits up to 1.0 s for `recording` to read false (the odrive package's own download would wait for ever otherwise), downloads, and sets `forced_stop`. The default stays the old raise (probe and timing use it).
- A forced window is saved as `<label>_<n>_<kind>_forced` (e.g. `_fault_forced`); its JSON has `forced_stop: true`, `trigger_position_known: false` and a note that the ring ends at the forced stop, so the fault may lie outside it if the scope kept recording meanwhile. `analyze` prints FORCED STOP and "trigger point UNKNOWN". Normal-path names and metadata are byte-for-byte as before (pinned).
- A download that fails after a forced stop says so in its error; a scope still recording after the forced stop says nothing was downloaded.
- `cmd_capture` catches an exception from one capture (traceback printed) and re-arms; 3 in a row (`MAX_CONSECUTIVE_FAILURES`) exit 4; a success resets the count. A failing analysis of an already saved window is printed, not fatal. Ctrl-C is unchanged.

Not exercised on the drive: whether a forced window actually holds the fault is unknown until the next fault capture.

## Verification

- Readers: `grep -rn hand_foc_scope . --exclude-dir=temp --exclude-dir=logbook` (2026-11-20): `tests/motion/test_hand_foc_scope.py`, `tools/probes/README.md`, plus `map/` items and sheets that only cite the tool.
- 2026-11-20, `python -m pytest -q tests/motion/test_hand_foc_scope.py`: 87 passed (8 new: forced window saved and tagged, normal path unchanged, Drive forced download, failed-after-forced message, still-recording-after-force message, one failure re-arms, cap exits 4, success resets the count).
