---
title: Tilt calibration Phase 3 — grid acquisition tool, offline analyser, session runbook
type: feature
date: 2026-08-03
status: resolved
phase: "tilt-calibration-grid Phase 3"
related_plan: tilt-calibration-grid.md
files_changed:
  - tests/hardware/tilt_cal_grid.py
  - tools/tilt_cal_analyse.py
  - tests/hardware/session_tilt_calibration.md
  - tests/motion/test_tilt_cal_grid.py
  - tests/sim/test_tilt_cal_analyse.py
  - ros_ws/docs/levelling_frame.md
  - tools/README.md
  - plans/active/tilt-calibration-grid.md
  - logbook/INDEX.md
subsystem:
  - testing
  - tools
  - docs
  - safety
---

# Tilt calibration Phase 3 — acquisition tool, analyser, runbook

## What changed

The software half of contract **C-LEVEL-2** is now complete: Phase 1 built the
map core, Phase 2 wired it into `trajectory_node`, and this phase builds the
thing that *produces* a map. Nothing here runs automatically and nothing here is
in a safety loop.

- **`tests/hardware/tilt_cal_grid.py`** — operator-run rclpy CLI in the
  `traj_ramp_battery.py` family. Drives an (x, y) grid at one height (5×5 over
  ±150 mm at z = 170 by default, home node first), dwells, reads the
  inclinometer N times per node, computes
  `residual = mean(raw) + radians(inclinometer_offset_deg)`, writes
  `config/tilt_calibration.yaml`, calls `trajectory/reload_tilt_map`, confirms
  the node's published `tilt_map_version` matches what it wrote, then
  re-measures off-node check poses and prints PASS/FAIL. It issues
  `trajectory/go_to_pose` requests **only** — never arms, never changes mode,
  never commands the hand — and every exit path returns the platform to the
  centre node first.
- **`tools/tilt_cal_analyse.py`** — offline. Heat maps + quiver, per-node sd
  table, outlier flagging, `--diff` for map-invariance (rung C2) with the rung's
  own `max(2 × noise, 0.05°)` bound computed from the captures' recorded sd.
- **`tests/hardware/session_tilt_calibration.md`** — the runbook: danger banner
  (multi-minute continuous motion, E-STOP in reach, hand quiescent), the
  two-package build gate, uptime discipline, the fresh-`level` precondition, and
  rungs C0–C3 with numeric PASS/ABORT and per-rung data obligations.

The pure core is importable with **zero ROS** — `rclpy` and the interface
packages are imported inside `run()` — which is what lets the tests exist at all.

## Discussion

Three things here were not obvious, and one of them I got wrong first.

**The outlier detector was wrong, and the fix reframed what it measures.** The
first version flagged a node whose residual departed from the mean of its
four neighbours. On the first realistic probe field that flagged **six of
twenty-five nodes, all on the boundary** — and correctly, by its own logic: an
edge node's neighbours all lie on its interior side, so on any monotonic field
an edge node always "departs". The residual field *is* curved (the 2026-07-28
table refutes a *linear* fit, not curvature), so a first-order test is
structurally a false-positive factory. The right statistic is the **second
difference**: a leg pinned on its stroke clamp does not bend the field, it
**kinks** it, and a second difference ignores a pure gradient by construction
(pinned in `test_curvature_ignores_a_pure_gradient`). Corner nodes have no
second difference along either axis and are simply not testable — stated as an
honest limit rather than a silent pass.

**But no threshold on that statistic survived scrutiny, and I stopped rather
than tuning one.** On a probe field with structured curvature, `4 × median`
missed a real 0.5° pin while `2 × median` flagged ten good nodes. The tempting
move was to pick whatever separated my synthetic and ship it as a validated
threshold; that is exactly what the probe-first rule exists to prevent, and the
SCL3300's noise floor and the real field's curvature scale are both unmeasured
in this repo. So the flag is a deliberately coarse net, marked PROVISIONAL in
code, and the report **leads with a scale-free number that needs no threshold at
all** — the top/median curvature ratio. On a smooth field every node's second
difference is nearly equal so the ratio sits near 1; a single pinned node raises
only its own, so it jumps. Rung C1 produces the first real field and is what
should pin the constants.

**A physical detail nearly cost the whole measurement.** The Murata SCL3300 runs
mode 4 with a **10 Hz internal low-pass filter**, read one sample per trigger.
Reading N times back-to-back therefore resamples the *same* filtered state: the
mean is unaffected, but the **spread collapses** — and the per-read sd is what
gates the home node and sizes θ_acc. The tool would have reported a noise floor
the sensor does not have, and C0 would have "pinned" it. Hence `--read-gap-s`,
defaulting to 0.15 s, just over one filter period. This is a plan-level gap, not
a coding detail: the plan's per-node sequence says "N sequential reads" and says
nothing about spacing them.

Two smaller decisions, both fail-closed. **Failed nodes are refused, never
interpolated** — `--on-fail continue` finishes the sweep so the operator learns
*which* nodes are bad, but no map is written and the exit is nonzero; filling a
node from its neighbours invents calibration data at exactly the place the
machine had trouble. And the **home-node gate is two-sided**: within
`max(3 × sd, floor)` *and* under an absolute ceiling. One-sided fails one of two
ways — `3 × sd` alone is degenerate on a quiet sensor (the Teensy persistence
quantum is 1 mrad/axis by itself, so a 3e-9 rad tolerance would abort a perfectly
good capture, and a gate that fires on good data is a gate the third operator
disables), while a *noisy* sensor makes `3 × sd` wide enough to launder a
genuinely stale reference.

## Fix / implementation notes

- **Three flags beyond the phase's list**, each because a Phase-4 rung is
  otherwise unrunnable: `--no-apply` (C0 — a read-noise probe must not leave a
  calibration behind), `--verify-only` (C2a is *defined* as "re-`level` only, NO
  recapture"; check poses come from the loaded map's own axes so they are
  identical to the capture run's), and `--read-gap-s` (above).
- **No `__file__` walk anywhere in path resolution** — the write target is
  `tilt_map.tilt_map_candidates()[0]`, honouring the authoritative
  `$JUGGLEBOT_TILT_CAL`. This was the Phase-2 audit blocker, reached from the
  other direction; the post-write `tilt_map_version` readback is the hard
  guarantee, and it fails loudly with `APPLIED THE WRONG FILE`.
- **A real CLI bug, surfaced by its own test:** `--x -150,-75,0,75,150` — the
  documented explicit-list form — fails, because argparse only treats a leading
  `-` as a value for bare negative *numbers*, and its native message
  ("expected one argument") gives no hint. The parser now appends the `=` fix to
  that specific error rather than leaving it to be found at the robot.
- **Test placement follows precedent, not the phase text**: `tests/motion/` for
  the `tests/hardware/` script (matching `test_bench_sysid_logic.py`,
  `test_kt_lib.py`) and `tests/sim/` for the `tools/` script (matching
  `test_check_vel_ff_plumbing.py`). No `serial`/`nightly` markers; `tmp_path`
  only.
- The strongest test drives the **real** `state_machine.LevellingHandler`
  `level_get_tilt` phase and compares its `pose_offset_rad` to the tool's
  reduction on the same reading. A residual is *defined* as that formula, so an
  assertion restating it would drift in lockstep with a sign flip — and a sign
  flip inverts every node, aiming the machine roughly twice as badly as no map
  at all.

## What the audit caught

An independent audit ran before commit; the findings were taken, not argued
down. The full list is in the plan's Phase-3 Outcome. The one that mattered:

**A second Ctrl-C escaped the return-to-centre guard.** The guard caught
`Exception`, which does not catch `KeyboardInterrupt`. The return blocks for up
to `--timeout-s` plus the move, so the reflex second interrupt — the one you
give a program that does not die on the first — landed inside it, propagated out
of the `finally`, and skipped the artefact writes, the rclpy shutdown **and** the
`RETURN TO CENTRE FAILED` message. The platform would have been left parked at a
raised displaced pose, in silence, which is exactly what this tool's docstring
and the runbook's danger banner both promise cannot happen. It is now
`except BaseException` with the artefact write and the shutdown each in their own
`finally`, and `test_return_to_centre_guard_catches_base_exception` pins the
handler type structurally — `run()` cannot be exercised without a robot, so a
well-meant tidy back to `except Exception` would otherwise be invisible.

The audit also found that **the contract overstated its own enforcement**:
`levelling_frame.md` claimed the tool "refuses to start" unless all four capture
preconditions hold, when only *no map loaded* is machine-checkable. The contract
now carries a per-precondition enforcement table. A tool trusted to enforce what
it cannot observe is worse than one that says it cannot — and this is the shape
of drift CLAUDE.md's "change the document first" rule exists to prevent, reached
from the other direction: the code was right and the document was ahead of it.

## Verification

- Scoped: `pytest tests/motion/test_tilt_cal_grid.py tests/sim/test_tilt_cal_analyse.py -q`
  (run 2026-08-03, post-audit-fix tree): **76 passed in 1.05 s** (55 + 21).
- Python 3.8 compatibility (the tool runs under the system interpreter with ROS
  sourced, not the venv): `/usr/bin/python3 -m py_compile
  tests/hardware/tilt_cal_grid.py` (run 2026-08-03): **clean**; `--dry-run`
  under `/usr/bin/python3` prints the plan and makes zero ROS calls.
- Threshold probe (`/tmp/probe_tiltcal.py`, run 2026-08-03) — smooth field,
  5×5/±150, 8 reads/node at 2e-5 rad: top/median curvature ratio **1.01 clean,
  2.43 at a 0.2° pin, 3.65 at 0.3°, 6.07 at 0.5°**; the `4 × median` flag line
  fires only at 0.5°. These are the numbers cited in the analyser docstring and
  the plan's Phase-3 Outcome.
- Phase gate: `./run_tests.sh --full` (run 2026-08-03, post-audit-fix tree):
  **parallel 4617 passed + 3 xfailed in 451.48 s, serial 9 passed in 40.12 s,
  total 497 s — RESULT: PASS**.
  **This run is also the mandatory pre-sitting gate for Phase 4**: CLAUDE.md
  requires `--full` before ANY hardware sitting, and the C0–C3 rungs in
  `tests/hardware/session_tilt_calibration.md` are the next thing that happens
  with this code.

**No hardware ran.** Every number above is offline. Nothing in this phase has
been exercised against a robot — that is Phase 4, and the runbook is its
authority.
