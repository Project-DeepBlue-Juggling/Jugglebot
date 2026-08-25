---
title: "Archival-review close-out: the vendored ODrive-decoder defect class, the UDP panel's units, the arc floor, and the silent sibling write"
type: bugfix
date: 2026-08-25
status: resolved
phase: "operator-observability — archival close-out"
related_plan: operator-observability.md
files_changed:
  - tests/hardware/free_platform_test.py
  - tests/hardware/supported_platform_test.py
  - tests/hardware/single_leg_test.py
  - tests/ros/test_gui_geometry.py
  - ros_ws/gui/js/udp-traffic.js
  - ros_ws/gui/js/odrive-errors.js
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py
  - tests/ros/test_bb_calibration_arc_span.py
  - config/generate_config.py
subsystem:
  - gui
  - ros
  - config
tags:
  - safety
  - testing
  - observability
---

# Observability close-out fixes

## Summary

Four fixable defects surfaced by the archival review of `operator-observability.md`, all fixed in the same session (house rule), in one review round.

## Fix

1. **Vendored ODrive-decoder defect class, closed across 3 harnesses.** `free_platform_test.py`'s `ERROR_CODES` had drifted (5 bits missing incl. `SPINOUT_DETECTED`, `0x1000000` mislabelled) and it never rendered `disarm_reason` — so the 2026-08-10 leg-0 spinout signature (`active=0x0`, `disarm=0x4000000`) printed **nothing**. The class proved wider: `supported_platform_test.py` was disarm-blind at all 11 report sites (Phase A had synced only its *table*); `single_leg_test.py` was disarm-blind **and** dropped unknown bits. All three now carry `decode_axis_errors(active, disarm)`, shape-matched to canonical `teensy_bridge_node._decode_axis_errors` (`:928`) under a "BOTH MASKS, ALWAYS" rule, with `has_fault` gates, `UNKNOWN(0x…)` residue, and a bounded 0.5 s post-`CLEAR_ERRORS` freshness wait (`ERROR_FRESH_TIMEOUT_S`). `tests/hardware/tilt_cal_grid.py` was checked and **exonerated** (already residue-correct and both-mask everywhere). Pins: `TestODriveErrorTablePins` 6 → 20 tests (14 added) — tables pinned, decoders pinned on *behaviour* (AST-lifted and executed against the canonical decoder, falsifiability verified pre-fix), both-mask usage tripwired, freshness wait pinned. A tautology hole went with it: `tests/motion/test_tilt_cal_grid.py:642` compares the imported table to *itself* whenever the import succeeds, so the fallback literal is now AST-lifted out of `tilt_cal_grid.py` and pinned regardless (`test_gui_geometry.py::test_tilt_cal_grid_fallback_table_matches_python`).
2. **UDP panel presentation** (`ros_ws/gui/js/udp-traffic.js`): the per-type **Gaps** column — proven a shared-wire-seq artifact, entry `2026-08-25-udp-gap-column-artifact` — is no longer rendered; headers unit-labelled `rx (msg/s)` / `tx (msg/s)`; hover totals labelled cumulative and blanking with the rates under LINK DOWN; footer aggregates keep last values, now visibly `cumulative:`. Counters and publisher untouched — their removal is P1 of `plans/archived/udp-channel-health.md` (P1 landed later the same day). Also one docstring correction in `odrive-errors.js` (`formatAxisErrors` no longer claims to match the bridge's shell rendering).
3. **`MIN_ARC_DEG` 20° → 60°** (`bb_calibration.py:80`), settled by the first real calibrate: owner-reported BB yaw span swept **118.8°**; the Phase-A probe showed ≤25° is noise-defeated; 60° sits ~2× under real operation. Test brackets moved (55° refused at all noise levels, 65° passes). `MIN_MARKER_RADIUS_MM` stays 20 — today's passing calibrate is its hardware confirmation. (The constant had three roles; only the sweep gate was raised. The per-marker INCLUSION threshold was split off as `MIN_MARKER_ARC_DEG = 20` — unchanged behaviour, threaded as its own `min_marker_arc_deg` parameter through `find_rotation_axis`/`run_calibration` — because a marker's fitted arc shrinks under occlusion inside a sweep that legally cleared 60°, and holding it to the sweep's floor would drop good markers out of a good calibrate.)
4. **`generate_config.py --no-external`** (owner-approved): an argparse flag suppressing every write outside this repo, default unchanged; external delivery now *always* prints an `EXTERNAL:` line naming the absolute destination. The silent sibling write had tripped two independent sessions.

## Discussion

**Climb to the class before fixing the instance.** The review named one drifted table in one harness. Fixing only that would have left two harnesses printing `active` alone — the same blindness one file over, and the exact failure the review had just caught. Enumerating the class first cost one extra pass and closed it: one decoder shape across all three, one canonical source to drift *from*, and behaviour pins rather than text pins so the next drift fails the gate instead of the bench.

**The B4 gate stays faithful, and may fail loudly on the bench.** A sticky-mask gate is only sound with a freshness proof — without one, a *previous* session's latch aborts the test that just cleared it — hence the bounded wait. The accepted risk runs the other way: `supported_platform_test.py` B4 / `test_estop` asserts a clean axis after commanding IDLE, so if the firmware records a reason for a *user-requested* disarm it now fails loudly with the named bit. Kept deliberately — a gate that ignores `disarm_reason` to stay green is the defect, not the fix. Same asymmetry drove the arc floor high rather than to the noise edge: a false refusal costs a re-run, a false accept ships a silently bad yaw offset.

**The footer keeps its numbers under LINK DOWN; the rates do not.** A frozen cumulative total is still true; a frozen rate is a lie about the present. So the rate columns and their hover totals blank together, while the footer aggregates keep their last values and carry the `cumulative:` label that earns them.

## Verification

All 2026-08-25, project venv:

- `pytest tests/ros/test_gui_geometry.py -q` → **106 passed in 1.51 s**
- `pytest tests/ros -q -p no:randomly` → **2343 passed, 1 skipped in 313.05 s**
- `pytest tests/motion/test_tilt_cal_grid.py -q` → **125 passed in 0.73 s**
- `pytest tests/firmware/test_config_drift.py tests/ros/test_bb_calibration_arc_span.py tests/ros/test_mocap_node.py tests/ros/test_mocap_status.py -q` → **102 passed, 1 skipped in 3.58 s**
- `./run_tests.sh` (mid-round, *before* the final two fixes) → **5923 passed, 4 skipped, 276 s, PASS**
- Decoder smoke: the spinout signature now renders `active=[] disarm=[SPINOUT_DETECTED] 0x0/0x4000000` in all three harnesses (was: nothing).
- Arc floor: probe `/tmp/probe_arc_floor60.py` (uncommitted) confirmed 65° also clears the downstream `max_dev` check under 0.5 mm noise.
- `--no-external`: a 510-file mtime snapshot of `~/Desktop/BallButler` was untouched under the flag.

**The `./run_tests.sh --full` gate triple for the complete tree is cited in the commit message, not here** — this entry lands alongside the other close-out edits.

## Outcome

Defect class closed across all four hardware harnesses (three fixed, one exonerated). Deploy: the GUI JS is static — **browser hard-refresh only**; the arc floor is live only after `colcon build`, because `mocap_node` imports the *installed* copy of `bb_calibration.py`.

## Open Questions

- **Watch on the first bench run:** `supported_platform_test.py` B4 / `test_estop` **and** `single_leg_test.py` `test_estop` may now fail with a named `disarm_reason` bit after a user-requested IDLE. Both report-and-score it — the `fault_summary` is printed and folded into the pass/fail terms, so it reads as a scored FAIL rather than a raised `RuntimeError`. That is the faithful gate working — read the bit before relaxing anything.
