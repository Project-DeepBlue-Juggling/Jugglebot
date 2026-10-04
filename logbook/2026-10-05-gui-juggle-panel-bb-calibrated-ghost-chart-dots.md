---
title: GUI — Juggle command panel with a live pattern diagram, Ball Butler placed by calibration (ghost until then), chart sample dots, Event Log layout
type: feature
date: 2026-10-05
status: resolved
phase: two-ball-skill-stack — R5 (operator surface)
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/gui/css/juggle-panel.css
  - ros_ws/gui/test_juggle_panel.html
  - ros_ws/gui/js/state-minimap.js
  - ros_ws/gui/css/state-minimap.css
  - ros_ws/gui/js/ball-butler-model.js
  - ros_ws/gui/js/viewer.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/test_bb_ghost.html
  - ros_ws/gui/test_robot_models.html
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/command-history.js
  - ros_ws/gui/js/event-store.js
  - ros_ws/gui/css/panels.css
  - ros_ws/gui/css/theme.css
  - ros_ws/gui/index.html
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - tests/ros/test_orchestrator_node.py
subsystem:
  - gui
---

# GUI: Juggle panel, calibrated Ball Butler, chart sample dots, Event Log layout

## What changed (owner's four asks, 2026-10-04)

1. **Chart sample dots.** While the pointer is over any ODrive chart, every chart dots its real
   samples (`drawSampleDots` in `telemetry-charts.js`, drawn in the existing draw hook). Opacity
   fades in between 4 and 10 px mean on-screen sample spacing, so at the default 10 s window
   (20 Hz ≈ 2 px apart) nothing is drawn, and close zooms show every sample.
2. **Event Log.** Rows are now dot | label | right-aligned time. The `ros2` (connection) colour
   moved from rose `#f43f5e`, which was too close to fault red, to `--accent-green`.
3. **Ball Butler placement.** The 3D model is placed from `bb/calibration_result` and no longer
   from the QTM `Ball_Butler` rigid body, which only labels BB's markers. Group heading =
   `yaw_offset_rad − π/2` (the CAD throws along +Y at yaw 0; the FK throws along
   `yaw + yaw_offset` from world +X). Origin = `position_mm − (0, 0, BB_PITCH_Z_OFFSET_MM)`,
   because `position_mm` is the yaw axis at pitch-axis height. Until a `success:true` result is
   known (initial load, websocket drop, failed calibration) BB is a translucent ghost: it orbits
   Jugglebot at ~700 mm and tumbles. It glides solid onto the calibrated pose over 1.25 s.
   `viewer.js` gained `onFrame(cb)` on its existing rAF loop.
4. **Juggle panel** (`juggle-panel.js`, mounted under ACTIVE in the state-machine minimap; the
   old one-row strip is removed). It has pattern, Throws, Apex, Sep. (off for self-toss),
   read-only Dwell, and Reload (forced on for `columns_1ball_fed`). Start is hold-to-confirm and
   gated on connected + ACTIVE + cooldown + valid fields; Stop is an immediate click gated on
   connection only. The panel also shows:
   - an animated SVG of the selected pattern (sites, apex, Ball 1/Ball 2, phantoms hollow, Ball
     Butler feed arc) on the schedule's own timing at ½× speed;
   - flight, beat and transit times plus a duration estimate;
   - the gate reason or the exact request string;
   - a RUNNING indicator and the last outcome or refusal from `skills/attempt`.
   Blank fields send nothing, so skill_node uses its own defaults; those defaults are fetched
   from `/skill_node/get_parameters` and shown as placeholders.
5. **Relay.** `orchestrator_node._parse_juggle_request` now accepts
   `<pattern>[,reload][,apex_m=…][,separation_mm=…][,num_cycles=…]`, with tokens in any order.
   An unknown, malformed, negative or non-finite token REFUSES the request rather than being
   dropped.

## Discussion

- **Refuse, never drop, in the relay.** Dropping an unparseable token would fly skill_node's
  default in place of a value the operator typed, and still return a success ACK. Zero stays
  legal on the wire as the "node default" sentinel; the GUI refuses a typed 0 so that a blank
  field is the only way to ask for the default.
- **Old-relay compatibility.** The installed `jugglebot` package is a copy (not symlinked), so a
  robot running the pre-change relay reads only `parts[1] == 'reload'`. The panel therefore
  always puts `reload` straight after the pattern. It also compares the next `juggle_start`'s
  RESOLVED apex, separation and throws with the typed values, and shows an amber "relay out of
  date?" warning on a mismatch, so a stale install cannot fly the defaults silently.
- **Dwell is read-only.** It is a skill_node parameter, not a Juggle goal field, and the
  admissible box is swept at it (`check_limits` refuses an unswept dwell). The owner decided on
  2026-10-05 to keep it view-only.
- **Mirrored BB hand side — the GUI is right; the FK sign is now the open question.** A numeric
  check of the CAD chain against `throw_ballistics.bb_release_state` matches along-throw and
  vertical exactly, but the lateral hand offset is mirrored. The owner confirmed on 2026-10-05
  that the physical hand is to the LEFT of the pitch axis, looking along the throw, which matches
  the CAD. The FK puts it on the RIGHT. Worked case: at world yaw 0 the throw is along +X, and
  the lateral term `s·(−sin ψ, cos ψ)` = `−105.65·(0, 1)` puts the release at −Y, which is the
  right-hand side with Z up. `yaw_solve_thetas` uses the same `s`, so the forward and inverse
  models agree with each other and the round-trip tests cannot see this. A 2|s| = 211 mm
  lateral release error would be visible in throws unless something absorbs it; the most likely
  candidate is the yaw-zero convention anchored on marker 4, which would cancel it at one range
  only (≈17° at 700 mm). NOT changed here: it is aim-bearing and needs a hardware check (same
  azimuth, two ranges — a mirrored `s` leaves a range-dependent lateral miss).

**Follow-up, 2026-10-05 (owner):** the expanded state-machine graph was trimmed so the Juggle panel
fits without scrolling. ACTIVE now ends ~13 px below the SPACEMOUSE/GUI chips (was ~75 px), and
the SVG ends just below ACTIVE (viewBox height 432 → 344). The ACTIVE-to-JUGGLE gap fell from
98 px to 29 px, most of which is the status line (CDP measurement on the live GUI).

## Verification

- `pytest tests/ros/test_orchestrator_node.py -q -k Juggle` (2026-10-04): **33 passed**, which
  includes the new parser tests (overrides, any order, explicit 0, ten refused bad tokens).
- Headless-chromium CDP shots, 2026-10-04/05, in `temp/probes/gui_shots/`:
  - `dots_combo.png`: 10 s / mid / close zoom / no hover, with 20 Hz synthetic telemetry;
  - `jp_montage_{a,b,c,d}.png`: every pattern and state, light theme, 12–16 px fonts;
  - `integ_expanded.png`: the panel mounted in the live GUI;
  - `bb_ghost_*.png` and `bb_cal_final2.png`: ghost and calibrated BB.
- `test_robot_models.html` self-test: every BB assertion passes, including "All CAD stays
  opaque". The page then stops at "Base, Platform and BallButler triads plus marker retained",
  which fails identically at HEAD (`c98819e2` GUI tree, served separately), so it pre-dates this
  change.
- Full gate: see the commit message.
