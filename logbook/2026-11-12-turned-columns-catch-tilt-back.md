---
title: "Turned Columns at 90 deg: every auto-banked Catch tilts the Platform back 4 deg toward +x, runtime-configurable (IDEA-22)"
type: feature
date: 2026-11-12
status: complete
phase: ""
subsystem: skill-stack
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tools/probes/columns_golden.py
  - tests/motion/test_turned_catch_tilt_back.py
  - tests/ros/test_skill_node.py
  - logbook/INDEX.md
tags: [IDEA-22, TKT-62, IDEA-11, columns_turned, catch-tilt]
related_items: [IDEA-22, IDEA-11]
---

# Turned Columns at 90 deg: the Catch tilt-back

## What and why

Owner, Sitting 2026-11-10: 90 deg turned Columns failed again (4 Attempts, 8/8, 3/3, 3/3, 3/3, all MISSED_CATCH; the ball struck the Platform top), "make the 90 deg version tilt the Platform slightly back (+ve x) when catching". TKT-62: "I lean Option 2 ... a default of ~3-5 deg tilt back from the currently-computed catch orientation, but it'd be good if this was runtime configurable."

**Interpretation (raised for the owner).** Option 2 was "pin a fixed attitude per Site instead of auto-banking"; the owner's words describe an OFFSET added to the currently computed (auto-banked) Catch orientation, which was Option 1's mechanism. Built what the words say: computed tilt plus a tilt-back offset.

New skill_node parameter `columns_turned_catch_tilt_back_deg` (default 4.0, |value| <= 12 deg, refused while a goal is active, like `columns_angle_deg`). Applied ONLY to `columns_turned` with `||angle| - 90| <= 0.5 deg`; every other Pattern and turn, and a value of 0.0, compile today's schedule.

## Discussion

**Direction.** The owner's "+ve x" is followed exactly: the offset is a rotation about Robot-frame **+y** by +4 deg, `(rx, ry) = (0, +0.0698 rad)`, so the Cup normal `tilt_geometry.cup_axis` is `(sin 4, 0, cos 4)`, leaning toward **+x**. At 90 deg the pair axis is y (P1@90 at -y, P2@90 at +y), so this rotation is about an axis PARALLEL to the pair axis and the lean is perpendicular to it. It is a Robot-frame direction, the same at -90 deg. Geometry check: a self-Throw's arrival is near vertical, so the auto bank is ~0 and the commanded Catch tilt is the offset itself. The 2026-11-10 fits landed 9-23 mm in -x; if the ball drifts -x it arrives moving -x, and the collinear rule (`tilt_to_receive`, Cup axis anti-parallel to the arrival) leans the Cup toward +x too. So the owner's +x and the banking rule agree in sign. Whether 4 deg is the right AMOUNT is the Sitting's question, not geometry's.

**Where the offset enters, and the tradeoff accepted.** The cleanest home was `unified_cycle._receive_pin_for` (auto tilt + offset, keeping the `'match'` catch program). It is NOT reachable: the Catch crosses the `InstallSegment` wire to `trajectory_node`, and `jugglebot_interfaces` cannot change on this branch. So the offset is folded on the Jetson side into the EXISTING `receive_tilt` pin by one function, `segments.catch_receive_tilt`. It takes `tilt_to_receive(landing_vel)` (the same call and the same 12 deg default `_receive_pin_for` uses), adds the offset in `(rx, ry)`, and clamps the sum back onto the `MAX_TILT_DEG` disc, direction kept. Both the executor's `_catch_terminal` (two sites) and the nomination's terminal build call it, so the probe and the flight plan the same pin. The accepted consequence is that a pinned Catch is planned under the pinned-catch semantics. `_events_for` retargets the soft catch MATCH onto the pinned Cup axis (`vel_target = axis (v . axis)`), so the Cup's matched velocity leans 4 deg: |v| sin 4 deg cos 4 deg (|v| = 4.35 m/s), about 300 mm/s of soft lateral target toward -x at touch-down. The evidence label reads `'level_pin'`. CATCH_CLOSING_SPEED treats `'level_pin'` and `'match'` alike and judges along the pinned axis. The ball's perpendicular velocity in the Cup frame (|v| sin 4, about 300 mm/s) is not changed by axial Cup motion, so the in-Cup skid is the tilt's own, as intended. Watch for a lateral slide at the Catch in the Sitting.

**Clamp.** The offset is applied AFTER the auto bank's own clamp and the sum is re-clamped, so the commanded tilt can never exceed 12 deg. Inside that, the gates see the final tilt: `cup_realize.prescribed_tilt_fn`'s tilt disc, `validate_cycle` (leg jerk/acc/vel) on the emitter, the online check (`_checker` builds the same request) and the nomination. A too-large offset is therefore refused, never flown. The parameter itself is refused outside +-12 deg, and a launch-file value is clamped there.

**Walk-through (one Catch, probe `/tmp/probe_idea22/probe4.py`, P4 limits 600/8000/200000/4500, Apex 0.95 m, Sep. 125, Dwell 0.27, 90 deg, hop entry).** Schedule: THROW 0 hop, the Feed CATCH (pinned Level, offset None, unchanged), then four auto-banked CATCHes each carrying `receive_tilt_offset = (0, 4 deg)`. Executor dispatch: `_catch_terminal` builds `receive_tilt = tilt_to_receive(v) + offset`. The first auto Catch is Ball 1's hop-entry Catch, arrival (0, 142, -4350) mm/s, auto bank (1.869, 0) deg, pin (1.869, 4.0) deg. Wire: `receive_tilt_set=True`. trajectory_node: `_receive_pin_for` returns the pin. The pose planner prescribes the tilt series start -> pin at `catch_k` -> throw tilt with the quintic sweeps (`h_catch_pre/post`), so the pin is a smooth target, not a step. At 3 deg the realised plan holds exactly (1.869, 3.000) deg at catch knot 12; the largest per-knot change is 0.62 deg (25 ms knots; 23.6 deg/s peak rate) against 0.39 deg at 0 deg. **The price is leg jerk.** The tilt swings on and off inside transit + Dwell, and nomination at P4 gives margin 0.346 at 0 deg, 0.228 at 2, 0.024 at 3, and a **refusal at 4** (LIMIT_JERK 212k > 200k, leg 5, t = 0.400 s, on CATCHes 2-4; no viable configuration within 4 steps of Apex / Sep. / Dwell). 45 deg is unchanged (0.0355 at every tilt-back). The default stays 4.0 as briefed, and the nomination is what stops it at P4.

**Rejected.** A per-Site fixed pin (Option 2 as labelled): it drops the arrival-matched seat, and it is not what the owner's words say. A trajectory_node param: that node does not know the Pattern. A Schedule-free executor setting: the Schedule is the one data structure (plan § 0), so the offset rides on `Pattern.catch_tilt_offset` -> `Skill.receive_tilt_offset` and survives `with_events` through `BeatSource`.

## Fix

`segments.catch_receive_tilt` (the one fold). `Skill.receive_tilt_offset` (CATCH only, exclusive with `receive_tilt`/`hold_tilt`), `Pattern.catch_tilt_offset`, and `compile_beats(receive_tilt_offset=)` carried in `BeatSource`. The Feed Catch never gets it. `nominate.turned_catch_tilt_offset` (the rule), `NominationConfig.catch_tilt_back_deg`, and `build_pattern`; `config_text` names the tilt-back when applied. skill_node param + set-time check, read only by `_nomination_config`, so the node's Pattern and the nomination's are one. `tools/probes/columns_golden.py`: a field added later whose None is the old compile is serialised only when set (`_ABSENT_WHEN_NONE`), so all 227 golden cases, the 45 and 90 deg pairs included, still pin it None without regenerating the fixture.

## Verification

- New tests (`tests/motion/test_turned_catch_tilt_back.py`, 23; `tests/ros/test_skill_node.py` +2): the original 21 + 2 were seen FAILING with the source changes stashed, passing with them (2026-11-12); the two added after the audit (negative tilt-back fold, t0 initial Catch carries the offset) and the strengthened clamp assertion were not re-run against the stashed tree. Post-audit run (2026-11-12, `tools/night_shift_nonet.sh python -m pytest -q tests/motion/test_turned_catch_tilt_back.py`): **23 passed in 0.59 s**.
- Scoped (2026-11-12): `tools/night_shift_nonet.sh python -m pytest -q -n 4 --dist loadfile -m "not nightly and not serial" <77 files>`: **3477 passed, 2 skipped in 267.87 s**. The 77 files are every test file that mentions `columns_turned` / `receive_tilt` / `tilt_to_receive` / the golden or imports schedule / executor / segments / nominate / skill_node, plus `tests/sim/test_logbook_front_matter.py` and `test_docs_links.py`. The same list with `-m "serial and not nightly"`: 4 passed, 1 failed (`test_solver_worker_lateness.py`, a wall-clock test, at load average ~6 with the parallel Night Shift units). Re-run alone the same morning: `tools/night_shift_nonet.sh python -m pytest -q tests/motion/test_solver_worker_lateness.py`, **1 passed**. That test does not touch the Catch tilt.
- Probes (uncommitted, `/tmp/probe_idea22/`): the nomination margins and the walk-through numbers above.
