---
title: The hand's end stop is 10.8 rev, not 11.1 — the shipped guard sat 0.3 rev past metal
type: bugfix
date: 2026-08-18
status: resolved
phase: "hand-command-continuity Phase 5 (anchor resolution)"
related_plan: hand-command-continuity.md
files_changed:  # sources of truth + the two firmware trees; the commit carries
                # the full set (generated copies, 7 test files, runbook, plan)
  - config/hardware_config.yaml
  - config/generate_config.py
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/can/odrive.py
  - ros_ws/src/jugglebot/jugglebot/motion/geometry.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - sim/hand/trajectory.py
  - tools/probes/hand_stroke_timeline.py
  - tests/hardware/session_anomaly_fixes.md
  - plans/active/hand-command-continuity.md
subsystem:
  - motion
tags:
  - safety
  - hardware
---

# Hand end stop corrected to 10.8 rev

## Summary

The hand's physical hard stop — metal contact — is **10.8 rev**. The operator
measured it on the sensorised hand; the hand changed slightly during ball-sensor
integration and the limit was never updated. Every declared value in the repo was
too high, and the error was **one-sided**:

| source | said | vs the real 10.8 rev stop |
|---|---|---|
| `hand_motor_max_position_revs` (shipped guard) | 11.1 | **0.3 rev / 9.5 mm PAST metal** |
| its own comment, "hand true max" | ~11.4 | 0.6 rev past |
| geometry from `hand_stroke_mm` 355 | 11.224 | 0.42 rev past |
| runbook H4.5 | 11.124 | 0.32 rev past |

The shipped guard was therefore **never protective**. `clip_position` — in the
host (`can/odrive.py`) *and* in the can-bridge firmware (`odrive_protocol.h`) —
clamped hand setpoints to `[0, 11.1]`, i.e. it would accept and pass a commanded
setpoint 9.5 mm into the stop.

It also mis-scored history. The 2026-07-27 sitting's five off-run-sheet ~1.2 m
tosses peaked at **10.8601–11.0621 rev**, recorded at the time as "1.2–7.6 mm
from the declared 11.1 rev limit". Against 10.8 they are **0.06–0.26 rev past
the stop** — that is the contact the operator felt, and the reason `logbook/
2026-07-29-hand-post-release-decel.md` § 2 could not close the question.

## Discussion

**The naming was the defect, not just the number.** `hand_motor_max_position_revs`
meant *"the hard limit"* to one reader and *"the hard limit minus a safety
margin"* to the next — its own comment asserted both at once ("hand true max
~11.4 rev. This value includes a safety margin"). A quantity with two meanings
cannot be checked against anything, which is exactly why a wrong value survived
three separate investigations that all quoted it. The sibling key
`leg_motor_max_position_revs` compounds it by meaning the *opposite* thing again:
its value sits **above** its stroke-derived limit.

So the key is renamed to **`hand_motor_hard_stop_revs`** — a measured physical
fact, with every margin now subtracted explicitly at its own site. The leg key is
deliberately left alone: the two now have different names because they have
different semantics, and the YAML says so, so nobody "fixes" the inconsistency
back into existence. The generated symbols follow automatically
(`GEOM_HAND_MOTOR_HARD_STOP_REVS`, `Geometry::HAND_MOTOR_HARD_STOP_REVS`) because
the geometry section is emitted by a generic loop; only the explicit JS export in
`generate_config.py` needed a hand edit.

**The smooth-move ceiling was deliberately held at 10.60 rev** (owner's call).
`smooth_move_excursion_margin_rev` moved 0.5 → 0.2 so that `10.8 − 0.2` lands on
the same 10.60 the firmware already used, which makes **every commanded
smooth-move profile bit-identical across this change**. The honest cost is stated
in the YAML rather than buried: 0.5 rev was 2.7× the measured +0.186 rev
position-loop tracking overshoot, and 0.2 rev is 1.08× it. That is not a
regression — the physical situation is unchanged, since the ceiling is unchanged
— but the arithmetic that believed it had 0.31 rev of clearance above the
commanded ceiling actually has **0.014 rev (0.44 mm)**. That is the "occasional
light tap" regime the owner explicitly accepted. Restoring a 2.7× ratio would
mean a 10.3 rev ceiling, declined because Phase 4's measured maximum commanded
prelude is 10.2259 rev and 10.3 would leave it 0.074 rev.

**One derived quantity genuinely moves**: `smoothMoveMaxDuration()` is defined as
the longest rest-to-rest move *the stroke admits*, so a shorter stroke shortens
it — **0.80054 → 0.78964 s**. That is the conservative direction (the cap fires
the rest-to-rest fallback marginally sooner), and the host window that consumes
it, `_PRIME_INFLIGHT_S`, was sized against the larger number. The velocity-
continuous band narrows with it: mid-stroke 20.90 → 19.96 rev/s, duration-capped
20.32 → 20.04 rev/s. The stroke-top figure (9.07 rev/s) is unchanged, because it
is sized off the ceiling, which did not move.

**Deliberately NOT changed: `hand_stroke_mm = 355.0`**, which now implies a top of
11.224 rev — 0.42 rev above the measured stop. It is tempting to "fix" it, and it
would be wrong to do so casually: 355 mm functions as a **throw-profile
parameter**, feeding `total_stroke → x2 / x3 / x5` (release point, stroke top,
catch point), all empirically validated on hardware. Re-deriving it would move the
release and catch points and needs its own validation round. Recorded here so the
inconsistency is a known, bounded one rather than a future surprise — it is the
same naming disease in a second place.

For context on what the margin actually buys: `x3` (9.9594 rev) now sits
**0.84 rev = 26.6 mm** below the stop, and the measured post-throw coast past x3
ran +0.51…+0.81 rev on 2026-08-10 and up to **+1.020 rev** at 4.858 m/s on
2026-07-27. A 0.84 rev coast reaches metal. The 2026-08-18 torque-clamp removal
([[2026-08-18-hand-torque-clamp-removed]]) is expected to shrink that coast: the
clamp bound total commanded torque at −10.00 A, and the fence is now
`current_soft_max` = 50 A, so the commanded braking torque (up to ~0.18 N·m =
32.7 A at the top tier) is no longer truncated. **By how much achieved braking
actually improved is unmeasured** — the HAND-7 ladder that would quantify it was
declined, and `logbook/2026-08-10-hand-drive-braking-clamp-diagnosis.md` leaves
open whether the clamp truncated the feedforward term only or the total, which is
the branch that would decide the ratio. No percentage is claimed here on purpose.

## Fix

Renamed `hand_motor_max_position_revs` → `hand_motor_hard_stop_revs` and set it to
`10.8`; `smooth_move_excursion_margin_rev` 0.5 → 0.2 to hold the 10.60 ceiling.
Swept per the grep-before-refactor rule: **~50 references enumerated**, 13 source
and test files patched, the generated artifacts regenerated via
`python config/generate_config.py`, and the live-code reference count for the old
symbol verified **down to zero**. The remaining textual hits are all intentional:
the YAML comment explaining the rename, a historical runbook blockquote, the
explicitly-superseded paragraph in `plans/active/hand-command-continuity.md`, and
the archived `plans/archived/simulation-development.md` table. Logbook entries and archived plans keep the old numbers — they were
true when written.

Docs corrected where the derivation was live rather than historical: the runbook's
"three sources disagree" note (now resolved), its `[0, 11.1]` bridge-range row
H3.4, H3.5's "0.5 rev short of 11.1", and H4.5's `11.1 − 0.5 = 10.60` clamp
derivation (now `10.8 − 0.2 = 10.60`, same ceiling).

## `hand_stroke_mm` corrected; `hand_stroke_m` deliberately NOT

The same disease lived in a second place, and the two halves are now split on
purpose (owner decision, 2026-08-18):

- **`jugglebot_geometry.hand_stroke_mm` 355.0 -> 344.75** — the physical travel
  between hard stops, a measured fact: `(10.8 + 0.1) x 31.6284 = 344.7496 mm`
  (the 0.1 rev is where the firmware sets encoder zero above the *bottom* stop).
  The old 355 was self-consistent with the old 11.1 anchor — `(11.1 + 0.1) x
  31.6284 = 354.24 ~ 355` — so it was wrong in exactly the same direction. Its
  consumers are the GUI render and the **MuJoCo model's hand joint `range` /
  `ctrlrange`** (`sim/model/generate_mjcf.py:92`): geometry, not control. Left
  uncorrected, the simulated hand could travel ~10 mm further than the real one.

- **`teensy_trajectory.hand_stroke_m` stays 0.355** — and its YAML comment now
  says at length why, because this is the one a future collaborator will
  reasonably try to "fix". It is not a measurement; it is the **basis of the
  throw profile**, feeding `total_stroke -> x2 / x3 / x5`. Setting it to 0.34475
  moves ball release -0.193 rev (-6.1 mm), the stroke top and therefore the catch
  prime -0.324 rev (-10.3 mm), and the catch point -0.199 rev (-6.3 mm) — which
  invalidates the tilt map, `config/toss_calibration.yaml`, the catch tuning, and
  the runbook's `peak <= 10.060` band (which *is* x3 + 0.10). It is safe as it
  stands: x3 = 9.9594 rev sits 0.84 rev (26.6 mm) below the stop, so the profile
  never commands into metal.

The old "Matches geometry `hand_stroke_mm`" claim is retired, and both keys now
carry a pointer to the other so the divergence reads as deliberate rather than as
drift someone forgot to reconcile. Re-deriving the profile is its own scheduled
piece of work with a re-validation round behind it, not a tidy-up.

## Deployment

**NOT fully live yet.** `Geometry::HAND_MOTOR_HARD_STOP_REVS` is compiled into two
firmware images:

- **Platform Teensy** — `Trajectory.h` uses it for `SMOOTH_MOVE_POS_CEIL_REV` and
  `smoothMoveMaxDuration()`. (Note: the `pio` image for this board is CAN-MUTE —
  flash via the Arduino IDE.)
- **can-bridge Teensy** — `canbridge_config.h` → `odrive_protocol.h::clip_position`.

Until both are flashed, the firmware still clamps at 11.1. That is safe in the
meantime because the **host-side** bound is upstream and goes live at the next
`colcon build` + relaunch: the live check is
`teensy_bridge_node._svc_smooth_move_hand`'s explicit `target_rev >
odrive.HAND_MOTOR_MAX_POSITION` range test (`teensy_bridge_node.py:6115`), which
reads the corrected constant. (`can/odrive.py::clip_position` also reads it but has
**no production caller** — `can_node.py` is deleted — so it is the constant, not
that function, that does the work host-side.)

## Verification

`./run_tests.sh --full` (every tier, `nightly` included), run 2026-08-18 on the
settled tree, AFTER the audit fixes: **5668 passed + 3 xfailed in 481.49 s**
(parallel phase, rc=0) plus **9 passed in 40.52 s** (serial phase, rc=0) — 529 s
total, `RESULT: PASS`. Total passed 5677.

Scoped run during development, `pytest` on the seven directly affected files
(2026-08-18): **711 passed**. Eight assertions pinned to the old anchor were
re-derived from the generated constants, not relaxed: the mirror value and its mm
conversion, `v_mid` 20.90 -> 19.96, the duration cap 0.80054 -> 0.78964, `v0_cap`
20.32 -> 20.04, the deepest honoured brake 3.213 -> 3.126, the prime's clearance
to the stop 1.1406 -> 0.8406 rev, and the GUI export's name. An independent audit
re-derived all of them from `A_MAX`/`S2`/`H`/`H2` rather than from the test files
and reproduced every one.

`python config/generate_config.py --check`, run 2026-08-18: `CONFIG FRESH: 14
artifact(s) match the generator`.

`pio run` in `Teensy_code_platform/`, run 2026-08-18: **SUCCESS in 9.26 s**
(text 102720 / data 17088 / bss 25472). Build-only — the image is NOT flashed,
and this board is flashed from the Arduino IDE, not `pio run -t upload`.

**One caveat on a bare-`pytest` run:** `tests/sim/test_hot_loop_allocation_contract.py`
carries `pytestmark = [serial, nightly]` and FAILS if run alongside the rest of
`tests/sim/` — its allocation baseline is corrupted by concurrent load, which is
exactly what the marker exists to prevent. Isolated, 2026-08-18: **3 passed**. Use
`./run_tests.sh`, which puts it in the serial phase; a bare `pytest tests/sim/`
result on that file is not signal.
