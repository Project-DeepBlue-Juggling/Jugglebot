---
title: "Platform ring identified offline from three IMU captures: x about 4.3 Hz, small at the planned release at Dwell 0.50; shaping not worth it there; IMU timestamps found 23 to 55 ms early (BUG-132)"
type: investigation
date: 2026-11-23
status: complete
phase: "carry-ring-zvd"
related_plan: "carry-ring-zvd.md"
related_issues: []
files_changed:
  - logbook/2026-11-23-platform-ring-identification-from-the-imu.md
  - plans/active/carry-ring-zvd.md
  - map/bug-platform-imu-capture-timestamps-run-23-to-55-ms-early.md
subsystem: [platform-imu, planner, analysis]
---

## Summary

Offline study, no hardware, by two agents (data preparation, then identification) on the three oval Attempt captures
`20261122T101355Z_5730`, `20261122T101614Z_5731` and `20261123T033855Z_4AFA` (`oval_1ball_fed`, Dwell 0.50, Sep. 150,
Apex 1.0, 8/8 caught each) and their bags. Scripts, figures and the full report are under
`temp/analysis/imu-ring-2026-11-23/` (not tracked; `ident/REPORT.md`). The numbers that matter are copied here.

1. The IMU capture timestamps are early against Mocap and the encoders: 53 to 57 ms on 2026-11-22, 22 to 24 ms on
   2026-11-23 (gyro against orientation rate, r 0.95 to 0.997; constant within a capture). Filed as BUG-132.
2. The lateral ring in x is about 4.3 Hz (free decay after release, 21 of 21 Throws: 4.35 Hz, interquartile 4.08 to
   4.44). Damping is not pinned: 0.10 from the decay, 0.30 to 0.37 from the forced fit.
3. In y the picture is unresolved: about 4.5 Hz from the forced fit (extrapolated) and a lightly damped 5.3 to 7 Hz
   decay that the command does not drive.
4. At the planned release the ring velocity is small (x 2 +- 6 mm/s, y 1 +- 10 mm/s). It then surges repeatably in x to
   about +43 mm/s 45 ms later, during the commanded hold. If the ball separates 20 to 40 ms late, that is 23 to 37 mm
   long with a spread of 7 to 8 mm.
5. The ring explains about 10 % (x) and at most 30 % (y) of the logged landing scatter (sd 24 and 27 mm).

## Discussion

- **The encoder-versus-Mocap disagreement is settled for x.** The IMU agrees with Mocap (gain 1.0 +- 0.1 to 9 Hz in y)
  and the encoders under-report Platform motion by 1.4 to 3 times between 4 and 7 Hz: there is compliance beyond the
  motors. The ring the ball sees is the Mocap-like 4.3 to 4.4 Hz; the encoders' 5.4 Hz is the motor-side view.
- **The carry_zvd rows did not fail because of a frequency error.** A ZVD shaper tuned to the provisional 4.9 Hz /
  0.20 leaves 3 to 4 % against the measured x mode (16 % against a 6.25 Hz y mode). The Hand stop-and-go at about
  5 Hz (BUG-90) is the shaper's own signature: impulses half a period apart, 230 ms in all, about half the Dwell.
- **Shaping the Platform command is not worth pursuing at Dwell 0.50.** What it could remove is under about 7 mm of
  landing spread in x; the bias is repeatable, so the Learner can absorb it. About half the x surge is excited by the
  Throw itself (the first Throw, with no carry, still surges +27 mm/s), and the y ring at release is about 90 % not
  command-driven. A Platform-command shaper cannot remove those parts.
- **What was not tested:** shorter Dwells (0.35 to 0.40), where the original 76 to 122 mm long landings were seen.
  All three captures are Dwell 0.50. The conclusion does not extend there.
- **Limits.** The oval excites almost nothing coherent above 5 Hz, and the captures end within 0.07 s of the Attempt,
  so there is no clean free decay. No measured separation instant exists. Dependence on Hand position and pose was
  not determined. Methods rejected: random decrement and AR poles (they returned the filter's centre).

If the mode needs pinning, the report's section 4 gives a short Sitting: profiled 30 to 40 mm moves with at least
4 m/s^2 deceleration and a 1.5 s hold, a 2 to 8 Hz chirp at about 1 m/s^2 per axis, Hand low then high, with the IMU
capture running 2 s past the motion.

## Verification

- 2026-11-23, `clock.py` (gyro against Mocap and encoder orientation rate, low-passed 8 Hz): lags -53.2 to -56.8 ms
  (5730, 5731) and -21.7 to -23.8 ms (4AFA); Mocap against encoders within 3 ms.
- 2026-11-23, `release_decay.py`: x decay 4.35 Hz, zeta 0.10, 21 of 21 fits with R^2 above 0.7; y 6.25 Hz, 17 of 21.
- 2026-11-23, `release.py` over 21 carried Throws: values in Summary items 4 and 5.
- No code changed. Not independently re-run by a second agent.
