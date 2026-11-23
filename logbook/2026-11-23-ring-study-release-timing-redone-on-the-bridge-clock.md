---
title: "Ring study release timing redone on the bridge clock: the x surge after release halves (19 mm/s, not 43), separation measured about 4 ms after the planned release, the ring is quiet there"
type: investigation
date: 2026-11-23
status: complete
phase: "carry-ring-zvd"
related_plan: "carry-ring-zvd.md"
related_issues: []
files_changed:
  - logbook/2026-11-23-ring-study-release-timing-redone-on-the-bridge-clock.md
  - map/idea-measure-the-platform-ring-at-dwell-0-40-with-the-imu.md
subsystem: [platform-imu, planner, analysis]
---

## Summary

Offline, no hardware, one agent. It corrects items 4 and 5 of `2026-11-23-platform-ring-identification-from-the-imu`
(that entry is left as written). The ring study had shifted the IMU by +54.6 / +55.6 / +22.8 ms to line it up with Mocap
and the encoders, which are themselves stamped late (`2026-11-23-bridge-stamp-on-robot-state-and-hand-telemetry`). On
the bridge clock the right shifts are +35.2 / +35.0 / -4.8 ms (captures 5730 / 5731 / 4AFA, each +-3 ms), about 20 to
28 ms less. Scripts and outputs: `temp/analysis/imu-ring-2026-11-23/clock_corrected/` (not tracked).

Carried Throws, x ring velocity in mm/s, time after the planned release (old shift, then corrected with the +-3 ms band):

| at | old | corrected | landing error if the ball separated then (old, corrected) |
|---|---|---|---|
| +0 ms | 2.3 +- 6.5 | -1.1 +- 5.4 (-2.2 to -0.3) | |
| +20 ms | 25.1 +- 8.4 | 11.3 +- 3.7 (8.8 to 13.5) | 23 mm, 10 mm |
| +30 ms | 35.2 +- 8.6 | 16.5 +- 2.7 (13.4 to 19.4) | 32 mm, 15 mm |
| +40 ms | 41.5 +- 7.6 | 18.9 +- 4.3 (15.5 to 22.3) | 37 mm, 17 mm |
| surge peak | 43 at +47 ms | 19 at +42 ms (15.5 to 22.4) | |

1. The surge is half the size reported. Its amplitude moves about 1.1 mm/s per ms of shift, so most of the old 43 mm/s
   was the clock error leaving commanded motion in the residual. The first, carry-free Throw: 31 became 23 mm/s. There is
   no y surge (-8.1 +- 12.6 mm/s at +0, -8.2 +- 7.3 at +40).
2. New: a measured separation instant. The Hand's measured acceleration first drops below -g at +8.6 +- 3.8 ms after
   the planned release (n = 24; per capture 8.5 / 5.7 / 11.5), with the Hand telemetry moved to the bridge clock by a
   per-capture constant. Less the Hand encoder's unknown sample age (0 to 10 ms) that is -1 to +9 ms, central +3.8 ms.
   The IMU z reaction (fastest change at +6 to +8 ms) agrees within about 3 ms, so the corrected shift is not
   contradicted. This is the earliest possible separation; adhesion in the Cup could only make it later.
3. At that instant the ring is quiet: x +1.1 +- 6.7 mm/s, landing +1.0 +- 6.0 mm; y landing -8 +- 11.5 mm.
4. Share of landing scatter the ring explains (r^2, n = 24): x at most 6 % (was 13 %), y 17 to 27 % (was 29 to 39 %).

## Discussion

- **Withdrawn (the orchestrating Session's guess before this run):** that the corrected clock would move the surge into
  the 20 to 40 ms separation window and make it matter more. It does start inside that window, but at half the size,
  and the measured separation is at about +4 ms, not 20 to 40 ms late.
- **Withdrawn from the ring study:** the "+25 to 35 mm repeatable long bias" from the surge, and the suggestion that the
  +22 mm common offset in the carry_zvd rows came partly from it. At the measured separation the x ring offset is about
  +1 mm; even a separation 40 ms late gives 17 mm. The +20 mm raw x landing mean in 4AFA is not explained by the ring.
- **Stands, and is stronger in x:** shaping the Platform command is not worth it at Dwell 0.50. Free-decay frequency
  and damping (x about 4.3 Hz) do not depend on an absolute shift.
- **Got worse, do not quote:** the forced-response fits. With the IMU on the bridge clock the free-delay x fits run to
  the -30 ms bound on all three captures, and with the delay fixed at 10 ms two run to the 20 Hz bound. A second-order
  plus delay model contradicts Legs that track within about 2 ms. The forced-fit modes (x 4.1 to 4.9 Hz, damping 0.32
  to 0.37) need the chirp Sitting, analysed on the bridge clock.
- **Clock facts found on the way:** the planned release in `/throw_announcements` is on the Jetson clock
  (`skill_node.py` about lines 2965 to 2987), not the bridge clock; the bridge-to-Jetson error was about 0 ms on
  2026-11-22 and +4.4 ms on 2026-11-23 and was removed. The launch log's "release +x ms" is fitted minus commanded
  (positive = late, `report.py` about lines 116 to 136); its means (+4.6, +4.6, +7.5 ms) include the Mocap lateness.
- **Limits:** shift +-3 ms; Hand encoder age +-5 ms on the separation instant; per-row restamping of the Hand telemetry
  was too noisy (sd 6 to 9 ms), so a per-capture constant was used (8.7 / 9.9 / 15.8 ms); Mocap left out. All three
  captures are Dwell 0.50: nothing here speaks to Dwell 0.40 (IDEA-27).

## Verification

- 2026-11-23, `python -I tools/probes/imu_clock_check.py --capture <stem>` for 5730, 5731, 4AFA: lags reproduced
  (-43.0 / -33.2, -40.0 / -33.0, -4.2 / +6.8 ms); shift = minus the executed-command lag plus 2 ms of Leg lag.
- 2026-11-23, `rel_cc.py`, `sep_cc.py`, `post_cc.py`, `fit_cc.py` in `clock_corrected/`, run at the old shift and at the
  low, adopted and high ends of the band: the tables above (`summary_{old,lo,new,hi}.txt`).
- Not independently re-run by a second agent. No code changed.
