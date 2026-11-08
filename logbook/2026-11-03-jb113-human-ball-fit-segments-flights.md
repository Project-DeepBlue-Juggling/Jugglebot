---
title: "IDEA-16: a human-thrown ball's flight fit now forgets the previous flight at the re-throw (carried-track hypothesis confirmed); usable landing 0.3 s ahead 24 -> 37 of 42 passes, 41 -> 51 of 71 cascade throws, phantoms unchanged"
type: bugfix
date: 2026-11-03
status: complete
phase: "IDEA-16 human-partner cascade, perception first rung"
subsystem: tracking
files_changed:
  - ros_ws/src/jugglebot/jugglebot/tracking/flight_fit.py
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py
  - ros_ws/src/jugglebot/jugglebot/tracking/tests/test_flight_fit.py
  - ros_ws/src/jugglebot/jugglebot/tracking/tests/test_matcher.py
  - map/idea-jp-17-three-ball-cascade-with-the-owner-as-the-second-hand.md
  - logbook/INDEX.md
---

# A human-thrown ball's fit segments its flights

**Problem.** The 2026-11-02 bench (`logbook/2026-11-02-jb113-human-throw-bench.md`) found
about half the human Throws served by a track already on the ball before the release, with
landing times 0.5-1.5 s early. The hypothesis: that track's `FlightFit` still held the
previous flight.

**Root cause, confirmed.** The tracker keeps a human-thrown ball's marker through the catch
and the re-throw: the hand's motion stays inside the 50 mm association gate, and the ball is
only declared CAUGHT once its marker vanishes. A human ball has no `throw_time`, so its
`FlightFit` admitted every sample above the freeze floor (landing plane + 250 mm), and
flight 2 pooled with flight 1 (up to 400 samples). A one-off probe classified the fit behind
each Throw's estimate at the 0.3 s deadline. A fit holding samples from more than 20 ms
before the release gave a usable estimate (within 50 mm and 30 ms) for 3 of 19 passes and
2 of 15 cascade Throws. A fit holding only this flight gave one for 20 of 20 and 36 of 37.
An announced ball cannot hit this, because each throw gets a new ball id.

**Fix.** `FlightFit(new_flight_rise_mm=...)`, off by default. The flight closes when a
sample falls at least that far below the freeze floor; that margin stops marker jitter at
the floor from closing it. A new flight starts, clearing the samples, when the ball then
rises that far above its lowest point since. The frozen fit stays readable while the ball
is in the hand. `BallTracker` turns it on, at 50 mm (`flight_fit_new_flight_rise_mm`), only
for balls with no announcement, so the robot's own throws are bit-identical. It is a
constructor default and not a YAML key, because `hardware_config.py` is a box-gated file
and a new key would force a box re-sweep for a knob the robot's throws never read.

**Result.** `python tools/probes/human_throw_bench.py <both 2026-11-02 bags>`, run
2026-11-03 12:06. Usable estimate at 0.30 s before the plane: passes 24 -> **37/42**,
cascade 41 -> **51/71**. Good estimate at any lead: 30 -> 39 and 55 -> 58, median lead
0.65 -> 0.72 s and 0.56 -> 0.60 s. Phantoms unchanged, 19 and 28.

**What is left.** 15 of the cascade's 20 remaining failures are low Throws (apex 830-1160
mm) that put no samples, or too few, above the 1080 mm freeze floor. The fit never converges
on them, so the Kalman fallback answers, and for a ball carried through a hand it is poor
(0 of 10 usable). The floor protects the Cup's push and catch; a human ball's push happens in
a hand near z 450. Lowering the floor for human balls is a design choice, now the first
IDEA-16 Next item.

**Verification.** `pytest -q ros_ws/src/jugglebot/jugglebot/tracking/tests/test_flight_fit.py
ros_ws/src/jugglebot/jugglebot/tracking/tests/test_matcher.py` (2026-11-03): 23 passed.
New: three `TestFlightSegmentation` cases, and `TestHumanThrowCarriedAcrossFlights`. That
class carries one ball through a smooth 0.3 s scoop at 300 Hz: landing error 0.0 ms with
segmentation, and a control pinning -100 ms without it. Default gate: see the commit message.
