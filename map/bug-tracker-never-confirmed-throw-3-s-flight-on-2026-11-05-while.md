---
id: JB-181
kind: bug
title: Tracker never confirmed Throw 3's flight on 2026-11-05 while the previous track kept running past its Catch, so that Throw has no learner row
status: inbox
created: '2026-11-06'
updated: '2026-11-06T19:54:16Z'
owners: []
priority: medium
effort: m
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: tracking
blocked_by: []
links: []
filed_by: claude
---

Found by the Night Shift on 2026-11-06 while reading JB-176's bag (2026-11-05 22:46, the first fed Attempt). Throw 3 got no learner row because the Tracker never confirmed its flight, not because of the identity gate.

## Findings

- Tracker id 4 (Throw 3) has no `tracker confirm ball 4` line in the launch log. Its status stays 0 or 1, tracking 0, never `from_fit`; at 354.96 its status goes to 4.
- Meanwhile id 3 (Throw 2's track) stayed tracking past its Catch with `from_fit` false, so the only estimate for that flight was unfitted.
- `_consider_landing` rejected every estimate for lack of a converged fit.
- Throws 1, 2, 7 and 9 had their own id confirmed and a fit before the crossing.

## Open

- Whether the previous track kept the marker, or the new flight's detection failed some other way. Replay the `/balls` and mocap topics of that bag through `ball_tracker_node`'s pure logic.

## Findings (2026-11-07)

Bag 2026-11-05_22-46-00, launch.log, replay of `/mocap_data` + `/throw_announcements` through `BallTracker.process_frame` (node's constructor args, frame stamps). Probes were deleted.

### What the markers did (measured)

- One physical ball is thrown, caught and re-thrown. Its marker (label `Catch...`) is continuous through the Catch, the Carry (z 705-711 mm) and the next rise.
- Replay, Throw 1 track (id 2): its Kalman filter follows that marker with `frames_without_measurement` 0 from the Catch (2.6 s) through the next flight to 3.4 s.
- Replay, Throw 2 (id 3): the marker is "used" by id 2 every frame, so id 3 never confirms; it goes UNKNOWN at 3.64 s (n=1 ball).

### Rule that keeps the old track alive

- `_associate_confirmed_balls` (matcher.py:492) runs before `_match_announced_balls` (:589) and takes the nearest marker within `gate_radius_mm` = 50 mm of the old filter.
- `_check_lifecycle` (:978-985) ends a CONFIRMED ball only when 15 frames pass with no marker. Its comment says a marker still visible past landing is "let continue".
- `_check_still_confirmed` does not fire: the marker moves.
- Nothing releases a CONFIRMED track at its own Catch or at the next announced throw_time.

### Live bag versus replay (does not match exactly)

- Live: id 3 confirmed late (`steps 2`, +0.205 s, launch.log:1006); id 4 never confirmed; id 3 shows tracking 1 with fit false at 4.345 s, z 1080, vz -6454, the same signature as id 2 at 3.056 s. So id 3 held Throw 3's marker, as id 2 held Throw 2's.
- Replay: the mirror image (id 3 never, id 4 confirmed at +0.002 s). The old track let go one throw earlier live, probably a dropped frame or stamp jitter.
- So the outcome is timing-sensitive, and the mechanism (older track holds the marker) is the same in both.

### Does it repeat?

- Launch log: no confirm line for ids 4, 12, 15, 16, and a late one for id 3. Replay: no confirm for 3, 12, 15, 16. The same ids except 3/4.
- Steady-state throws (ids 5-11, 14, 17+) confirm at +0.001 to +0.003 s.
- Replay id 5 (steady): its filter lost the marker at the re-throw (fwm 5 at 6.85 s, 15 at 6.90 s) before id 6 needed it, so id 6 confirmed.
- Not settled: why the filter lets go in the steady case but not after Throw 1 or 2. Hypothesis, not tested: how fast the hand accelerates the marker out of the Carry relative to the filter's velocity.
- Other fed Attempt: only the second Attempt in this same bag (ids 15, 16). No second bag checked.

### What a fix would touch (not written)

- matcher.py: retire a CONFIRMED track whose announced successor reaches throw_time, or let the ANNOUNCED gate take the marker from a track already past its landing.
- Risk to a live Catch: a wrongly stolen marker would move the old ball's landing and the Catch aim. Retire only a track whose landing time has passed.
- Pin by replay: this bag (ids 3/4), its second Attempt (15/16), and a synthetic marker with slow versus fast re-throw ramp. Assert each id confirms at about +0.003 s.
- Control view at 40 Hz: the tracker publishes after the Catch of the old ball, so a retired old track is not read by a live Catch.
