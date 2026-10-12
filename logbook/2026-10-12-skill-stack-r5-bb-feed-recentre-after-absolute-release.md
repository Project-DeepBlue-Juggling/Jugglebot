---
title: BB feeds landed 50.6 ms early after the absolute-release change - announcement constants re-centred from a free-fall fit (0.052 -> 0.001 s)
type: investigation
date: 2026-10-12
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / two-ball columns"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-12-skill-stack-r5-bb-absolute-release-time.md
  - 2026-10-12-skill-stack-r5-bb-feed-no-drift-release-jitter.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - tests/ros/test_ball_butler_node.py
  - plans/active/two-ball-skill-stack.md
  - logbook/2026-10-12-skill-stack-r5-bb-feed-recentre-after-absolute-release.md
  - logbook/INDEX.md
subsystem:
  - ball-butler
tags:
  - timing
---

# BB feed re-centre after the absolute release

## What happened

Sitting 7, third attempt (2026-10-12 ~23:24): the first sitting on bridge FW 29 (absolute BB release, `355a9138`).
- Bag: `~/Desktop/rosbags/2026-10-12_23-24-02`.
- launch.log: `~/.ros/log/2026-10-12-23-24-01-939035-jetson-3607045`.
- 17 BB throws, all 17 `bb/throw C1:` lines matched to their announcements.

The owner reported: "the BB throws are consistently landing early", and the `columns_1ball_fed` feeds "fell short
and collided with part of the Platform structure"; the few that were caught were caught messily. 6 of the 9 fed
feeds missed (cup empty).

## Measurements

Method: a fixed-gravity free-fall fit of the last ~0.25 s of mocap, extrapolated to the 830 mm plane. It does not
use `feed_catch_bag_probe.py`'s crossing, because that crossing includes cup contact. BB's release comes from its
hand encoder. Scripts are in that session's scratchpad: `bb_extract.py`, `bb_flight.py`, `w_*.py`.

- **Dispatch latency,** announcement `now` -> bridge receipt: 35.7 ms (sd 14.1, range 20.6-61.0).
- **Release:** BB's hand stroke now lands 3.9 ms before the commanded `release_wall` (sd 2.6).
  - On the old relative path the sd was 14-22 ms.
  - The `355a9138` prediction (sd <= 6 ms) holds: the release jitter is gone.
- **Arrival** against the announcement's `landing_time`: -50.6 ms (sd 2.2, n 17).
  - The 2026-10-11 sittings read about -10.5 ms.
  - The extra -40 ms splits into -33.6 ms of earlier release (the removed mean latency) and -7 ms of lower flight
    (apex 2481 mm against 2502).
  - The feed catch's scheduled landing equals `landing_time` within 0.5 ms on every throw.
- **Landing position:** dx -17.0 mm (sd 12.5), dy -9.4 mm (sd 6.8) against the site.
  - The 2026-10-11 sittings read about +5 / -1 mm.
  - Feeds k8-k11 flew 31-46 mm short at a horizontal speed of 884 mm/s, against 911 mm/s for k12-k16.
  - Command, aim correction, hand stroke and `columns_feed_bb_bias_mm` (0, 0) were all unchanged. So this is
    physical variation in BB between sittings, not code.

## Discussion

- **Why the fed catches collided: timing and position together.**
  - The early ball first touched the level cup 70-78 ms before the scheduled landing, at z about 955-977 mm, while
    the cup was still high and descending. It hit the rim and bounced back toward BB; k8 reversed in x at about
    -1 m/s.
  - The early contact also moved the hit point about 20 mm toward BB, on top of the short landing.
  - Feeds whose first contact was <= 36 mm from the cup axis were caught; those >= 38 mm missed.
  - The tilted reload catches (k0-k7) met the same -50 ms and were all caught. The level-pinned feed catch is the
    one with little tolerance.
- **Fixed in the announcement, not in BB's release.**
  - The feed catch is compiled from `landing_time`, so making the announcement predict the real arrival fixes the
    catch directly.
  - Delaying BB's release back to the old timing would mean changing `BB_OP_THROW_RELEASE_LATENCY_MS`, which is
    shared with the generated firmware headers. It would re-create a number that has no physical meaning.
  - By the hand-speed peak, BB physically releases about 40 ms before `now + delay`: the 44 ms latency constant
    also contained the old dispatch latency. Its comment now says so.
- **Only the sum of the two constants is measurable.** The new values are `BB_RELEASE_PUSH_LAG_S = 0.037 - 0.036 =
  0.001` and `BB_FLIGHT_BIAS_S = 0.000`.
  - The flight's remaining 14.9 ms: about 10 ms was already there on 2026-10-11, and about 5 ms comes from this
    sitting's lower flight.
  - Sitting-to-sitting spread in the stroke-to-arrival time is about +/-5 ms (933-943 ms). That is the expected
    residual for the next sitting.
- **The position shortfall is not fixed here.** It changed sign between sittings (+5 mm on 2026-10-11, -17 mm
  now), so a bias set from one sitting would chase noise. The options are re-running BB's accuracy calibration, or
  setting `columns_feed_bb_bias_mm` from a median over several sittings. That is the owner's call.
- **Unverified aside:** the cup sensor read empty for 2 s before every fed hop-entry throw, and mocap showed no
  second ball rising (checked on k8). Ball 1 may not have been in the cup, which would fit the "throw 2/4 never left
  the hand" endings. This is an operator question.

## Fix

`ball_butler_node.py`: `BB_RELEASE_PUSH_LAG_S` 0.037 -> 0.001 and `BB_FLIGHT_BIAS_S` 0.015 -> 0.000, with the bag, n,
mean and sd in the comment. The `BB_OP_THROW_RELEASE_LATENCY_MS` comment is updated. The plan's § R5 constants
line is updated.

## Verification

- **Offline replay** with the new constants (scratchpad `w_verify.py`, which reads the constants from the edited
  file): arrival minus `landing_time` went from -50.6 ms to +0.4 ms, sd 2.2, n 17. Fed throws only: +0.4 ms, sd 2.8,
  n 9.
- **New test:** `test_constants_pinned_to_the_2026_10_12_free_fall_recentre` in `tests/ros/test_ball_butler_node.py`.
  Before it, nothing pinned the literals.
- **Scoped run,** 2026-10-12: `pytest -q -p no:cacheprovider tests/ros/test_ball_butler_node.py
  tests/ros/test_ball_butler.py tests/ros/test_teensy_bridge_node_bb.py tests/ros/test_skill_node.py
  tests/ros/test_perception_console_lines.py tests/ros/test_catching_cone.py` gave 399 passed.
- **Gate:** in the commit message.
- **Next sitting:**
  - arrival within 0 +/- 5 ms of `landing_time`; if the mean is outside +/-8 ms over >= 10 throws, re-centre
    `BB_FLIGHT_BIAS_S` with the same scripts;
  - first contact of the level feed about 25 ms before landing at z about 875 mm, not 75 ms at 965 mm;
  - if BB again lands 20-45 mm short, contacts beyond about 36 mm from the axis can still miss.
