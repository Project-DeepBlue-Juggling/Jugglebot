---
title: "Scheduled-promotion continuity fields (hand and leg dp/dv/da maxima, over counts) now ride /link_status so a rosbag holds them (BUG-99 row 2)"
type: feature
date: 2026-11-26
status: complete
phase: "Offline 2026-11-26"
related_plan: ""
related_issues: [BUG-99]
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_read.py
subsystem: [ros, observability]
tags:
  - bridge
---

# /link_status carries the promotion-continuity fields

**What.** `teensy_bridge_node` now publishes eight more `/link_status` KeyValues, named as the HeartbeatT2J wire fields: `hand_promo_dp_max`, `hand_promo_dv_max`, `hand_promo_da_max`, `leg_promo_dp_max`, `leg_promo_dv_max`, `leg_promo_da_max` (floats, `%.6f`; armed-session maxima of |dp| in rev, |dv| in rev/s, |da| in rev/s^2 at a scheduled promotion; Legs are the max over legs) and `hand_promo_over`, `leg_promo_over` (cumulative-since-boot counts over the firmware's continuity tolerance). No wire or firmware change, no new log line, no threshold.

**Why.** BUG-99 row 2: a clock-offset slew of at most 1.25 ms per 30 s anchor makes a promotion between scheduled segments slightly discontinuous, and the firmware only counts it. The 2026-11-26 analysis of nine bags found the slew negligible (worst applied 0.77 ms, about 1.3 mm at the Hand at 50 rev/s) but could not check it against the counters, which no bag held. The first powered Sitting now records them.

**Control path.** None touched: the values are read from the already-decoded heartbeat inside the 10 Hz `_publish_link_status` build. Readers of `/link_status` look keys up by name (GUI, replay allowlist is topic-level, probes); no reader pins the key list or order.

**Verification.** 2026-11-26, `pytest tests/ros/test_teensy_bridge_node_read.py tests/ros/test_teensy_bridge_node_bridge_diag.py tests/ros/test_teensy_bridge_node_recover_lane.py -q`: 124 passed in 42.21 s. The new `test_link_status_surfaces_promotion_continuity_fields` FAILS with the node change stashed (KeyError on the first promo key) and passes with it.
