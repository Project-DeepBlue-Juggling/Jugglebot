---
title: "Bridge tick stamp appended to /robot_state and /hand_telemetry; Mocap stamp corrected in analysis tools only (BUG-134)"
type: investigation
date: 2026-11-23
status: complete
phase: "carry-ring-zvd"
related_plan: "carry-ring-zvd.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot_interfaces/msg/RobotState.msg
  - ros_ws/src/jugglebot_interfaces/msg/HandTelemetryMessage.msg
  - teensy_link/bridge_stamp.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/conftest.py
  - tests/ros/_replay_fixture.py
  - tests/ros/test_replay_overview.py
  - tests/ros/test_bridge_stamp_or_none.py
  - tests/ros/test_teensy_bridge_node_bridge_stamp.py
  - tools/probes/_bag_time.py
  - tools/probes/imu_clock_check.py
  - tools/probes/README.md
  - tests/sim/test_bag_time.py
  - map/bug-reference-stamps-run-late-robot-state-is-stamped-at-publish.md
subsystem: [ros, tools, tracking]
---

## Summary

BUG-134: the reference stamps used to validate bridge-clock streams run late. `/robot_state.timestamp` and
`/hand_telemetry.timestamp` are Jetson publish time, 12 to 14 ms after the can-bridge telemetry tick that produced
the values, and the Mocap envelope stamp sits about 13 ms after the bridge-stamped executed command.

- Both messages gain two appended fields, `builtin_interfaces/Time bridge_stamp` (bridge wall clock at the tick) and
  `bool bridge_stamp_valid` (true only when the bridge is time-synced and within 1 s of the Jetson clock). `timestamp`
  keeps its meaning. Old bags lack the fields.
- `tools/probes/_bag_time.py` holds the offline helpers: `robot_state_bridge_time` (field, else paired
  `/leg_cmd_executed` stamp, else NaN) and `mocap_time_on_bridge_clock` (envelope stamp minus `err_us` minus 13 ms).
  Its docstring is the one list of which stamp of each stream to use for timing analysis.
- The live Mocap stamp is NOT changed (owner decision, 2026-11-23).

## Discussion

**Why a new field and not a change to `timestamp`.** About nine offline probes and every old bag read `timestamp`
as publish time (`platform_mode_fit.py` fits a 17 to 22 ms input delay against it, and `platform_mode_delay_s`
0.020 absorbs that lateness). Changing its meaning would break every comparison across the date of the change
without any marker in the data. No live code reads either stamp (survey section B: the watchdogs use arrival on the
node monotonic clock), so appending costs nothing live and keeps old and new bags distinguishable by the field.

**Why the validity gate.** An unsynced bridge clock reads uptime, not wall time. Copying it into a `Time` field
would put 1970 stamps beside current ones. The same synced-or-fallback rule already exists for the cone catch event.
A consumer that sees `bridge_stamp_valid` false falls back to pairing, never to the publish stamp.

**Why the live Mocap stamp stays.** Correcting it moves Tracker landing times about 13 ms earlier, trips
`bb_calibration`'s 13 ms lag advisory and invalidates the Ball Butler release constants (the `BB_FLIGHT_BIAS` sum and
the release push lag rest on the uncorrected stamp). The true latency is only bounded to about 4 to 13 ms, so a
correction would be a guess that needs a Ball Butler re-centring Sitting to undo. The 13 ms alignment constant lives
in analysis tools, labelled as an alignment constant and not as QTM latency: it includes the Legs' own tracking lag
behind the command (modelled 1.3 to 3 ms).

**An earlier estimate revised.** The first estimate was Mocap 13.7 to 17.5 ms late, using the encoders as the
reference with an assumed 5.5 ms encoder sample age. Against the executed command (no encoders, fewest assumptions)
the figure is 13.05 ms after the command stamp; the 5.5 ms encoder age was the likely error (about 4 ms fits).

**Unresolved: the Ball Butler yaw cross-check.** `bb_calibration` fits the yaw stamp (bridge tick, `yaw_age_us` not
applied) against the Mocap envelope stamp and logged tau +2.0 ms in the 2026-10-24 bag. Working that through (BB
transmit age median 3.0 ms, hold 0 to 6.7 ms assumed uniform, CAN about 0.3 ms) puts Mocap lateness at 1 to 8 ms
(mean about 4.4), against 11 to 13 ms by the command route for a Leg lag of 1.3 to 3 ms. The 3 to 11 ms gap closes
only if the Leg command-to-motion lag is 7 to 10 ms or the hold exceeds 6.7 ms. For alignment to command-time events
the command route is trusted; for lateness against physical truth neither is trusted beyond 4 to 13 ms.

## Fix

Offline half (this session): `tools/probes/_bag_time.py` and `tests/sim/test_bag_time.py`; `imu_clock_check.py`
restamps through the helper and reads the appended `bridge_stamp` when the bag has it (its decoder is generic CDR by
the schema embedded in the bag, so the appended tail decodes with no change beyond reading it with a default for
old bags). One behaviour change: a `/robot_state` sample with no pair within 4 ms is now dropped instead of paired
to the nearest command at any distance; coverage is reported as `restamp_sources`.

Message and bridge half:

- `RobotState.msg` and `HandTelemetryMessage.msg`: `builtin_interfaces/Time bridge_stamp` and `bool bridge_stamp_valid`
  appended at the very end, so old bags and the GUI replay's positional CDR walk (`ros_ws/gui/replay/overview.py`
  reads the flags before the tail) still decode.
- `teensy_link/bridge_stamp.py`: pure `bridge_stamp_or_none(t_teensy_us, time_synced, now_jetson_us, max_skew_us=1_000_000)`;
  None when unsynced, missing or zero, or `abs(t - now) > max_skew` (a skew of exactly the limit is accepted).
- `teensy_bridge_node.py`: `_bridge_stamp_us(telem, hb)` (never raises) and an inline assignment at both publish sites,
  from the same local `telem` that built the published data (a newer tick landing mid-publish replaces only
  `_latest_telemetry`). `bridge_stamp` and `bridge_stamp_valid` join `_ROBOT_STATE_REQUIRED_FIELDS`, so a stale
  `jugglebot_interfaces` build names itself with INTERFACES_STALE at construction. `timestamp`, the rates and
  `/leg_cmd_executed` are untouched.
- `tests/ros/conftest.py` (mock messages gain the fields), `tests/ros/_replay_fixture.py` (builds from the live `.msg`),
  `tests/ros/test_replay_overview.py` (flag walk identical with and without the tail), and the new
  `tests/ros/test_bridge_stamp_or_none.py` and `tests/ros/test_teensy_bridge_node_bridge_stamp.py`.

## Verification

- 2026-11-23, `python -m pytest -q tests/sim/test_bag_time.py tests/sim/test_imu_clock_check.py`: 9 passed in 0.84 s.
- 2026-11-23, `python -I tools/probes/imu_clock_check.py --capture <AEDF|5730>` on the old bags (no `bridge_stamp`,
  all samples paired): AEDF lag1 -2.4 / lag2 +5.2 ms PASS (reference -2.2 / +5.2); 5730 lag1 -43.0 / lag2 -33.2 ms FAIL
  (reference -43.0 / -33.2). Pairing coverage 1634 of 2071 (AEDF) and 1009 of 1343 (5730); the rest had no command
  within 4 ms and were dropped.
- 2026-11-23, `pytest tests/ros/test_replay_overview.py tests/ros/test_replay_convert.py tests/ros/test_replay_api.py tests/ros/test_replay_stdlib.py -q`: 35 passed.
- 2026-11-23, `pytest tests/ros/test_bridge_stamp_or_none.py tests/ros/test_teensy_bridge_node_bridge_stamp.py tests/ros/test_teensy_bridge_node_robot_state_freshness.py tests/ros/test_teensy_bridge_node_hand_sensor.py tests/ros/test_teensy_bridge_node_coldstart.py -q`: 47 passed. An earlier wider run (adding version, read, bb, orchestrator_conduit_contract, trajectory_tilt_map) gave 236 passed, 1 failed: the coldstart drift guard, which needs the literal `msg.bridge_stamp` in the publish source; fixed by the inline assignment and rerun green.
- 2026-11-23, `cd wt/ros_ws && source /opt/ros/foxy/setup.bash && colcon build --packages-select jugglebot_interfaces` (scratch worktree): success in 2 min 9 s, installed `.msg` carries both fields. The `jugglebot` package was not built (pure Python change, covered by pytest).
- Audit finding fixed before the commit: on a stale `jugglebot_interfaces` build the new assignment raised
  AttributeError (rosidl messages use `__slots__`), which stopped `/robot_state` (throttled publish error, orchestrator
  stalled in BOOT) and stopped `/hand_telemetry` silently. Both publish sites now catch it and publish without the
  stamp; `test_stale_interfaces_build_still_publishes` pins it (2026-11-23, `python -m pytest -q
  tests/ros/test_teensy_bridge_node_bridge_stamp.py tests/ros/test_teensy_bridge_node_coldstart.py`: 29 passed).
- Default gate (2026-11-23, `flock /tmp/jugglebot-tests.lock ./run_tests.sh` in the scratch worktree): parallel 11042
  passed, 1 failed, 64 skipped, 1 xfailed in 738.67 s; serial 7 passed in 29.41 s. The one failure was
  `test_every_item_is_rated[bug-reference-stamps-run-late-...]` (BUG-134 set in progress without `hardware` and `owner`
  ratings). Rated `hardware: none`, `owner: none`; that touches one map file, so the rerun was the map and docs tests
  (below), not the whole gate.
- After the rating (2026-11-23, `python -m pytest -q tests/sim/test_mapkit*.py tests/sim/test_map_items.py
  tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py
  tests/sim/test_no_legacy_item_ids.py tests/sim/test_plans_index.py tests/sim/test_glossary_avoid_words.py`): 340 passed
  in 31.10 s.
- Live checkout rebuild and launch check (2026-11-23, live checkout at 5c1d5321, `colcon build --packages-select
  jugglebot_interfaces jugglebot`, then `ros2 launch jugglebot jugglebot_launch.py` with 48 V off): build OK; firmware
  checks v49 and v15; no INTERFACES_STALE and no "Robot state publish error" line. `bridge_stamp_valid` true on 100 % of
  messages (1491 + 395 `/robot_state`, 1500 + 398 `/hand_telemetry`). `timestamp` minus `bridge_stamp`: `/robot_state`
  min 2.9, median 3.2 to 3.3, max 14.6 ms; `/hand_telemetry` min 3.5, median 3.9 to 4.0, max 16.6 ms; never negative.
  That is smaller than the 12 to 14 ms measured in the Sitting bags. Not investigated; the likely reasons are the
  unloaded box and the free-running phase between the node's 100 Hz publish timer and the bridge's 100 Hz tick (a
  per-launch offset of 0 to 10 ms), which is the variation the field exists to remove. `/clock_diag` published
  nothing in 40 s, so the clock error for this run is unknown. NOT verified on hardware: the invalid-before-sync case
  (the capture began about 20 s after launch, already synced); it is covered by the unit tests only.
- Mocap envelope stamp minus the executed-command stamp, `err_us` removed (survey, 2026-11-20/22/23, ms; orientation
  fit, position fit):

| capture | lag minus err, ori | pos | err_us |
| --- | --- | --- | --- |
| AEDF | 12.4 | 13.2 | 3.36 |
| AEE0 | 11.8 | 12.4 | 2.82 |
| AEE1 | 11.7 | 12.9 | 1.95 |
| 572E | 13.5 | n/a | 0.35 |
| 5730 | 13.2 | 14.0 | 0.18 |
| 5731 | 13.0 | 14.2 | -0.24 |
| 4AFA | 15.8 | 13.2 | 4.38 |

Orientation mean 13.05 ms (sd 1.39; 12.6, sd 0.8 without 4AFA); position mean 13.3 ms (sd 0.7). The raw per-body
stamps in `/rigid_body_poses` scatter 10 to 27 ms and are unusable.

## Open

- Encoder sample age (0 to 10 ms per axis) stays unexported until can-bridge FW 50: FEAT-64.
- QTM's true latency is unmeasured: IDEA-28 (LED or strobe edge), then a decision on the live stamp.
- The invalid-before-sync case has not been seen on hardware (unit tests only).
