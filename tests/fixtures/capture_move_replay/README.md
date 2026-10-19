# capture_move_replay - replay inputs for the held-axis reload CATCH refusals

`../capture_move_replay_2026-10-19.yaml` holds one record per ball for the
2026-10-19 sitting (rehearsal row 33): balls 17 and 18, the two reload CATCHes
refused `CATCH_AXIS` at the fresh-origin dispatch (28.3 mm and 20.8 mm off the
held-axis line; attempt ENDED, 0/0 caught), and ball 10, the control (the reload
CATCH installed from the schedule aim and caught). Every field in the YAML is
commented with its source topic / launch-log line and bag time. Frames: the
announcement and tracker landings are the bag's world (mocap) frame in mm; commanded
xyz is the trajectory/schedule frame.

- Bag: `~/Desktop/rosbags/2026-10-19_11-04-03` (MCAP).
- Launch log: `~/.ros/log/2026-10-19-11-04-03-078330-jetson-3068207/launch.log`.
- Installed code: commit `84519014` (the `Jugglebot-skills` install built 10:35 is
  byte-identical to the source at 84519014 for `skill_node.py` and `motion/skills/*.py`).
- Extraction (venv, ~20 s; run from the repo root):
  `python -I tools/probes/capture_move_replay_extract.py --bag ~/Desktop/rosbags/2026-10-19_11-04-03 --launch-log ~/.ros/log/2026-10-19-11-04-03-078330-jetson-3068207/launch.log --balls 10,17,18 --emit-fixture tests/fixtures/capture_move_replay_2026-10-19.yaml`
- Check built into the fixture: `reproduced_last_before_dispatch.clamped_norm_mm` =
  `hypot(clip(fit - announced, +-20 mm))` = 28.28 (ball 17) and 20.77 (ball 18),
  matching the logged refusals 28.3 / 20.8; the raw (unclamped) re-aims are 41.3 and 24.9 mm.
- Caveats: the commanded TILT has no direct topic in the bag (`/platform_pose_topic` is empty;
  `/trajectory/commanded_position` is a bare xyz `Point`), so `commanded_fk` is forward
  kinematics of `/leg_setpoint_echo` (the legs' physical command, which includes the
  ~0.55 deg level baseline); the planner-side hold tilt is `hold_tilt_deg` from the launch
  log. Which fit `skill_node` held at dispatch is not recoverable from bag `log_time`
  (its `/balls` receipt time is unrecorded; ball 10 had a fit 2 ms before dispatch yet
  aimed at the schedule), so the record carries first / last-before / first-after fits.
