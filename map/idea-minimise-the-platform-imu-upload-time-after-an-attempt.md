---
id: IDEA-25
kind: idea
title: Minimise the Platform IMU upload time after an Attempt
status: in_progress
created: '2026-11-19'
updated: '2026-11-23T03:48:24Z'
owners: []
priority: low
effort: m
hardware: robot
owner: decision
radius: local
area: platform-teensy
blocked_by: []
links:
- plans/archived/platform-imu.md
- logbook/2026-11-20-sitting-debrief-robot-2.md
- logbook/2026-11-21-platform-imu-upload-hole-fill.md
- logbook/2026-11-21-platform-imu-fw13-chaining-pacing-2khz.md
parent: two-ball-oval
filed_by: claude
night_skip: owner-session
night_skip_on: '2026-11-23'
night_skip_note: 'Your TKT-88 answer: a day Session is on it (FW 14 chunk CRC, pipelined refills, 2026-11-22); the night leaves it pinned.'
---

Keep this area open (owner, 2026-11-19): once the MVP works, explore shortening the post-Attempt IMU upload. Three levers: tighter sample packing or compression; a slower setpoint heartbeat while holding (the 500 Hz stream stays as is for now: it feeds the ODrive watchdog and avoids a rate switch on the Leg path); a higher measured rate from the P4 ramp.

## Feasibility (2026-11-19)

- What it would take: once P4's rate ramp has measured the clean frame rate, pick one lever. A sample is 6 x int16 = 12 bytes against 8 data bytes per classic CAN frame, so the payload is the bound: a 12-bit or delta pack fits about 1.5 samples a frame, cutting the 64000 data frames of a full window to about 43000. A slower setpoint heartbeat while holding frees bus time but touches the Leg path and the ODrive watchdog. A higher rate needs only the measured number.
- Effort: m (packing in the Platform Teensy firmware and the Jetson decoder, with native and decode tests). Hardware: robot (the bus rate is measurable only on the Platform). The owner chooses the lever.
- One risk: packing changes the wire format of a CRC-checked upload, so firmware and decoder out of step corrupt captures silently unless the header carries a format version.
- Waits on: the MVP (plan `platform-imu.md` P1 to P4) working, as the owner set when filing.

## Options (2026-11-21)

The trigger fired: Rung P4 passed at the Sitting of 2026-11-20, and that debrief measured Armed uploads at 79-84 s against 56 s at rest. Its diagnosis: about 0.4 % of IMU data frames are lost when Armed, and each hole-fill READ (`_Region.next_block`, `teensy_link/platform_imu.py`) re-sends every held word between the first and last hole of a 256-word span.

1. **Jetson-only hole-fill change, no flash** (the debrief's proposal): end each hole-fill READ at a run of more than 16 present words and cut the post-block settle from 30 ms to about 5 ms. Modelled 79 s to about 64 s Armed, duplicates about 14500 to about 200. The Night Shift can build it with tests against the region logic; it then waits for a Sitting check (at rest under 100 duplicates and 56 s or less; Armed under 1000 duplicates, 62-66 s, CRC OK).
2. **Firmware counters first**: transmit and receive counters for the IMU data frame on the Platform Teensy and the can-bridge, to find where the 0.4 % is lost before changing the Jetson. Needs two flashes; a day Session.
3. **Sample packing** (the 2026-11-19 feasibility): about 1.5 samples a frame, 64000 to about 43000 frames; a wire-format change on both sides with a format version in the header. Largest gain at rest, largest effort.
4. **Leave it**: about 80 s between Attempts is tolerated for now.

Recommendation: 1 now (cheap, Jetson-only, attacks the measured 13 s of re-reading), 2 when the next firmware flash is due anyway.

## Sitting

- setup: level, armed
- why: re-measure after the next change (the faster pace as default, or a can-bridge change to stop its CAN receive losing frames). The Sitting of 2026-11-23 gave 57.7 s and 49.3 s on Platform firmware 14 with pipelined refill reads.
- do:
  - With the launch up, Level done and ACTIVE:TRAJECTORY held (Armed), leave the robot untouched for 40 s, then upload the whole ring at the default pace: `ros2 service call /platform_imu/upload jugglebot_interfaces/srv/SetString "{data: \"{}\"}"`. Wait for "recording" on the GUI IMU panel and read `grep -n "Platform IMU upload finished" ~/Desktop/Jugglebot/temp/logs/launch_<date>_1.log | tail -1`.
  - Faster pace: `ros2 param set /teensy_bridge_node imu_read_gap_10us 63`, leave the robot untouched for 40 s from the end of the last upload, then the same upload call and the same grep.
  - `ros2 param set /teensy_bridge_node imu_read_gap_10us 80` and `ros2 param get /teensy_bridge_node imu_read_gap_10us` must read 80.
- pass:
  - Both uploads read "32000 samples", "complete yes, crc ok"; the default pace takes 57 s or less and the faster pace 48 s or less.
  - Computed at the debrief from the capture files: bridge and Jetson frame counts agree, and the transmit counters name the side that loses frames.
- record:
  - Wall-clock time of each upload. The finished lines are in the launch log, nothing to type.
- minutes: 10

## Sitting result (2026-11-23 14:55)

- verdict: fail
- notes: First upload took 57.7 sec, while second took 49.3. (pasted log lines are in the sheet results file) (sheet 2026-11-23-robot, step 6) [derived: 57.7 s at gap 80 (limit 57), 49.3 s at gap 63 (limit 48); both 32000 samples, crc ok; bridge minus Jetson frames 0; Platform sent 66518 of 66518, bridge received 66116: the bridge CAN receive loses the frames]

## Progress

- 2026-11-20: Sitting 2026-11-20: Armed uploads took 79 s (ring) and 84 s (32 s span) against 56 s at rest, with 14651-19974 duplicate frames (158 at rest), up to 6 block timeouts, CAN1 at 74-77 %. A juggle goal was refused 7 s into a 37 s upload. Diagnosis in logbook 2026-11-20-sitting-debrief-robot-2.
- 2026-11-21: 2026-11-21: Jetson-only hole-fill change landed (READs cover one cluster of holes, settle 5 ms); modelled 79 s to about 64 s Armed. To verify on the robot from the new upload-finished log line (logbook 2026-11-21-platform-imu-upload-hole-fill).
- 2026-11-21: 2026-11-21: FW 13 flashed; at rest a whole-ring upload takes 52.6 s at gap 80 and 41.4 s at gap 63 with 0 duplicates (56.25 s and 158 on FW 12); 250 of 263 READs chained. Armed check and the 1600 frames/s trial are on IDEA-20's Sitting row (logbook 2026-11-21-platform-imu-fw13-chaining-pacing-2khz).
- 2026-11-22: Sitting 2026-11-22, FW 13 Armed: whole ring 59.7 s (24 duplicates) and 63.3 s (1694) against 79 to 84 s on 2026-11-20. Gap 63 gave no gain per sample (2.08 ms against 1.87 to 1.98): lost frames and refills ate it. 2 kHz Attempt upload 71.7 s, 723 lost frames against 5 at rest. Why duplicates vary 24 to 1694 is unexplained.
- 2026-11-23: Sitting 2026-11-23, FW 14 Armed: whole ring 57.7 s at gap 80 and 49.3 s at gap 63 (limits 57 and 48 missed). Refill reads now about 20 ms each. The Platform sent every frame (66518 of 66518, error counters 0); the bridge counted about 400 fewer: its CAN receive loses them. Logbook 2026-11-23-sitting-debrief-robot.
