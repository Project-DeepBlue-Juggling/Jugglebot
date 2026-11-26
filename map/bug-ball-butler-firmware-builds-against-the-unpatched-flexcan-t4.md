---
id: BUG-133
kind: bug
title: Ball Butler firmware builds against the unpatched FlexCAN_T4 and stamps the sync at dispatch
status: inbox
created: '2026-11-23'
updated: '2026-11-26T01:17:20Z'
owners: []
priority: medium
effort: m
hardware: robot
radius: subsystem
diagnosis: confirmed
area: ball-butler
blocked_by: []
links:
- logbook/2026-11-23-platform-imu-clock-early-rx-ring-leak.md
filed_by: claude
night_skip: out-of-reach
night_skip_on: '2026-11-24'
night_skip_note: The fix is in the BallButler repo's firmware, not in this clone, and needs a flash; its deciding measurement needs Ball Butler uptime per Throw.
---

Ball Butler firmware (BallButler repo, `ball_butler_main`) builds against upstream FlexCAN_T4 master and stamps the 0x7DD sync at dispatch. The Platform had the same defect, diagnosed in BUG-132 (logbook entry 2026-11-23-platform-imu-clock-early-rx-ring-leak).

## Findings

- Same unprotected RX pop, so stranded frames turn the RX ring into a delay line: the wall clock reads early and the Throw command arrives late by the same delay, so Ball Butler would release late.
- Cost is about 2.6 ms per stranded frame (about 390 frames/s on its bus), growing with uptime. Inferred growth is 0.3 to 8 ms per hour; saturated it is about 0.65 s.
- On 2026-10-12 the release landed 3.9 ms before `release_wall` (sd 2.6), so the delay was about zero then. Ball Butler is power-cycled each Sitting, which resets it.

## Proposed fix

- Build against the can-bridge's patched copy (pop with the RX interrupt masked), and stamp 0x7DD at reception.

## Deciding measurement

- Release minus `release_wall` per Throw, against Ball Butler uptime. A slow ratchet toward late in 2.6 ms steps convicts it.

## Sitting

- setup: bridge-cycled
- why: the held Ball Butler FW 10 builds against the patched FlexCAN_T4; it must be flashed and shown to Feed as before. Owner decision pending on flashing it.
- do:
  - Launch down. Merge BallButler branch `offline/bug133-flexcan` and set `BB_FW_VERSION_EXPECTED` to 10 in `teensy_link/rpc_args.py`.
  - Flash the Ball Butler from `ball_butler_main` and read back its firmware version.
  - Launch and fly Feed Throws for as long as the Sitting allows without power-cycling the Ball Butler.
- pass: the Ball Butler reports FW 10; Feeds land as before; Feed arrival minus `landing_time` shows no upward slope over the Sitting (under about 3 ms per hour).
- record: the hex sha256, the version read back, the bag paths, and per bag n, median, sd and slope of arrival minus `landing_time`.
- minutes: 15

## Progress

- 2026-11-26: offline, the bag proxy (Feed arrival minus landing_time; 11-13, 11-18, 11-20) cannot tell: slopes +22, +4.0, +0.3 ms/h, per-Throw sd 7 to 9 ms against a 2.6 ms step, and no bag topic carries Ball Butler uptime. The saturated case is excluded. Held fix: BallButler branch offline/bug133-flexcan (FW 10, patched library only, built, not flashed). Outputs: temp/probes/bug133-bb/.
