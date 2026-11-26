---
id: BUG-133
kind: bug
title: Ball Butler firmware builds against the unpatched FlexCAN_T4 and stamps the sync at dispatch
status: inbox
created: '2026-11-23'
updated: '2026-11-26T03:18:15Z'
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

- setup: bridge-cycled, bb-fed
- why: the held Ball Butler FW 10 builds against the patched FlexCAN_T4 and reports its receive-ring health and uptime; it must be flashed and shown to Feed as before, with the ring numbers in the bag.
- do:
  - Launch down. Check out BallButler branch `offline/bug133-flexcan` and Jugglebot branch `offline/bb-uptime` together, then colcon build; both merge to main after a pass.
  - Flash the Ball Butler from `ball_butler_main` and read back its firmware version.
  - Launch; confirm the `BB_STATUS: first frame` log line and `bb_fw_version` 10 on `/link_status`.
  - Fly Feed Throws for as long as the Sitting allows without power-cycling the Ball Butler, including at least 2 minutes with the Ball Butler at rest.
- pass: the Ball Butler reports FW 10; Feeds land as before; `bb_rx_ring_leak_max` is 0 in every frame; `bb_rx_ring_depth_min` is 0 in every frame of the idle stretch; `bb_uptime_s` rises with no reset; Feed arrival minus `landing_time` shows no upward slope against `bb_uptime_s` (under about 3 ms per hour).
- record: the hex sha256, the version read back, the bag paths, the largest `bb_rx_ring_leak_max`, `bb_rx_ring_depth_min` and `bb_rx_ring_depth_max` per bag, the last `bb_rx_fifo_overflows`, the uptime span, and per bag n, median, sd and slope of arrival minus `landing_time`.
- minutes: 20
- branch: offline/bb-uptime

## Progress

- 2026-11-26: offline, the bag proxy (Feed arrival minus landing_time; 11-13, 11-18, 11-20) cannot tell: slopes +22, +4.0, +0.3 ms/h, per-Throw sd 7 to 9 ms against a 2.6 ms step, and no bag topic carries Ball Butler uptime. The saturated case is excluded. Held fix: BallButler branch offline/bug133-flexcan (FW 10, patched library only, built, not flashed). Outputs: temp/probes/bug133-bb/.
- 2026-11-26: Held FW 10 now also sends a status frame every 500 ms (receive-ring depth min and max, leak, uptime, FIFO overruns); the host shows them on /link_status as bb_* keys, absent on FW 9. No can-bridge change. Branches to merge together at the flash: BallButler offline/bug133-flexcan 5128a50 (hex sha256 e49156f1...cff89ffe) and Jugglebot offline/bb-uptime.
