---
id: TKT-88
kind: ticket
title: 'IDEA-25: which lever shortens the Armed IMU upload: the Jetson-only hole-fill change, firmware counters first, sample packing, or leave it?'
status: open
created: '2026-11-21'
updated: '2026-11-20T15:51:58Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: platform-teensy
blocked_by: []
links: []
parent: idea-minimise-the-platform-imu-upload-time-after-an-attempt
type: question
---

Rung P4 passed at the Sitting of 2026-11-20, so IDEA-25's trigger has fired. That debrief measured Armed IMU uploads at 79-84 s against 56 s at rest, and traced most of the loss to the hole-fill re-reading words it already holds after about 0.4 % of frames go missing.

## Options

1. **Jetson-only hole-fill change, no flash.** Each hole-fill READ ends at a run of more than 16 present words, and the settle after a holed block drops from 30 ms to about 5 ms. This is modelled at 79 s to about 64 s Armed. The Night Shift builds it with tests, holds it for a Sitting check, and you merge it after that check.
2. **Firmware counters first.** Add transmit and receive counters on the Platform Teensy and the can-bridge to find where the frames are lost. This needs two flashes and a day Session.
3. **Sample packing.** About 1.5 samples go in each frame, a wire-format change on both sides. It gives the largest gain and costs the most effort.
4. **Leave it.** About 80 s between Attempts is acceptable for now.

## What each answer unblocks

- 1: the next night builds the change and holds it for a Sitting check.
- 2 or 3: a day Session does it. The Item records your choice.
- 4: IDEA-25 goes back to backlog.

Recommendation: 1 now, then 2 at the next firmware flash.
