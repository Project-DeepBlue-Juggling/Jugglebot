---
id: FEAT-40
kind: feature
title: Install the host hold within one tick of a firmware scheduler STOP so a guard latch becomes a recoverable LANE_REFUSED
status: backlog
created: '2026-11-04'
updated: '2026-11-07T17:49:27Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: decision
radius: subsystem
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-04-sitting-debrief.md
- bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid
filed_by: claude
---

Filed by the 2026-11-04 debrief (2026-11-03 Sitting).

- After a firmware STOP the host learns of it from the 10 Hz /link_status poll (~100 ms), then installs the hold 130-214 ms after the skill END; goal 15's Hand guard tripped MAX_DEVIATION 68 ms after the END (STOP ~022.74, END 022.839, trip 022.907, hold 023.045).
- Candidate: read sched_stops/sched_refused on the faster telemetry path and install the hold in the same tick (skill_node _on_tick LANE_REFUSED branch / force_hold path).
- Converts goal-3/15-style guard latches (FAULT, operator CLEAR_ERRORS) into goal-9-style LANE_REFUSED with auto-recovery; does not remove the Attempt end.
- Owner decision: touches the hold path next to the Teensy guard (the guard stays the authority).
- Probe STOP->hold latency first.

## Options (2026-11-05)

Where the host learns of a STOP today: `skill_node` reads `sched_refused` only from the bridge's 10 Hz `link_status` diagnostic (`_on_link_status`, `_hand_lane_refused`), so a STOP can sit up to ~100 ms plus a 25 ms tick before `_on_tick`'s LANE_REFUSED branch installs the hold.

1. Probe first: measure STOP to hold latency per STOP from the launch logs and bags already flown (the Item's own Next). No code on the robot path. xs.
2. Publish `link_status` on change as well as at 10 Hz, when `sched_stops` or `sched_refused` moves, so `skill_node` acts within one tick. Same message type, so no interface rebuild. s, needs a Sitting.
3. Raise `link_status` to 40 Hz. One parameter, but 4x the diagnostic traffic on a loaded box. xs, needs a Sitting.
4. Leave it: the Teensy guard stays the authority and a latch costs one operator CLEAR_ERRORS.

Recommendation: 1, then 2 if the probe shows the latch would have been a LANE_REFUSED inside one tick.

## Findings (2026-11-06)

Correction to the Options of 2026-11-05: option 2 cannot do what it said. The firmware reports a STOP only through `sched_stops`, `sched_refused` and the `SCHED_HOLD_LATCHED` flag in its heartbeat, and the heartbeat is 10 Hz (`HEARTBEAT_HZ` in `config/generated/udp_protocol.py`). The 100 Hz `TELEMETRY` frame does not carry them.

| Leg | Today | With publish on change |
|---|---|---|
| STOP to the next heartbeat | 0-100 ms | 0-100 ms, unchanged |
| Heartbeat to `/link_status` | 0-100 ms (timer phase) | about 0 |
| `/link_status` to the END | up to 25 ms (one tick) | up to 25 ms |
| END to hold installed | 110-210 ms (probe) | 110-210 ms, unchanged |

- Publishing on change saves at most 100 ms, about 50 ms on average.
- The probe's two guard latches beat the hold by 170 ms and 51 ms, so at best it turns one of those two around.
- The Option 2 recommendation was conditional on the probe showing a LANE_REFUSED inside one tick. It does not.
- FEAT-41's FW 40 may remove most of these STOPs, since each clean STOP fits one missing setpoint frame.

No code was written for this Item tonight.

## Findings (2026-11-07)

Verdict: FW 40 did not change the race. The firmware already holds a stopped lane itself; the guard latch comes from the host plan still streaming frames the firmware refuses, and the host hold cannot beat a guard that trips in tens of ms.

### 1. What a STOP does today (FW 40, main)

Source: `leg_interp.cpp` block comment at 362-408, `sched_apply` 725-752, Hand guard 1455-1500, `canbridge_config.h` 499-544.

| Step | Owner |
|---|---|
| Last frame's 50 ms cover ends, span-2 cubic continues for the grace (legs 20 ms, Hand 14 ms if rising over 50 rev/s) | firmware |
| Grace out: C2 stop (Hand: C1 stop) to rest, then HOLD | firmware |
| Next frame within tolerance (Hand 0.05 rev, 8 rev/s): resumes | firmware, no host action |
| Next frame discontinuous while armed: REFUSED, lane latched, `sched_refused` +1 | firmware |
| Refused frame becomes the base command, so the guard measures the walk-away | firmware |

- Hand guard: `MAX_DEVIATION_HAND_REV` 2.5 rev, counts only anchors fresher than 30 ms.
- Leg guard: 1.0 rev.
- By design a refusal "fails loud": the guard is meant to trip if the plan keeps walking.

### 1b. The host side

- The host learns of a refusal only through `sched_refused` on the 10 Hz `/link_status` (`skill_node.py` `_on_link_status` 1966, `_hand_lane_refused` 2074).
- `_on_tick` then ends the Attempt LANE_REFUSED and sets `_force_hold` (3665-3671). `trajectory/hold` builds a hand-less plan (`trajectory_node._svc_hold`), so no further Hand frame is sent.
- A STOP that resumes cleanly never reaches this path; only a refusal does.
- Race: the firmware holds from the STOP on. The danger window is refusal to hold installed: the plan keeps walking and the refused base command widens the deviation.

### 1c. Why the host hold cannot win (hypothesis, arithmetic only)

- Hand command speeds at a STOP were 52-100 rev/s (BUG-66 Progress).
- At 50 rev/s the 2.5 rev band fills in about 50 ms. At 78 rev/s, about 32 ms.
- Measured STOP to hold: 130-340 ms; END to hold installed: 110-210 ms; guard beat the hold in 2 of 4 (by 170 ms and 51 ms) (FEAT-40 probe, 2026-11-05).
- Publish on change saves at most 100 ms (about 50 ms mean). That is under the END to hold term alone.
- A same-tick install would still leave `trajectory/hold` service time. It cannot turn a 32-50 ms guard budget into a win.
- Not measured: the hold's service latency split. Option 4 of the ticket (read where 110-210 ms goes) is still open.

### 2. How often since FW 39 / FW 40

FW 40 flashed 2026-11-06 00:52 and has flown (debrief `can-bridge v40`, FEAT-41 closed pass: lost 0, queue_dropped +0, 12613 setpoints).

| Sitting | STOPs | Refused ends / guard latches from a STOP |
|---|---|---|
| 2026-11-06 first debrief | 9 WARNs, 2 at Attempt start | no LANE_REFUSED end (BUG-58 row) |
| 2026-11-06 endurance (14:55) | 25 on a clean stream in 51 min | none refused the lane (BUG-66 Progress); 2 LANE_REFUSED ends were GC pauses in `trajectory_node` (58-64 ms) |
| 2026-11-06 20:20 | 3, none in a goal | none |

- Hand C1 stops still fire under FW 40 with lost 0: about 1 per 1175 Throw upstrokes, host gaps (14:55 L7017, forward gap 60 ms). The 14 ms grace stays (owner, `2026-11-06-hand-c1-grace-stays-14-ms`).
- No guard latch in these Sittings is attributed to the STOP-to-hold race. The FW 40-era fault ends the debriefs name were drive faults (SPINOUT, CURRENT_LIMIT_VIOLATION); one end is labelled GUARD_LATCHED (BUG-68) with no STOP timing given, so it is unclassified.
- Not read: raw logs. The debriefs do not give a per-STOP guard split, so "none" is as far as the entries go.
- No Runbook row exists for FEAT-40 (`docs/runbook.md` lists it as needing a section). The only row touching STOPs is BUG-58's, a one-minute watch for a LANE_REFUSED under `gc_probe`.

### 3. Where the Item sits

Needed in a smaller form, or waiting on BUG-58's watch row. The 2026-11-03 STOP supply (one missing frame, 1.3-1.5 %) is gone; the remainder is host gaps from GC pauses, which BUG-58's `gc_freeze` trial addresses.

Options:
1. Wait: the BUG-58 `gc_freeze` arm and the next LANE_REFUSED end under `gc_probe` decide whether any STOP-to-guard latch still occurs. Touches nothing on the robot path.
2. Publish on change anyway: saves about 50 ms mean, cannot beat a 32-50 ms guard. Not recommended; the code no longer supports the case for it.
3. Firmware edge heartbeat on a STOP (ticket option 3): about 1 ms to the host, plus a host change. Still leaves the hold service time. Touches firmware and the host hold path; owner Session, safety-bearing.
4. Firmware stops adopting a refused frame as base command, or freezes it while the host hold lands (hypothesis, not in the ticket). Removes the race at its source but reverses the "fails loud" design, so the guard would no longer trip on a walking plan. Safety-bearing; offline replay first.
5. Park: the guard stays the authority, a latch costs one CLEAR_ERRORS, and STOPs that refuse have not recurred since FW 40.

Recommendation: 5 now, with option 1 as the unpark condition (a LANE_REFUSED or guard latch within 300 ms of a STOP on a Sitting on FW 40, with `lost 0`). If that appears, take option 3 first, since 4 weakens the guard.

### Gaps

- No STOP-to-hold latency was re-measured on FW 40; the 110-210 ms is from 2026-11-05 data (FW 38/39).
- GC-pause LANE_REFUSED ends on FW 40 (73 and 62 Throw runs) did recover through the host hold without a latch; the debriefs do not say how long it took.

## Parked (2026-11-08)

Parked on the owner's answer in TKT-45. The guard stays the authority and a latch costs one CLEAR_ERRORS.

- **Unpark when:** a Sitting on FW 40 or later shows a LANE_REFUSED or a guard latch within 300 ms of a STOP, with `lost 0`.
- **Then:** take the firmware edge heartbeat first, because stopping the firmware adopting a refused frame weakens the guard.

## Progress

- 2026-11-05: Probe done 2026-11-05 (option 1): STOP -> hold 130-340 / 300 / 320 / 275 ms; WARN lag from 10 Hz link_status 0-130 ms, then 110-210 ms END -> hold installed. Guard latched before the hold in 2 of 4 (L1399 by 170 ms, L2361 by 51 ms). Publishing on change helps; the END -> install leg is the larger term.
- 2026-11-08: Night Shift 2026-11-08: parked on the owner's answer in TKT-45 until a LANE_REFUSED or guard latch within 300 ms of a STOP on FW 40 with lost 0 (detail below)
