---
id: BUG-99
kind: bug
title: can-bridge firmware accepts four discontinuities on rare paths and has no armed interlock on reconfiguration RPCs (to confirm)
status: inbox
created: '2026-11-14'
updated: '2026-11-27T04:12:10Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: none
radius: normative
diagnosis: hypothesis
area: firmware
blocked_by: []
links: []
filed_by: claude
night_skip: owner-session
night_skip_on: '2026-11-28'
night_skip_note: The 2026-11-27 powered Sitting on FW 53 met its open condition (nothing regressed); closing it is yours.
---

Four can-bridge firmware paths the 2026-11-13 maintainer review (Opus read of the firmware) reported as accepting a discontinuity or lacking an interlock. None was re-read by the coordinating Session, so each needs confirming before any change. All are on rare paths.

## Findings to confirm

| Where | Report |
| --- | --- |
| `leg_interp.cpp:1107`, `:1147`, `:815-816` | Switching between scheduled and arrival-phased playback has no continuity check: `latch_from_staging` clears `active`, and `sched_apply` only checks continuity when `active` is set. Estimated step about velocity times lateness. |
| `leg_interp.cpp:833-848` | During PLAY a discontinuous promotion is accepted and only counted; a time-base slew of up to 1.25 ms per anchor re-maps later stamps. |
| `leg_interp.cpp:1317-1339` | On the older arrival-phased Leg path, starvation holds the last position with the last velocity feedforward until the staleness stop, where the Hand block beside it decays. |
| `rpc.cpp:283-322`, `:408-411` | Reconfiguration RPCs (absolute position, controller mode, axis state, SDO write, position gain) are forwarded while the stream is live. A per-axis clear of errors also releases the global Soft E-stop latch (`:355`). |

## Also reported

- Scheduled stamps are bounded only in the future direction (`leg_interp.cpp:1037`, `:895`); a far-past stamp can overflow the signed arithmetic. Reachable only through a host fault.
- UDP sources are not checked against the Jetson address (`udp_link.cpp:132-150`).
- The limit-cycle invariant "position gain times `MAX_LEAD_REV` stays under the Leg velocity limit" lives only in a YAML comment; the header has `static_assert` for its siblings but not this one (`canbridge_config.h:222`).

## Wanted

Confirm each row against the code and the native tests, then decide per row: fix, pin with a `static_assert` or test, or record as accepted with its bound.

## Progress

- 2026-11-25: FW 52 flashed 20:54 (BridgeIdentity 52/14, unpowered launch check clean). It carries the stamp bound, the lead-clamp assert and the pins (FW 50) and the owner decisions D1 to D5: Armed refusals, per-axis clear scope, held clock offset, Jetson-only UDP. Contract row I-FW-18. Open until a powered Sitting confirms nothing regressed.
- 2026-11-26: row 2 measured from 9 bags (clock_diag): the offset slews at most 1.25 ms once per 30 s; worst applied 0.77 ms, about 1.3 mm at the Hand at 50 rev/s (computed). Accepted with that bound, no rate limit. /link_status now carries the eight promo fields so a powered Sitting records the counters. Steps over 20 ms were seen only at bag start.
- 2026-11-27: Powered Sitting 2026-11-27 on FW 53 (which includes FW 52): five launches, 11 rises and Stows, 14 Attempts, no new refusal, HELD bit or Guard latch beyond a launch before the 45 V supply was on and the FAULT after a commanded drive reboot. The Armed refusal was exercised once (drive reboot refused at the Active pose).
