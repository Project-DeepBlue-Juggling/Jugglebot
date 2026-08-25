---
title: The UDP panel's Gaps column is a measurement artifact, not message loss
type: investigation
date: 2026-08-25
status: resolved
phase: "operator-observability F2"
related_plan: operator-observability.md
subsystem:
  - gui
  - teensy_link
tags:
  - observability
  - udp
---

# UDP Gaps column — by-construction artifact

## Symptoms

First sitting with the F2 UDP panel (2026-08-25): `TELEMETRY`, `LEG_CMD` and
`BB_AXIS_ESTIMATES` all showed **Gaps ≡ RX**; `HAND_SENSOR` showed **Gaps ≈
RX/2**. At face value, a link losing nearly every frame.

## Diagnosis

**VERDICT: by-construction measurement artifact, not message loss.** The wire
carries ONE shared sequence counter per (channel, direction) —
`docs/teensy-udp-protocol.md:21`, incremented at
`ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.cpp:171`
(`const uint16_t seq = (*seq_ctr)++;`, one counter per socket at `:41-42`). But
`teensy_link/client.py::_track_seq` (`:595`) keeps `_last_rx_seq` **per message
type** and scores a gap unless `seq == prev+1` — so *any* interleaving frame of
another type scores a gap on the next frame of the type you are reading.
`telemetry.cpp:619-622` closes it: `telemetry_step()` calls `send_telemetry();
send_bb_estimates(); send_leg_cmd();` back-to-back every tick at
`TELEM_RATE_HZ = 100`, making `gaps ≡ rx − 1` deterministic for those three.

## Discussion

Per-type loss is **unrecoverable** from a shared counter: no definition over
`_last_rx_seq[type]` can separate "a frame of this type was lost" from "a frame
of another type went out in between". The intent was unsatisfiable from day one,
and the only unit test drove a single type through a silent link — where the two
are indistinguishable.

`HAND_SENSOR ≈ RX/2` is **not** this mechanism (which predicts `≡ rx − 1` there
too) and cannot be loss — receiver-side loss cannot create sender-side seq
adjacency. Leading explanation is the panel's unit mixing: rx/tx are **rates**
(msg/s), Gaps is a **cumulative total**, so the eyeballed ratio was cross-unit.
Consistent with that, `HAND_SENSOR` is not in the per-tick trio — it comes from
`hand_sensor_uplink_step()` (`telemetry.cpp:260-289`), gated on reply freshness
with a 1 Hz keepalive, and a `static_assert` pins the poll no faster than the
telemetry tick (`CHECK_INTERVAL_MS` 20 ms against a 10 ms tick), so its true
rate genuinely is about half `TELEMETRY`'s. A discriminator was pre-registered
here — one `ros2 topic echo /udp_diag`, comparing cumulative `rx_HAND_SENSOR`
against `gap_HAND_SENSOR` in the same message — but it was **never run, and is
now DROPPED by decision**: P1 deleted the `gap_<TYPE>` keys the same day, and the
firmware answers the question without it. See Open Questions.

## Fix

**This close-out session implemented only the column removal.** The
investigation itself was read-only; the one exception was taken in the same
commit — the misleading column was removed from the panel
(`udp-traffic.js:511`), while the producer counters and the `gap_<TYPE>` keys
were left untouched as P1. The fix proper was filed as
`plans/archived/udp-channel-health.md`, written in parallel with this entry and
at `status: proposed` **at that moment** — nothing beyond the column
implemented, nothing yet approved. **The producer counters and the
channel-health metrics landed LATER THE SAME DAY** as P1–P4, after the owner
redirected the plan from *counting gaps* to *gauging channel health*; that plan
is now archived `completed`
(`logbook/2026-08-25-udp-channel-health-implementation.md`).

That plan was **rescoped the same day after owner review**: the question is
*gauge channel health*, not *count gaps*, so it now carries four independently
landable phases ranked by diagnostic value — P1 deletes this column and
`seq_gaps_by_type`; P2 puts the alarmed `latency_monitor` verdict on the panel
(latency being the failure class loss counters are blind to, per the five-week
uptime-lag arc); P3 adds per-flow rate conformance against a firmware-pinned
nominal table and fixes the units mixing; and P4 keeps the original intended fix
as an honesty-check in the footer — a **per-socket aggregate**, mirroring what
the Teensy already does for the downlink (`track_downlink_seq`,
`udp_link.cpp:72` — one `s_last_dn_seq` per socket, one `seq_gaps` scalar, RPC
socket excluded), which makes the two link ends measure the same quantity.

Ruled out: a per-type sequence counter in firmware. It would break the wrap-safe
monotonic-seq guard at `leg_interp.cpp:186-193`, which correctly depends on the
shared counter — its own comment records that setpoint seqs are "strictly
MONOTONIC but NOT contiguous" because heartbeats consume intervening values, and
a `seq == last+1` guard there was already rejected once.

## Outcome

**No link-health concern.** `crc_errors`, `decode_errors` and `drain_capped` are
the honest numbers on that panel and were clean for the sitting. The per-type
gap counters survived on `/udp_diag` only until P1 landed later the same day
(2026-08-25); they were never to be read as loss, and they are gone.

## Verification

Code-reading, not a test run: (2026-08-25, read-only investigation of
`teensy_link/client.py` plus
`ros_ws/src/jugglebot/Teensy_code_canbridge/{udp_link,telemetry,leg_interp}.cpp`,
mechanism confirmed at every file:line cited above).

## Open Questions

**None. The one that was open is CLOSED — by the firmware read, not by the
measurement (2026-08-25).**

`HAND_SENSOR ≈ RX/2` was to be settled by the `/udp_diag` echo above. That echo
was destroyed unrun: P1 landed the same day and deleted the `gap_<TYPE>` keys it
would have read. It is deliberately NOT being reinstated, because the source
already answers it. `send_to` consumes ONE sequence counter per socket
(`udp_link.cpp:171`) for every message type, and `telemetry_step` emits its trio
back-to-back on a single tick (`telemetry.cpp:619-622`) — so **any** per-type
ratio taken over that shared counter is an artifact BY CONSTRUCTION, and the
particular value of the ratio is a fact about how often another type interleaved
with the flow being read, not about the link. For `HAND_SENSOR` that cadence is
round-trip paced: it sends once per poll REPLY (`telemetry.cpp:273-275` gates on
the reply's wall stamp, plus a flags flip — never on the ball verdict changing
value), so the interleave pattern carries the CAN round-trip jitter of the poll.

Running the echo would therefore have confirmed a mechanism already proven from
the firmware, at the cost of implying the exact ratio meant something. It has no
remaining diagnostic value, and nothing downstream is waiting on it.
