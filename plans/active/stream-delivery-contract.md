---
title: Stream delivery contract - no Attempt lost to a host delay (prevent, tolerate, fail soft)
created: 2026-11-29
status: active
owner: Harrison
last_updated: 2026-11-29
pathway: stream-delivery-no-attempt-lost-to-a-host-delay
branch: main
scope: >-
  About 1 in 10 two-ball Attempts ends in a Hand MAX_DEVIATION latch after a host delay of 34-58 ms in delivering
  the Knot stream. Three layers close the class: the forwarding path leaves the rclpy process (prevent), FW 54 frames
  carry 7 Knots of which 4 are committed (tolerate), and the firmware reports a STOP at once so the forwarder holds
  (fail soft). The Guard is not loosened.
steps:
  - {id: PS0, title: Evidence and offline reproduction, kind: phase, status: in_progress, effort: m, hardware: none}
  - {id: PS1, title: Forwarder process and stall instruments, kind: phase, status: in_progress, effort: m, hardware: none}
  - {id: PS2, title: Sitting S1 on FW 53, kind: phase, status: not_started, effort: s, hardware: robot}
  - {id: PS3, title: Committed horizon in the planner, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: PS4, title: FW 54 deep frames, kind: phase, status: in_progress, effort: l, hardware: none}
  - {id: PS5, title: FW 54 STOP event and the forwarder hold, kind: phase, status: not_started, effort: l, hardware: none}
  - {id: PS6, title: Bench B1, kind: phase, status: not_started, effort: s, hardware: bench-hand}
  - {id: PS7, title: Sitting S2 and close-out, kind: phase, status: not_started, effort: m, hardware: robot}
related_logbook:
  - 2026-11-28-first-flight-of-the-learner-merge-and-the-15-mm-catch-authority-trial.md
  - 2026-11-25-emit-lead-planner-derivation.md
  - 2026-11-25-emit-lead-p2-measurements.md
  - 2026-11-23-bridge-stall-watchdog.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py (the setpoint thread and the :5557 drain)
  - teensy_link/stream_forwarder.py (new in PS1), teensy_link/setpoint_pump.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp (cover, STOP, refusal, the latched Hand guard), fault_machine.cpp (the 10 Hz latch)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py (WIRE_READ_KNOTS, EMIT_LEAD_KNOTS), emitter.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md (I-FW-1, I-FW-2; new rows in § 4)
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py (the firmware's Python twin)
---

# Stream delivery contract

Branch `main`. Pathway PWAY-12 (`map/pathway-stream-delivery-no-attempt-lost-to-a-host-delay.md`).
It takes over BUG-89, BUG-58, FEAT-40 and FEAT-31. Narrative of the investigation: the logbook entry
written at PS0's close.

## 1. Context

The chain, identical in the three events of 2026-11-28 and behind 15 of 17 Hand MAX_DEVIATION latches
since 2026-11-18:

1. The setpoint thread in `teensy_bridge_node` stalls 34-58 ms. The emitter is on time.
2. The frame it then forwards reaches the can-bridge firmware 4.7-7.7 ms after the playing frame's
   cover (50 ms) plus Hand grace (20 ms) ran out. The firmware STOPs the Hand and latches a hold.
3. The firmware refuses the following frames, because the plan is still moving.
4. The host learns of the STOP in the 10 Hz heartbeat and its hold lands 80-340 ms later. The plan
   walks 2.7-8.1 rev from the held Hand and the 2.5 rev Guard trips.

The Guard reports a real walk-away and is not loosened anywhere in this plan.

**Root cause, from the code.** A frame stamped S with N Knots covers S + (N-1)T, T = 25 ms. In the
worst phase the largest delivery delay with no STOP is (N-2)T + grace - l, where l is the normal
stamp-to-firmware latency (6-13 ms today). With N = 3 that is about 29 ms on the fast upstroke and
35 ms elsewhere. The observed stalls exceed it by construction.

**What the evidence says about the stall** (2026-11-18 to 2026-11-28, 121 Attempts):

| Finding | Evidence |
|---|---|
| The stall is inside the bridge process | The bridge's own publishers gap in 6 of 9 stalls; the tracker in 0 of 9; emitter max gap 30.3 ms |
| Solver load raises the rate, it is not required | 15 of 23 stalls overlap a solve, 2.1 times chance (p = 0.0007); pre-registered bar was 3 times: UNDETERMINED |
| None of the three latch events had a solve running | Each stall ended 5-21 ms before the next CATCH solve began (log time, +/- 10-20 ms) |
| CPU contention is not the lever | `cpu_partition` A/B 2026-11-18: 3.2 against 3.3 drops per minute |
| GC is a second contributor | Excluded for the three latch events (probe n = 0); 61-94 ms pauses at two other stalls |
| A larger class exists | Five watchdog dumps of 313-372 ms show an executor thread inside rclpy `publish` |
| The watchdog is blind to this class | It needs a stall of 100-150 ms to dump |
| 2026-11-28 was not unusual | 3 latches in 18 Attempts, p = 0.34 against the base rate |

Working hypothesis, not yet shown: the setpoint thread shares the GIL with the executor, and a
blocking call under the GIL in an executor thread freezes it. PS1's instruments test this at PS2.
The fix in PS1 does not depend on it: it removes every in-process cause at once.

**Offline reproduction** (the firmware's Python twin, synthetic 60 rev/s stroke): a STOP from 38 ms
of delay at peak upward speed and 46 ms elsewhere; the Guard band fills 62-88 ms after the STOP at
peak speed and 104-132 ms on the way down; a stall at the slow top of the stroke stops and resumes
with no refusal. PS0 replaces these with numbers from the real firmware build.

## 2. Owner decisions 2026-11-29

- "Fixed for good" means no Attempt lost at all to this class.
- A firmware change is in scope (FW 54, PROTOCOL 15).
- The cost of one more Knot of planner horizon is accepted.
- The stall watchdog may be lowered for the next Sitting.
- Nothing on the box is known to coincide with the events.

Open, for the owner (§ 6): the Leg guard reference change, and whether sessions may run suites
during a Sitting.

## 3. Design

Three layers. Each row says what it removes and what it leaves.

| Layer | What | Removes | Leaves | Cost |
|---|---|---|---|---|
| Prevent | The forwarding path runs in a process that never imports rclpy (`teensy_link/stream_forwarder.py`), blocking on the socket | Every in-process stall of the bridge: GIL, DDS lock, bridge GC | Emitter-side delays in `trajectory_node`; box-wide starvation | A second process to supervise; flag off by default until PS2 |
| Tolerate | FW 54 frames carry 7 Knots (150 ms of plan), the first 4 committed | A STOP for any delay up to X_g; for most delays up to X_c | A delay past X_c; a delay past X_g with an install inside the stall | One Knot of planner horizon (re-aim budget p50 276 to 263 ms); wire 240 to about 466 B; a flash |
| Fail soft | FW 54 sends a STOP event within 2 ms with the stop's final position; the forwarder sends no frame that fails the resume test and holds the stopped group | The Guard latch after a STOP, while the forwarder lives | The Attempt itself: a Hand STOP mid-Throw cannot be survived | A new message and port; hold logic nearest the wire |

**X, the no-STOP delay, worst phase.** X_g = (C-2)T + grace - l, about 66 ms at l = 4 ms, for any
install timing. X_c = (N-2)T + grace - l, about 135-141 ms, when no Splice, re-aim or hold changed a
tentative Knot during the stall. l under the forwarder is unmeasured and X moves one for one with
it; PS2 measures it. Five committed Knots would give X_g of about 91 ms for a second Knot of horizon.

**Why deep frames and not an emit lead.** Both extend the cover and both cost one Knot of horizon.
The lead also moves the first Knot, which drops the fresh-REST rebase threshold from 75 to 50 ms
(the measured 40 % of RESTs newly rebased), relies on the firmware queue, and needs the drain
rewritten: with today's newest-only drain the twin shows lead 1 tolerating only 52 ms. Deep frames
at lead 0 keep the drain and the rebase threshold. `emit_lead_knots` stays 0. Only the
holds-at-the-wire-horizon change from `offline/emit-lead-p3` is still needed.

**Firmware promotion rule (FW 54).** While the playing frame is inside its committed cover plus
grace, a successor is accepted exactly as FW 53 accepts it. Past that, in the tentative spans, a
successor is accepted only within the existing RESUME tolerances; otherwise the group stops at once
and latches, as FW 53 does at exhaustion. Nothing that flies today is refused, and no tolerance
widens.

**Can an Attempt survive a Hand STOP mid-Throw?** No. The STOP takes the Hand to rest; the release
is gone. "No Attempt lost" therefore rests on the first two layers. The third only makes a loss soft:
no FAULT, no CLEAR_ERRORS.

**One 25 ms cycle under the design** (lead 0, N = 7, C = 4):

- On time: the successor is promoted about 4 ms into its first span on the same committed cubic.
  Position, velocity and acceleration feedforward are continuous, as today. A Splice changes Knots
  at least 4 periods ahead, which were sent as tentative and never played.
- 60 ms late: the playing frame runs on through committed Knots; the successor lands inside
  committed cover plus grace and is accepted. No STOP. Today this is the chain in § 1.
- 120 ms late: the playing frame is in its tentative spans. An unchanged plan passes the resume test
  and plays on. A plan changed inside the stall fails it: immediate stop, event, forwarder hold, no
  latch.
- Segment boundary: Knots are Knots; nothing special.

**Rejected.** Widening the firmware's resume tolerances (a 17 rev/s step into the Hand). SCHED_FIFO
on the solver worker (it stalled the stream 60-96 ms per solve on 2026-10-28). CPU pinning (no
effect on drops). A host hold from `skill_node` (it shares the paths that cost 80-340 ms).

## 4. Contract

Rows to land in `INVARIANTS.md` with their phase, each with one enforcement point and one test.

| ID | Statement | Enforcement point | Test | Phase |
|---|---|---|---|---|
| I-STREAM-3 | The forwarding path runs in a process that never imports rclpy; loop gaps over 10 ms are counted and reach /link_status | `stream_forwarder.py` | subprocess import test | PS1 |
| I-WIRE-8 | A frame carries FRAME_KNOTS Knots and declares COMMITTED_KNOTS; no install changes a committed Knot of a published frame | `schedule.py`, emitter `frame()` | published-Knot contract test, every install kind | PS3 |
| I-FW-20 | The promotion rule of § 3; exhaustion at (FRAME_KNOTS-1)T + grace | `leg_interp.cpp` `sched_apply` / `sched_advance` | native scheduler test | PS4 |
| I-FW-21 | The Leg MAX_DEVIATION reference is the start Knot of the span being played (amends I-FW-2; owner sign-off) | `leg_interp.cpp`, `fault_machine.cpp` | native fault-machine test | PS4 |
| I-FW-22 | Every STOP, refusal and resume edge is sent within 2 ms with the stop's final position | firmware net task | native test and host decode test | PS5 |
| I-STREAM-1 | A delivery delay up to X_g never produces a Hand STOP; up to X_c never, absent an install in the stall | firmware cover with the forwarder | twin stall sweep: delay by phase by install time | PS4 |
| I-STREAM-2 | While the forwarder lives, a STOP or refusal never produces a Guard latch | `stream_forwarder.py` hold rules | closed loop on the twin, stalls 60-240 ms | PS5 |

## 5. Phases

| Phase | Lands | Flies at |
|---|---|---|
| PS0 | main (tests, probe, logbook) | none |
| PS1 | main, flag off | PS2 |
| PS2 | Sitting | FW 53 |
| PS3 | main, defaults 3 and 3 | PS7 |
| PS4, PS5 | one held branch (PROTOCOL 15 lockstep) | PS6, PS7 |
| PS6 | bench | FW 54 |
| PS7 | Sitting, then merge | FW 54 |

### Phase S0: Evidence and offline reproduction

The tabulation and the digests are done (§ 1). Remaining: the chain promoted to
`tools/probes/stall_chain_probe.py`; `tests/firmware/test_stall_stop_guard_chain.py` pinning it on
FW 53 at lead 0; one native scenario on the real `leg_interp.cpp` and `fault_machine.cpp` that
measures the STOP-to-exceed time and the largest safe hold latency, replacing the twin's estimates.
Close: the logbook entry in full investigation form, with the pre-registered criteria and the
UNDETERMINED verdict.

### Phase S1: Forwarder process and stall instruments

- `teensy_link/stream_forwarder.py`: spawned by the bridge (posix_spawn, a socketpair, exit on parent
  death); newest-only drain and the existing pump unchanged; enable and disable by explicit message,
  never a liveness lease; its own sequence counter; stats to the bridge at 10 Hz (loop-gap
  histogram, forward gap, drops, stamp-to-send latency, scheduling class); a 40 ms stall dump of its
  own. SCHED_FIFO 20 is tried and falls back; `gc.freeze()` after import.
- Bridge integration behind `stream_forwarder` (default false); stats on /link_status.
- Instruments, flag-independent: a watchdog able to dump at 30 ms; a stall line carrying its own
  event time; begin and end times on each solve line.
- Safe state: forwarder death means no frames, the firmware's cover STOP, then the 250 ms
  stale-setpoint E-stop (I-FW-1), unchanged.
- Gate: `./run_tests.sh --full` (bridge-node change). I-STREAM-3 lands here.

### Phase S2: Sitting S1 on FW 53

Forwarder off against on, at least 15 minutes of two-ball Attempt time per arm, watchdog at 30 ms,
`gc_freeze` on in both arms. Record: forwarder loop-gap histogram, l, forward-gap p99 and max,
drops, STOPs and latches per Attempt, emitter gap, every stall dump. Pass: no forwarder loop gap
over 25 ms, and no STOP attributed to a bridge-side stall in the on arm. It also names the stall
(or kills the hypothesis in § 1) and gives the emitter-side delay distribution that decides 4
against 5 committed Knots. The hardware Item with its `## Sitting` section is filed when PS1 lands,
so its commands are the real ones.

### Phase S3: Committed horizon in the planner

`FRAME_KNOTS` and `COMMITTED_KNOTS` in `schedule.py` and config, defaults 3 and 3 (bit-identical,
checked with `tools/probes/skill_constants_dump.py`). `WIRE_READ_BASE_KNOTS` follows
`COMMITTED_KNOTS`. Holds and graceful stops seed at the wire horizon (the P3 change, rebased). The
fresh-REST rule at C = 4 is decided and documented. Tests extend the published-Knot contract to
C = 4 for every install kind. I-WIRE-8 lands here. Opus: planner timing.

### Phase S4: FW 54 deep frames

One held branch. In order: the wire format and N-Knot playback, behaviour-identical at 3 and 3,
with frames deeper than 3 refused until the next unit; the promotion rule and the Leg guard
reference; twin parity, with the 10 Hz fault task and the encoder-freshness gate added to the twin;
the host pump and emitter building N Knots. The twin stall sweep prints the X table and is
I-STREAM-1's test. The Hand C1 grace (14 ms) has no written derivation: the stale comment is
corrected and the value is not relied on. Opus: firmware.

### Phase S5: FW 54 STOP event and the forwarder hold

Firmware: the edge detector in the net task, the event message and port, the stop's final position
computed at entry. Forwarder: events handled before any frame; no frame that fails the resume test
against the reported stop curve is sent; the stopped group is held at its final position until the
next arm edge; exhaustion is also predicted locally in case all three event datagrams are lost. The
bridge publishes the event at once and `skill_node` ends the Attempt and runs the existing lane
recovery. Opus: firmware and the hold rules.

### Phase S6: Bench B1

Bench Hand, production FW 54 image (never the bench-sysid image on the robot; the bench stroke is
shorter, so the top stop is found first). A bench-only hook in the forwarder injects delays of 30
to 240 ms at peak-up, slow-top and down phases, with and without a forced change to a tentative
Knot. Pass: no STOP up to X_g at any phase; no STOP up to X_c without the change; in every case no
MAX_DEVIATION latch, deviation under 2.5 rev, the hold within one frame of the event, recovery by
re-arm with no CLEAR_ERRORS. Record l and the event latency (target under 3 ms).

### Phase S7: Sitting S2 and close-out

FW 54 on the robot, N = 7, C = 4, forwarder and hold on. At least 150 two-ball Attempts. Pass: none
lost to a STOP, a refused lane or a Hand MAX_DEVIATION from delivery delay; no Guard latch after
any STOP; Catch rate not below baseline; emitter tick CPU under 2 ms. Then: merge the held branch,
flip the defaults, close BUG-89, BUG-58 and FEAT-40, decide FEAT-31 on the solve-time lines, one
`/audit`, `./run_tests.sh --full`, archive.

## 6. Open questions

1. **Leg guard reference (I-FW-21).** Today the Leg guard compares the promoted frame's first Knot
   with the encoder. Under a 150 ms frame the Legs legitimately move away from that Knot, so above
   about 10 rev/s the 1.0 rev guard would false-trip. The proposal moves the reference to the start
   Knot of the span being played. It stays "a Knot of the incoming plan against the encoder", and a
   runaway first Knot still trips. It amends I-FW-2 and needs the owner's sign-off before PS6.
2. **Suites during a Sitting.** The evidence does not show box load causing these stalls. A rule is
   still cheap: no session runs a suite while `live_session` reports a Sitting.
3. **Emitter-side delays.** If PS2 shows delays in `trajectory_node` above about 50 ms, the choices
   are 5 committed Knots or moving the emitter into the stream process. Not designed here.
4. **Accepted steps today.** FW 53 accepts a spliced successor inside the grace window with a small
   step. FW 54 keeps that and counts it; PS7 reports the count.

## 7. What this plan does not do

BUG-160 (Feed bias trim), BUG-78 (25 mm Catch authority) and BUG-161 (re-aim fix) are owned
elsewhere. The Hand and Leg guard thresholds, the grace values and SETPOINT_STALE do not change.
