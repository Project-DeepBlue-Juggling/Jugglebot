---
title: Prompt: can-bridge CAN3 RX-drain throughput investigation
archived: 2026-07-05
---
# Prompt: can-bridge CAN3 RX-drain throughput investigation

*(Hand this to a fresh Claude Code session on the Jetson. It is self-contained.
Surfaced as finding #10 of the adversarial review of the three-bus firmware
refactor — see `plans/archived/HANDOFF-firmware-three-bus-WIP.md`.)*

---

You are investigating a **firmware throughput** concern in the Jugglebot
can-bridge Teensy 4.1 firmware (`ros_ws/src/jugglebot/Teensy_code_canbridge/`).
This is a real-time robotics control codebase — incorrect handling of motor
telemetry can let the fault watchdog miss a real fault, so reason about safety
implications, not just code.

Read these first, in order:
1. `CLAUDE.md` (project conventions — especially the Workflow Rules and
   Engineering Philosophy).
2. `docs/adr/0013-three-can-buses.md` (the three-bus topology; CAN3 = Jugglebot
   core: 6 leg ODrives + Hand ODrive + platform Teensy + can-bridge, ~5,340
   frames/s steady, ~5,840 with a throw).
3. `plans/archived/HANDOFF-firmware-three-bus-WIP.md` (the refactor + the
   adversarial review; this issue is finding #10, and findings #1/#4/#5/#7 —
   the FlexCAN TX serialisation and 64-bit timestamp atomics — are the relevant
   concurrency context you must not regress).

## The problem

The per-axis motor-state cache (`axis_state.h` `axes[]`) is populated by
decoding incoming ODrive CAN frames. The decode path is:

- The FlexCAN hardware RX interrupt pushes each received frame into the
  peripheral's `rxBuffer` (a `RX_SIZE_256` circular buffer).
- `can_buses_service()` (`can_buses.cpp`) calls `can_jugglebot.events()` once
  per invocation; `task_can_rx` (`Teensy_code_canbridge.ino`) calls
  `can_buses_service()` on a **1 ms** `vTaskDelayUntil` tick (~1 kHz).
- **`FlexCAN_T4::events()` pops exactly ONE frame** from `rxBuffer` per call
  (verified: `FlexCAN_T4.tpp` `events()` — a single `if (rxBuffer.size()) {
  rxBuffer.pop_front(...); mbCallbacks(...); }`, NOT a loop). It returns
  `(rxBuffer.size() << 12) | txBuffer.size()`, so the remaining RX depth is
  readable from the high bits of the return value.

So the **decode drain rate is ~1,000 frames/s**, but **CAN3 carries ~5,340
frames/s**. The `rxBuffer` (256 deep) fills within ~50 ms of bring-up; from then
on the circular buffer overwrites its oldest entry on overflow, and `events()`
pops oldest-first — so the cache is fed stale frames at ~1/5 of the true rate,
lagging real time by roughly the buffer depth.

**Scope honesty:** this is **pre-existing** — the two-bus base (`07a895b`) had the
same one-frame-per-`events()` drain, and the leg ODrives were already ~5,000
frames/s on a single bus (CAN2). The three-bus refactor did **not** introduce or
worsen it; it merely makes CAN3 the one bus where it matters (all ODrive
telemetry is now there). Treat it as its own issue, not a refactor regression.

## Why it matters (quantify this FIRST — do not fix before you have)

The cache feeds two safety-relevant consumers: the fault state machine's
heartbeat watchdog (`fault_machine.cpp`, leg-bus-down detection + deferred stow)
and the per-axis staleness/telemetry path. Before proposing any fix, **quantify
the real impact** — the issue may be benign or may be serious, and the answer
determines the fix:

- Decompose the ~5,340 frames/s into per-(axis, command-id) streams (heartbeat,
  encoder estimate, error, iq, temps, bus-voltage — see `decode_into_cache`).
  Roughly how many independent streams, and what is each stream's native rate?
- At a ~1,000 frames/s drain (and FIFO-with-overwrite semantics that pop oldest),
  what is the *effective* per-stream update rate and the *latency* the cache
  sees? Compare against the ported thresholds in `canbridge_config.h`:
  `MOTOR_FB_STALENESS_US` (0.15 s), `CAN_HEARTBEAT_TIMEOUT_US` (2.0 s),
  and the interp lead-clamp's dependence on `axes[i].pos_rev` freshness.
- Worst case: can a one-shot critical frame (an ODrive `get_error`/disarm, or a
  heartbeat gap that should trip the watchdog) be dropped or delayed past a
  threshold? That is the safety crux.

There is **no bench** in this environment, so this is a quantitative/analytical
argument (per-stream-rate math + buffer dynamics), not a measurement. If you can
build a small host-side simulation of the circular-buffer fill/drain dynamics to
make the argument concrete, do (`/tmp` probe is fine per the project's
empirical-probe rule). Document the conclusion either way.

## Candidate approaches (and their trade-offs)

1. **Bounded drain loop.** Loop `events()` until `rxBuffer` is empty or a
   per-tick budget is hit, e.g. `for (int n = 0; n < BUDGET && (can_jugglebot.events() >> 12); ++n) {}`.
   Cheapest, smallest change. **Must** bound the loop so a saturated bus can't
   starve `task_can_rx`'s `vTaskDelayUntil` cadence or the lower-priority tasks.
   Pick the budget from the quantified rate (≈6 frames/ms covers steady CAN3 with
   margin). Keep the decode in task context (no concurrency-model change).
2. **Higher `task_can_rx` tick rate.** Marginal — even 1 kHz only buys ~1,000
   fps; you'd need an impractically fast tick to cover 5,340 fps one-frame-per-tick.
   Inferior to (1); mention and dismiss with the arithmetic.
3. **Decode in the FlexCAN onReceive ISR** (don't use `events()` for RX;
   FlexCAN_T4 can dispatch `onReceive` from the actual RX interrupt). Removes the
   drain-rate ceiling entirely. **But** it moves `decode_into_cache` (the seqlock
   `write_pos_vel`, the heartbeat/error cache writes, the `atomic_write_u64`
   timestamp) into ISR context, which **interacts directly with the just-landed
   review fixes**: the A1 TX path now masks IRQs (`__disable_irq`) around the
   FlexCAN write, and the 500 Hz interp ISR reads `axes[i].pos_rev` for its lead
   clamp. Re-derive the full ISR-vs-ISR / ISR-vs-task hazard set before choosing
   this — it is the most robust throughput fix and the most invasive.

Lead with the root-cause justification for whichever you choose (the concrete
failure mode it prevents), per CLAUDE.md — not "the review said so".

## Constraints & build

- **Firmware only.** PlatformIO is the gate. Recipe (5-min Bash timeout):
  ```bash
  source /home/jetson/Desktop/PDJ_venv/venv/bin/activate
  cd /home/jetson/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_canbridge
  pio run
  ```
  `pio` lives only in that venv (`/home/jetson/Desktop/PDJ_venv/venv/bin/pio`,
  PlatformIO 6.1.19). `pytest` is not in your loop unless you touch config codegen.
- **Branch:** base off `teensy-can-offload-firmware-three-bus-wip` (so you have
  the A1/A2 concurrency fixes, which option 3 must not regress) into a new WIP
  branch. Do **not** `git push`, `--amend`, `rebase`, or rewrite history.
  Build-green at every commit; one concept per commit; cite the build in the
  commit body.
- **Do not regress** the A1 FlexCAN TX serialisation or the A2 64-bit timestamp
  atomics; do not touch the safety-critical interp ISR timing
  (`leg_interp_init` `s_timer.priority(32)`, the 500 Hz cadence) without
  re-deriving the implications.
- Also fix the stale `can_buses.h` header comment that claims the RX path has
  "no queue hop" — `events()` does use the library's `rxBuffer` queue (the
  comment predates this finding).

## Deliverables

1. A written **impact analysis** (the quantification above) — is this a real
   safety problem, a quality issue, or benign? This gates everything else.
2. A recommended approach with root-cause justification and the rejected
   alternatives.
3. If the diagnosis is clear and the fix well-scoped: the implementation, in
   build-green commits, with the bench-validation items spelled out (e.g.
   confirm no RX overflow and bounded `task_can_rx` cadence under real
   5,340 fps load — that needs the bench).
4. A short logbook or HANDOFF note capturing the analysis and decision (this is
   pre-bench firmware; do not invent a logbook entry for unvalidated hardware
   behaviour unless the project's `/log` conventions fit — a plan/HANDOFF note
   is the safe default).

## References

- `ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp` —
  `can_buses_service()`, `on_jugglebot_rx()`, `decode_into_cache()`,
  `can_jugglebot_send()` (A1 IRQ-off guard).
- `ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino` —
  `task_can_rx()` (1 ms tick).
- `~/.platformio/packages/framework-arduinoteensy/libraries/FlexCAN_T4/FlexCAN_T4.tpp` —
  `events()` (one `rxBuffer.pop_front` per call; return `rxBuffer.size()<<12 | txBuffer.size()`),
  and the `onReceive` / `mbCallbacks` ISR path for option 3.
- `docs/adr/0013-three-can-buses.md` (the ~5,340 / ~5,840 frames/s figures).
- `plans/archived/HANDOFF-firmware-three-bus-WIP.md` (finding #10 + the A1/A2 fixes).

---

## Resolution (2026-06-04, commit e2b4cfb)

**Confirmed**, with one correction to the framing: the drain *ceiling* is real,
but the *received* load is lower than the 5,340 fps the review assumed.

### Ground truth (verified against the pinned FlexCAN_T4)

- `events()` (`FlexCAN_T4.tpp:1074`) pops exactly ONE `rxBuffer` frame per call —
  a single `if (rxBuffer.size()) { pop_front; mbCallbacks; }`, not a loop.
  `task_can_rx` calls it once per 1 ms tick (`configTICK_RATE_HZ = 1000`,
  preemptive), so RX decode is capped at **~1,000 fps/bus**.
- The RX path **does** use a library queue: the FlexCAN FIFO interrupt
  (`flexcan_interrupt`, enabled by `enableFIFOInterrupt()`) pushes each frame via
  `struct2queueRx` into the `RX_SIZE_256` `rxBuffer`; `events()` drains it. Once
  `events()` has run once, `isEventsUsed=1` routes all RX through that buffer (so
  decode is **not** in the ISR today). On overflow `circular_buffer.h` write()
  advances `head` → **overwrite-oldest**, and `events()` pops oldest-first.
- Self-reception is disabled (`MCR_SRX_DIS`, set unconditionally in `begin()`),
  so the can-bridge does **not** receive its own ~3,000 fps setpoint TX or
  ~100 fps time-sync TX. **Received CAN3 load = 5,340 − 3,100 ≈ 2,240 fps steady;
  with-throw 5,840 − 3,100 ≈ 2,740 fps** (the platform Teensy's +500 fps hand
  trajectory IS received).

### Quantified impact (host simulation of the ring fill/drain, `/tmp` probe)

At 2,240 fps in / 1,000 fps out of a 256-deep overwrite-oldest ring:

- **~55% of telemetry dropped** before decode (63% with a throw).
- **Survivors reach the cache ~112 ms stale** (mean; ≈ buffer depth / arrival
  rate = 256 / 2,240). Per-stream effective rate falls from 100 Hz native to
  ~37–48 Hz (with realistic ±5% ODrive-timer jitter; deterministic timing
  aliases far worse).

Consumer-by-consumer:

- **Heartbeat watchdog + deferred stow** (`fault_machine.cpp`,
  `CAN_HEARTBEAT_TIMEOUT_US = 2.0 s`): **robust.** Even decimated heartbeats
  arrive ≫ once / 2 s, and a real heartbeat silence still stops the timestamp
  advancing → still trips. The most safety-critical path is unaffected.
- **500 Hz lead-clamp** (`leg_interp.cpp:246`, reads `pos_rev`) and the 10 Hz
  **max-deviation / overspeed E-stops** (`fault_machine.cpp:198,212`): **degraded
  but not broken** — they operate on ~112 ms-stale, decimated feedback. At leg
  speed, 112 ms can exceed the 0.15 rev lead band, so the backstop references
  badly-old position. This is the real (non-acute) harm.
- `MOTOR_FB_STALENESS_US` (0.15 s) is **defined but not yet wired** to any guard,
  so 112 ms staleness auto-trips nothing today.
- Telemetry uplink to the Jetson (`telemetry.cpp`): stale + decimated — a quality
  issue.

**Verdict:** real freshness/quality degradation with a concrete (non-acute)
safety-backstop cost; not an acute fault. Worth fixing; fix is low-risk.

### Decision — option 1 (bounded drain), root-cause justified

Drain each bus to empty per tick, capped at `CAN_RX_DRAIN_BUDGET = 32`
(`can_buses.cpp`). 32,000 fps drain capacity is ~4× the ~7,700 fps physical
1 Mbps classical-CAN frame ceiling, so the ring cannot grow even on a saturated
bus; the sim shows 0% drop / ~0.5 ms staleness. **It prevents the specific
failure mode that the decode falls arbitrarily far behind real time whenever
arrival exceeds 1,000 fps** — the cap restores the invariant *drain ≥ arrival*.

Keeps the 500 Hz interp ISR un-starved **by construction**: decode stays in the
priority-5 `task_can_rx`, below the leg-setpoint IntervalTimer ISR (NVIC 32,
above the FreeRTOS syscall ceiling) which preempts the task. A larger per-tick
drain can only shorten the slice left to lower-priority tasks, and only during a
backlog (the loop breaks the instant the ring empties). It also *reduces* the
rxBuffer/seqlock race surface (a near-empty ring never hits the overwrite-oldest
`head` advance that races task-side pops).

**Rejected alternatives:**

- **Option 2 (faster tick).** Still one pop per call; even a 2 kHz tick buys only
  ~2,000 fps vs ~2,240 needed — and burns CPU on empty polls. Arithmetic kills it.
- **Option 3 (decode in the FlexCAN onReceive ISR, `isEventsUsed=false`).** Highest
  throughput but the most invasive and **needs bench measurement to choose safely**
  (per the prompt's escape hatch). Concrete blockers, not authority: (a) it forces
  *all* RX into ISR context, where `decode_into_cache` (seqlock `write_pos_vel`,
  `atomic_write_u64`) runs at up to ~2,740 frames/s — worst-case ISR-vs-ISR latency
  against the interp tick's 500 µs jitter budget is unmeasured here (the handoff
  already flags the interp-priority-vs-syscall-ceiling as bench-pending); (b) to get
  RX-in-ISR you must never call `events()` on that instance, so CAN3 TX-queue
  draining would rely solely on the ISR TX-complete path — a behaviour change to the
  3,000 fps setpoint TX with its own untested risk; (c) the seqlock itself is fine
  (writer-in-higher-priority is exactly what it supports; the interp ISR's `pos_rev`
  read is single-word atomic regardless), so seqlock correctness is *not* the
  blocker — (a) and (b) are. Revisit only if option 1 proves insufficient on the
  bench.

### Docs corrected

- `can_buses.h` header comment: the "no queue hop" claim was wrong (events()
  dispatch routes through the library `rxBuffer`); corrected, with the D7 nuance
  noted (D7's "not by enqueuing" meant *no application-level* queue, which holds).

### Observability (landed 2026-06-04, commit 18e22fd)

Per-bus RX-health counters now make the bench validation a direct read rather than
an inference. Collected in `can_buses_service()` (1 kHz) and printed by `task_diag`
over USB Serial as `[canhealth]` lines (cumulative/sticky since boot):

- `depth_hwm` — peak `rxBuffer` occupancy at a service tick (via `getRXQueueCount()`
  before draining). **This is the zero-drop witness**: healthy ⇒ single digits;
  climbing toward 256 ⇒ `task_can_rx` starvation.
- `cap_hits` — ticks the `CAN_RX_DRAIN_BUDGET` bound with frames still queued (the
  overflow precursor; must read 0).
- `err_events` / `err_flags` / `rec_max` / `fault_conf` — FlexCAN ESR1 history
  (wire-error type, peak REC, worst fault-confinement: active/passive/bus-off).
- `decode_short` / `decode_bad_axis` — CAN3 frames that arrived but were not cached
  (truncated DLC, or node id ≥ NUM_AXES).

Surfaced over Serial, not the UDP uplink: the wire format + its Jetson consumer are
owned by a parallel session mid-edit. Promoting these to a telemetry frame is a
follow-up once that lands.

### Bench-validation TODO (cannot be done without hardware)

- Under real ~5,340 fps CAN3 load, confirm `[canhealth] jugglebot` reads
  `depth_hwm` ≈ single-digit and `cap_hits` = 0 (⇒ zero `rxBuffer` overflow and the
  bounded drain keeping up), with `task_can_rx` holding its `vTaskDelayUntil`
  cadence.
- Confirm `interp_max_jitter_us` / `interp_deadline_misses` stay within budget
  with the bounded drain + error polling active under load.
