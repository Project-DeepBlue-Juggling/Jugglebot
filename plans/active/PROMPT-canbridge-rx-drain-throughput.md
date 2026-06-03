# Prompt: can-bridge CAN3 RX-drain throughput investigation

*(Hand this to a fresh Claude Code session on the Jetson. It is self-contained.
Surfaced as finding #10 of the adversarial review of the three-bus firmware
refactor — see `plans/active/HANDOFF-firmware-three-bus-WIP.md`.)*

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
3. `plans/active/HANDOFF-firmware-three-bus-WIP.md` (the refactor + the
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
- `plans/active/HANDOFF-firmware-three-bus-WIP.md` (finding #10 + the A1/A2 fixes).
