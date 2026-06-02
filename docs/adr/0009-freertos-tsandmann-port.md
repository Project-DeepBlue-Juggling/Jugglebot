# ADR-0009: FreeRTOS as the leg-bridge firmware substrate (tsandmann fork)

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-28 (FreeRTOS), 2026-06-02 (tsandmann fork)
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0002](0002-dedicated-second-teensy.md), [parent plan](../../plans/active/teensy-can-offload.md), [firmware handoff](../../plans/active/HANDOFF-teensy-can-offload-firmware-wip.md) D10

## Context

The leg-bridge Teensy has several concurrent responsibilities:

- Hard real-time: 500 Hz Hermite interpolator ISR (deadline 2 ms).
- Time-sensitive: 100 Hz time-sync master broadcast on CAN1, 100 Hz
  telemetry uplink over UDP, 10 Hz heartbeats both directions.
- Event-driven: UDP RPC dispatch, CAN RX decode, fault state machine,
  encoder-search SDO polling.
- Diagnostic: 1 Hz profiling/diagnostics frames.

Three substrate options:

1. **Bare-metal super-loop** — same model as the existing platform Teensy
   4.0 and catching cone Teensy. Cooperative; uses hardware timers for
   periodic tasks.
2. **FreeRTOS** — preemptive task scheduling; classic embedded RTOS.
3. **Zephyr** — modern, feature-rich (DTS, drivers, OTA), but a large
   leap from the project's existing Arduino-on-Teensy expertise.

The existing platform Teensy and cone Teensy use bare-metal super-loops
successfully, but they have much simpler concurrency — a handful of
periodic things, no hard 500 Hz deadline competing with multi-kB UDP TX.

## Decision

Use **FreeRTOS** via the **tsandmann/freertos-teensy** library port.

- The 500 Hz interpolator runs as a **bare hardware-timer ISR above the
  FreeRTOS syscall ceiling** (see handoff doc D10). It cannot be preempted
  by any RTOS critical section — stronger than even a max-priority task.
- Everything else (UDP RX/TX, CAN decode, fault state machine, time-sync
  broadcast, heartbeats, telemetry, diagnostics) runs as FreeRTOS tasks
  with priorities ordered by latency sensitivity.
- Inter-task communication uses FreeRTOS queues + semaphores; ISR↔task
  handoff uses volatile flags with IRQ-guarded atomic 64-bit access for
  larger values (`atomic_read_u64` / `atomic_write_u64` in
  [`time_base.h`](../../ros_ws/src/jugglebot/Teensy_code_legbridge/time_base.h)).

### Why specifically tsandmann's fork

The originally-cited fork (greiman/FreeRTOS_TEENSY4) is **404 as of
2026-06-02** — the repo has been deleted or moved. The
**tsandmann/freertos-teensy** fork is actively maintained, provides the
standard FreeRTOS C API in the global namespace, and includes a Cortex-M7
port with PRIMASK intrinsics that we need for atomic 64-bit access.

The standard FreeRTOS C API (xTaskCreate, vTaskDelay, queues, semaphores)
is identical across forks — the firmware code is unaffected by the choice
of fork. Only the umbrella include (`<arduino_freertos.h>`) and a few
quirks differ; see
[`freertos_shim.h`](../../ros_ws/src/jugglebot/Teensy_code_legbridge/freertos_shim.h).

## Consequences

**Positive:**

- **Hard real-time guarantee for the 500 Hz tick is non-negotiable.** A
  bare timer-ISR above the syscall ceiling cannot be delayed by any
  scheduler or critical section. The whole point of moving off Linux.
- **Task isolation.** Each subsystem (UDP, CAN, fault, telemetry, etc.)
  has its own task, stack, and explicit priority. Bugs and timing issues
  in one don't silently leak into others.
- **Standard FreeRTOS API.** Well-documented, large ecosystem, lots of
  prior art to reference.

**Negative:**

- **C++ std::thread support drags in libstdc++ exception machinery**,
  which causes a real linker headache on the stock Teensy platform
  (R_ARM_PREL31 overflow in `pr-support.o`). Worked around by an
  `extra_script.py` that patches the linker script's `.ARM.exidx`
  pattern. Documented in the firmware
  [README](../../ros_ws/src/jugglebot/Teensy_code_legbridge/README.md)
  and `extra_script.py` header. This is *the* gotcha to know about when
  building from scratch.
- **Heap configuration is real.** `configTOTAL_HEAP_SIZE` and per-task
  stack sizes need to be sized correctly and validated on the bench.
- **Two failure modes to learn**: priority inversion, deadlock on locks.
  Mitigated by keeping inter-task communication patterns simple
  (single-writer queues, no nested locks).

## Alternatives considered

- **Bare-metal super-loop.** Considered. Rejected because the 500 Hz
  interp ISR competing with potentially-many-kB UDP TX from a
  cooperatively-scheduled main loop becomes hard to reason about. With
  FreeRTOS, priorities make the trade explicit.
- **Zephyr.** Considered for its richer driver stack and OTA support.
  Rejected as too big a leap from the project's current Arduino-on-Teensy
  expertise — revisit later if OTA or device-tree management becomes
  important.
- **greiman/FreeRTOS_TEENSY4 (originally cited in the plan).** Cannot
  use: the repo is 404. tsandmann's fork is the active replacement and
  is API-compatible.
