---
title: Catching cone hardware bringup
type: feature
date: 2026-05-23
status: in-progress
phase: catching-cone-bringup
related_plan: ""   # planning artefact lives outside the repo (~/.claude/plans/i-d-like-to-improve-ticklish-cat.md)
files_changed:
  # Config / codegen
  - config/protocol_config.yaml
  - config/hardware_config.yaml
  - config/generate_config.py
  - config/generated/protocol_config.h
  - config/generated/protocol_config.py
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - config/generated/geometry-config.js
  - ros_ws/src/jugglebot/Teensy_code/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/gui/js/geometry-config.js
  # Host decoder + tests
  - ros_ws/src/jugglebot/jugglebot/can/catching_cone.py
  - tests/ros/test_catching_cone.py
  # ROS2 messages
  - ros_ws/src/jugglebot_interfaces/msg/CatchEvent.msg
  - ros_ws/src/jugglebot_interfaces/msg/CatchingConeHeartbeat.msg
  - ros_ws/src/jugglebot_interfaces/msg/CatchTimingResult.msg
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  # can_node integration
  - ros_ws/src/jugglebot/jugglebot/can_node.py
  - tests/ros/conftest.py
  - tests/ros/test_can_node.py
  # Correlation node
  - ros_ws/src/jugglebot/jugglebot/catch_correlation_node.py
  - ros_ws/src/jugglebot/setup.py
  - tests/ros/test_catch_correlation.py
  # GUI panel
  - ros_ws/gui/index.html
  - ros_ws/gui/css/panels.css
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/js/main.js
  # Firmware
  - ros_ws/src/jugglebot/CatchingCone_code/CatchingCone_code.ino
  - ros_ws/src/jugglebot/CatchingCone_code/platformio.ini
  - ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  # Logbook
  - logbook/2026-05-23-catching-cone-hardware-bringup.md
  - logbook/INDEX.md
commits: []
subsystem:
  - can
  - ros
  - gui
  - config
tags:
  - feature
  - hardware
  - firmware
  - testing
  - timing
---

# Catching cone hardware bringup

## Summary

Added an instrumented catching cone — a new Teensy 4.x microcontroller with a
piezo contact sensor on the cup — that detects ball-landing impacts and
broadcasts precisely-timed `CATCH_EVENT` CAN frames synced to the Jugglebot
global clock. The host correlates each catch against the nearest throw
announcement and publishes the timing error on `cone/timing_result`; a new
"Catching Cone" GUI panel displays predicted/actual times, the headline
timing delta, and a "sound bar" of recent offsets on a fixed ±50 ms axis.
The cone reuses the platform-Teensy's `TimeSync` layer verbatim and lives on
the same 1 Mbps CAN bus as the existing devices. Software, firmware, and
hardware (Zener clamp + 57 kΩ load on soldered veroboard, CAD-printed cone
mount) are complete; formal dual-trigger timing-precision validation and
Jetson-side live integration are still pending.

## Motivation

The existing catching cone measured only **spatial** throw accuracy (did the
ball land in the cup, and where). It said nothing about **temporal**
accuracy — whether the ball arrived when the catch trajectory said it
would. As Phase 7 ball-catching work moves toward closed-loop predictive
catching, the timing axis of throw quality is now load-bearing: a 30 ms
arrival error at typical ball speeds (~3 m/s) is ~90 mm of position error
that no spatial-only sensor will surface. Building the instrumentation as a
peer CAN device — rather than a host-side video pipeline — keeps the
timing-precision budget in the µs range (CAN bus, TimeSync) instead of the
multi-ms range (video frame interval, USB jitter, OS scheduling).

Target precision: tens of µs (formal validation deferred). Even at 1 ms
precision the device is useful; the µs goal is what justifies the
dedicated-MCU + CAN architecture over a simpler USB-tethered sensor.

## Design

**Topology.** New Teensy 4.x on the existing 1 Mbps CAN bus (node added to
the bus map). Piezo wired into a single analog/digital GPIO; impact detected
in an ISR on rising edge; CAN frame broadcast from the main loop after a
short report delay (see Firmware below). The cone Teensy is a peer device,
not a slave — it timestamps and broadcasts on its own clock, synced to the
global clock by the same `TimeSync` slave logic used by the platform Teensy
(no change to that code; same `.h`/`.cpp` files reused verbatim).

**CAN protocol.**
- `CATCH_EVENT` (0x7E0): 8-byte little-endian frame (`struct '<IBBH'`)
  carrying the **low 32 bits of wall-time µs** (wraps every ~71.5 min), a
  monotonic 8-bit `sequence` counter (for host-side drop detection), and an
  8-bit `flags` byte (bit 0 `time_synced`, bit 1 `retrigger_suppressed`).
  Two bytes reserved.
- `CONE_HEARTBEAT` (0x7E1): 10 Hz status frame (cone state enum, last-event
  count, time-sync health).
- Decision: **microsecond timestamps, not milliseconds.** A 1 ms field
  would quantize away the entire measurement budget. The host reconstructs
  the high 32 bits via the same wrap-trick the firmware uses for
  `reconstructWallMs()`, mirrored exactly in
  `catching_cone.reconstruct_catch_time_us`.

**Single namespace, two headers.** `CatchingCone` namespace is reopened in
both `protocol_config.h` (for CAN IDs + state enums) and `hardware_config.h`
(for piezo pin, threshold windows, GUI thresholds). Legal in C++ (namespaces
are open); avoids forcing a contrived rename of one or the other; keeps
"everything cone-related" under one symbol on the embedded side.

**Single owner of the timing delta.** Correlation lives in a dedicated
`catch_correlation_node`, *not* in `can_node`. `can_node` stays pure CAN
transport — it publishes raw `CatchEvent` and `CatchingConeHeartbeat`
messages, and the correlation node consumes those plus throw-arrival
announcements to publish `CatchTimingResult`. The GUI only displays — it
doesn't re-compute. One place to change if the matching policy ever moves
from "nearest throw" to something stricter.

**GUI: "sound bar", not a sparkline.** Recent timing offsets render as
vertical bars on a fixed `−50 … 0 … +50 ms` axis (newest at full opacity,
older fading) rather than a time-series sparkline (x = time). Reads like a
tuning meter: bias and spread visible at a glance, which is what you
actually want when tuning throw timing. The first draft used a sparkline;
user pushed back and the sound bar is genuinely better for the data.

**Predicted / Actual cells display wall-clock times.** The mockup used
placeholder durations, but the real `CatchTimingResult` carries absolute
`builtin_interfaces/Time` stamps, so the cells became `HH:MM:SS.mmm` clock
times. Flagged this to the user during integration as a deliberate, faithful
realization rather than an unauthorized change.

## Implementation

The work is structured to land as seven feature commits, in this order:

1. **Config / codegen.** New `catching_cone` section in
   `protocol_config.yaml` (`CATCH_EVENT` 0x7E0, `CONE_HEARTBEAT` 0x7E1,
   state enum) and `hardware_config.yaml` (piezo pin, refractory +
   report-delay windows, heartbeat period, GUI thresholds).
   `generate_config.py` learned to emit the cone C++ namespaces + Python
   constants, gained a new firmware-dir copy destination
   (`CatchingCone_code/`), and exports cone GUI constants to
   `geometry-config.js`. All consumer dirs receive the regenerated artefacts.

2. **Host decoder + tests.** `ros_ws/src/jugglebot/jugglebot/can/catching_cone.py`
   defines `CatchEvent` and `ConeHeartbeat` dataclasses and the wrap-safe
   32→64-bit µs timestamp reconstructor. 28 unit tests in
   `tests/ros/test_catching_cone.py` cover field decoding, wrap reconstruction
   at the boundary, retrigger-flag plumbing, and bad-frame rejection.

3. **ROS2 messages.** `CatchEvent.msg`, `CatchingConeHeartbeat.msg`,
   `CatchTimingResult.msg` added under `jugglebot_interfaces/msg/` and
   wired into `CMakeLists.txt`.

4. **can_node integration.** Dispatch branches for the two new arbitration
   IDs, publishers for both raw topics, and a 10 Hz republish of the cone
   heartbeat. `tests/ros/conftest.py` gained mock msg types and a
   `MockTime.nanoseconds` accessor; 4 new dispatch/publish tests in
   `test_can_node.py`.

5. **Correlation node.** `catch_correlation_node.py` matches each catch
   event against the most recent throw announcement (0.5 s match window —
   currently a module-level constant `_MATCH_WINDOW_S`; not yet exposed as
   a ROS parameter) and publishes `CatchTimingResult` on
   `cone/timing_result`. Entry point added to `setup.py`. 7 tests in
   `test_catch_correlation.py` cover matching, no-throw fallback, and
   timing-window edges.

6. **GUI panel.** New `panel-catching-cone` container in `index.html`,
   `.cc-*` styles in `panels.css`, and `initCatchingConePanel` /
   `updateConeHeartbeat` / `updateConeTimingResult` in `panels.js` driving
   the sound-bar canvas. `main.js` subscribes to the new topics and routes
   to the handlers.

7. **Firmware.** `CatchingCone_code.ino` — Teensy 4.x sketch with
   `TimeSync` reused verbatim, piezo ISR on rising edge, split refractory /
   report-delay timing (see Discussion), and CAN broadcast in the main
   loop. `platformio.ini` is the first PlatformIO config in this repo
   (existing firmware uses Arduino IDE); the `.ino` builds under both, so
   the `platformio.ini` is purely a convenience for `pio run -t upload`.

## Discussion

This is the load-bearing section. The final design — soldered veroboard
with a single GND-referenced Zener, a 57 kΩ load resistor, a 30 ms report
delay split from a 500 ms refractory — is not what the original plan
specified. Each deviation came from a specific bench symptom that revealed
an assumption that didn't survive contact with the hardware. The
catching-cone topology is what it is because of this arc, not in spite of
it.

### Bench-up on breadboard surfaced cascading issues

The plan called for the conventional rail-referenced clamp: two 1N4148s
(one anode→node, cathode→3V3; one anode→GND, cathode→node), R_load 1 MΩ
across the piezo, R_series 1 kΩ. Textbook protection.

First-build symptoms on breadboard:

- **Piezo alone (disconnected) generated reliable ~35 V spikes; piezo
  in-circuit showed flat 0 V across its own terminals.** Wiring confirmed
  correct, piezo confirmed good in isolation. The signal was just *gone*
  once the protection circuit was attached.
- **Teensy 3V3 pin read 3.45 V when scope-probed and 3.35 V un-probed.**
  A probe physically *raising* a regulated rail is impossible from passive
  loading. The probe was perturbing a parasitic-laden node and the clamp
  diode was rectifying noise into the rail. (3.35 V is itself fine —
  within spec; the artefact was the probe-induced *change*.)

The diagnosis was that a node which is simultaneously high-impedance
(≥1 MΩ), high-voltage (35 V), and fast-edged is the textbook worst case
for a breadboard: long leads + uncertain contacts + measurable insulation
leakage + parasitic capacitance.

**Fix:** moved to soldered veroboard. Both symptoms cleared. No further
breadboard work on this circuit.

### Hum pickup — the second iteration

Once the circuit was soldered, the piezo registered impacts cleanly. New
symptom: pressing and holding the piezo produced (a) a perceptible *buzz*
the user could feel under the finger and (b) rapid spurious `CATCH` events
from a static hold.

The user correctly noted: a piezo is a dF/dt sensor; a continuous press
should generate nothing. So the events were interference, not sensing.

A piezo is a bidirectional transducer. Feeling a buzz meant AC was being
driven *onto* the piezo. The user's finger, coupled to mains wiring through
pF-scale body capacitance, was injecting **50/60 Hz mains hum** into the
high-impedance node — driving the piezo (audible/tactile buzz) and crossing
the GPIO threshold once per mains cycle (spurious "catches"). The finger
was both the antenna and the thing feeling the result.

**Fix:** lowered `R_load` from 1 MΩ to **57 kΩ**. With ~35 V of piezo
signal margin against a ~1.6 V threshold, the loss in peak signal amplitude
is irrelevant; the order-of-magnitude reduction in source impedance shunted
the hum below the threshold. Buzz gone, spurious events gone.

**Class-of-failures lesson:** don't default high-impedance inputs to 1 MΩ
when external coupling is possible. Mains hum on high-Z nodes is universal,
not a special edge case — the bench was not unusually noisy.

### The dead-rail discovery — the decisive iteration

With all power off and the Teensy disconnected, contacting the piezo still
produced spikes >10 V at the GPIO node. The user (sharply) asked: this
seems unsafe — surely the clamp should clamp regardless of power state?

Diagnosis: a diode-to-rail clamp only works when the rail is a *live,
low-impedance node that can absorb the dumped current*. With the rail dead
or floating, `D_high` had nowhere to dump charge — it just dragged the
floating rail node up with the signal. `D_low` to GND still worked (GND is
always GND), but the high side was effectively unclamped.

The hazard surface this exposed: a Teensy connected but powered off → a
piezo hit drives an unclamped 10 V+ spike straight into the GPIO's
internal ESD diodes. The disconnected-Teensy bench test harmed nothing,
but it correctly surfaced the failure mode for the powered-off case.

**Fix:** replaced the two-diode rail clamp with a **single GND-referenced
Zener (1N4728, 3.3 V)** — cathode→node, anode→GND. One part clamps both
polarities (reverse breakdown limits positives, forward conduction limits
negatives), works whether the Teensy is on / off / absent, and **never
dumps charge into the 3V3 rail** (so it also eliminated the earlier
rail-pumping artefact). At the operating currents involved (well below
the part's 76 mA test current) the clamp voltage actually sits at or
slightly below 3.3 V — better GPIO protection than the rail-diode topology's
~3.9 V clamp, which was already slightly *over* the Teensy's 3.6 V abs-max.

Verified: peak voltage at the pin stays ≤ 3.3 V even with the Teensy
unpowered.

**Class-of-failures lesson — and the most important takeaway from this
arc:** rail-referenced clamps protect only when the rail is live. **For
inputs that may see active external signals when the host is unpowered,
GND-referenced clamps (Zener / TVS) should be the default.** Always
bench-test clamps with the host powered off. This deserves to be in a
"hardware design conventions" doc the day there's a second instrumented
peripheral to write one for.

### Firmware timing — double-counts on hard hits

Once the circuit was clean, the firmware reported two `CATCH` events for
hard impacts. The user noted real catches are ≥ 1 s apart (faster than that
isn't physically reachable) and asked for the refractory to be extended
from its initial bench value.

**First attempt:** during bench iteration `DEAD_TIME_US` was bumped from
the initial 10 ms (an unversioned local value) up to 500 ms. This worked
for double-count suppression — but because the firmware was deferring the
`CATCH_EVENT` send until the refractory closed (so `retrigger_suppressed`
could capture the full ring/bounce tail), it also pushed the CAN frame's
*send latency* to 500 ms. The timestamp was still precise, but the GUI was
laggy and tail-event correlation against external observations became
unnecessarily hard.

**Clean split:** introduced a separate `REPORT_DELAY_US` (30 ms) that
controls send latency and the `retrigger` flag window, and kept
`DEAD_TIME_US` (500 ms) as the ISR refractory. Best of both — frame on the
bus within ~30 ms of impact, double-count suppression out to 500 ms. The
`retrigger` flag now reflects ring inside 30 ms (captures piezo
electrical/mechanical ring); longer-tail ball-bounce is suppressed by the
refractory but isn't flagged. Acceptable trade — bounces are physically
distinguishable as separate impacts only at >> 30 ms cadence, by which
point they're real re-strikes and the refractory is the right tool.

## Verification

- **pytest:** 1102 pass, 4 skipped. (The 4 skipped are pre-existing
  `tests/sim/test_make_feasible_events.py` cases that need the `hypothesis`
  dep, which isn't installed in this environment — not caused by this
  feature.) 42 of the passing tests are new with this feature: 31 decoder
  + 7 correlation + 4 can_node dispatch/publish.
- **Codegen:** `python config/generate_config.py` runs clean; new constants
  present in all consumer files.
- **GUI JS:** `node --check` passes on `panels.js` and `main.js`.
- **Live bench:** firmware flashed via PlatformIO; `candump can0` confirmed
  `CATCH_EVENT` and `CONE_HEARTBEAT` frames on the bus when the cone was
  plugged into the Jugglebot CAN.
- **Hardware:** peak voltage at the GPIO pin verified ≤ 3.3 V even with the
  Teensy unpowered (the dead-rail test that drove the Zener choice).
- **Hardware build complete:** Zener clamp + 57 kΩ R_load + transceiver
  wired on soldered veroboard; CAD-printed cone mount installed.

## Outcome

Software, firmware, and hardware are complete and bench-validated to the
point that the cone broadcasts well-formed CAN frames on the live bus.
Status is `in-progress` because two pieces of verification remain before
this can move to `resolved`:

1. **Formal dual-trigger timing-precision validation** — comparing the cone
   Teensy's timestamp against the platform Teensy's timestamp on a shared
   known electrical edge, to confirm the ≤ ~30 µs std-dev claim the
   architecture is built around. Software and hardware are ready; the test
   is not yet run.
2. **Jetson-side live integration** — cone on the bus, `can_node` decoding,
   `catch_correlation_node` matching, GUI panel showing live data with a
   real throw → catch sequence. User is about to start this.

If either of those surfaces an issue, the entry stays open and the new arc
gets appended to Discussion. If both pass clean, the entry moves to
`resolved`.

The hardware design conventions established in the Discussion — 57 kΩ
default load on high-impedance inputs, GND-referenced clamps on
externally-driven inputs, bench-test clamps with the host powered off —
are worth promoting to a `docs/hardware/` design-conventions doc the next
time an instrumented peripheral is added. Filing as an Open Question rather
than a contract since one data point isn't enough to justify the doc on its
own.

## Open Questions

- **Formal timing-precision validation.** What's the actual std-dev of the
  cone-vs-platform timestamp delta on a shared electrical edge? The
  architecture targets tens of µs; until measured, that's a claim not a
  fact.
- **Live Jetson integration.** Does the full pipeline (cone → CAN →
  `can_node` → `catch_correlation_node` → GUI) work end-to-end with a real
  throw?
- **Richer per-event data.** The `CATCH_EVENT` frame's two reserved bytes
  are available if a future use wants more than just time + sequence + flags
  (e.g. impact peak amplitude for false-positive rejection or catch-energy
  estimation). That would add an analog-sampling step in the ISR — feasible
  but not yet specified.
- **Hardware design conventions doc.** Worth writing once there's a second
  instrumented peripheral to draw from.
