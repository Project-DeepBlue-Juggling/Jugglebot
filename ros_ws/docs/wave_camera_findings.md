# Wave Camera: measured findings (running log)

> **Wave Camera sub-project.** Jugglebot starts and stops recording on a
> Freefly Wave high-speed camera from ROS 2 through an ESP32 on the cone-role
> CAN bus (CAN3), and stamps each clip start on the robot wall clock. The ESP32
> firmware lives in [`ros_ws/src/jugglebot/WaveCamera_ESP_code/`](../src/jugglebot/WaveCamera_ESP_code/README.md)
> (a standalone PlatformIO project, built and flashed from a PC, never from the
> Jetson). Decision: [ADR-0014](../../docs/adr/0014-wave-camera-on-can3.md); plan:
> [wave-camera-can3-integration.md](../../plans/active/wave-camera-can3-integration.md). Sibling docs:
> [GPIO explorer design](wave_camera_gpio_explorer.md),
> [CAN adapter design](wave_camera_can_adapter.md),
> [measured findings](wave_camera_findings.md).
> Moved here from the former standalone adapter repo on 2026-10-25 with content
> unchanged except paths: log paths `bench_logs/NN_*.log` are relative to the
> firmware folder (formerly `logs/` in the adapter repo). Short summary:
> [logbook entry](../../logbook/2026-10-25-wave-camera-gpio-findings.md).


Camera: Freefly Wave, firmware v1.4.1. Host: ESP32-WROOM-32D DevKitC.
All timestamps are ESP32 `esp_timer` microseconds since boot unless stated.
Each entry records the date, the firmware commit, and the camera settings.

## Prior knowledge (logic analyser, before this project)

- One pin outputs 115200 8-N-1 ASCII. At every record start (button or GPI2)
  it sends `Created new clip.\r\n`. Nothing on record stop.
- One pin outputs a continuous 30 Hz pulse train whenever the camera is on,
  independent of recording and frame rate. Possibly HDMI timing.
- One pin stayed high, one stayed low. Pin-to-channel mapping uncertain.

## Documentation cross-check (2026-10-19)

- Wave connector page names only pin 1 (GND), pin 2 (VIN) and pin 6
  (GPI2/RS) in text. Pin 3-5 names come from the owner's notes, not Freefly.
- Wave: GPI setting EDGE (default, toggles on rising edge) or LVL (records
  while low). Pulse limits are only in an image Freefly hosts.
- Ember only: edge mode min 10 ms low / 500 ms high; level mode min 500 ms
  each way. GPO2 "Frame Pulse" = short negative pulse per frame, falling
  edge at exposure start. QX10001 UART remote protocol at 115200 8-N-1.
- Wave specs page: UART API "details TBD". No text console is documented
  for either camera.

## Pin identification

Session 2026-10-19, firmware uncommitted first build, camera in GPI:EDGE,
standby, host 3V3 on VIN. Logs: `bench_logs/01_probe_camera_on.log`,
`bench_logs/02_record_button_uart_probe.log`.

| Wave pin | ESP GPIO | Camera off | Camera on, standby         | On record start                      | Role        |
|----------|----------|------------|----------------------------|--------------------------------------|-------------|
| 3        | 26       | high       | high, no edges             | `Created new clip.\r\n` 115200 8-N-1 | UART TX out |
| 4        | 27       | high       | 30 Hz positive pulse, idle low | unchanged                        | PULSE out   |
| 5        | 32       | high       | high, no edges             | no edges                             | GPI1 (input to camera), idle high |
| 6        | 25       | high       | high                       | unchanged                            | GPI2/RS     |

Resolves the earlier uncertainty: the logic-analyser "stayed high" channel
was Pin 5 (camera input with its pull-up) and "stayed low" was Pin 4 between
its narrow pulses.

The UART message is exactly 19 bytes: `43 72 65 61 74 65 64 20 6E 65 77 20
63 6C 69 70 2E 0D 0A`. The first byte's start-bit edge to end-of-line is
about 2.4 ms, consistent with 19 bytes at 115200 (1.65 ms) plus the ESP's
read latency.

## 30 Hz pulse characterisation

Pin 4 (GPIO27), 10 s probe then a 63-period window, camera in standby,
not recording:

| Quantity          | Value                      |
|-------------------|----------------------------|
| Rate              | 29.999 Hz                  |
| Period mean       | 33334.3 us                 |
| Period stdev      | 1.08 us                    |
| Period min / max  | 33332 / 33337 us           |
| Jitter (p-p)      | 5 us                       |
| Polarity          | positive pulse, idle low   |
| High width        | 7-10 us as seen by the ISR |
| Missed edges      | 1 in 599 over 10 s         |

Width caveat: the pulse is only a few microseconds, about the same as the
GPIO interrupt latency, so the width figures are an upper bound and one
edge was missed in 10 s. A logic analyser or the RMT peripheral would be
needed for a trustworthy width. Period and jitter are unaffected because
they use the leading edge only.

Period of 33.3343 ms is 30.000 Hz to within 2 ppm, not 29.97 Hz
(33.3667 ms), so if this is HDMI timing it is 30p, not 29.97p.

HDMI unplug test (2026-10-19, log `bench_logs/09_hdmi_check_pulse_watch.log`):
13 pulse samples over 100 s while the owner unplugged a connected HDMI
cable. Period mean stayed 33334.2-33334.3 us in every sample, rate
29.9992 Hz, 6043 edges, no ring overflow. Two samples showed jitter blips
of 117 us and 49 us with a 68 us max width; these coincide with console
output and match the ISR-latency artefacts seen elsewhere, not a camera
event. Conclusion: Pin 4 is a free-running 30.000 Hz camera clock, not
slaved to a live HDMI link. Whether changing the HDMI *setting* moves it
was not tested, and also no longer matters for the design, since the
pulse is present and stable with no sink attached.

Camera clock vs ESP32 clock: the 30 ppm offset was identical across all
sessions, so it is a fixed crystal difference, not drift.

## CAN-phase bench, 2026-10-19 (no CAN hardware yet; simulated intent)

Level mode, camera power-cycled before this session, HDMI cable reconnected (owner confirmed) and
recording settings unchanged from the previous session. Firmware CAN phase uncommitted.
Logs: `bench_logs/12_can_fsm_camera_on_sim_intent.log`,
`bench_logs/13_can_fsm_two_more_cycles.log`, `bench_logs/14_console_lat5_after_can.log`.

CAN state machine, driven by the bench-only `sim intent` command, camera on:
IDLE -> STARTING -> START (clip confirmed, tick_stamped=1) -> RECORDING ->
AUTO_STOP (reason INTENT_TIMEOUT) -> IDLE. Three cycles, three clips,
recording_seq 1..3.

- Auto-stop fired 3.0 s after the last simulated intent frame (log 12:
  intent off at t=16893142, STOPPING at 19845934).
- Command-to-pad latency 19-40 us.
- CAN TX failed on every frame, as expected with no transceiver fitted.
  No resets, no ring overflow.

Per-cycle timing (us, ESP clock):

| Run | edge     | start tick | clip first byte | edge->tick | tick->clip | edge->clip |
|-----|----------|------------|-----------------|------------|------------|------------|
| 1   | 11846327 | 11867325   | 11888115        | 20998      | 20790      | 41788      |
| 2   | 64140203 | 64168861   | 64185566        | 28658      | 16705      | 45363      |
| 3   | 74287033 | 74302489   | 74345682        | 15456      | 43193      | 58649      |

Run 1 was the first clip after power-up; runs 2 and 3 followed 8 s
recordings with gaps of about 2.1 s. These do not fit the earlier
"latency + phase is constant" model: (tick-before-edge -> clip) = 54.1,
50.0, 76.4 ms.

Console-driven series straight afterwards: `lat 5 2 3` (2 s recordings,
3 s gaps, console owns GPI2, no pulse pin set so phase not recorded):

| Run | edge -> first byte (us) |
|-----|-------------------------|
| 1   | 42579                   |
| 2   | 42652                   |
| 3   | 42708                   |
| 4   | 42787                   |
| 5   | 42774                   |

Mean 42700 us, stdev 87 us, eol mean 45206 us. The 5.000210 s run period
shifts the tick phase by about 65 us per run and the latency grew about
50 us per run, consistent with tick quantisation within this series.

| Series (level mode)                          | edge -> first byte  |
|----------------------------------------------|---------------------|
| Previous session, HDMI connected, camera on for a while (3 runs) | 28467 / 28592 / 28694 us |
| This session, `lat 5 2 3`, HDMI connected (5 runs)                | 42579-42787 us, mean 42700 |

Hypothesis, not established: the clip-message latency is stable within a
series of identical short cycles but shifts between conditions by tens of
ms. Candidates are camera readiness after a longer clip with a short gap,
first clip after power-up, or power-cycle state. HDMI was connected in both sessions with the same settings, so it is ruled out. The recorded START
stamp uses the first tick after the GPI2 falling edge and is unaffected,
but whether that tick marks frame 0 of the clip under all conditions is
unknown.

## Repin verification, 2026-10-25

Firmware flashed with the new pins (GPI2 on GPIO26, Pin 3 on GPIO32, Pin 4
on GPIO27, Pin 5 unwired). Logs: `bench_logs/16_repin_check.log`,
`bench_logs/17_repin_fsm_cycle.log`. GPI1 is dropped, so these runs used the
CAN-owned simulated intent, camera in level mode.

- Pin map confirmed: GPIO27 classified PULSE (29.999 Hz, stdev 0.96 us,
  5 us p-p, same as before the move). GPIO32 idles high and carries the
  camera UART. GPIO26 idles high with no edges until driven.
- Full cycle on the new GPI2 pin: IDLE, STARTING, START (clip confirmed,
  tick stamped), RECORDING, AUTO_STOP 3.0 s after the intent stopped, IDLE.
  Falling edge to pad 30 us, tick to clip 21.3 ms, edge to first byte
  36.0 ms. No ring overflow.
- CAN: transceiver now fitted (pad reads recessive), no other node yet, so
  every transmit fails with no ACK and the controller sits error-passive
  without going bus-off or resetting. Expected until the bridge is connected.
- New finding, camera boot console. While the camera boots, Pin 3 prints
  about 33 lines of text, none of which appear in normal operation:
  `WAVE Hello! Bootloader entry.`, `WAVE HELLO!`, a PCIe link and
  initialisation report (`xdma_pcie` vendor 10EE, device 9134, then an NVMe
  end point 144D:A808), `NVMe initialization successful. PCIe link is Gen3
  x4.` and `SSD mount successful.` Boot to a running tick took roughly 12 s.
  The tick tracker counted 345 missed ticks during that window, so
  `ticks_missed_total` and the camera-on flag are only meaningful after the
  camera has finished booting. The `Created new clip.` matcher is unaffected.

## Open items

1. Which captured frame corresponds to the start tick, and what is the
   fixed offset between a Pin 4 tick and the sensor exposure at the
   recording frame rate. Needs an optical reference (Electronic-Clapboard
   flash, timestamped on CAN) in the recorded footage.
2. Button-press starts: only one of two button cycles produced a UART
   message in the one trial. Retest with a long listen and called-out
   presses. Low priority, since the robot will trigger via GPI2.
3. The slow upward drift of (message latency + phase) with camera uptime,
   about 18 ppm. Harmless for the tick-based design; worth a 10-minute
   log later to see whether it wraps at one tick period.
4. Pin 4 true pulse width: ISR-limited at 7-10 us. Use the logic analyser
   or the ESP32 RMT peripheral if the width ever matters.
5. Pin 5 drive level through 1 kOhm not measured. The `tx hold low`
   command exists in source for this (not flashed as of 2026-10-19).
6. GPI left in LVL mode at the end of the session. Choose the mode for the
   adapter: level mode gives a deterministic falling-edge start and a
   release stop with no toggle-state ambiguity, which suits a CAN-driven
   controller better than edge mode.
7. Clip-message latency depends on camera condition (see the CAN-phase
   bench section). Before relying on the tick stamp as frame 0: run a
   controlled series (HDMI in/out; 2 s vs 8 s clips; 2 s vs 6 s gaps; 10
   runs each) with `probe` run first so the phase column is populated, and
   consider a minimum gap after stop in the firmware if readiness is the
   cause. The optical calibration (clapboard flash in frame) remains the
   only way to settle frame 0.
8. The CAN path's `tick_to_clip_us` (INFO page 3, surfaced as camera_diag
   on the Jetson) should be watched during bring-up; a value far from the
   series norm flags a late start.

### Pin change 2026-10-23

Owner-approved rewiring. New mapping: Wave Pin 6 GPI2/RS -> GPIO26
(direct, open-drain), Pin 3 GPO1 camera UART out -> GPIO32 (1 kOhm series +
10 kOhm pull-up), Pin 4 GPO2 30 Hz tick -> GPIO27 (unchanged), Pin 5 GPI1
not wired, GPIO25 unused; CAN pins unchanged (TX GPIO13, RX GPIO34). Pin 5
was found inert (no response to QX packets or text, see "QX10001 packets
on the Wave") and dropped, so the firmware has no transmit path to the
camera and item 5 above is closed. Every measurement in this file was taken
with the old mapping (GPI2 on GPIO25, Pin 3 on GPIO26, Pin 5 on GPIO32);
GPIO numbers in the sections below and in `bench_logs/` refer to that mapping.

## GPI2 edge / level behaviour

Edge mode, 2026-10-19, log `bench_logs/03_gpi2_two_cycles.log`. Two full
start/stop cycles driven by the ESP32: 20 ms low pulse, released, 4 s apart.

- Every pulse toggled recording as expected: start, stop, start, stop.
  Owner confirmed the clip counter rose by exactly 2.
- Both starts produced `Created new clip.` on Pin 3; neither stop produced
  anything.
- Measured low width 20.003-20.009 ms (ISR stamps on GPIO25 itself).
- Command-to-pad latency 3-27 us.
- Rising edge to first UART byte: 22.3 ms and 24.6 ms for the two starts
  (full statistics in the latency section).

Button presses: in a 90 s listen the owner did two button start/stop cycles
but only one `Created new clip.` burst was seen (edge count on Pin 3
confirms only one burst physically occurred). Owner thinks the second press
may have been late or short. Open item: retest button starts with a long
listen and called-out presses.

Level mode, 2026-10-19, GPI = LVL, log `bench_logs/08_level_mode_lat3.log`,
`lat 3 2 3`: Pin 6 held low 2.000 s per run, released 3 s between runs.

| Run | latency_first_us | latency_eol_us | phase_us | first + phase |
|-----|------------------|----------------|----------|---------------|
| 1   | 28467            | 30853          | 21313    | 49780         |
| 2   | 28592            | 31008          | 21269    | 49861         |
| 3   | 28694            | 31046          | 21184    | 49878         |

- Falling edge starts recording, `Created new clip.` follows; release stops
  it with no message. Hold width measured 2000.03-2000.07 ms.
- latency + phase is again about 49.8 ms, so level mode is quantised to the
  same 30 Hz tick as edge mode. The three latencies agree to 0.1 ms only
  because the 5 s run spacing is exactly 150 ticks, which repeats the phase.
- The slow upward drift of latency + phase continues across sessions
  (48.84 ms at the start of the edge test, 49.88 ms here, about 25 minutes
  later), so it is a function of time since camera power-on, not of mode.

## GPI2 edge to "Created new clip." latency

2026-10-19, `lat 10 2 3`, edge mode, log `bench_logs/04_latency_10_runs.log`.
t_edge is the rising edge of GPIO25 as stamped by its own ISR. t_first is
the start-bit edge of the first UART byte on Pin 3. t_eol is when the LF was
read. Phase is t_edge minus the leading edge of the last 30 Hz pulse before
it.

| Run | latency_first_us | latency_eol_us | phase_us | first + phase |
|-----|------------------|----------------|----------|---------------|
| 1   | 38540            | 40926          | 10303    | 48843         |
| 2   | 31974            | 34327          | 16903    | 48877         |
| 3   | 25510            | 27867          | 23506    | 49016         |
| 4   | 18941            | 21310          | 30106    | 49047         |
| 5   | 45800            | 48146          | 3366     | 49166         |
| 6   | 39257            | 41618          | 9978     | 49235         |
| 7   | 32758            | 35176          | 16564    | 49322         |
| 8   | 26200            | 28572          | 23172    | 49372         |
| 9   | 19861            | 22217          | 29784    | 49645         |
| 10  | 46623            | 48946          | 3058     | 49681         |

| Statistic (n = 10) | first byte | end of line |
|--------------------|------------|-------------|
| min                | 18.94 ms   | 21.31 ms    |
| mean               | 32.55 ms   | 34.91 ms    |
| max                | 46.62 ms   | 48.95 ms    |
| stdev              | 9.95 ms    | 9.94 ms     |

Interpretation:

- The latency varies over a 27.7 ms range with 10 samples, consistent with
  a uniform spread over one 33.33 ms pulse period. The sum latency + phase
  is nearly constant (48.8-49.7 ms). So the camera samples GPI2 on its
  30 Hz tick and sends the message a fixed delay after the first tick that
  follows the edge: about 15.5-16.3 ms after that tick.
- Consequence for alignment: the record start is quantised to the 30 Hz
  pulse train. The UART message alone gives the start to about +-0.5 ms;
  the pulse train plus the message pins down which tick started the clip.
  The ESP32 can therefore timestamp the start tick itself, not the message.
- The sum drifts upward by about 90 us per run (18 ppm of the 5 s run
  spacing), monotonic over the 10 runs. Cause unknown: possibly a beat
  between the 30 Hz tick and another periodic process in the camera, or
  UART transmit scheduling. Worth a longer run later to see if it wraps.
- The pulse period measured 33334.3 us against a nominal 33333.3 us, a
  30 ppm offset between the ESP32 crystal and the camera clock. Irrelevant
  for alignment once the ESP is slaved to the robot's wall clock, because
  pulses are timestamped individually.

## UART console probe

2026-10-19, log `bench_logs/05_console_probe.log`. TX open-drain on Pin 5 through
1 kOhm, RX on Pin 3, armed. Sent, 3 s apart: `\r\n`, `help\r\n`, `?\r\n`.

Result: no reply. Zero bytes and zero edges on Pin 3 during and after the
sends. The 64 edges counted on GPIO32 are our own 11 transmitted bytes.

## QX10001 packets on the Wave

2026-10-19, logs `bench_logs/06_qx_rec.log` and `bench_logs/07_qx_stream_rec.log`.
Packets byte-exact to the Ember spec (idle ends BE, record-pressed ends BC),
115200 8-N-1, open-drain through 1 kOhm on Pin 5.

- Single press: one idle packet, then 3 x rec=02 and 3 x rec=00 at 10 ms
  spacing, repeated 5 s later. No visible reaction on the camera, no
  `Created new clip.`, no reply bytes.
- Streamed: 50 Hz idle stream (751 packets over 15 s) with the same press
  sequence twice inside it. Same result: nothing.

Conclusion (owner and session agree): on firmware 1.4.1 the Wave's GPIO
UART is output-only. Pin 3 carries a firmware log line at record start;
Pin 5 does not respond to the Ember QX protocol or to text. This matches
Freefly's Wave page ("UART API details TBD"). Not verified: that the
1 kOhm series drive pulls Pin 5 below the camera's input threshold. A
`tx hold low` diagnostic exists in the firmware (built, not yet flashed) if
that ever needs closing.

## Implications for the CAN-triggered auto-record design

- Trigger: GPI2 edge mode, 20 ms low pulse, >= 500 ms between pulses.
- Acknowledge: `Created new clip.` on Pin 3, 19 bytes, first byte about
  15.5 ms after the first 30 Hz tick following the trigger edge.
- Clock: Pin 4, 30.000 Hz, 1 us stdev. Record start is quantised to it.
  Timestamp every tick against the robot wall clock (0x7DD TIME_SYNC) and
  report the tick that started the clip, not the message time.
- Still unknown: which captured frame corresponds to the start tick, and
  whether Pin 4 follows the HDMI setting or is internal. Needs an optical
  check (clapboard flash) and an HDMI-setting test.
