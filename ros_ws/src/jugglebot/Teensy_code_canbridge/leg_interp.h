#pragma once
// =============================================================================
//  leg_interp.h — 500 Hz Hermite/Taylor interpolator (port of motor_guard)
// =============================================================================
//  Port of motor_guard.py:894-1048 — the Hermite → Taylor → velocity-decay
//  ladder + lead-clamp + stroke-clamp. The math mirrors, line-for-line, the
//  validated Python reference
//    tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
//  which the xref harness proves matches the real motor_guard to 0.0 rev in
//  float64 across all three modes + clamps over recorded data. On the Teensy
//  the math runs in float32 (single-precision FPU) — the only expected residual
//  is float32-vs-float64 rounding, a bench-validation item.
//
//  Runs in a 500 Hz IntervalTimer ISR at a hardware priority ABOVE the FreeRTOS
//  syscall ceiling, so nothing the RTOS does can preempt it (the entire point of
//  moving off Linux). The ISR therefore makes NO
//  FreeRTOS calls; the setpoint hand-off uses a plain volatile pending flag.
// =============================================================================

#include <cstdint>
// UNIFIED7_BENCH_BUILD gates the bench-only declarations at the bottom of this
// header. It MUST be included here rather than left to the includer: an undefined
// macro evaluates to 0 in `#if`, so a translation unit that reached this header
// first would silently drop those declarations while leg_interp.cpp still defined
// them — a link error at best, and an include-order-dependent build at worst.
// (can_buses.h pulls the same header for BUS_PARTNER_STALENESS_US.)
#include "canbridge_config.h"

namespace CanBridge {

void leg_interp_init();   // start the 500 Hz IntervalTimer

// Reset every interpolator file-static to its power-on value. For (a) the native
// test harness (isolate interp statics between cases) and (b) on-target re-arm.
// NOT ISR-safe — call only when the interp is quiescent (output disabled).
void interp_reset();

// udp_link setpoint handler (runs in the net task): stages the new waypoints.
void interp_on_setpoint(uint16_t seq, const uint8_t* payload, uint16_t len);

// Wall-clock (us) of the last setpoint received — for the staleness watchdog.
uint64_t interp_last_setpoint_us();

// Monotonic (us) of the most recent 500 Hz interp tick. Torn-load-guarded, so it is
// safe to call from a FreeRTOS task even though the ISR is the only writer.
// Diagnostic use: `micros64() - interp_last_tick_us()` is the caller's PHASE within
// the 2 ms interp cycle — hand_ops stamps every HAND_TRAJ_CMD with it (2026-08-09),
// which is the falsifiable test of the phase-locked-dispatch-quantisation verdict in
// logbook/2026-08-02-err-timeout-attribution-instrumentation.md § A3. Returns 0 until
// the first tick (and after interp_reset()), so a stamp taken before the interp is
// running is meaningless, not merely large.
uint64_t interp_last_tick_us();

// Latched interpolation base (= the incoming MPC command u0) for leg i, and
// whether any setpoint has been latched — used by the fault machine's
// max-deviation E-STOP (motor_guard.py:539-551, incoming-command vs encoder).
float interp_base_pos(uint8_t i);
bool  interp_have_latched();

// Enable/disable command output (the fault state machine / guard-mode gate).
// When false the ISR computes but does not transmit to CAN3.
void interp_set_output_enabled(bool en);
bool interp_output_enabled();

// Profiling counters.
uint32_t interp_deadline_misses();
uint32_t interp_max_jitter_us();
void     interp_reset_jitter();

// ── 500 Hz ladder occupancy census (FW 11 → CLOCK_DIAG 0x8F) ─────────────────
// CUMULATIVE SINCE BOOT — the caller differences two consecutive reads to get a
// window count, exactly as profiling.cpp does with the CAN frame counters. There
// is deliberately NO reset entry point: a read-then-clear at the emit site would
// silently drop any ISR increment that landed between the two, and at 500 Hz that
// loss is indistinguishable from a real change in occupancy. u32 wrap (~99 days)
// is harmless because unsigned subtraction is wrap-correct.
//   interp_tick_count()        — EVERY ISR entry, including early returns (stow,
//                                pre-first-latch). The duty-cycle denominator and
//                                a tick-census cross-check of interp_deadline_misses.
//   interp_recover_slew_ticks()— ticks where the re-enable recovery ramp actually
//                                OVERRODE the streamed command (not merely armed).
//   interp_extrap_ticks()      — ticks that took the Mode-2 cubic-Taylor branch,
//                                i.e. ran open-loop with no u1 knot available.
uint32_t interp_tick_count();
uint32_t interp_recover_slew_ticks();
uint32_t interp_extrap_ticks();

// Per-leg lead-clamp-engaged bitmask from the most recent computed 500 Hz tick
// (bit i = leg i). Diagnostic telemetry (surfaced on HeartbeatT2J) for the
// 2026-07-10 stutter/lead diagnosis.
uint8_t  interp_lead_clamp_mask();

// Per-leg torque_ff-ingest-clamp bitmask from the most recent ACCEPTED setpoint
// frame (bit i = leg i; set when |torque_ff[i]| was clamped to
// Dynamics::TORQUE_FF_FIRMWARE_CLAMP_WIRE_NM at UDP ingest). Diagnostic telemetry
// (surfaced on HeartbeatT2J flags bits 8-13, mirroring interp_lead_clamp_mask).
uint8_t  interp_torque_clamp_mask();

// ── Deferred-stow profiled descent (driven by the fault machine) ──────────────
// When stow is active the 500 Hz ISR ignores the MPC ladder and runs a
// velocity-limited descent of every leg to the off pose (stroke min), emitting
// the setpoints on CAN3 (the Jugglebot core bus). This is the Teensy analog of
// can_node's _gently_move_to_setpoint(0.0, deactivating=True).
void interp_begin_stow();    // capture current encoder positions, start the descent
void interp_end_stow();      // stop the descent (back to MPC ladder / hold)
bool interp_stow_active();
bool interp_stow_complete();  // true once all legs reached the off pose

#if UNIFIED7_BENCH_BUILD
// ── BENCH ONLY — 7th-frame bus-headroom probe (unified-7dof Phase 0 probe 3) ──
// Compiled out of every production image (canbridge_config.h defaults the flag to
// 0). See that header's UNIFIED7_BENCH_BUILD block for what the probe measures and
// why it exists; the sitting runbook is
// tests/hardware/session_unified7_bus_headroom.md.
//
// WHY THE SERIAL CONSOLE AND NOT AN ADDITIVE RPC / MsgType. Three reasons, in
// descending weight. (1) The measurement channel is ALREADY the console: the
// per-class [cantx] deferral census (Teensy_code_canbridge.ino), [canhealth]'s
// defer=/txq= pair and the [handphase] stamp are all console-only, so the sitting
// holds a `pio device monitor` open regardless — the toggle costs the operator no
// new plumbing and no new host process to fight over the single-owner UDP link.
// (2) An additive RpcMethod/MsgType is generated from config/generate_udp_protocol.py
// and lands in the COMMITTED generated headers on BOTH ends, so a bench-only, one
// sitting, never-flashed-to-production experiment would leave a permanent entry in
// the production wire spec and in teensy_link — the opposite of "the bench image
// leaves no trace". (3) The console seam already exists and has a settled idiom
// (gpio_poll_console: own your prefix, print your own status line, return false for
// anything else), so this is one more handler on an established dispatch chain
// rather than a new mechanism.
//
// Console grammar, mirroring gpio_poll's:
//   bench7        → print the status line (toggle state + counters), change nothing
//   bench7 on     → arm the 7th frame   (arm B)
//   bench7 off    → disarm it           (arm A / baseline; the BOOT DEFAULT)
// Returns false for any line this handler does not own, so the caller can still
// report an unknown command.
bool interp_bench7_console(const char* line);

// 1 Hz console step (task_diag). Prints the repeating self-identification +
// counter line. Repeating rather than one-shot for the gpio_poll_diag_step reason:
// a boot banner has scrolled away long before an operator attaches a monitor, and a
// bench image that looks stock on the console is exactly the hazard the
// never-flash-bench-firmware discipline exists to prevent.
void interp_bench7_diag_step();
#endif  // UNIFIED7_BENCH_BUILD

}  // namespace CanBridge
