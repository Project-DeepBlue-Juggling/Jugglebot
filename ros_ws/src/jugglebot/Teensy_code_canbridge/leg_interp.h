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

}  // namespace CanBridge
