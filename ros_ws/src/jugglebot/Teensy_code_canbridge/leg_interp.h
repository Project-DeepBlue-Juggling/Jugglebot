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
#include "canbridge_config.h"   // NUM_LEGS/NUM_AXES + the hand-lane guard constants
                                // (can_buses.h pulls the same header for BUS_PARTNER_STALENESS_US)

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

// Latched interpolation base (= the incoming MPC command u0) for axis i
// (legs 0-5 + the hand lane at 6), and whether any setpoint has been latched —
// used by the fault machine's max-deviation E-STOP (motor_guard.py:539-551,
// incoming-command vs encoder) and the [guard] diag line.
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

// Per-axis lead-clamp-engaged bitmask from the most recent computed 500 Hz tick
// (bits 0-5 = legs, bit 6 = the hand lane since FW 17). Diagnostic telemetry
// (surfaced on HeartbeatT2J) for the 2026-07-10 stutter/lead diagnosis and the
// FW 17 hand lead-duty read.
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

// ── Hand lane (axis 6) — unified-7dof FW 17 ──────────────────────────────────
// The 7th interpolated channel. Active only while a latched setpoint carried
// HAS_HAND *and* hand_source == STREAMED (hand_source.h); inert otherwise.
// Counters are CUMULATIVE SINCE BOOT (the interp census idiom above — the
// consumer differences two reads; interp_reset() zeroes them with the rest).

// True iff the hand lane holds latched state (a HAS_HAND frame was accepted
// while STREAMED). The fault task gates its hand-deviation read on this.
bool interp_hand_lane_active();

// Ticks whose transmitted hand command engaged the MAX_LEAD_HAND_REV clamp —
// THE REQUIRED lead-duty counter (Phase 0 Decision 4): non-zero duty during a
// throw is a hard-abort-the-sitting signal (FW 14 clamp-duty-0 precedent).
// Live per-tick engagement also rides interp_lead_clamp_mask() bit 6.
uint32_t interp_hand_lead_clamp_ticks();

// 7th-frame TX census, carried over from the retired UNIFIED7_BENCH_BUILD
// probe because the real lane needs the same honesty: sent = hand frames
// actually transmitted; unseen_skips = ticks the lane was active but axis 6's
// encoder has NEVER been seen (commanding anything would be a guess — skip and
// say so, never silently); stale_holds = ticks whose lead-clamp anchor age was
// capped at MOTOR_FB_STALENESS_US because the axis-6 feedback went stale (the
// frame still transmits; the counter is what makes the gap visible).
uint32_t interp_hand_sent();
uint32_t interp_hand_unseen_skips();
uint32_t interp_hand_stale_holds();

// Setpoint index-6 discards while hand_source == LEGACY — the § 2.4 "discarded
// on a visible counter". A climbing value with a v6 host means the host is
// emitting hand knots the firmware is refusing by mode: switch the source, or
// stop the producer.
uint32_t interp_hand_discard_legacy();

// ── Hand deviation guard (MAX_DEVIATION_HAND_REV, observe-first) ─────────────
// The residual is computed EVERY 500 Hz tick while the lane is active:
//   dev = (raw interpolated hand command, pre-clamp) − (fb + fb_vel·age)
// (velocity-compensated both sides — Phase 0 Decision 4). The tick's verdict
// is a cumulative exceed-tick counter the 10 Hz fault task differences (race-
// free single-writer census — no read-then-clear); the max |residual| and the
// worst-tick snapshot are the observe-first sitting's read.
uint32_t interp_hand_dev_over_ticks();   // ticks with |dev| > MAX_DEVIATION_HAND_REV
float    interp_hand_dev_last();         // most recent tick's residual (rev)
float    interp_hand_dev_max();          // max |residual| since boot/reset (signed value at the max)
float    interp_hand_dev_snap_cmd();     // raw command at the worst-residual tick
float    interp_hand_dev_snap_fb();      // age-extrapolated fb at that tick
// Trip-dedicated snapshot (2026-09-02 review fix): the residual trio at the
// most recent EXCEED tick (|dev| > MAX_DEVIATION_HAND_REV). The fault machine
// freezes THESE into an armed MAX_DEVIATION latch — never the boot-cumulative
// dev_max trio above, whose worst tick can predate the trip by a whole observe
// block and misattribute the excursion in /link_status and bags.
float    interp_hand_dev_trip_dev();     // residual at the most recent exceed tick
float    interp_hand_dev_trip_cmd();     // raw command at that tick
float    interp_hand_dev_trip_fb();      // age-extrapolated fb at that tick
// Observe→arm switch (explicit, per the Phase 0 observe-first decision): boots
// FALSE (observe — report only); `hand7 arm` on the console arms the E-STOP
// trip for the second sitting. Runtime, not a build flag, so arming needs no
// reflash and every boot returns to the safe observe state.
bool interp_hand_dev_guard_armed();
void interp_set_hand_dev_guard_armed(bool armed);

// Console handler + 1 Hz status line (task_diag), the gpio_poll idiom: owns the
// `hand7` prefix, prints its own status, returns false for anything else.
//   hand7            → status line only
//   hand7 arm        → arm the hand deviation E-STOP (second-sitting step)
//   hand7 observe    → back to observe-only (the boot default)
bool interp_hand7_console(const char* line);
void interp_hand7_diag_step();

}  // namespace CanBridge
