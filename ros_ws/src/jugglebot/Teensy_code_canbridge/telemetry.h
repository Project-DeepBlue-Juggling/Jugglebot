#pragma once
// =============================================================================
//  telemetry.h — motor-state uplink + on-change diagnostics + cone relay
// =============================================================================
//  100 Hz motor-state stream (pos/vel for all 7 axes), plus per-axis diagnostics
//  (iq, temps, bus voltage, error/disarm bitmasks, state) published on-change
//  (delta over threshold) OR on a 1 Hz per-axis heartbeat. The 1 Hz forced
//  refreshes are
//  staggered across axes so they never burst in a single tick.
//
//  Also home to the cone uplink: cone_uplink_step() drains the
//  CAN2 SPSC ring (can_buses.h) into CONE_FRAME UDP messages. It lives here —
//  not in can_buses.cpp — to keep the bus layer free of UDP dependencies; both
//  steps run on task_telem.
// =============================================================================

#include <cstdint>

namespace CanBridge {

void telemetry_init();
// Call at TELEM_RATE_HZ (100 Hz). Sends the Telemetry frame every tick and any
// due Diagnostic frames (changed axes immediately; unchanged axes once/second).
void telemetry_step();

// Call at TELEM_RATE_HZ (100 Hz), after telemetry_step(). Forwards queued cone
// CAN2 frames (catch events + cone heartbeats) to the Jetson, bounded per tick.
void cone_uplink_step();

// Call at TELEM_RATE_HZ (100 Hz), after telemetry_step(). Forwards queued BB
// CAN1 CMD_RESULT frames (loud command-outcome channel) to the Jetson,
// bounded per tick.
void cmd_result_uplink_step();

// Call at TELEM_RATE_HZ (100 Hz), after telemetry_step(). Forwards queued
// Platform-Teensy relay replies (CAN3 0x6E0 RobotState / 0x7DE tilt) verbatim to
// the Jetson as PLATFORM_FRAMEs (relay seam), bounded per tick.
void platform_uplink_step();

// Call at TELEM_RATE_HZ (100 Hz), after telemetry_step(). Emits the latest sniffed
// hand Set_Input_Pos (Platform→hand ODrive on CAN3) as a HAND_CMD_ECHO when a fresh
// command is pending (hand command-echo — feeds hand_telemetry's cmd fields).
void hand_cmd_echo_uplink_step();

// Call at TELEM_RATE_HZ (100 Hz), after telemetry_step(). Publishes gpio_poll's
// hand ball-sensor cache as a HAND_SENSOR frame on every NEW good TxSdo reply
// (wire rate = the poll rate) plus a 1 Hz keepalive while none lands, so a stale
// or never-answering sensor is observable on the wire rather than silent.
// Deliberately NOT gated on JBBallDetect::ENABLED: a kill-switched build still
// emits the honest-UNKNOWN keepalive, so the operator can tell "bridge alive,
// sensor compiled out" from "no bridge at all".
void hand_sensor_uplink_step();

// Call at TELEM_RATE_HZ (100 Hz), after telemetry_step(). Emits the CAN3
// wire-error + fault-confinement counters as a CAN_ERRORS frame at a flat 1 Hz.
// These counters have been computed every 1 kHz service tick since the 2026-07-05
// marginal-CAN3 work and were reachable ONLY over the USB serial console; the
// 2026-07-29 CAN3 flap could not be root-caused from a rosbag for exactly that
// reason (logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md, layer 2).
// Unconditional rather than on-change ON PURPOSE: an operator differencing an A/B
// needs a continuous baseline, and "no frame" must never be ambiguous with "no
// errors".
void can_errors_uplink_step();

// Call at TELEM_RATE_HZ (100 Hz), after telemetry_step(). Emits the per-bus CAN
// TX deferral/queue counters plus hand_ops' per-stage exit tally as a
// BRIDGE_TX_DIAG frame at a flat 1 Hz. Unconditional for the same reason as
// can_errors_uplink_step: the consumer reads this by DIFFERENCING two captures
// (e.g. with and without the 500 Hz leg stream), which needs a continuous
// baseline, and "no frame" must never be ambiguous with "no pressure".
void bridge_tx_diag_uplink_step();

// Call at TELEM_RATE_HZ (100 Hz), after telemetry_step(). Folds ONE per-axis
// encoder-cache-age sample into the current window on EVERY call, and emits the
// window's per-axis age floor/peak plus the CAN RX-ring occupancy as a
// CACHE_DIAG frame at a flat 1 Hz. Unconditional, like its two 1 Hz siblings:
// the consumer reads it as a trend across a multi-hour soak, so a gap must never
// be ambiguous with health.
//
// WHY IT SAMPLES AT 100 Hz BUT SENDS AT 1 Hz. The quantity under test is the
// cache-age FLOOR (see the CacheDiag summary in config/generate_udp_protocol.py:
// this is the instrument that separates "the leg genuinely trails" from "the
// lead clamp is measuring against a stale encoder cache"). Between two ODrive
// encoder broadcasts the age ramps 0 -> one broadcast period, so a single 1 Hz
// read is a uniform sample of that ramp and says nothing about the floor; the
// MINIMUM over ~100 reads IS the floor. The extra cost is one seqlock read per
// axis per tick — the same read send_telemetry() already performs — and NO ISR
// work at all.
//
// CONCURRENCY: every accumulator behind this frame is written and read ONLY by
// task_telem, so there is no cross-context counter, no reader-side clear of a
// counter another context writes, and no IRQ-off window.
void cache_diag_uplink_step();

// Call at TELEM_RATE_HZ (100 Hz), after telemetry_step(). Emits the running
// build's FW_VERSION + PROTOCOL_VERSION as a BRIDGE_IDENTITY frame at a flat
// 1 Hz. FW_VERSION previously existed only in the USB serial boot banner, so a
// Jetson session had no way to tell WHICH firmware answered it. Unconditional
// rather than send-once-on-connect: the bridge has no notion of a Jetson
// re-attaching, so a one-shot would be missed by every consumer that started
// after it.
void bridge_identity_uplink_step();

}  // namespace CanBridge
