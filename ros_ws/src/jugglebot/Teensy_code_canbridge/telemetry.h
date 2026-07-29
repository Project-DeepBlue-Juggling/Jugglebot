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

}  // namespace CanBridge
