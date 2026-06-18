#pragma once
// =============================================================================
//  telemetry.h — motor-state uplink + on-change diagnostics + cone relay
// =============================================================================
//  100 Hz motor-state stream (pos/vel for all 7 axes), plus per-axis diagnostics
//  (iq, temps, bus voltage, error/disarm bitmasks, state) published on-change
//  (delta over threshold) OR on a 1 Hz per-axis heartbeat. Matches the parent
//  plan's telemetry design (§"Telemetry uplink"). The 1 Hz forced refreshes are
//  staggered across axes so they never burst in a single tick.
//
//  Also home to the cone uplink (phase-10b): cone_uplink_step() drains the
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

}  // namespace CanBridge
