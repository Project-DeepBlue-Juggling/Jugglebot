#pragma once
// =============================================================================
//  telemetry.h — motor-state uplink + on-change diagnostics
// =============================================================================
//  100 Hz motor-state stream (pos/vel for all 7 axes), plus per-axis diagnostics
//  (iq, temps, bus voltage, error/disarm bitmasks, state) published on-change
//  (delta over threshold) OR on a 1 Hz per-axis heartbeat. Matches the parent
//  plan's telemetry design (§"Telemetry uplink"). The 1 Hz forced refreshes are
//  staggered across axes so they never burst in a single tick.
// =============================================================================

#include <cstdint>

namespace LegBridge {

void telemetry_init();
// Call at TELEM_RATE_HZ (100 Hz). Sends the Telemetry frame every tick and any
// due Diagnostic frames (changed axes immediately; unchanged axes once/second).
void telemetry_step();

}  // namespace LegBridge
