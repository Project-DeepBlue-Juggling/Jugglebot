#pragma once

// 30 Hz pulse-train statistics. Every capture pin keeps a rolling history of
// its last PULSE_HISTORY_EDGES edges; `pulse` analyses the PULSE pin found by
// `probe` (or every pin with activity if none is known yet).

#include "pin_io.h"

#include "event_ring.h"
#include "stats_math.h"

namespace pulse_stats {

void on_edge(const EdgeEvent& ev);
void report();  // prints PULSE lines

// Time of the most recent leading edge of the pulse on `gpio` at or before
// t_us (leading = falling for a negative pulse). -1 if unknown.
int64_t last_leading_edge_us(uint8_t gpio, int64_t t_us);

}  // namespace pulse_stats
