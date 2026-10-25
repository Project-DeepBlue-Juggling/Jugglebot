#pragma once

// Edge ISRs on every capture pin -> lock-free ring of {gpio, level, t_us}.
// The ISR stamps esp_timer_get_time(), reads the real pad level and pushes;
// it never prints or allocates. loop() drains the ring in time order.

#include "pin_io.h"

#include "event_ring.h"

namespace edge_capture {

using EdgeRingT = EventRing<EDGE_RING_CAPACITY>;

// Configures all capture pins (GPI2 open-drain released, the rest inputs with
// pull-ups) and installs the GPIO ISR service + per-pin handlers.
bool begin();

// Pops up to max_events and calls handler for each. Returns the count.
size_t drain(void (*handler)(const EdgeEvent&), size_t max_events);

uint32_t overflows();
uint32_t high_water();
uint32_t pending();

}  // namespace edge_capture
