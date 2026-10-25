#pragma once

// WAVE_HEARTBEAT at HEARTBEAT_RATE_HZ from boot, unconditionally (the bridge
// opens its TX gate only after hearing from us), and one WAVE_INFO page every
// INFO_PAGE_PERIOD_MS in rotation. Both are scheduled drift-free from loop():
// the next due time advances by exactly one period, and only a sender that has
// fallen SCHEDULE_RESYNC_PERIODS behind re-bases instead of bursting.

#include "pin_io.h"

namespace heartbeat {

void begin(int64_t now_us);  // sends the first heartbeat (state BOOT) immediately
void poll(int64_t now_us);

uint32_t heartbeats_queued();
uint32_t heartbeats_refused();  // TX request queue full
uint32_t info_queued();
uint32_t slips();               // schedule re-bases

// Per-second rates for INFO pages and `can`.
uint8_t accepted_per_s();
uint8_t rejected_per_s();
uint8_t intent_rx_per_s();
uint8_t ticks_missed_last_s();

}  // namespace heartbeat
