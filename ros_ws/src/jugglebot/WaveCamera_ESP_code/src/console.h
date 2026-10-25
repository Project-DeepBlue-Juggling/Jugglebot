#pragma once

// USB console (UART0 through the CP210x): line parser, help, and the output
// formatter every module prints through.
//
// Every output line is `<t_us> <TAG> <fields>`, t_us from esp_timer_get_time().
// Tags: EDGE UART UARTHEX PROBE PULSE REC LAT QX CAN INFO WARN ERR.

#include <cstdint>

#include "event_ring.h"

namespace console {

void begin();
void poll();  // read console bytes, dispatch complete lines

// Print one line stamped with the current time.
void out(const char* tag, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
// Print one line stamped with an explicit time (EDGE uses the ISR stamp).
void out_at(int64_t t_us, const char* tag, const char* fmt, ...) __attribute__((format(printf, 3, 4)));

bool armed();
bool edges_streaming();

// Per-pin edge bookkeeping; prints EDGE (stamped with the ISR time) when
// `edges on`.
void on_edge(const EdgeEvent& ev);

}  // namespace console
