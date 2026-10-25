#pragma once

// `lat <n> [rec_s] [gap_s]`: N-run GPI2 trigger -> "Created new clip." timing.
//
// Per run: start trigger (edge mode: toggle; level mode: start) -> stamp the
// GPIO26 edge from its own ISR -> wait for a camera UART line containing
// LAT_MATCH_TEXT -> record t_first_byte - t_edge and t_eol - t_edge, plus the
// phase to the last 30 Hz pulse -> hold rec_s -> stop trigger -> wait gap_s.
// LAT_RUN_TIMEOUT_MS without the line aborts and sets the rec state unknown.
//
// Non-blocking: polled from loop(), so edge capture and UART stamping keep
// running throughout.

#include "pin_io.h"

#include "line_assembler.h"

namespace latency_test {

bool start(uint32_t runs, float rec_s, float gap_s);
void abort(const char* why);
void poll(int64_t now_us);
void on_uart_line(const UartLine& line, int64_t t_first_us, int64_t t_eol_us);
bool running();

}  // namespace latency_test
