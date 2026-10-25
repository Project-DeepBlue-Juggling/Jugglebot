#pragma once

// `probe [seconds]`: per-pin edge statistics over a window, then classify
// each pin (pin_classify.h). Roles persist until reboot or the next probe.

#include "pin_io.h"

#include "event_ring.h"
#include "pin_classify.h"

namespace pin_probe {

bool start(uint32_t seconds);  // false if a probe is already running
void on_edge(const EdgeEvent& ev);
void poll(int64_t now_us);     // finishes the window and prints PROBE lines
bool running();

PinRole role(uint8_t gpio);
int pulse_pin();  // first pin classified PULSE, or -1
int uart_pin();   // first pin classified UART, or -1

}  // namespace pin_probe
