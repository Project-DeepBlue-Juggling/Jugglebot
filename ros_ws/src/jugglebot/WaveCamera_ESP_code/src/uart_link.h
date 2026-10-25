#pragma once

// UART2 bridged to camera pins through the GPIO matrix, chosen at runtime.
//
// RX: GPIO32 (Wave pin 3, the camera's UART out; the boot default) or GPIO27
// (Wave pin 4). GPIO26 is the GPI2 output and is refused. The pad keeps its edge ISR while routed to
// U2RXD (the matrix only adds an input path; the GPIO input register and
// interrupt still see the pad), so t_first_byte is stamped from the start
// bit's falling edge when one is available, else from the read time.
// `status` reports rx_edges_while_bridged so this can be confirmed on the
// bench; if it ever reads 0 while bytes arrive, stamps fall back to read
// time automatically (src=read on the UART line).
// TX: unwired since 2026-10-23 (Wave pin 5 dropped). The open-drain TX path
// below is kept compiled, but set_tx() refuses while UART_TX_WIRED is false,
// so tx_pin() stays -1 and write_bytes() / tx_hold_low() always fail.

#include "pin_io.h"

#include "event_ring.h"
#include "line_assembler.h"

namespace uart_link {

// Called for every completed (or partial) line. t_first_us is the best
// available first-byte stamp, t_eol_us the read time of the terminator (or of
// the last byte for partial lines).
using LineHook = void (*)(const UartLine& line, int64_t t_first_us, int64_t t_eol_us);
void set_line_hook(LineHook hook);

bool set_rx(uint8_t gpio);  // prints INFO / ERR itself
bool set_tx(uint8_t gpio);
void off();
int  rx_pin();  // -1 if unassigned
int  tx_pin();
void set_hex(bool on);
bool hex();

void on_edge(const EdgeEvent& ev);
void poll(int64_t now_us);

// Send to the camera. Caller enforces arm / blocked-word guards.
// Bench: hold the TX pad low (UART break), open-drain only. Caller enforces arm.
// Auto-released from poll() after TX_HOLD_MAX_MS. Both print their own INFO.
bool tx_hold_low();
void tx_hold_release();
bool tx_holding();

bool write_bytes(const uint8_t* data, size_t len);

uint32_t rx_bytes();
uint32_t rx_edges_while_bridged();
uint32_t lines_edge_stamped();
uint32_t lines_read_stamped();

}  // namespace uart_link
