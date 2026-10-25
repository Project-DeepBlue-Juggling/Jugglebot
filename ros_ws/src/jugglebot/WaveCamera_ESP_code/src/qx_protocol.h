#pragma once

// QX10001 packet sender over the bridged UART2 TX (open-drain).
// Packet layout and checksum live in qx_packet.h (pure, native-tested).
//
// `qx rec|sel|jog` queue a press: QX_PRESS_PACKETS packets with the button
// set, then QX_PRESS_PACKETS released (idle), QX_PRESS_GAP_MS apart. Each
// packet is printed as hex immediately before it is written.
// `qx stream <hz>` sends idle packets continuously (cap QX_STREAM_MAX_HZ).
// Arm state and TX availability are checked by the caller (console).

#include "pin_io.h"

#include "qx_packet.h"

namespace qx_sender {

bool send_now(const uint8_t pkt[qx::PACKET_BYTES], const char* label);
bool queue_press(uint8_t rec, uint8_t jog, uint8_t sel, const char* label);
void set_stream_hz(uint32_t hz);  // 0 = off
uint32_t stream_hz();
void stop_all();
void poll(int64_t now_us);
bool busy();

}  // namespace qx_sender
