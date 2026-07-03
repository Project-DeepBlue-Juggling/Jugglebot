#pragma once
// =============================================================================
//  udp_link.h — UDP framing, RX dispatch, and TX, over QNEthernet
// =============================================================================
//  Owns both UDP sockets (PORT_STREAM, PORT_RPC). Validates inbound frames with
//  JbUdp::decode_frame (magic/version/length/CRC-16), tracks sequence gaps, and
//  dispatches by msg_type to registered handlers. Provides TX helpers that
//  build frames with JbUdp::encode_frame and per-(channel) sequence counters.
//
//  Threading: all QNEthernet/lwIP access is serialised by an internal recursive
//  mutex (lwIP is not reentrant). RX runs in the udp_rx task (udp_link_service);
//  TX helpers may be called from any task. See README "lwIP threading" for the
//  alternative single-net-task + TX-queue design (a hardware-validation item).
// =============================================================================

#include <cstdint>

namespace CanBridge {

// Registered RX handler. `seq` is the frame sequence; `payload`/`len` are the
// validated payload (already CRC-checked). Handlers must be fast / non-blocking.
typedef void (*FrameHandler)(uint16_t seq, const uint8_t* payload, uint16_t len);

// Register typed handlers (null = drop). Set during setup, before the scheduler.
void udp_on_setpoint(FrameHandler h);
void udp_on_heartbeat_j2t(FrameHandler h);
// RPC requests are handled by the rpc dispatcher (see rpc.h); udp_link routes
// RPC_REQUEST frames to it via this hook.
void udp_on_rpc_request(FrameHandler h);
// RPC responses arrive when the Teensy is the RPC *client* (time-of-day query —
// the time-sync master bootstrap). Routed to the time master via this hook.
void udp_on_rpc_response(FrameHandler h);

// Bind sockets. Call after net_ethernet_begin(), before the scheduler starts.
void udp_link_init();

// Service the network: pump lwIP, drain RX on both sockets, dispatch. Loop this
// from the udp_rx task.
void udp_link_service();

// ── TX helpers ───────────────────────────────────────────────────────────────
// Send an uplink frame to the Jetson on the STREAM port (telemetry/diag/profile/
// heartbeat). Returns false on encode/socket error. Thread-safe.
bool udp_send_stream(uint8_t msg_type, const uint8_t* payload, uint16_t len);

// Send a frame on the RPC port to the Jetson (RPC responses). Thread-safe.
bool udp_send_rpc(uint8_t msg_type, const uint8_t* payload, uint16_t len);

// ── Stats (for the profiling frame) ───────────────────────────────────────────
struct UdpStats {
  uint32_t rx_frames;      // valid frames received
  uint32_t tx_frames;      // frames transmitted
  uint32_t crc_errors;     // frames dropped on CRC/magic/version/length
  uint32_t seq_gaps;       // detected downlink sequence discontinuities
  uint32_t drain_cap_hits; // RX drain hit UDP_RX_DRAIN_BUDGET with a frame still queued
};
UdpStats udp_link_stats();

// Last measured Jetson round-trip (us) and jitter estimate. Reported by the
// time-of-day RPC round-trip (the natural request/response we already perform).
uint32_t udp_last_rtt_us();
uint32_t udp_rtt_jitter_us();
void     udp_note_rtt(uint32_t rtt_us);   // called by the time master on each TOD round-trip

// Time (wall-clock us) of the last valid downlink frame from the Jetson — used
// by the link-health watchdog.
uint64_t udp_last_rx_us();

}  // namespace CanBridge
