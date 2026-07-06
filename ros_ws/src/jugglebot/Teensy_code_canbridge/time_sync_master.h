#pragma once
// =============================================================================
//  time_sync_master.h — multi-bus 0x7DD time-sync master + time-of-day RPC client
// =============================================================================
//  The can-bridge replaces the Jetson as the system time-sync MASTER. It:
//    * broadcasts (wall_offset + monotonic) at 100 Hz on ALL THREE subsystem
//      buses (CAN1/CAN2/CAN3), ID 0x7DD, using the EXACT payload format the
//      Jetson emitted — struct.pack('<II', sec, usec) (see bus.py
//      broadcast_time) — so the three existing slave Teensys (platform 4.0 on
//      CAN3, Ball Butler on CAN1, catching cone on CAN2) consume it with ZERO
//      change (per-bus fan-out is invisible to each slave — ADR-0013);
//    * bootstraps + drift-corrects its wall clock by querying the Jetson over
//      UDP (RpcMethod::TIME_OF_DAY_QUERY) at boot and every ~30 s.
//
//  Broadcasting is gated on time_synced(): until the first Jetson anchor lands,
//  the Teensy has only a monotonic clock, and emitting that as wall-clock would
//  corrupt the slaves' epoch. During that brief gap no 0x7DD is sent and the
//  slaves hold their last IIR-filtered offset.
// =============================================================================

#include <cstdint>

namespace CanBridge {

// Register the RPC-response handler (captures time-of-day replies). Call in setup.
void time_sync_master_init();

// Drive one step. Call at TIME_SYNC_RATE_HZ (100 Hz): broadcasts 0x7DD if synced,
// and internally paces the time-of-day query (fast retry until first anchor,
// then every TIMEOFDAY_RESYNC_MS).
void time_sync_step();

}  // namespace CanBridge
