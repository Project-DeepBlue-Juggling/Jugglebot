#pragma once
// =============================================================================
//  imu_ring.h — the Platform IMU_DATA ring and its batch framing (FW 45, IDEA-20 P3a)
// =============================================================================
//  Contract: ros_ws/docs/platform_imu_protocol.md section 6 ("Uplink").
//
//  Every IMU_DATA (PlatformCanId::IMU_DATA, 0x6E6) frame the bridge receives on
//  the jugglebot bus is copied here by on_jugglebot_rx (can_buses.cpp, called from
//  FlexCAN events() in task_can_rx, priority 5), and platform_imu_uplink_step
//  (telemetry.cpp, task_telem, priority 3, 100 Hz) drains it into
//  PLATFORM_IMU_BATCH datagrams. A dedicated ring, not the 8-slot Platform reply
//  ring: at the 1600 frames/s Armed cap one 10 ms tick brings 16 frames, which
//  would overflow that ring twice per tick and evict tilt / fan / FwUpdate replies.
//
//  LOCK-FREE SPSC. One producer task, one consumer task; free-running u16 indices
//  (CAP divides 65536, so `head - tail` is the occupancy across index wrap); each
//  side writes only its own index, publishing with release and reading the other
//  side's with acquire. No PRIMASK section and no FreeRTOS lock, so nothing here
//  can delay the 500 Hz interp ISR or contend with the net task's NetLock.
//  The counters are written only by the producer (rx_frames, drops, hwm) and read
//  by the consumer as single aligned words, so a read is never torn.
//
//  FULL POLICY: drop the NEWEST frame and count it (drops). The Jetson rebuilds the
//  image by `seq` and re-requests every hole with a READ (spec section 7), so which
//  frame is lost does not matter to recovery; drop-newest keeps the producer from
//  ever touching `tail`, which is what keeps the ring single-writer per index.
//
//  Header-only and FreeRTOS-free so the native harness tests it as compiled.
// =============================================================================
#include <atomic>
#include <cstdint>
#include <cstring>
#include "udp_protocol.h"   // JbUdp::PlatformImuBatchPayload, PLATFORM_IMU_BATCH_MAX_FRAMES

namespace CanBridge {

// 128 frames = 80 ms at the 1600 frames/s Armed cap (102 ms at the 1250 target):
// eight 100 Hz telemetry ticks of slack for task_telem jitter, 1 KB of RAM.
// A power of two so the free-running u16 indices stay consistent across wrap.
constexpr uint16_t IMU_RING_CAP = 128;
constexpr uint8_t  IMU_FRAME_BYTES = 8;   // one IMU_DATA payload: [seq u16 LE][word 6 B]

static_assert((IMU_RING_CAP & (IMU_RING_CAP - 1)) == 0 && 65536u % IMU_RING_CAP == 0,
              "IMU_RING_CAP must be a power of two dividing the u16 index space");
static_assert(sizeof(JbUdp::PlatformImuBatchPayload::frames) ==
                  (size_t)JbUdp::PLATFORM_IMU_BATCH_MAX_FRAMES * IMU_FRAME_BYTES,
              "PlatformImuBatch frames[] must hold PLATFORM_IMU_BATCH_MAX_FRAMES records");

template <uint16_t CAP>
class ImuDataRing {
 public:
  // Producer (task_can_rx). Copies `len` bytes (at most 8) and zero-pads the rest,
  // so a short frame can never uplink stale bytes. Never blocks. False = dropped.
  bool push(const uint8_t* d, uint8_t len) {
    rx_frames_.store(rx_frames_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    const uint16_t h = head_.load(std::memory_order_relaxed);
    const uint16_t t = tail_.load(std::memory_order_acquire);
    const uint16_t used = (uint16_t)(h - t);
    if (used >= CAP) {
      drops_.store(drops_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
      return false;
    }
    uint8_t* slot = buf_[h % CAP];
    const uint8_t n = len > IMU_FRAME_BYTES ? IMU_FRAME_BYTES : len;
    memcpy(slot, d, n);
    if (n < IMU_FRAME_BYTES) memset(slot + n, 0, IMU_FRAME_BYTES - n);
    head_.store((uint16_t)(h + 1), std::memory_order_release);
    if ((uint16_t)(used + 1) > hwm_.load(std::memory_order_relaxed))
      hwm_.store((uint16_t)(used + 1), std::memory_order_relaxed);
    return true;
  }

  // Consumer (task_telem). False when empty.
  bool pop(uint8_t out[IMU_FRAME_BYTES]) {
    const uint16_t t = tail_.load(std::memory_order_relaxed);
    const uint16_t h = head_.load(std::memory_order_acquire);
    if (h == t) return false;
    memcpy(out, buf_[t % CAP], IMU_FRAME_BYTES);
    tail_.store((uint16_t)(t + 1), std::memory_order_release);
    return true;
  }

  uint16_t size() const {
    return (uint16_t)(head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire));
  }
  uint32_t rx_frames() const { return rx_frames_.load(std::memory_order_relaxed); }
  uint32_t drops() const { return drops_.load(std::memory_order_relaxed); }
  uint16_t hwm() const { return hwm_.load(std::memory_order_relaxed); }

 private:
  uint8_t buf_[CAP][IMU_FRAME_BYTES] = {};
  std::atomic<uint16_t> head_{0};
  std::atomic<uint16_t> tail_{0};
  std::atomic<uint32_t> rx_frames_{0};
  std::atomic<uint32_t> drops_{0};
  std::atomic<uint16_t> hwm_{0};
};

using ImuDataRingT = ImuDataRing<IMU_RING_CAP>;

// Fill one PLATFORM_IMU_BATCH from the ring: up to PLATFORM_IMU_BATCH_MAX_FRAMES
// records in FIFO order, zeroes past `count`, and the ring's counters. The caller
// stamps t_bridge_us and sends. Returns `count` (0 = the ring was empty).
template <class Ring>
inline uint8_t imu_batch_fill(Ring& ring, JbUdp::PlatformImuBatchPayload& p,
                              uint32_t batches_sent, uint32_t send_fails) {
  memset(&p, 0, sizeof(p));
  uint8_t n = 0;
  while (n < JbUdp::PLATFORM_IMU_BATCH_MAX_FRAMES && ring.pop(&p.frames[n * IMU_FRAME_BYTES])) ++n;
  p.count = n;
  p.ring_hwm = ring.hwm();
  p.rx_frames = ring.rx_frames();
  p.ring_drops = ring.drops();
  p.batches_sent = batches_sent;
  p.send_fails = send_fails;
  return n;
}

// Datagrams per telemetry tick. Steady state at the 1600 frames/s cap is ONE
// (16 frames per 10 ms tick); four drain a full 128-frame ring (6 batches) in two
// ticks after a stall while bounding the tick's added UDP work to four sends.
constexpr uint8_t IMU_BATCHES_PER_TICK_MAX = 4;

}  // namespace CanBridge
