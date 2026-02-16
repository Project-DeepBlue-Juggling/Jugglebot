// HandTrajectoryStreamer.h
#pragma once
#include "BallButlerConfig.h"
#include "CanInterface.h"
#include "TrajFrame.h"

struct HandTrajectoryStreamer {
  // ODrive enums (axis state constants live in BallButlerConfig.h)
  static constexpr uint32_t CONTROL_MODE_POSITION  = 3u;
  static constexpr uint32_t INPUT_MODE_PASSTHROUGH = 1u;

  CanInterface& can;
  const TrajFrame* frames = nullptr;
  size_t n = 0, idx = 0;
  uint32_t node = 0;
  bool active = false;
  uint64_t t_offset_us = 0;  // absolute wall-clock offset in microseconds

  explicit HandTrajectoryStreamer(CanInterface& c) : can(c) {}

  // time_offset_us: absolute wall-clock reference in microseconds.
  //   frames[i] is sent when wallTimeUs() >= time_offset_us + frames[i].t_s * 1e6.
  //   • For "start now": pass can.wallTimeUs()
  //   • For "decel at absolute time T": pass T (in µs)
  bool arm(uint32_t node_id, const TrajFrame* f, size_t count, uint64_t time_offset_us = 0) {
    if (!f || count == 0) return false;
    node = node_id; frames = f; n = count; idx = 0; this->t_offset_us = time_offset_us;

    if (!can.isAxisHomed(node_id)) {
      if (Serial) Serial.printf("[Gate] Trajectory arm rejected for node %lu (not homed)\n", (unsigned long)node_id);
      return false;
    }

    // 1) CLOSED_LOOP + POSITION/PASSTHROUGH
    if (!can.setRequestedState(node, ODriveState::CLOSED_LOOP)) return false;
    if (!can.setControllerMode(node, CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH)) return false;

    active = true;
    return true;
  }

  // Call frequently from loop()
  void tick() {
    if (!active) return;

    const uint64_t now_us = can.wallTimeUs();

    // Send any frames whose time has arrived (burst out catch-up frames too)
    while (idx < n) {
      // frames[idx].t_s is small (±few seconds), so float→int64 conversion is precise.
      // Adding to uint64_t t_offset_us preserves full microsecond accuracy at any epoch.
      const int64_t frame_off_us = (int64_t)llroundf(frames[idx].t_s * 1e6f);
      const uint64_t t_us = (uint64_t)((int64_t)t_offset_us + frame_off_us);
      if (now_us < t_us) break;
      (void)can.sendInputPos(node, frames[idx].pos_cmd, frames[idx].vel_ff, frames[idx].tor_ff);
      ++idx;
    }

    // Finished? → IDLE
    if (idx >= n) {
      (void)can.setRequestedState(node, ODriveState::IDLE);
      active = false;
    }
  }

  bool isActive() const { return active; }
};
