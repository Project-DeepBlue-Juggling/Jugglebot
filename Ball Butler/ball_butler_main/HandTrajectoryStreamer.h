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
  float t_offset_s = 0.0f;  // set via arm()

  explicit HandTrajectoryStreamer(CanInterface& c) : can(c) {}

  // time_offset_s:
  //   • If frames[].t_s are ABSOLUTE wall times → pass 0.
  //   • If RELATIVE (start at 0 "now") → pass can.wallTimeUs()/1e6f
  //   • If ABSOLUTE in local micros() time → pass (can.wallTimeUs()-can.localTimeUs())/1e6f
  bool arm(uint32_t node_id, const TrajFrame* f, size_t count, float time_offset_s = 0.0f) {
    if (!f || count == 0) return false;
    node = node_id; frames = f; n = count; idx = 0; t_offset_s = time_offset_s;

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
      const uint64_t t_us = (uint64_t) llroundf((frames[idx].t_s + t_offset_s) * 1e6f);
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
