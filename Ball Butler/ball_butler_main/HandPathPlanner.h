// HandPathPlanner.h
#pragma once

/*
  HandPathPlanner — pure planning class (no Serial, no CAN)
  ---------------------------------------------------------
  Purpose
    • Given the current feedback state (position, velocity) and a desired
      linear throw velocity [m/s], generate a full hand trajectory sampled
      at 500 Hz: optional smooth move → configurable pause → throw.

  P2-1 REFACTOR: All heap allocations removed.
    • Intermediate Trajectory objects use a TrajArena (bump allocator)
      owned by this class. The arena is reset at the start of each plan call.
    • Output TrajFrames are written into a caller-provided fixed buffer.
    • HandPlanResult no longer contains a vector — just a frame count.

  Inputs (set by parent)
    • setFeedback(pos_rev, vel_rev_s, timestamp_us)
    • setPauseSeconds(seconds) — default 0.5 s
    • planThrow*(...) — multiple variants (see below)

  Outputs
    • HandPlanResult { time_to_ready_s, frame_count }
    • Optional streaming overloads planThrow*(v, emit) that call a callback
      per frame to avoid building a large vector.
    • Telemetry getters: last feedback, last plan duration, last frame count.

  Units
    • Position  : rev (motor revolutions)
    • Velocity  : rev/s
    • Torque FF : N·m (spool torque)
    • Time      : seconds

  Dependencies
    • Trajectory.h — provides:
        - struct Trajectory (arena-backed)
        - class TrajArena
        - bool makeSmoothMove(TrajArena&, Trajectory&, start, target)
        - class HandTrajGenerator(throwVel_mps)
        - TrajCfg::SAMPLE_RATE
        - TrajCfg::HAND_MAX_SMOOTH_POS
*/

#include <stdint.h>
#include <algorithm>
#include <cmath>
#include "TrajFrame.h"
#include "Trajectory.h"

struct HandPlanResult {
  float time_to_ready_s;             // smooth_move_duration + pause
  size_t frame_count;                // number of frames written to output buffer
};

class HandPathPlanner {
public:
  explicit HandPathPlanner(float sample_hz = (float)TrajCfg::SAMPLE_RATE);

  // Configuration setters (no Serial in this class)
  void setPauseSeconds(float s);

  // Provide a way to set FB when you want (also used by PV-at-call guard)
  void setFeedback(float pos_rev, float vel_rev_s, uint32_t timestamp_us);

  // Decel-zero planners (time origin at decel start)
  // Output frames written to out_buf[0..out_cap-1]. Returns result with frame_count.
  HandPlanResult planThrowDecelZero(float throw_vel_mps,
                                    TrajFrame* out_buf, size_t out_cap);
  float          planThrowDecelZero(float throw_vel_mps, void (*emit)(const TrajFrame&));

  // Build a throw plan and write frames to out_buf.
  // Times start at 0 = beginning of the plan (smooth/pause precede throw).
  HandPlanResult planThrow(float throw_vel_mps,
                           TrajFrame* out_buf, size_t out_cap);

  // Streaming variant of the above (time origin at plan start)
  float planThrow(float throw_vel_mps, void (*emit)(const TrajFrame&));

  // Build a plan with t_s = 0 at the INSTANT THE THROW DECELERATION BEGINS.
  HandPlanResult planThrowDecelZero(float throw_vel_mps,
                                    float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                                    TrajFrame* out_buf, size_t out_cap);
  float          planThrowDecelZero(float throw_vel_mps,
                                    float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                                    void (*emit)(const TrajFrame&));

  // Build a smooth move plan
  HandPlanResult planSmoothTo(float target_pos_rev,
                              float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                              TrajFrame* out_buf, size_t out_cap);
  float          planSmoothTo(float target_pos_rev,
                              float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                              void (*emit)(const TrajFrame&));

  // Telemetry getters
  float lastPosRev() const { return fb_pos_rev_; }
  float lastVelRevS() const { return fb_vel_rev_s_; }
  uint32_t lastFeedbackTimestampUs() const { return fb_ts_us_; }

  float lastTimeToReadyS() const { return last_time_to_ready_s_; }
  size_t lastFrameCount() const { return last_frame_count_; }

private:
  // Fixed sample settings (should match TrajCfg::SAMPLE_RATE)
  float fs_hz_;
  float dt_s_;
  float pause_s_;

  // Latest feedback
  float fb_pos_rev_;
  float fb_vel_rev_s_;
  uint32_t fb_ts_us_;

  // Last-plan telemetry
  float  last_time_to_ready_s_;
  size_t last_frame_count_;
  float  last_decel_time_in_throw_;  // Decel start time within intrinsic throw (cached for planThrowDecelZero)

  // Arena for intermediate Trajectory float arrays (reset per planning call)
  alignas(4) uint8_t arena_buf_[TrajCfg::TRAJ_ARENA_BYTES];
  TrajArena arena_;

  // Helpers
  static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }

  static inline float trajDuration(const Trajectory& tr) {
    return tr.empty() ? 0.0f : tr.t[tr.count - 1];
  }

  struct AnchorResult {
    float anchor_rev;
    float target_start_rev;
  };
  AnchorResult computeAnchor_(const Trajectory& throwTr) const;

  friend struct FBGuard;

  // Find index/time where deceleration begins in a throw trajectory
  static size_t findDecelStartIndex(const Trajectory& tr);

  // Append helpers — write to raw TrajFrame buffer
  size_t appendTrajectoryRebased(const Trajectory& tr,
                                 float desired_first_pos_rev,
                                 float t_start,
                                 TrajFrame* out, size_t out_cap) const;

  void emitTrajectoryRebased(const Trajectory& tr,
                             float desired_first_pos_rev,
                             float t_start,
                             void (*emit)(const TrajFrame&)) const;
  size_t appendPause(float pos_rev, float t_start, float duration,
                     TrajFrame* out, size_t out_cap) const;
  void emitPause(float pos_rev, float t_start, float duration,
                 void (*emit)(const TrajFrame&)) const;
};
