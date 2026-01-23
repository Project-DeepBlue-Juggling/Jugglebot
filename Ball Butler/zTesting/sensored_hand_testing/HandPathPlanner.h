// HandPathPlanner.h
#pragma once

/*
  HandPathPlanner — pure planning class for smooth moves
  -------------------------------------------------------
  Purpose
    • Given the current feedback state (position, velocity),
      generate a smooth point-to-point hand trajectory sampled at 500 Hz.

  Inputs (set by parent)
    • setFeedback(pos_rev, vel_rev_s, timestamp_us)
    • planSmoothTo(...) — smooth move variants (see below)

  Outputs
    • HandPlanResult { time_to_ready_s, trajectory<vector<TrajFrame>> }
    • Optional streaming overload planSmoothTo(target, pos, vel, ts, emit) 
      that calls a callback per frame to avoid building a large vector.
    • Telemetry getters: last feedback, last plan duration, last frame count.

  Units
    • Position  : rev (motor revolutions)
    • Velocity  : rev/s (not used for smooth moves)
    • Time      : seconds

  Dependencies
    • Trajectory.h — provides:
        - struct Trajectory { std::vector<float> t, x, v, tor; };
        - inline Trajectory makeSmoothMove(float start_rev, float target_rev);
        - constexpr int SAMPLE_RATE = 500;
    • TrajFrame.h
*/

#include <stdint.h>
#include <vector>
#include <cmath>
#include "TrajFrame.h"
#include "Trajectory.h"

struct HandPlanResult {
  float time_to_ready_s;             // duration of smooth move
  std::vector<TrajFrame> trajectory; // frames at 500 Hz
};

class HandPathPlanner {
public:
  explicit HandPathPlanner(float sample_hz = (float)SAMPLE_RATE);

  // Provide feedback state
  void setFeedback(float pos_rev, float vel_rev_s, uint32_t timestamp_us);
  
  // Build a smooth move plan
  HandPlanResult planSmoothTo(float target_pos_rev,
                              float pos_rev, float vel_rev_s, uint32_t timestamp_us);
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
  // Fixed sample settings
  float fs_hz_;
  float dt_s_;

  // Latest feedback
  float fb_pos_rev_;
  float fb_vel_rev_s_;
  uint32_t fb_ts_us_;

  // Last-plan telemetry
  float  last_time_to_ready_s_;
  size_t last_frame_count_;

  // Helpers
  static inline float trajDuration(const Trajectory& tr) {
    return tr.t.empty() ? 0.0f : tr.t.back();
  }

  friend struct _FBGuard;

  // Append helpers
  void appendTrajectoryRebased(const Trajectory& tr,
                               float desired_first_pos_rev,
                               float t_start,
                               std::vector<TrajFrame>& out) const;

  void emitTrajectoryRebased(const Trajectory& tr,
                             float desired_first_pos_rev,
                             float t_start,
                             void (*emit)(const TrajFrame&)) const;
};
