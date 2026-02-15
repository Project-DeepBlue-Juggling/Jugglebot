// HandPathPlanner.h
#pragma once

/*
  HandPathPlanner — pure planning class (no Serial, no CAN)
  ---------------------------------------------------------
  Purpose
    • Given the current feedback state (position, velocity) and a desired
      linear throw velocity [m/s], generate a full hand trajectory sampled
      at 500 Hz: optional smooth move → configurable pause → throw.

  Inputs (set by parent)
    • setFeedback(pos_rev, vel_rev_s, timestamp_us)
    • setPauseSeconds(seconds) — default 0.5 s
    • planThrow*(...) — multiple variants (see below)

  Outputs
    • HandPlanResult { time_to_ready_s, trajectory<vector<TrajFrame>> }
    • Optional streaming overloads planThrow*(v, emit) that call a callback
      per frame to avoid building a large vector.
    • Telemetry getters: last feedback, last plan duration, last frame count.

  Units
    • Position  : rev (motor revolutions)
    • Velocity  : rev/s
    • Torque FF : N·m (spool torque)
    • Time      : seconds

  Dependencies
    • Trajectory.h (unchanged) — provides:
        - struct Trajectory { std::vector<float> t, x, v, tor; };
        - inline Trajectory makeSmoothMove(float start_rev, float target_rev);
        - class  HandTrajGenerator(throwVel_mps).makeThrow();
        - constexpr int SAMPLE_RATE = 500;
        - constexpr float HAND_MAX_SMOOTH_MOVE_POS;
*/

#include <stdint.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include "TrajFrame.h"
#include "Trajectory.h"

struct HandPlanResult {
  float time_to_ready_s;             // smooth_move_duration + pause
  std::vector<TrajFrame> trajectory; // concatenated frames at 500 Hz
};

class HandPathPlanner {
public:
  explicit HandPathPlanner(float sample_hz = (float)SAMPLE_RATE);

  // Configuration setters (no Serial in this class)
  void setPauseSeconds(float s);

  // Provide a way to set FB when you want (also used by PV-at-call guard)
  void setFeedback(float pos_rev, float vel_rev_s, uint32_t timestamp_us);
  
  // Decel-zero planners (time origin at decel start)
  HandPlanResult planThrowDecelZero(float throw_vel_mps);
  float          planThrowDecelZero(float throw_vel_mps, void (*emit)(const TrajFrame&));

  // Build a throw plan and return all frames in a vector.
  // Times start at 0 = beginning of the plan (smooth/pause precede throw).
  HandPlanResult planThrow(float throw_vel_mps);

  // Streaming variant of the above (time origin at plan start)
  float planThrow(float throw_vel_mps, void (*emit)(const TrajFrame&));

  // Build a plan with t_s = 0 at the INSTANT THE THROW DECELERATION BEGINS.
  // This means smooth and pause (and early throw acceleration) have negative t_s.
  HandPlanResult planThrowDecelZero(float throw_vel_mps,
                                    float pos_rev, float vel_rev_s, uint32_t timestamp_us);
  float          planThrowDecelZero(float throw_vel_mps,
                                    float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                                    void (*emit)(const TrajFrame&));

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
  // Fixed sample settings (should match Trajectory.h SAMPLE_RATE)
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

  // Helpers
  static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }

  static inline float trajDuration(const Trajectory& tr) {
    return tr.t.empty() ? 0.0f : tr.t.back();
  }

  struct AnchorResult {
    float anchor_rev;
    float target_start_rev;
  };
  AnchorResult computeAnchor_(const Trajectory& throwTr) const;

  friend struct FBGuard;

  // Find index/time where deceleration begins in a throw trajectory
  static size_t findDecelStartIndex(const Trajectory& tr);

  // Append helpers
  void appendTrajectoryRebased(const Trajectory& tr,
                               float desired_first_pos_rev,
                               float t_start,
                               std::vector<TrajFrame>& out) const;

  void emitTrajectoryRebased(const Trajectory& tr,
                             float desired_first_pos_rev,
                             float t_start,
                             void (*emit)(const TrajFrame&)) const;
  void appendPause(float pos_rev, float t_start, float duration,
                   std::vector<TrajFrame>& out) const;
  void emitPause(float pos_rev, float t_start, float duration,
                 void (*emit)(const TrajFrame&)) const;
};

