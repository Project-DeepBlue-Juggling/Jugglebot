// HandPathPlanner.cpp
#include "HandPathPlanner.h"

/* ============================================================================
   CONSTRUCTOR
   ============================================================================
   Initializes the path planner with a given sample rate (default 500 Hz).
   
   Parameters:
     sample_hz - Trajectory sample rate in Hz (should match SAMPLE_RATE from
                 Trajectory.h, typically 500 Hz)
   
   Initializes:
     - Sample rate and delta-time (dt_s_)
     - Feedback state variables to zero
     - Telemetry counters to zero
   ============================================================================ */
HandPathPlanner::HandPathPlanner(float sample_hz)
: fs_hz_(sample_hz),
  dt_s_(1.0f / sample_hz),
  fb_pos_rev_(0.0f),
  fb_vel_rev_s_(0.0f),
  fb_ts_us_(0),
  last_time_to_ready_s_(0.0f),
  last_frame_count_(0) {}

/* ============================================================================
   FEEDBACK STATE SETTER
   ============================================================================
    Updates the current hand position and velocity feedback state.
   
   Parameters:
     pos_rev      - Current hand position in motor revolutions
     vel_rev_s    - Current hand velocity in revolutions per second
     timestamp_us - Timestamp in microseconds when this feedback was captured
   
   This feedback is used as the starting point for planning smooth moves.
   Call this before calling planSmoothTo() to ensure planning starts from
   the correct current state.

   ============================================================================
   TRAJECTORY REBASING - VECTOR OUTPUT
   ============================================================================
   Converts a Trajectory (from makeSmoothMove) into TrajFrame format and
   appends it to an output vector, with position rebasing and time shifting.
   
   Parameters:
     tr                   - Source trajectory with relative positions
     desired_first_pos_rev - Absolute position where trajectory should start
     t_start              - Time offset to add to all frame timestamps
     out                  - Output vector to append frames to
   
   What is "rebasing"?
     The trajectory from makeSmoothMove() has positions relative to its
     start (tr.x.front() is the first position). We need to convert these
     to absolute positions in the hand's coordinate system.
     
     base_rev = desired_first_pos_rev - tr.x.front()
     absolute_pos = base_rev + tr.x[i]
   
   Why the time adjustment?
     The first frame's timestamp is slightly adjusted (t_start + dt_s_) to
     ensure proper timing when concatenating multiple trajectory segments.
   ============================================================================ */
void HandPathPlanner::setFeedback(float pos_rev, float vel_rev_s, uint32_t timestamp_us) {
  fb_pos_rev_   = pos_rev;
  fb_vel_rev_s_ = vel_rev_s;
  fb_ts_us_     = timestamp_us;
}

/* ============================================================================
   FEEDBACK GUARD UTILITY (RAII pattern)
   ============================================================================
   Internal utility struct that temporarily overrides the planner's feedback
   state, then automatically restores the original state when it goes out of
   scope.
   
   Purpose:
     Allows planSmoothTo() to accept explicit pos/vel parameters while still
     using the internal fb_pos_rev_ for planning calculations, without
     permanently modifying the stored feedback state.
   
   Usage:
     {
       _FBGuard g(*this, new_pos, new_vel, new_ts);
       // Planning code here uses new_pos as fb_pos_rev_
     } // Original feedback automatically restored here
   ============================================================================ */
struct _FBGuard {
  HandPathPlanner& self;
  float p0, v0; uint32_t t0;
  _FBGuard(HandPathPlanner& s, float p, float v, uint32_t ts)
  : self(s), p0(s.fb_pos_rev_), v0(s.fb_vel_rev_s_), t0(s.fb_ts_us_) {
/* ============================================================================
   TRAJECTORY REBASING - STREAMING OUTPUT
   ============================================================================
   Same as appendTrajectoryRebased(), but instead of appending to a vector,
   calls a user-provided callback function for each frame.
   
   Parameters:
     tr                   - Source trajectory with relative positions
     desired_first_pos_rev - Absolute position where trajectory should start
     t_start              - Time offset to add to all frame timestamps
     emit                 - Callback function to receive each frame
   
   Benefits of streaming:
     - Avoids allocating a large vector in memory
     - Frames can be processed/transmitted immediately as they're generated
     - Useful for real-time trajectory streaming to motor controllers
   ============================================================================ */
    self.setFeedback(p, v, ts);
  }
  ~_FBGuard(){ self.fb_pos_rev_ = p0; self.fb_vel_rev_s_ = v0; self.fb_ts_us_ = t0; }
};

void HandPathPlanner::appendTrajectoryRebased(const Trajectory& tr,
                                              float desired_first_pos_rev,
                                              float t_start,
                                              std::vector<TrajFrame>& out) const {
  if (tr.x.empty()) return;
  const float t0       = tr.t.front();
  const float base_rev = desired_first_pos_rev - tr.x.front();
  float t_cursor       = t_start;

  for (size_t i = 0; i < tr.x.size(); ++i) {
    TrajFrame f;
    f.t_s     = (i == 0) ? (t_cursor + dt_s_) : (t_cursor + (tr.t[i] - t0));
    f.pos_cmd = base_rev + tr.x[i];
    f.vel_ff  = tr.v[i];
    f.tor_ff  = tr.tor[i];
    out.push_back(f);
  }
}

void HandPathPlanner::emitTrajectoryRebased(const Trajectory& tr,
                                            float desired_first_pos_rev,
                                            float t_start,
                                            void (*emit)(const TrajFrame&)) const {
  if (!emit || tr.x.empty()) return;
  const float t0       = tr.t.front();
  const float base_rev = desired_first_pos_rev - tr.x.front();
  float t_cursor       = t_start;

  for (size_t i = 0; i < tr.x.size(); ++i) {
    TrajFrame f;
    f.t_s     = (i == 0) ? (t_cursor + dt_s_) : (t_cursor + (tr.t[i] - t0));
    f.pos_cmd = base_rev + tr.x[i];
    f.vel_ff  = tr.v[i];
    f.tor_ff  = tr.tor[i];
    emit(f);
  }
}

/* -------------------------- Smooth Move Planner --------------------------- */
HandPlanResult HandPathPlanner::planSmoothTo(float target_pos_rev,
                                             float pos_rev, float vel_rev_s, uint32_t timestamp_us) {
  _FBGuard g(*this, pos_rev, vel_rev_s, timestamp_us);
  HandPlanResult out;
  Trajectory smoothTr = makeSmoothMove(fb_pos_rev_, target_pos_rev);
  if (smoothTr.x.empty()) {
    out.time_to_ready_s = 0.0f;
    last_time_to_ready_s_ = 0.0f;
    last_frame_count_ = 0;
    return out;
  }
  out.trajectory.reserve(smoothTr.x.size());
  appendTrajectoryRebased(smoothTr, /*desired_first_pos=*/fb_pos_rev_, /*t_start=*/0.0f, out.trajectory);
  out.time_to_ready_s = trajDuration(smoothTr);
  last_time_to_ready_s_ = out.time_to_ready_s;
  last_frame_count_     = out.trajectory.size();
  return out;
}

float HandPathPlanner::planSmoothTo(float target_pos_rev,
                                    float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                                    void (*emit)(const TrajFrame&)) {
  _FBGuard g(*this, pos_rev, vel_rev_s, timestamp_us);
  if (!emit) return 0.0f;
  Trajectory smoothTr = makeSmoothMove(fb_pos_rev_, target_pos_rev);
  if (smoothTr.x.empty()) {
    last_time_to_ready_s_ = 0.0f;
    last_frame_count_ = 0;
    return 0.0f;
  }
  emitTrajectoryRebased(smoothTr, /*desired_first_pos=*/fb_pos_rev_, /*t_start=*/0.0f, emit);
  last_time_to_ready_s_ = trajDuration(smoothTr);
  last_frame_count_     = smoothTr.x.size();
  return last_time_to_ready_s_;
}
