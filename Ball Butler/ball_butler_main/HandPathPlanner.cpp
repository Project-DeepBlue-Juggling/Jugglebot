// HandPathPlanner.cpp
#include "HandPathPlanner.h"

HandPathPlanner::HandPathPlanner(float sample_hz)
: fs_hz_(sample_hz),
  dt_s_(1.0f / sample_hz),
  pause_s_(0.5f),
  fb_pos_rev_(0.0f),
  fb_vel_rev_s_(0.0f),
  fb_ts_us_(0),
  last_time_to_ready_s_(0.0f),
  last_frame_count_(0),
  last_decel_time_in_throw_(0.0f)
{
  arena_.init(arena_buf_, sizeof(arena_buf_));
}

void HandPathPlanner::setPauseSeconds(float s) { pause_s_ = (s < 0.0f) ? 0.0f : s; }

void HandPathPlanner::setFeedback(float pos_rev, float vel_rev_s, uint32_t timestamp_us) {
  fb_pos_rev_   = pos_rev;
  fb_vel_rev_s_ = vel_rev_s;
  fb_ts_us_     = timestamp_us;
}

// Utility to save/restore FB around a planning call
struct FBGuard {
  HandPathPlanner& self;
  float p0, v0; uint32_t t0;
  FBGuard(HandPathPlanner& s, float p, float v, uint32_t ts)
  : self(s), p0(s.fb_pos_rev_), v0(s.fb_vel_rev_s_), t0(s.fb_ts_us_) {
    self.setFeedback(p, v, ts);
  }
  ~FBGuard(){ self.fb_pos_rev_ = p0; self.fb_vel_rev_s_ = v0; self.fb_ts_us_ = t0; }
};

/* --------------------------------------------------------------------------
   Anchor selection — shared by all throw planners
   -------------------------------------------------------------------------- */
HandPathPlanner::AnchorResult HandPathPlanner::computeAnchor_(const Trajectory& throwTr) const {
  const float x0       = throwTr.x[0];
  const auto  minmaxX  = std::minmax_element(throwTr.x, throwTr.x + throwTr.count);
  const float throwMin = *minmaxX.first;
  const float throwMax = *minmaxX.second;
  const float anchorMin = -throwMin;
  const float anchorMax = TrajCfg::HAND_MAX_SMOOTH_POS - throwMax;
  const float desiredAnchor = fb_pos_rev_ - x0;
  const float anchor_rev    = clampf(desiredAnchor, anchorMin, anchorMax);
  const float target_start_rev = anchor_rev + x0;
  return { anchor_rev, target_start_rev };
}

/* --------------------------------------------------------------------------
   Core planner: writes frames into caller-provided buffer
   -------------------------------------------------------------------------- */
HandPlanResult HandPathPlanner::planThrow(float throw_vel_mps,
                                          TrajFrame* out_buf, size_t out_cap) {
  HandPlanResult out{0.0f, 0};
  arena_.reset();

  // 1) Build the intrinsic throw profile at 500 Hz using Trajectory.h
  HandTrajGenerator tg(throw_vel_mps);
  Trajectory throwTr;
  if (!tg.makeThrow(arena_, throwTr) || throwTr.empty()) {
    last_time_to_ready_s_ = 0.0f;
    last_frame_count_     = 0;
    return out;
  }

  // Cache decel start time for planThrowDecelZero() to avoid recomputing
  const size_t i_decel = findDecelStartIndex(throwTr);
  last_decel_time_in_throw_ = throwTr.t[i_decel];

  // 2) Choose anchor within bounds, minimizing pre-position distance
  const auto [anchor_rev, target_start_rev] = computeAnchor_(throwTr);

  // 3) Smooth move (if necessary) to the starting sample of the throw
  Trajectory smoothTr;
  if (std::abs(fb_pos_rev_ - target_start_rev) > 1e-4f) {
    makeSmoothMove(arena_, smoothTr, fb_pos_rev_, target_start_rev);
  }

  // 4) Concatenate: smooth → pause → throw (times start at 0 at plan start)
  size_t written = 0;
  float t_cursor = 0.0f;
  if (!smoothTr.empty()) {
    written += appendTrajectoryRebased(smoothTr, fb_pos_rev_, 0.0f,
                                       out_buf + written, out_cap - written);
    t_cursor = smoothTr.t[smoothTr.count - 1];
  }
  if (pause_s_ > 0.0f) {
    written += appendPause(target_start_rev, t_cursor, pause_s_,
                           out_buf + written, out_cap - written);
    t_cursor += pause_s_;
  }
  written += appendTrajectoryRebased(throwTr, anchor_rev + throwTr.x[0],
                                     t_cursor,
                                     out_buf + written, out_cap - written);

  // 5) Fill summary telemetry
  out.time_to_ready_s    = trajDuration(smoothTr) + pause_s_;
  out.frame_count        = written;
  last_time_to_ready_s_  = out.time_to_ready_s;
  last_frame_count_      = written;
  return out;
}

/* --------------------------------------------------------------------------
   Streaming variant — emits frames, avoids building a big vector
   Returns: time_to_ready_s
   -------------------------------------------------------------------------- */
float HandPathPlanner::planThrow(float throw_vel_mps, void (*emit)(const TrajFrame&)) {
  arena_.reset();

  // 1) Build throw
  HandTrajGenerator tg(throw_vel_mps);
  Trajectory throwTr;
  if (!tg.makeThrow(arena_, throwTr) || throwTr.empty()) {
    last_time_to_ready_s_ = 0.0f; last_frame_count_ = 0; return 0.0f;
  }

  // 2) Anchor selection
  const auto [anchor_rev, target_start_rev] = computeAnchor_(throwTr);

  // 3) Smooth move (if needed)
  Trajectory smoothTr;
  if (std::abs(fb_pos_rev_ - target_start_rev) > 1e-4f) {
    makeSmoothMove(arena_, smoothTr, fb_pos_rev_, target_start_rev);
  }

  // Emit smooth
  float t_cursor = 0.0f;
  if (!smoothTr.empty()) {
    emitTrajectoryRebased(smoothTr, fb_pos_rev_, 0.0f, emit);
    t_cursor = smoothTr.t[smoothTr.count - 1];
  }
  if (pause_s_ > 0.0f) {
    emitPause(target_start_rev, t_cursor, pause_s_, emit);
    t_cursor += pause_s_;
  }
  emitTrajectoryRebased(throwTr, anchor_rev + throwTr.x[0],
                        t_cursor, emit);

  // Telemetry
  last_time_to_ready_s_ = trajDuration(smoothTr) + pause_s_;
  last_frame_count_ = smoothTr.count + (size_t)(pause_s_ * (float)TrajCfg::SAMPLE_RATE + 0.5f) + throwTr.count;
  return last_time_to_ready_s_;
}

/* ------------------------ decel detection + helpers ------------------------- */
size_t HandPathPlanner::findDecelStartIndex(const Trajectory& tr) {
  if (tr.count < 3) return 0; // degenerate → treat start as pivot
  // Find first index after a rising phase where velocity starts to drop
  const float eps = 1e-6f;
  // 1) Find a span of rising samples
  size_t i = 0;
  while (i + 1 < tr.count && tr.v[i+1] >= tr.v[i] - eps) ++i; // climb to local max
  // i is at local maximum (or last rising point)
  // 2) Deceleration begins at the next index where v decreases
  if (i + 1 < tr.count) return i + 1;
  // Fallback: use global max index
  size_t imax = std::distance(tr.v, std::max_element(tr.v, tr.v + tr.count));
  return (imax + 1 < tr.count) ? (imax + 1) : imax;
}

/* ------------------------ helpers: append / emit ------------------------- */
size_t HandPathPlanner::appendTrajectoryRebased(const Trajectory& tr,
                                                float desired_first_pos_rev,
                                                float t_start,
                                                TrajFrame* out, size_t out_cap) const {
  if (tr.empty()) return 0;
  const float t0       = tr.t[0];
  const float base_rev = desired_first_pos_rev - tr.x[0];
  float t_cursor       = t_start;
  size_t written = 0;

  for (size_t i = 0; i < tr.count && written < out_cap; ++i) {
    TrajFrame f;
    f.t_s     = (i == 0) ? (t_cursor + dt_s_) : (t_cursor + (tr.t[i] - t0));
    f.pos_cmd = base_rev + tr.x[i];
    f.vel_ff  = tr.v[i];
    f.tor_ff  = tr.tor[i];
    out[written++] = f;
  }
  return written;
}

void HandPathPlanner::emitTrajectoryRebased(const Trajectory& tr,
                                            float desired_first_pos_rev,
                                            float t_start,
                                            void (*emit)(const TrajFrame&)) const {
  if (!emit || tr.empty()) return;
  const float t0       = tr.t[0];
  const float base_rev = desired_first_pos_rev - tr.x[0];
  float t_cursor       = t_start;

  for (size_t i = 0; i < tr.count; ++i) {
    TrajFrame f;
    f.t_s     = (i == 0) ? (t_cursor + dt_s_) : (t_cursor + (tr.t[i] - t0));
    f.pos_cmd = base_rev + tr.x[i];
    f.vel_ff  = tr.v[i];
    f.tor_ff  = tr.tor[i];
    emit(f);
  }
}

size_t HandPathPlanner::appendPause(float pos_rev, float t_start, float duration,
                                    TrajFrame* out, size_t out_cap) const {
  if (duration <= 0.0f) return 0;
  const size_t N = (size_t)(duration * fs_hz_ + 0.5f);
  float t = t_start;
  size_t written = 0;
  for (size_t i = 0; i < N && written < out_cap; ++i) {
    t += dt_s_;
    out[written++] = TrajFrame{t, pos_rev, 0.0f, 0.0f};
  }
  return written;
}

void HandPathPlanner::emitPause(float pos_rev, float t_start, float duration,
                                void (*emit)(const TrajFrame&)) const {
  if (!emit || duration <= 0.0f) return;
  const size_t N = (size_t)(duration * fs_hz_ + 0.5f);
  float t = t_start;
  for (size_t i = 0; i < N; ++i) {
    t += dt_s_;
    TrajFrame f{t, pos_rev, 0.0f, 0.0f};
    emit(f);
  }
}

/* ------------------------ decel-zero planners ------------------------ */

HandPlanResult HandPathPlanner::planThrowDecelZero(float throw_vel_mps,
                                                   float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                                                   TrajFrame* out_buf, size_t out_cap) {
  FBGuard g(*this, pos_rev, vel_rev_s, timestamp_us);
  return planThrowDecelZero(throw_vel_mps, out_buf, out_cap);
}

float HandPathPlanner::planThrowDecelZero(float throw_vel_mps,
                                          float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                                          void (*emit)(const TrajFrame&)) {
  FBGuard g(*this, pos_rev, vel_rev_s, timestamp_us);
  return planThrowDecelZero(throw_vel_mps, emit);
}

HandPlanResult HandPathPlanner::planThrowDecelZero(float throw_vel_mps,
                                                   TrajFrame* out_buf, size_t out_cap) {
  // Build the unshifted plan (planThrow caches last_decel_time_in_throw_)
  HandPlanResult base = planThrow(throw_vel_mps, out_buf, out_cap);
  if (base.frame_count == 0) return base;

  // Shift all frame times so that decel start = t=0
  const float t_zero_global = last_time_to_ready_s_ + last_decel_time_in_throw_;

  for (size_t i = 0; i < base.frame_count; ++i)
    out_buf[i].t_s -= t_zero_global;

  return base; // last_time_to_ready_s_ remains (smooth + pause)
}

float HandPathPlanner::planThrowDecelZero(float throw_vel_mps,
                                          void (*emit)(const TrajFrame&)) {
  if (!emit) return 0.0f;
  arena_.reset();

  // Build throw to discover decel index/time and anchor decisions
  HandTrajGenerator tg(throw_vel_mps);
  Trajectory throwTr;
  if (!tg.makeThrow(arena_, throwTr) || throwTr.empty()) {
    last_time_to_ready_s_ = 0.0f; last_frame_count_ = 0; return 0.0f;
  }

  // Anchor selection (same as planThrow)
  const auto [anchor_rev, target_start_rev] = computeAnchor_(throwTr);

  // Smooth (if needed)
  Trajectory smoothTr;
  if (std::abs(fb_pos_rev_ - target_start_rev) > 1e-4f) {
    makeSmoothMove(arena_, smoothTr, fb_pos_rev_, target_start_rev);
  }

  // Precompute timings
  const float t_smooth = trajDuration(smoothTr);
  const float t_pause  = pause_s_;
  const size_t i_decel = findDecelStartIndex(throwTr);
  const float t_decel_in_throw = throwTr.t[i_decel];
  const float t_zero_global = t_smooth + t_pause + t_decel_in_throw;

  // Emit with a time shift so decel occurs at t=0
  float t_cursor = 0.0f;

  // smooth (with decel-zero time shift)
  if (!smoothTr.empty()) {
    const float shift = -t_zero_global;
    const float desired_first_pos = fb_pos_rev_;
    const float t0 = smoothTr.t[0];
    float t_start = 0.0f;
    const float base_rev = desired_first_pos - smoothTr.x[0];

    for (size_t i = 0; i < smoothTr.count; ++i) {
      TrajFrame f;
      f.t_s     = ((i == 0) ? (t_start + dt_s_) : (t_start + (smoothTr.t[i] - t0))) + shift;
      f.pos_cmd = base_rev + smoothTr.x[i];
      f.vel_ff  = smoothTr.v[i];
      f.tor_ff  = smoothTr.tor[i];
      emit(f);
    }
    t_cursor = t_smooth;
  }

  if (t_pause > 0.0f) {
    float t = t_cursor;
    for (size_t i = 0, N=(size_t)(t_pause*fs_hz_ + 0.5f); i<N; ++i) {
      t += dt_s_;
      TrajFrame f{t - t_zero_global, target_start_rev, 0.0f, 0.0f};
      emit(f);
    }
    t_cursor += t_pause;
  }

  // Emit throw
  {
    const float shift = -t_zero_global;
    const float desired_first_pos = anchor_rev + throwTr.x[0];
    const float base_rev = desired_first_pos - throwTr.x[0];
    const float t0 = throwTr.t[0];
    float t_start = t_cursor;

    for (size_t i = 0; i < throwTr.count; ++i) {
      TrajFrame f;
      f.t_s     = ((i == 0) ? (t_start + dt_s_) : (t_start + (throwTr.t[i] - t0))) + shift;
      f.pos_cmd = base_rev + throwTr.x[i];
      f.vel_ff  = throwTr.v[i];
      f.tor_ff  = throwTr.tor[i];
      emit(f);
    }
  }

  last_time_to_ready_s_ = t_smooth + t_pause;
  last_frame_count_     = smoothTr.count + (size_t)(t_pause * (float)TrajCfg::SAMPLE_RATE + 0.5f) + throwTr.count;
  return last_time_to_ready_s_;
}

/* -------------------------- Smooth Move Planner --------------------------- */
HandPlanResult HandPathPlanner::planSmoothTo(float target_pos_rev,
                                             float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                                             TrajFrame* out_buf, size_t out_cap) {
  FBGuard g(*this, pos_rev, vel_rev_s, timestamp_us);
  HandPlanResult out{0.0f, 0};
  arena_.reset();

  Trajectory smoothTr;
  makeSmoothMove(arena_, smoothTr, fb_pos_rev_, target_pos_rev);
  if (smoothTr.empty()) {
    last_time_to_ready_s_ = 0.0f;
    last_frame_count_ = 0;
    return out;
  }

  size_t written = appendTrajectoryRebased(smoothTr, fb_pos_rev_, 0.0f,
                                           out_buf, out_cap);
  out.time_to_ready_s = trajDuration(smoothTr);
  out.frame_count     = written;
  last_time_to_ready_s_ = out.time_to_ready_s;
  last_frame_count_     = written;
  return out;
}

float HandPathPlanner::planSmoothTo(float target_pos_rev,
                                    float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                                    void (*emit)(const TrajFrame&)) {
  FBGuard g(*this, pos_rev, vel_rev_s, timestamp_us);
  if (!emit) return 0.0f;
  arena_.reset();

  Trajectory smoothTr;
  makeSmoothMove(arena_, smoothTr, fb_pos_rev_, target_pos_rev);
  if (smoothTr.empty()) {
    last_time_to_ready_s_ = 0.0f; last_frame_count_ = 0; return 0.0f;
  }

  emitTrajectoryRebased(smoothTr, fb_pos_rev_, 0.0f, emit);
  last_time_to_ready_s_ = trajDuration(smoothTr);
  last_frame_count_     = smoothTr.count;
  return last_time_to_ready_s_;
}
