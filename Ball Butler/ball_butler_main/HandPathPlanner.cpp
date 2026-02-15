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
  last_frame_count_(0) {}

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
  const float x0       = throwTr.x.front();
  const auto  minmaxX  = std::minmax_element(throwTr.x.begin(), throwTr.x.end());
  const float throwMin = *minmaxX.first;
  const float throwMax = *minmaxX.second;
  const float anchorMin = -throwMin;
  const float anchorMax = HAND_MAX_SMOOTH_MOVE_POS - throwMax;
  const float desiredAnchor = fb_pos_rev_ - x0;
  const float anchor_rev    = clampf(desiredAnchor, anchorMin, anchorMax);
  const float target_start_rev = anchor_rev + x0;
  return { anchor_rev, target_start_rev };
}

/* --------------------------------------------------------------------------
   Core planner: returns vector (pre-reserved for minimal heap churn)
   -------------------------------------------------------------------------- */
HandPlanResult HandPathPlanner::planThrow(float throw_vel_mps) {
  HandPlanResult out;

  // 1) Build the intrinsic throw profile at 500 Hz using Trajectory.h
  HandTrajGenerator tg(throw_vel_mps);
  Trajectory throwTr = tg.makeThrow();

  if (throwTr.x.empty()) {
    out.time_to_ready_s = 0.0f;
    out.trajectory.clear();
    last_time_to_ready_s_ = out.time_to_ready_s;
    last_frame_count_     = 0;
    return out;
  }

  // 2) Choose anchor within bounds, minimizing pre-position distance
  const auto [anchor_rev, target_start_rev] = computeAnchor_(throwTr);

  // 3) Smooth move (if necessary) to the starting sample of the throw
  Trajectory smoothTr;
  if (std::abs(fb_pos_rev_ - target_start_rev) > 1e-4f) {
    smoothTr = makeSmoothMove(fb_pos_rev_, target_start_rev);
  }

  // 4) Pre-allocate output
  const size_t Ns = smoothTr.x.size();
  const size_t Np = (size_t) (pause_s_ * (float)SAMPLE_RATE + 0.5f);
  const size_t Nt = throwTr.x.size();
  out.trajectory.reserve(Ns + Np + Nt);

  // 5) Concatenate: smooth → pause → throw (times start at 0 at plan start)
  float t_cursor = 0.0f;
  if (!smoothTr.t.empty()) {
    appendTrajectoryRebased(smoothTr, /*desired_first_pos=*/fb_pos_rev_, /*t_start=*/0.0f, out.trajectory);
    t_cursor = smoothTr.t.back();
  }
  if (pause_s_ > 0.0f) {
    appendPause(/*pos_rev=*/target_start_rev, t_cursor, pause_s_, out.trajectory);
    t_cursor += pause_s_;
  }
  // First throw sample should equal (anchor_rev + throwTr.x.front())
  appendTrajectoryRebased(throwTr, /*desired_first_pos=*/anchor_rev + throwTr.x.front(),
                          /*t_start=*/t_cursor, out.trajectory);

    // 6) Fill summary telemetry
    out.time_to_ready_s    = trajDuration(smoothTr) + pause_s_;
    last_time_to_ready_s_  = out.time_to_ready_s;
    last_frame_count_      = out.trajectory.size();
    return out;
  }

/* --------------------------------------------------------------------------
   Streaming variant — emits frames, avoids building a big vector
   Returns: time_to_ready_s
   -------------------------------------------------------------------------- */
float HandPathPlanner::planThrow(float throw_vel_mps, void (*emit)(const TrajFrame&)) {
  // 1) Build throw
  HandTrajGenerator tg(throw_vel_mps);
  Trajectory throwTr = tg.makeThrow();
  if (throwTr.x.empty()) { last_time_to_ready_s_ = 0.0f; last_frame_count_ = 0; return 0.0f; }

  // 2) Anchor selection
  const auto [anchor_rev, target_start_rev] = computeAnchor_(throwTr);

  // 3) Smooth move (if needed)
  Trajectory smoothTr;
  if (std::abs(fb_pos_rev_ - target_start_rev) > 1e-4f) {
    smoothTr = makeSmoothMove(fb_pos_rev_, target_start_rev);
  }

  // Emit smooth
  float t_cursor = 0.0f;
  if (!smoothTr.t.empty()) {
    emitTrajectoryRebased(smoothTr, /*desired_first_pos=*/fb_pos_rev_, /*t_start=*/0.0f, emit);
    t_cursor = smoothTr.t.back();
  }
  if (pause_s_ > 0.0f) {
    emitPause(/*pos_rev=*/target_start_rev, t_cursor, pause_s_, emit);
    t_cursor += pause_s_;
  }
  emitTrajectoryRebased(throwTr, /*desired_first_pos=*/anchor_rev + throwTr.x.front(),
                        /*t_start=*/t_cursor, emit);

  // Telemetry
  last_time_to_ready_s_ = trajDuration(smoothTr) + pause_s_;
  last_frame_count_ = smoothTr.x.size() + (size_t)(pause_s_ * (float)SAMPLE_RATE + 0.5f) + throwTr.x.size();
  return last_time_to_ready_s_;
}

/* ------------------------ decel detection + helpers ------------------------- */
size_t HandPathPlanner::findDecelStartIndex(const Trajectory& tr) {
  if (tr.v.size() < 3) return 0; // degenerate → treat start as pivot
  // Find first index after a rising phase where velocity starts to drop
  const float eps = 1e-6f;
  // 1) Find a span of rising samples
  size_t i = 0;
  while (i + 1 < tr.v.size() && tr.v[i+1] >= tr.v[i] - eps) ++i; // climb to local max
  // i is at local maximum (or last rising point)
  // 2) Deceleration begins at the next index where v decreases
  if (i + 1 < tr.v.size()) return i + 1;
  // Fallback: use global max index
  size_t imax = std::distance(tr.v.begin(), std::max_element(tr.v.begin(), tr.v.end()));
  return (imax + 1 < tr.v.size()) ? (imax + 1) : imax;
}

/* ------------------------ helpers: append / emit ------------------------- */
void HandPathPlanner::appendTrajectoryRebased(const Trajectory& tr,
                                              float desired_first_pos_rev,
                                              float t_start,
                                              std::vector<TrajFrame>& out) const {
  if (tr.x.empty()) return;
  const float t0       = tr.t.front();
  const float base_rev = desired_first_pos_rev - tr.x.front(); // <-- rebase
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
  const float base_rev = desired_first_pos_rev - tr.x.front(); // <-- rebase
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

void HandPathPlanner::appendPause(float pos_rev, float t_start, float duration,
                                  std::vector<TrajFrame>& out) const {
  if (duration <= 0.0f) return;
  const size_t N = (size_t)(duration * fs_hz_ + 0.5f);
  float t = t_start;
  for (size_t i = 0; i < N; ++i) {
    t += dt_s_;
    TrajFrame f{t, pos_rev, 0.0f, 0.0f};
    out.push_back(f);
  }
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
                                                   float pos_rev, float vel_rev_s, uint32_t timestamp_us) {
  FBGuard g(*this, pos_rev, vel_rev_s, timestamp_us);
  return planThrowDecelZero(throw_vel_mps);
}

float HandPathPlanner::planThrowDecelZero(float throw_vel_mps,
                                          float pos_rev, float vel_rev_s, uint32_t timestamp_us,
                                          void (*emit)(const TrajFrame&)) {
  FBGuard g(*this, pos_rev, vel_rev_s, timestamp_us);
  return planThrowDecelZero(throw_vel_mps, emit);
}

HandPlanResult HandPathPlanner::planThrowDecelZero(float throw_vel_mps) {
  // Build the unshifted plan first to compute cursor timings
  HandPlanResult base = planThrow(throw_vel_mps);
  if (base.trajectory.empty()) return base;

  // To find where decel begins, regenerate the intrinsic throw
  HandTrajGenerator tg(throw_vel_mps);
  Trajectory throwTr = tg.makeThrow();
  const size_t i_decel = findDecelStartIndex(throwTr);
  const float t_decel_in_throw = throwTr.t.empty() ? 0.0f : throwTr.t[i_decel];

  // The throw segment in base begins at time t_throw_begin = time_to_ready_s
  const float t_throw_begin = last_time_to_ready_s_;
  const float t_zero_global = t_throw_begin + t_decel_in_throw; // make this -> 0

  for (auto& f : base.trajectory) f.t_s -= t_zero_global;

  return base; // last_time_to_ready_s_ remains (smooth + pause)
}

float HandPathPlanner::planThrowDecelZero(float throw_vel_mps,
                                          void (*emit)(const TrajFrame&)) {
  if (!emit) return 0.0f;

  // Build throw to discover decel index/time and anchor decisions
  HandTrajGenerator tg(throw_vel_mps);
  Trajectory throwTr = tg.makeThrow();
  if (throwTr.x.empty()) { last_time_to_ready_s_ = 0.0f; last_frame_count_ = 0; return 0.0f; }

  // Anchor selection (same as planThrow)
  const auto [anchor_rev, target_start_rev] = computeAnchor_(throwTr);

  // Smooth (if needed)
  Trajectory smoothTr;
  if (std::abs(fb_pos_rev_ - target_start_rev) > 1e-4f) {
    smoothTr = makeSmoothMove(fb_pos_rev_, target_start_rev);
  }

  // Precompute timings
  const float t_smooth = trajDuration(smoothTr);
  const float t_pause  = pause_s_;
  const size_t i_decel = findDecelStartIndex(throwTr);
  const float t_decel_in_throw = throwTr.t.empty() ? 0.0f : throwTr.t[i_decel];
  const float t_zero_global = t_smooth + t_pause + t_decel_in_throw;

  // Emit with a time shift so decel occurs at t=0
  float t_cursor = 0.0f;

  // smooth (with decel-zero time shift)
  if (!smoothTr.t.empty()) {
    const float shift = -t_zero_global;
    const float desired_first_pos = fb_pos_rev_;
    const float t0 = smoothTr.t.front();
    float t_start = 0.0f;
    const float base_rev = desired_first_pos - smoothTr.x.front();

    for (size_t i = 0; i < smoothTr.x.size(); ++i) {
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
    const float desired_first_pos = anchor_rev + throwTr.x.front();
    const float base_rev = desired_first_pos - throwTr.x.front();
    const float t0 = throwTr.t.front();
    float t_start = t_cursor;

    for (size_t i = 0; i < throwTr.x.size(); ++i) {
      TrajFrame f;
      f.t_s     = ((i == 0) ? (t_start + dt_s_) : (t_start + (throwTr.t[i] - t0))) + shift;
      f.pos_cmd = base_rev + throwTr.x[i];
      f.vel_ff  = throwTr.v[i];
      f.tor_ff  = throwTr.tor[i];
      emit(f);
    }
  }

  last_time_to_ready_s_ = t_smooth + t_pause;
  last_frame_count_     = smoothTr.x.size() + (size_t)(t_pause * (float)SAMPLE_RATE + 0.5f) + throwTr.x.size();
  return last_time_to_ready_s_;
}

/* -------------------------- Smooth Move Planner --------------------------- */
HandPlanResult HandPathPlanner::planSmoothTo(float target_pos_rev,
                                             float pos_rev, float vel_rev_s, uint32_t timestamp_us) {
  FBGuard g(*this, pos_rev, vel_rev_s, timestamp_us);
  // use existing smooth-only variant (if you added it) or inline:
  HandPlanResult out;
  Trajectory smoothTr = makeSmoothMove(fb_pos_rev_, target_pos_rev);
  if (smoothTr.x.empty()) { out.time_to_ready_s = 0.0f; last_time_to_ready_s_ = 0.0f; last_frame_count_ = 0; return out; }
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
  FBGuard g(*this, pos_rev, vel_rev_s, timestamp_us);
  if (!emit) return 0.0f;
  Trajectory smoothTr = makeSmoothMove(fb_pos_rev_, target_pos_rev);
  if (smoothTr.x.empty()) { last_time_to_ready_s_ = 0.0f; last_frame_count_ = 0; return 0.0f; }
  emitTrajectoryRebased(smoothTr, /*desired_first_pos=*/fb_pos_rev_, /*t_start=*/0.0f, emit);
  last_time_to_ready_s_ = trajDuration(smoothTr);
  last_frame_count_     = smoothTr.x.size();
  return last_time_to_ready_s_;
}


