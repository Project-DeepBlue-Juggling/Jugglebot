// CachedHandTrajectory.cpp
#include "CachedHandTrajectory.h"

/* ============================================================================
   CONSTRUCTOR
   ============================================================================ */
CachedHandTrajectory::CachedHandTrajectory(float target1_rev, float target2_rev)
  : target1_rev_(target1_rev),
    target2_rev_(target2_rev),
    is_built_(false),
    cached_accel_(0.0f) {}

/* ============================================================================
   CACHE MANAGEMENT
   ============================================================================ */

void CachedHandTrajectory::build() {
  // Store the current acceleration for change detection
  cached_accel_ = getMaxSmoothMoveHandAccel();

  // Clear any existing cached data
  traj_1_to_2_.clear();
  traj_2_to_1_.clear();

  // Generate trajectory: target1 → target2
  {
    Trajectory tr = makeSmoothMove(target1_rev_, target2_rev_);
    if (!tr.x.empty()) {
      duration_1_to_2_s_ = tr.t.back();
      trajectoryToFrames(tr, target1_rev_, traj_1_to_2_);
    } else {
      duration_1_to_2_s_ = 0.0f;
    }
  }

  // Generate trajectory: target2 → target1
  {
    Trajectory tr = makeSmoothMove(target2_rev_, target1_rev_);
    if (!tr.x.empty()) {
      duration_2_to_1_s_ = tr.t.back();
      trajectoryToFrames(tr, target2_rev_, traj_2_to_1_);
    } else {
      duration_2_to_1_s_ = 0.0f;
    }
  }

  // Shrink vectors to fit (release any extra capacity)
  traj_1_to_2_.shrink_to_fit();
  traj_2_to_1_.shrink_to_fit();

  is_built_ = true;
}

bool CachedHandTrajectory::rebuildIfAccelChanged() {
  float current_accel = getMaxSmoothMoveHandAccel();
  
  // Use small epsilon for float comparison
  if (fabsf(current_accel - cached_accel_) > 1e-6f) {
    build();
    return true;
  }
  return false;
}

void CachedHandTrajectory::setTargets(float target1_rev, float target2_rev) {
  target1_rev_ = target1_rev;
  target2_rev_ = target2_rev;
  // Invalidate cache - must rebuild
  is_built_ = false;
}

/* ============================================================================
   INITIAL MOVE (ON-THE-FLY)
   ============================================================================ */

HandPlanResult CachedHandTrajectory::planInitialMove(float current_pos_rev,
                                                      float vel_rev_s,
                                                      uint32_t timestamp_us,
                                                      HandPathPlanner& planner) {
  // Check if cache needs rebuilding due to accel change
  rebuildIfAccelChanged();

  // Use the planner for on-the-fly computation to target1
  return planner.planSmoothTo(target1_rev_, current_pos_rev, vel_rev_s, timestamp_us);
}

/* ============================================================================
   CACHED TRAJECTORY ACCESS
   ============================================================================ */

const TrajFrame* CachedHandTrajectory::getTrajectory1to2() const {
  if (!is_built_ || traj_1_to_2_.empty()) {
    return nullptr;
  }
  return traj_1_to_2_.data();
}

const TrajFrame* CachedHandTrajectory::getTrajectory2to1() const {
  if (!is_built_ || traj_2_to_1_.empty()) {
    return nullptr;
  }
  return traj_2_to_1_.data();
}

/* ============================================================================
   INTERNAL HELPERS
   ============================================================================ */

void CachedHandTrajectory::trajectoryToFrames(const Trajectory& tr,
                                               float start_pos_rev,
                                               std::vector<TrajFrame>& out) {
  if (tr.x.empty()) return;

  const size_t n = tr.x.size();
  out.reserve(n);

  // The trajectory from makeSmoothMove already has absolute positions
  // starting from start_pos_rev, so we just need to copy the data
  // with appropriate time values.
  //
  // Note: Unlike HandPathPlanner::appendTrajectoryRebased, we store
  // RELATIVE times starting from 0, since the time offset is applied
  // when arming the streamer.

  const float t0 = tr.t.front();
  const float dt_s = 1.0f / SAMPLE_RATE;

  for (size_t i = 0; i < n; ++i) {
    TrajFrame f;
    // Use relative time (starting from near zero)
    // First frame gets a small offset to match HandPathPlanner behavior
    f.t_s = (i == 0) ? dt_s : (tr.t[i] - t0);
    f.pos_cmd = tr.x[i];
    f.vel_ff = tr.v[i];
    f.tor_ff = tr.tor[i];
    out.push_back(f);
  }
}
