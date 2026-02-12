// CachedHandTrajectory.h
#pragma once

/*
  CachedHandTrajectory — Pre-computed trajectory cache for fixed hand moves
  -------------------------------------------------------------------------
  Purpose:
    For repetitive back-and-forth motion between two fixed positions, this class
    pre-computes and caches the trajectory frames so they don't need to be
    recalculated each cycle. Only the initial move (from arbitrary start position
    to the first target) needs on-the-fly computation.

  Workflow:
    1. Initial move: start_pos → target1 (computed on-the-fly via HandPathPlanner)
    2. Cached move A: target1 → target2 (pre-computed)
    3. Cached move B: target2 → target1 (pre-computed)
    4. Repeat steps 2-3...

  Acceleration handling:
    The cache stores the acceleration value used when trajectories were computed.
    If the max acceleration changes (via setMaxSmoothMoveHandAccel), call
    rebuildIfAccelChanged() to detect and rebuild the cache as needed.

  Usage:
    CachedHandTrajectory cache(8.5f, 1.0f);  // target1, target2
    cache.build();  // Pre-compute trajectories

    // For initial move (arbitrary start):
    auto plan = cache.planInitialMove(current_pos, vel, ts, planner);

    // For cached moves:
    const TrajFrame* frames = cache.getTrajectory1to2();
    size_t count = cache.getTrajectory1to2Count();

  Units:
    - Position: rev (motor revolutions)
    - Time: seconds
    - Acceleration: rev/s² (from Trajectory.h)
*/

#include <vector>
#include "TrajFrame.h"
#include "Trajectory.h"
#include "HandPathPlanner.h"

class CachedHandTrajectory {
public:
  // ============================================================================
  // Construction
  // ============================================================================

  /**
   * @param target1_rev First target position in revolutions
   * @param target2_rev Second target position in revolutions
   */
  CachedHandTrajectory(float target1_rev, float target2_rev);

  // ============================================================================
  // Cache Management
  // ============================================================================

  /**
   * Build (or rebuild) the cached trajectories using current max acceleration.
   * Call this at startup after setting the desired acceleration.
   */
  void build();

  /**
   * Check if acceleration has changed since last build, and rebuild if needed.
   * @return true if cache was rebuilt, false if unchanged
   */
  bool rebuildIfAccelChanged();

  /**
   * Force a rebuild of the cache regardless of acceleration state.
   */
  void forceRebuild() { build(); }

  /**
   * Check if the cache has been built and is ready to use.
   */
  bool isBuilt() const { return is_built_; }

  /**
   * Get the acceleration value used when the cache was built.
   */
  float getCachedAccel() const { return cached_accel_; }

  // ============================================================================
  // Target Position Accessors
  // ============================================================================

  float getTarget1() const { return target1_rev_; }
  float getTarget2() const { return target2_rev_; }

  void setTargets(float target1_rev, float target2_rev);

  // ============================================================================
  // Initial Move (On-the-fly computation)
  // ============================================================================

  /**
   * Plan the initial move from an arbitrary starting position to target1.
   * This is computed on-the-fly since the starting position varies.
   *
   * @param current_pos_rev Current hand position
   * @param vel_rev_s Current velocity (unused for smooth moves, but kept for API consistency)
   * @param timestamp_us Current timestamp
   * @param planner Reference to HandPathPlanner for computation
   * @return HandPlanResult with trajectory frames
   */
  HandPlanResult planInitialMove(float current_pos_rev,
                                  float vel_rev_s,
                                  uint32_t timestamp_us,
                                  HandPathPlanner& planner);

  // ============================================================================
  // Cached Trajectory Access
  // ============================================================================

  /**
   * Get pointer to cached trajectory: target1 → target2
   * @return Pointer to first frame, or nullptr if not built
   */
  const TrajFrame* getTrajectory1to2() const;

  /**
   * Get frame count for trajectory: target1 → target2
   */
  size_t getTrajectory1to2Count() const { return traj_1_to_2_.size(); }

  /**
   * Get duration of trajectory: target1 → target2
   */
  float getDuration1to2() const { return duration_1_to_2_s_; }

  /**
   * Get pointer to cached trajectory: target2 → target1
   * @return Pointer to first frame, or nullptr if not built
   */
  const TrajFrame* getTrajectory2to1() const;

  /**
   * Get frame count for trajectory: target2 → target1
   */
  size_t getTrajectory2to1Count() const { return traj_2_to_1_.size(); }

  /**
   * Get duration of trajectory: target2 → target1
   */
  float getDuration2to1() const { return duration_2_to_1_s_; }

  // ============================================================================
  // Telemetry
  // ============================================================================

  /**
   * Get total memory used by cached trajectories (bytes).
   */
  size_t getMemoryUsage() const {
    return (traj_1_to_2_.capacity() + traj_2_to_1_.capacity()) * sizeof(TrajFrame);
  }

private:
  // Target positions
  float target1_rev_;
  float target2_rev_;

  // Cached trajectories (stored as vectors for memory safety)
  std::vector<TrajFrame> traj_1_to_2_;
  std::vector<TrajFrame> traj_2_to_1_;

  // Trajectory durations
  float duration_1_to_2_s_ = 0.0f;
  float duration_2_to_1_s_ = 0.0f;

  // Cache state
  bool is_built_ = false;
  float cached_accel_ = 0.0f;  // Acceleration used when cache was built

  // Internal helper: convert Trajectory to TrajFrame vector with rebasing
  static void trajectoryToFrames(const Trajectory& tr,
                                  float start_pos_rev,
                                  std::vector<TrajFrame>& out);
};
