/*  Smooth point-to-point hand move — Teensy 4.0
    For generating quintic S-curve trajectories
*/
#pragma once

#include <Arduino.h>
#include <vector>
#include <cmath>

/* ───────── data container ───────── */
struct Trajectory {
  std::vector<float> t, x, v, tor;
};

/* ───────── constants ───────── */
constexpr float HAND_SPOOL_R        = 0.0052493f; // m (Max pos = 9.399  rev, min pos = 0 rev, travel = 310 mm)
constexpr float LINEAR_GAIN_FACTOR  = 1.0f;     // Just 'cuz
constexpr float LINEAR_GAIN         = LINEAR_GAIN_FACTOR / (M_PI * HAND_SPOOL_R * 2.f);  // rev per metre
constexpr float INERTIA_HAND_ONLY   = 0.281f;   // kg
constexpr int   SAMPLE_RATE         = 500;      // Hz

/* ----- smooth-move tuning ------------------------------------- */
constexpr float DEFAULT_SMOOTH_MOVE_HAND_ACCEL = 200.0f;   // [rev s⁻²] default value
constexpr float QUINTIC_S2_MAX                 = 5.7735027f; // max |s''| for 10t³−15t⁴+6t⁵

constexpr float HAND_MAX_SMOOTH_MOVE_POS = 8.9; // rev

inline float accelToTorque(float a) { return a * INERTIA_HAND_ONLY * HAND_SPOOL_R; }

/* ----- runtime-adjustable max accel --------------------------- */
inline float& _maxSmoothMoveHandAccelRef() {
    static float value = DEFAULT_SMOOTH_MOVE_HAND_ACCEL;
    return value;
}

inline bool setMaxSmoothMoveHandAccel(float a_max_rev_s2) {
    if (a_max_rev_s2 <= 0.0f) return false;  // reject non-positive values
    _maxSmoothMoveHandAccelRef() = a_max_rev_s2;
    return true;
}

inline float getMaxSmoothMoveHandAccel() {
    return _maxSmoothMoveHandAccelRef();
}

/* =====================================================================
   Smooth point-to-point move
   ─────────────────────────────────────────────────────────────────────
   * start position  = start_rev (argument)
   * end   position  = target_rev   (argument)
   * boundary cond.  = v=a=0 at both ends   (quintic "S-curve")
   * duration chosen such that  |a_max| ≤ MAX_SMOOTH_MOVE_HAND_ACCEL
   ===================================================================== */
inline Trajectory makeSmoothMove(float start_rev, float target_rev)
{
    Trajectory tr;

    if (target_rev > HAND_MAX_SMOOTH_MOVE_POS){
      target_rev = HAND_MAX_SMOOTH_MOVE_POS;
    } else if (target_rev < 0.0){
      target_rev = 0.0;
    }
    const float delta_rev = target_rev - start_rev;

    if (fabsf(delta_rev) < 1e-6f)          // already there → empty traj.
        return tr;

    /* ----- derive duration from accel limit ------------------------- *
     * pos(t) = delta_rev · s(τ) + start_rev ,   τ = t / T
     * a(t)   = delta_rev / T² · s''(τ)
     * so  |a_max| = |delta_rev| · QUINTIC_S2_MAX / T²  ≤  A_max
     */
    const float maxAccel = getMaxSmoothMoveHandAccel();
    const float T = sqrtf(fabsf(delta_rev) * QUINTIC_S2_MAX / maxAccel);

    /* guard against silly small values (numerics, rounding, …) */
    const float duration  = fmaxf(T, 0.05f);

    const float dT   = 1.0f / SAMPLE_RATE;
    const float invT = 1.0f / duration;
    const float invT2 = invT * invT;

    /* pre-allocate for speed */
    const size_t N = (size_t)ceilf(duration * SAMPLE_RATE) + 1;
    tr.t.reserve  (N);
    tr.x.reserve  (N);
    tr.v.reserve  (N);
    tr.tor.reserve(N);

    for (float t = 0.0f; t <= duration; t += dT) {

        const float tau = t * invT;                // 0 … 1
        const float tau2 = tau * tau;
        const float tau3 = tau2 * tau;
        const float tau4 = tau2 * tau2;

        /*  s(τ)   = 10τ³ − 15τ⁴ + 6τ⁵  */
        const float s_pos = 10.0f*tau3 - 15.0f*tau4 + 6.0f*tau4*tau;
        /*  s'(τ)  = 30τ² − 60τ³ + 30τ⁴ */
        const float s_vel = 30.0f*tau2 - 60.0f*tau3 + 30.0f*tau4;
        /*  s''(τ) = 60τ − 180τ² + 120τ³ */
        const float s_acc = 60.0f*tau - 180.0f*tau2 + 120.0f*tau3;

        const float pos_rev = start_rev + delta_rev * s_pos;
        const float vel_rev = delta_rev * s_vel * invT;
        const float acc_rev = delta_rev * s_acc * invT2;

        const float acc_lin = acc_rev / LINEAR_GAIN;        // [m/s²]
        const float tor     = accelToTorque(acc_lin);       // [N·m]

        tr.t  .push_back(t);
        tr.x  .push_back(pos_rev);
        tr.v  .push_back(vel_rev);
        tr.tor.push_back(tor);
    }

    return tr;
}
