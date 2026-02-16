#pragma once
/*  Hand‑trajectory generator — Teensy 4.0
    Heavily based on code written by Jon Beno, May 13 2024
    Harrison Low · Aug 2025
    ------------------------------------------------------------
    Generates three motion profiles sampled at 500 Hz:
      • makeThrow()  – forward throw
      • makeCatch()  – reverse catch
      • makeFull()   – throw + flight + catch
    Returned Trajectory:
      t   [s]   — time since trajectory‑zero
      x   [rev] — hand winch position (motor revs)
      v   [rev/s]
      tor [N·m] — spool torque
*/

#include <Arduino.h>
#include <vector>
#include <cmath>
#include "BallButlerConfig.h"

/* ───────── data container ───────── */
struct Trajectory {
  std::vector<float> t, x, v, tor;
};

/* ───────── constants (aliases to BallButlerConfig.h / TrajCfg::) ───────── */
constexpr float G                   = TrajCfg::G;
constexpr float HAND_SPOOL_R        = TrajCfg::HAND_SPOOL_R;
constexpr float LINEAR_GAIN_FACTOR  = TrajCfg::LINEAR_GAIN_FACTOR;
constexpr float LINEAR_GAIN         = TrajCfg::LINEAR_GAIN;
constexpr float INERTIA_HAND_ONLY   = TrajCfg::INERTIA_HAND_ONLY;
constexpr float INERTIA_RATIO       = TrajCfg::INERTIA_RATIO;
constexpr float THROW_VEL_HOLD_PCT  = TrajCfg::THROW_VEL_HOLD_PCT;
constexpr float CATCH_VEL_RATIO     = TrajCfg::CATCH_VEL_RATIO;
constexpr float CATCH_VEL_HOLD_PCT  = TrajCfg::CATCH_VEL_HOLD_PCT;
constexpr float HAND_STROKE         = TrajCfg::HAND_STROKE;
constexpr float STROKE_MARGIN       = TrajCfg::STROKE_MARGIN;
constexpr float END_PROFILE_HOLD    = TrajCfg::END_PROFILE_HOLD;
constexpr int   SAMPLE_RATE         = TrajCfg::SAMPLE_RATE;

/* ----- smooth-move tuning ------------------------------------- */
constexpr float MAX_SMOOTH_MOVE_HAND_ACCEL = TrajCfg::MAX_SMOOTH_ACCEL;
constexpr float QUINTIC_S2_MAX             = TrajCfg::QUINTIC_S2_MAX;

constexpr float HAND_MAX_SMOOTH_MOVE_POS = TrajCfg::HAND_MAX_SMOOTH_POS;

inline float accelToTorque(float a) { return a * INERTIA_HAND_ONLY * HAND_SPOOL_R; }

/* ───────── helper to shift whole trajectory in time ───────── */
inline void shiftTime(Trajectory& tr, float offset)
{
  for (float& tt : tr.t) tt += offset;
}

/* ───────── trajectory generator class ───────── */
class HandTrajGenerator {
public:
  explicit HandTrajGenerator(float throwVel)
      : v_throw(throwVel),
        throwHeight(v_throw * v_throw / (2.f * G)),
        totalStroke(HAND_STROKE - 2.f * STROKE_MARGIN),
        deltaT(1.f / SAMPLE_RATE)
  {
    calcThrow();
    calcCatch();
  }

  /* ----- public builders ----- */
  Trajectory makeThrow() {
    Trajectory tr = buildThrow();
    shiftTime(tr, -t2);               // set t=0 at start of decel
    return tr;
  }
  Trajectory makeCatch() {
    Trajectory tr = buildCatch();
    shiftTime(tr, -(t5 - t4));        // t=0 at mid velocity‑hold
    return tr;
  }
  Trajectory makeFull()  {
    Trajectory tr = buildCommand();
    shiftTime(tr, -t2);               // align to throw timeline
    return tr;
  }

private:
  /* ----- member variables ----- */
  float v_throw, throwHeight, totalStroke, deltaT;
  float throwA, throwD, t1, t2, t3;
  float catchA, catchD, t4, t5, t6, t7, t8;
  float x1, x2, x3, x5, x6, airT;

  /* ----- internal helpers ----- */
  void calcThrow()
  {
    airT = 2.f * v_throw / G;
    float velHold = THROW_VEL_HOLD_PCT * totalStroke;
    float accelSt = totalStroke - velHold;

    float t_acc = 2.f / (INERTIA_RATIO + 1.f) * accelSt / v_throw;
    float t_vel = velHold / v_throw;
    float t_dec = t_acc * INERTIA_RATIO;

    throwA =  v_throw / t_acc;
    throwD = -throwA / INERTIA_RATIO;

    t1 = t_acc;
    t2 = t1 + t_vel;
    t3 = t2 + t_dec;

    x1 = 0.5f * throwA * t_acc * t_acc;
    x2 = x1   + v_throw * t_vel;
    x3 = x2   + v_throw * t_dec + 0.5f * throwD * t_dec * t_dec;
  }

  void calcCatch()
  {
    float vC   = -CATCH_VEL_RATIO * v_throw;
    float irC  = 1.f / INERTIA_RATIO;
    float velH = CATCH_VEL_HOLD_PCT * totalStroke;
    float accS = totalStroke - velH;

    float t_acc = -(2.f / (irC + 1.f)) * accS / vC;
    float t_vel = -velH / vC;
    float t_dec =  t_acc * irC;

    catchA =  vC / t_acc;
    catchD = -catchA / irC;

    t5 = t2 + airT - 0.5f * t_vel;   // centre of vel hold
    t4 = t5 - t_acc;
    t6 = t5 + t_vel;
    t7 = t6 + t_dec;
    t8 = t7 + END_PROFILE_HOLD;

    x5 = x3 + 0.5f * catchA * t_acc * t_acc;
    x6 = x5 + vC * t_vel;
  }

  /* generic 3‑segment builder (a–v–a or v–a–v etc.) */
  Trajectory buildSegment(float start,
                          const float tA[4], const float xA[4],
                          const float vA[4], const float aA[4])
  {
    Trajectory tr;
    unsigned idx = 0;
    float   t    = 0.f;
    float   end  = tA[3] - start;

    while (t < end) {
      while (t > (tA[idx + 1] - start)) ++idx;
      float tau = (start + t) - tA[idx];
      float pos = xA[idx] + vA[idx] * tau + 0.5f * aA[idx] * tau * tau;
      float vel = vA[idx] + aA[idx] * tau;

      tr.t  .push_back(t);
      tr.x  .push_back(pos * LINEAR_GAIN);
      tr.v  .push_back(vel * LINEAR_GAIN);
      tr.tor.push_back(accelToTorque(aA[idx]));
      t += deltaT;
    }
    return tr;
  }

  /* ----- concrete builders ----- */
  Trajectory buildThrow()
  {
    float tA[4] = {0.f, t1, t2, t3};
    float xA[4] = {0.f, x1, x2, x3};
    float vA[4] = {0.f, v_throw, v_throw, 0.f};
    float aA[4] = {throwA, 0.f, throwD, 0.f};
    return buildSegment(0.f, tA, xA, vA, aA);
  }

  Trajectory buildCatch()
  {
    float vC   = -CATCH_VEL_RATIO * v_throw;
    float tA[4] = {t4, t5, t6, t7};
    float xA[4] = {x3, x5, x6, 0.f};
    float vA[4] = {0.f, vC, vC, 0.f};
    float aA[4] = {catchA, 0.f, catchD, 0.f};
    return buildSegment(t4, tA, xA, vA, aA);
  }

  /* full 9‑segment throw‑flight‑catch */
  Trajectory buildCommand()
  {
    const char  typ[9] = {'a','v','a','x','a','v','a','x','e'};
    const float tA[9]  = {0.f,t1,t2,t3,t4,t5,t6,t7,t8};
    const float xA[9]  = {0.f,x1,x2,x3,x3,x5,x6,0.f,0.f};
    const float vA[9]  = {0.f, v_throw, v_throw, 0.f,
                          0.f,-CATCH_VEL_RATIO*v_throw,
                          -CATCH_VEL_RATIO*v_throw, 0.f, 0.f};
    const float aA[9]  = {throwA,0.f,throwD,0.f,
                          catchA,0.f,catchD,0.f,0.f};

    Trajectory tr;
    unsigned idx = 0;
    float t = 0.f;
    const float dT = 1.f / SAMPLE_RATE;

    while (t < t8) {
      while (t > tA[idx + 1]) ++idx;
      float tau = t - tA[idx];

      float acc, vel, pos;
      switch (typ[idx]) {
        case 'a':
          acc = aA[idx];
          vel = vA[idx] + acc * tau;
          pos = xA[idx] + vA[idx] * tau + 0.5f * acc * tau * tau;
          break;
        case 'v':
          acc = 0.f;
          vel = vA[idx];
          pos = xA[idx] + vel * tau;
          break;
        default:              /* 'x' or 'e' */
          acc = vel = 0.f;
          pos = xA[idx];
          break;
      }
      tr.t  .push_back(t);
      tr.x  .push_back(pos * LINEAR_GAIN);
      tr.v  .push_back(vel * LINEAR_GAIN);
      tr.tor.push_back(accelToTorque(acc * LINEAR_GAIN));
      t += dT;
    }
    return tr;
  }
};

/* =====================================================================
   Smooth point-to-point move
   ---------------------------------------------------------------------
   * start position  = start_rev (argument)
   * end   position  = target_rev   (argument)
   * boundary cond.  = v=a=0 at both ends   (quintic “S-curve”)
   * duration chosen such that  |a_max| ≤ MAX_SMOOTH_MOVE_HAND_ACCEL
   ===================================================================== */
inline Trajectory makeSmoothMove(float start_rev, float target_rev)
{
    Trajectory tr;

    if (target_rev > HAND_MAX_SMOOTH_MOVE_POS){
      target_rev = HAND_MAX_SMOOTH_MOVE_POS;
    } else if (target_rev < 0.0f){
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
    const float T = sqrtf(fabsf(delta_rev) * QUINTIC_S2_MAX / MAX_SMOOTH_MOVE_HAND_ACCEL);

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

