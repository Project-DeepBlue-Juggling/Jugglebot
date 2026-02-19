#pragma once
/*  Hand‑trajectory generator — Teensy 4.0
    Heavily based on code written by Jon Beno, May 13 2024
    Harrison Low · Aug 2025
    ------------------------------------------------------------
    Generates three motion profiles sampled at 500 Hz:
      • makeThrow()  – forward throw
      • makeCatch()  – reverse catch
      • makeFull()   – throw + flight + catch
    Returned Trajectory:
      t   [s]   — time since trajectory‑zero
      x   [rev] — hand winch position (motor revs)
      v   [rev/s]
      tor [N·m] — spool torque

    P2-1 REFACTOR: All trajectory storage is now arena-backed.
    No heap allocations occur during trajectory planning.
*/

#include <Arduino.h>
#include <cmath>
#include <algorithm>
#include "BallButlerConfig.h"

/* ───────── bump allocator for trajectory working memory ───────── */
class TrajArena {
public:
  void init(uint8_t* buf, size_t cap) { buf_ = buf; cap_ = cap; offset_ = 0; }
  void reset() { offset_ = 0; }

  float* allocFloats(size_t n) {
    // Align to 4-byte boundary
    size_t aligned = (offset_ + 3u) & ~3u;
    size_t needed = aligned + n * sizeof(float);
    if (needed > cap_) return nullptr;
    float* p = reinterpret_cast<float*>(buf_ + aligned);
    offset_ = needed;
    return p;
  }

  size_t used() const { return offset_; }
  size_t capacity() const { return cap_; }

private:
  uint8_t* buf_ = nullptr;
  size_t cap_ = 0;
  size_t offset_ = 0;
};

/* ───────── arena-backed data container ───────── */
struct Trajectory {
  float* t   = nullptr;
  float* x   = nullptr;
  float* v   = nullptr;
  float* tor = nullptr;
  size_t count    = 0;
  size_t capacity = 0;

  // Allocate backing storage from arena. Returns false on arena overflow.
  bool init(TrajArena& arena, size_t max_samples) {
    t   = arena.allocFloats(max_samples);
    x   = arena.allocFloats(max_samples);
    v   = arena.allocFloats(max_samples);
    tor = arena.allocFloats(max_samples);
    if (!t || !x || !v || !tor) return false;
    capacity = max_samples;
    count = 0;
    return true;
  }

  void push(float tt, float xx, float vv, float torr) {
    if (count < capacity) {
      t[count]   = tt;
      x[count]   = xx;
      v[count]   = vv;
      tor[count]  = torr;
      ++count;
    }
  }

  bool   empty() const { return count == 0; }
  size_t size()  const { return count; }
};

/* ───────── helper functions ───────── */
inline float accelToTorque(float a) { return a * TrajCfg::INERTIA_HAND_ONLY * TrajCfg::HAND_SPOOL_R; }

/* ───────── helper to shift whole trajectory in time ───────── */
inline void shiftTime(Trajectory& tr, float offset)
{
  for (size_t i = 0; i < tr.count; ++i) tr.t[i] += offset;
}

/* ───────── trajectory generator class ───────── */
class HandTrajGenerator {
public:
  explicit HandTrajGenerator(float throwVel)
      : v_throw(throwVel),
        throwHeight(v_throw * v_throw / (2.f * TrajCfg::G)),
        totalStroke(TrajCfg::HAND_STROKE - 2.f * TrajCfg::STROKE_MARGIN),
        deltaT(1.f / TrajCfg::SAMPLE_RATE)
  {
    calcThrow();
    calcCatch();
  }

  /* ----- public builders (arena-backed) ----- */
  bool makeThrow(TrajArena& arena, Trajectory& out) {
    if (!buildThrow(arena, out)) return false;
    shiftTime(out, -t2);               // set t=0 at start of decel
    return true;
  }
  bool makeCatch(TrajArena& arena, Trajectory& out) {
    if (!buildCatch(arena, out)) return false;
    shiftTime(out, -(t5 - t4));        // t=0 at mid velocity‑hold
    return true;
  }
  bool makeFull(TrajArena& arena, Trajectory& out) {
    if (!buildCommand(arena, out)) return false;
    shiftTime(out, -t2);               // align to throw timeline
    return true;
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
    airT = 2.f * v_throw / TrajCfg::G;
    float velHold = TrajCfg::THROW_VEL_HOLD_PCT * totalStroke;
    float accelSt = totalStroke - velHold;

    float t_acc = 2.f / (TrajCfg::INERTIA_RATIO + 1.f) * accelSt / v_throw;
    float t_vel = velHold / v_throw;
    float t_dec = t_acc * TrajCfg::INERTIA_RATIO;

    throwA =  v_throw / t_acc;
    throwD = -throwA / TrajCfg::INERTIA_RATIO;

    t1 = t_acc;
    t2 = t1 + t_vel;
    t3 = t2 + t_dec;

    x1 = 0.5f * throwA * t_acc * t_acc;
    x2 = x1   + v_throw * t_vel;
    x3 = x2   + v_throw * t_dec + 0.5f * throwD * t_dec * t_dec;
  }

  void calcCatch()
  {
    float vC   = -TrajCfg::CATCH_VEL_RATIO * v_throw;
    float irC  = 1.f / TrajCfg::INERTIA_RATIO;
    float velH = TrajCfg::CATCH_VEL_HOLD_PCT * totalStroke;
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
    t8 = t7 + TrajCfg::END_PROFILE_HOLD;

    x5 = x3 + 0.5f * catchA * t_acc * t_acc;
    x6 = x5 + vC * t_vel;
  }

  /* generic 3‑segment builder (a–v–a or v–a–v etc.) */
  bool buildSegment(TrajArena& arena, Trajectory& tr,
                    float start,
                    const float tA[4], const float xA[4],
                    const float vA[4], const float aA[4])
  {
    if (!tr.init(arena, TrajCfg::MAX_THROW_SAMPLES)) return false;
    unsigned idx = 0;
    float   t    = 0.f;
    float   end  = tA[3] - start;

    while (t < end) {
      while (t > (tA[idx + 1] - start)) ++idx;
      float tau = (start + t) - tA[idx];
      float pos = xA[idx] + vA[idx] * tau + 0.5f * aA[idx] * tau * tau;
      float vel = vA[idx] + aA[idx] * tau;

      tr.push(t, pos * TrajCfg::LINEAR_GAIN, vel * TrajCfg::LINEAR_GAIN, accelToTorque(aA[idx]));
      t += deltaT;
    }
    return true;
  }

  /* ----- concrete builders ----- */
  bool buildThrow(TrajArena& arena, Trajectory& tr)
  {
    float tA[4] = {0.f, t1, t2, t3};
    float xA[4] = {0.f, x1, x2, x3};
    float vA[4] = {0.f, v_throw, v_throw, 0.f};
    float aA[4] = {throwA, 0.f, throwD, 0.f};
    return buildSegment(arena, tr, 0.f, tA, xA, vA, aA);
  }

  bool buildCatch(TrajArena& arena, Trajectory& tr)
  {
    float vC   = -TrajCfg::CATCH_VEL_RATIO * v_throw;
    float tA[4] = {t4, t5, t6, t7};
    float xA[4] = {x3, x5, x6, 0.f};
    float vA[4] = {0.f, vC, vC, 0.f};
    float aA[4] = {catchA, 0.f, catchD, 0.f};
    return buildSegment(arena, tr, t4, tA, xA, vA, aA);
  }

  /* full 9‑segment throw‑flight‑catch */
  bool buildCommand(TrajArena& arena, Trajectory& tr)
  {
    const char  typ[9] = {'a','v','a','x','a','v','a','x','e'};
    const float tA[9]  = {0.f,t1,t2,t3,t4,t5,t6,t7,t8};
    const float xA[9]  = {0.f,x1,x2,x3,x3,x5,x6,0.f,0.f};
    const float vA[9]  = {0.f, v_throw, v_throw, 0.f,
                          0.f,-TrajCfg::CATCH_VEL_RATIO*v_throw,
                          -TrajCfg::CATCH_VEL_RATIO*v_throw, 0.f, 0.f};
    const float aA[9]  = {throwA,0.f,throwD,0.f,
                          catchA,0.f,catchD,0.f,0.f};

    // Full trajectory can be much longer than a throw-only; use total t8 to estimate
    const size_t est = (size_t)(t8 * TrajCfg::SAMPLE_RATE) + 2;
    if (!tr.init(arena, est)) return false;

    unsigned idx = 0;
    float t = 0.f;
    const float dT = 1.f / TrajCfg::SAMPLE_RATE;

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
      tr.push(t, pos * TrajCfg::LINEAR_GAIN, vel * TrajCfg::LINEAR_GAIN, accelToTorque(acc * TrajCfg::LINEAR_GAIN));
      t += dT;
    }
    return true;
  }
};

/* =====================================================================
   Smooth point-to-point move
   ---------------------------------------------------------------------
   * start position  = start_rev (argument)
   * end   position  = target_rev   (argument)
   * boundary cond.  = v=a=0 at both ends   (quintic "S-curve")
   * duration chosen such that  |a_max| ≤ TrajCfg::MAX_SMOOTH_ACCEL
   ===================================================================== */
inline bool makeSmoothMove(TrajArena& arena, Trajectory& tr,
                           float start_rev, float target_rev)
{
    if (target_rev > TrajCfg::HAND_MAX_SMOOTH_POS){
      target_rev = TrajCfg::HAND_MAX_SMOOTH_POS;
    } else if (target_rev < 0.0f){
      target_rev = 0.0;
    }
    const float delta_rev = target_rev - start_rev;

    if (fabsf(delta_rev) < 1e-6f) {
      // Already there — leave tr empty (count=0)
      tr = Trajectory{};
      return true;
    }

    /* ----- derive duration from accel limit ------------------------- *
     * pos(t) = delta_rev · s(τ) + start_rev ,   τ = t / T
     * a(t)   = delta_rev / T² · s''(τ)
     * so  |a_max| = |delta_rev| · TrajCfg::QUINTIC_S2_MAX / T²  ≤  A_max
     */
    const float T = sqrtf(fabsf(delta_rev) * TrajCfg::QUINTIC_S2_MAX / TrajCfg::MAX_SMOOTH_ACCEL);

    /* guard against silly small values (numerics, rounding, …) */
    const float duration  = fmaxf(T, 0.05f);

    const float dT   = 1.0f / TrajCfg::SAMPLE_RATE;
    const float invT = 1.0f / duration;
    const float invT2 = invT * invT;

    const size_t N = (size_t)ceilf(duration * TrajCfg::SAMPLE_RATE) + 1;
    if (!tr.init(arena, N)) return false;

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

        const float acc_lin = acc_rev / TrajCfg::LINEAR_GAIN;  // [m/s²]
        const float torque  = accelToTorque(acc_lin);       // [N·m]

        tr.push(t, pos_rev, vel_rev, torque);
    }

    return true;
}
