#ifndef TRAJECTORY_H
#define TRAJECTORY_H
/*  Hand‑trajectory generator — Teensy 4.0
    Heavily based on code written by Jon Beno, May 13 2024
    Harrison Low · May 2025
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
#include "hardware_config.h"

/* ───────── data container ───────── */
struct Trajectory {
  std::vector<float> t, x, v, tor;
};

/* ───────── constants — sourced from hardware_config.h ───────── */
constexpr float G                   = Physics::GRAVITY_MPS2;
constexpr float HAND_SPOOL_R        = TeensyTraj::HAND_SPOOL_RADIUS_M;
constexpr float LINEAR_GAIN_FACTOR  = TeensyTraj::LINEAR_GAIN_FACTOR;
constexpr float LINEAR_GAIN         = LINEAR_GAIN_FACTOR / (M_PI * HAND_SPOOL_R * 2.f);  // rev per metre (derived)
constexpr float INERTIA_HAND_ONLY   = TeensyTraj::INERTIA_HAND_ONLY_KG;
constexpr float INERTIA_RATIO       = TeensyTraj::INERTIA_RATIO;
constexpr float THROW_VEL_HOLD_PCT  = TeensyTraj::THROW_VEL_HOLD_PCT;
constexpr float CATCH_VEL_RATIO     = TeensyTraj::CATCH_VEL_RATIO;
constexpr float CATCH_VEL_HOLD_PCT  = TeensyTraj::CATCH_VEL_HOLD_PCT;
constexpr float HAND_STROKE         = TeensyTraj::HAND_STROKE_M;
constexpr float STROKE_MARGIN       = TeensyTraj::STROKE_MARGIN_M;
constexpr float END_PROFILE_HOLD    = TeensyTraj::END_PROFILE_HOLD_S;
constexpr int   SAMPLE_RATE         = (int)TeensyTraj::SAMPLE_RATE_HZ;

/* ----- smooth-move tuning — sourced from hardware_config.h ---- */
constexpr float MAX_SMOOTH_MOVE_HAND_ACCEL = TeensyTraj::MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2;
constexpr float QUINTIC_S2_MAX             = TeensyTraj::QUINTIC_S2_MAX;
constexpr float QUINTIC_H2_MAX             = TeensyTraj::QUINTIC_H2_MAX;
/* TeensyTraj::QUINTIC_H_MAX (= 16/81, the maximum of quinticH below) is
   deliberately NOT aliased here: smoothMoveExcursion computes the EXACT
   excursion rather than the |v0|*T*H_MAX bound, which widens the band over which
   velocity continuity is affordable.  The constant is consumed host-side by
   jugglebot/motion/trajectory/hand_stroke.py, where a closed-form bound is what
   a caller needs to size a window with.                                       */
constexpr float SMOOTH_MOVE_V0_DEADBAND    = TeensyTraj::SMOOTH_MOVE_V0_DEADBAND_RPS;
constexpr float SMOOTH_MOVE_MIN_DURATION_S = 0.05f;   // fmaxf guard, historical
/* Stroke end stops the smooth-move overshoot is checked against.  Upper carries
   a margin (the overextension guard sits only 0.76 mm below the top of the
   355 mm stroke).  Lower is ENCODER ZERO — JBOp::HAND_RETRACT_REV, which is also
   the host's own declared floor for this axis (teensy_bridge_node rejects a
   smooth-move target below 0, and can/odrive.py clips a hand setpoint below 0
   and warns).  It is NOT Homing::HAND_ABS_POS_REV = -0.1 rev: that value IS the
   bottom hard stop (the axis homes DOWNWARD into it at -3 rev/s), so using it
   here would let a honoured prelude plan commanded travel all the way onto the
   stop with no allowance for the position loop's own undershoot — measured
   +0.186 rev of tracking overshoot on this profile family (bench H3.5) against
   0 rev of margin.  Brute-forced over the honoured branch, the -0.1 floor
   admitted commanded troughs to -0.0999 rev on the retract, prime AND
   catch-descent paths, and a PRIME dispatched at the mid-point of a live retract
   (5.0 rev, -24.6 rev/s) dived to -0.057 rev — 5 rev BELOW its start, for a move
   whose target is the top of the stroke.  A tighter floor costs nothing: the
   bound is relaxed to fminf(FLOOR, start, target) below, so every legal target
   stays servable at any floor value, and a profile whose bulge would go under
   simply takes the documented rest-to-rest fallback (today's behaviour).       */
constexpr float SMOOTH_MOVE_POS_CEIL_REV   = Geometry::HAND_MOTOR_HARD_STOP_REVS
                                             - TeensyTraj::SMOOTH_MOVE_EXCURSION_MARGIN_REV;
constexpr float SMOOTH_MOVE_POS_FLOOR_REV  = JBOp::HAND_RETRACT_REV;
/* Slack on the two end-stop comparisons.  NOT a margin — it absorbs float32
   rounding in the excursion's interior turning point, which can report a trough
   a few ulps BELOW a profile's own endpoint even when the profile is monotone in
   exact arithmetic (a 5.0 -> 0.0 retract at -7.5 rev/s measures -2.4e-6 rev).
   Without it, endpoint rounding alone would send those moves to the fallback and
   the fix would be a no-op on the retract path.  1e-4 rev = 3.2 um of cable
   travel: 1000x tighter than the 0.1 rev the old floor gave away, and three
   orders of magnitude below the measured position-loop tracking error.        */
constexpr float SMOOTH_MOVE_END_STOP_EPS_REV = 1e-4f;
extern volatile float current_hand_position;
extern volatile float current_hand_velocity;

/* ───────── torque feedforward — TWO conversions, deliberately ─────────
   The torque stream is packed into every setpoint frame (packTrajectory ->
   ODrive `input_torque`).  It is the ONLY term that commands braking
   OPEN-LOOP; every other newton-metre the drive produces on the decel it has
   to earn from tracking error, through a loop that is far too slow to do it
   inside a 47-93 ms ramp (hand_pos_gain 35 rev/s per rev, hand_vel_gain 0.007
   Nm/(rev/s) = 1.27 A/(rev/s), hand_vel_int_gain 0.07 => a 0.100 s integrator
   unwind constant, 1.1-2.1x the whole ramp).

   accelToTorque is the HISTORICAL conversion and models the axis as a pure
   translating mass on a spool.  Its IMPLIED reflected inertia is
       m*r / (2*pi*LINEAR_GAIN) = 0.281*0.00521 / 198.6588 = 7.3695e-6 kg m^2
   — it omits the motor's rotor entirely and uses the RAW spool radius instead
   of the effective radius LINEAR_GAIN_FACTOR = 1.035 implies.  Measured
   reflected inertia is 1.02e-5 - 1.05e-5 (two independent identifications off
   the 2026-07-27 sitting; see ros_ws/docs/hand_decel_feedforward.md), so the
   feedforward delivered ~70 % of the torque the commanded acceleration
   physically requires.  On the DECEL that shortfall is what the hand coasts
   out: 0.06 / 0.06 / 0.35 / 1.02 rev past x3 at 2.74 / 3.44 / 3.97 / 4.86 m/s,
   ending in light physical contact with the end stop.

   throwDecelToTorque is the corrected conversion.  It is applied ONLY to the
   post-release decel segment of a kind-0 throw (buildThrow's torA[2]).
   Everything else keeps accelToTorque, and that scoping is load-bearing:
     * ASCENT — correcting the accel feedforward would make the hand track the
       ascent better and therefore RAISE the achieved release velocity,
       re-calibrating every throw height the machine has ever flown.  Operator
       decision, not this header's.
     * kind-1 CATCH and makeSmoothMove — bit-identical to before, so the
       C-HAND-1 arm gate, _PRIME_INFLIGHT_S and every host window sized on a
       commanded hand move stay valid without moving.
   The decel segment is also the only place where the inertia is UNAMBIGUOUS:
   the ball has left, so the axis is exactly rotor + hand.                    */
inline float accelToTorque(float a) { return a * INERTIA_HAND_ONLY * HAND_SPOOL_R; }
constexpr float THROW_DECEL_TORQUE_K =
    TeensyTraj::THROW_DECEL_REFLECTED_INERTIA_KGM2 * 2.f * (float)M_PI * LINEAR_GAIN;
inline float throwDecelToTorque(float a) { return a * THROW_DECEL_TORQUE_K; }

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

  /* generic 3‑segment builder (a–v–a or v–a–v etc.)
     torA carries the per-segment torque FEEDFORWARD explicitly rather than
     deriving it from aA inside the loop, because the throw's decel segment is
     the one place that uses a different acceleration->torque conversion (see
     throwDecelToTorque above).  Passing it in keeps that fact at the caller,
     where the segment's meaning is known, instead of hiding a branch here.   */
  Trajectory buildSegment(float start,
                          const float tA[4], const float xA[4],
                          const float vA[4], const float aA[4],
                          const float torA[4])
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
      tr.tor.push_back(torA[idx]);
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
    /* segment 2 (x2 -> x3) is POST-RELEASE: the ball is gone, so the axis is
       exactly rotor + hand and the feedforward can be sized from the real
       reflected inertia.  Segments 0/1 keep the historical conversion — see
       throwDecelToTorque's comment for why the ascent must not move.        */
    float torA[4] = {accelToTorque(throwA), 0.f,
                     throwDecelToTorque(throwD), 0.f};
    return buildSegment(0.f, tA, xA, vA, aA, torA);
  }

  Trajectory buildCatch()
  {
    float vC   = -CATCH_VEL_RATIO * v_throw;
    float tA[4] = {t4, t5, t6, t7};
    float xA[4] = {x3, x5, x6, 0.f};
    float vA[4] = {0.f, vC, vC, 0.f};
    float aA[4] = {catchA, 0.f, catchD, 0.f};
    /* kind-1 is UNTOUCHED by the 2026-07-28 decel-feedforward change. */
    float torA[4] = {accelToTorque(catchA), 0.f,
                     accelToTorque(catchD), 0.f};
    return buildSegment(t4, tA, xA, vA, aA, torA);
  }

  /* full 9‑segment throw‑flight‑catch (kind 2, `makeFull`)
     TWO KNOWN DEFECTS, deliberately left alone as of 2026-07-28 — no live host
     dispatches kind 2 (`grep -rn traj_type ros_ws/src/jugglebot/jugglebot`:
     only kind 0/1 outside `archived/`), and both are commanded-magnitude
     changes on a path this phase is not chartered for:
       1. `accelToTorque(acc * LINEAR_GAIN)` below feeds a rev/s^2 quantity into
          a conversion whose argument is m/s^2, so kind 2's torque feedforward
          is LINEAR_GAIN = 31.6x too large.  Every other builder passes m/s^2.
       2. it does NOT carry buildThrow's corrected post-release decel
          feedforward, so kind 2's decel would differ from kind 0's even after
          defect 1 is fixed.
     Fix them together, with a bench validation, if kind 2 is ever revived.   */
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

/* ───────── smooth-move quintic shape functions ─────────
   pos(τ) = x0 + δ·s(τ) + u·h(τ)     with  u = v0·T,  δ = target − x0.
   s is the historical rest-to-rest shape; h is what a non-zero start velocity
   contributes.  h(0) = h(1) = 0 and h ≥ 0, so u never moves the endpoints — it
   only bulges the profile by u·QUINTIC_H_MAX in the direction of travel.  The
   `u *` terms vanish EXACTLY at u = 0, so every COMMANDED POSITION of every
   rest-to-rest hand move the robot already makes is bit-identical to before this
   change.  One caveat, checked rather than assumed: the FIRST velocity sample of
   a move with δ < 0 (every retract, every catch-descent prelude) changes from
   −0.0f to +0.0f, because (δ·s'(0)) is −0.0f while (δ·s'(0) + u·h'(0)) is
   −0.0f + 0.0f = +0.0f.  int16_t(lrintf(±0.0f · vel_scale)) is 0 either way, so
   the wire is unchanged; 1426 of 4000 at-rest cases differ in that one field and
   in nothing else.                                                            */
inline float quinticS  (float tau) { const float t2=tau*tau, t3=t2*tau, t4=t2*t2;
                                     return 10.0f*t3 - 15.0f*t4 + 6.0f*t4*tau; }
inline float quinticS1 (float tau) { const float t2=tau*tau, t3=t2*tau, t4=t2*t2;
                                     return 30.0f*t2 - 60.0f*t3 + 30.0f*t4; }
inline float quinticS2 (float tau) { const float t2=tau*tau, t3=t2*tau;
                                     return 60.0f*tau - 180.0f*t2 + 120.0f*t3; }
inline float quinticH  (float tau) { const float w=1.0f-tau;
                                     return tau*w*w*w*(3.0f*tau + 1.0f); }
inline float quinticH1 (float tau) { const float w=1.0f-tau;
                                     return w*w*(1.0f + 2.0f*tau - 15.0f*tau*tau); }
inline float quinticH2 (float tau) { return 12.0f*tau*(1.0f-tau)*(5.0f*tau - 3.0f); }

/* Shortest duration whose peak |acceleration| is bounded by the accel limit.
   a(τ)·T² = δ·s''(τ) + u·h''(τ), bounded by |δ|·S2 + |v0|·T·H2, so requiring
   that ≤ A_max gives  A_max·T² − |v0|·H2·T − |δ|·S2 ≥ 0.  The v0 == 0 branch is
   the HISTORICAL expression, kept verbatim so a rest-to-rest duration (and hence
   the sample count and smoothDur_us) is bit-identical to before.              */
inline float smoothMoveDuration(float delta_rev, float v0_rps)
{
    const float c = fabsf(delta_rev) * QUINTIC_S2_MAX;
    if (v0_rps == 0.0f)
        return sqrtf(c / MAX_SMOOTH_MOVE_HAND_ACCEL);
    const float b = fabsf(v0_rps) * QUINTIC_H2_MAX;
    return (b + sqrtf(b*b + 4.0f*MAX_SMOOTH_MOVE_HAND_ACCEL*c))
           / (2.0f*MAX_SMOOTH_MOVE_HAND_ACCEL);
}

/* Longest duration a VELOCITY-CONTINUOUS prelude may take before it falls back.
   Arresting v0 costs time as well as travel: the duration grows linearly in |v0|
   without bound, so the excursion clamp alone does not stop a prelude from
   outlasting a move the firmware could previously command.  The cap is the
   longest REST-TO-REST smooth move the stroke admits (full travel, 0 -> 10.8
   rev = 0.78964 s), so an honoured prelude can never take longer than a profile
   this firmware already emitted before the change — which means every host-side
   window sized on a commanded hand move stays valid without moving.

   The window that made this concrete: catch_coordinator_node._PRIME_INFLIGHT_S =
   1.2 s suppresses a re-prime while a prime ascent could still be RUNNING (a
   kind-3 re-dispatch mid-ascent rebuilds the profile from the live position and
   yanks the hand backwards — the 2026-07-23 stutter, 5/12 ascents stalled 60-70
   ms).  It is sized from the rest-to-rest ascent, 0.7583 s.  A prime dispatched
   into a live retract near the retract's own peak descent speed (start ~5.04
   rev, v0 ~-24.6 rev/s) solves to 1.206 s and OUTGREW it.  Capping is the
   conservative half of that fix: the profile degrades to today's, rather than a
   validated host timing constant moving on the strength of an unobserved case.
   Pinned host-side by tests/ros/test_catch_coordinator_node.py::
   test_prime_inflight_window_covers_the_commanded_prime_ascent.                */
inline float smoothMoveMaxDuration()
{
    return sqrtf(Geometry::HAND_MOTOR_HARD_STOP_REVS * QUINTIC_S2_MAX
                 / MAX_SMOOTH_MOVE_HAND_ACCEL);
}

/* Lowest / highest commanded position over the move, RELATIVE to the start.
   pos'(τ) = (1−τ)²·[30δτ² + u(1 + 2τ − 15τ²)], so the interior turning point is
   a quadratic root.  Both endpoints (0 and δ) are always included, so the
   interval spans the whole excursion, not only the overshoot.                 */
inline void smoothMoveExcursion(float delta_rev, float v0_rps, float duration,
                                float& lo_out, float& hi_out)
{
    const float u = v0_rps * duration;
    float lo = 0.0f, hi = delta_rev > 0.0f ? delta_rev : 0.0f;
    if (delta_rev < 0.0f) lo = delta_rev;

    const float qa = 30.0f*delta_rev - 15.0f*u;
    const float qb = 2.0f*u;
    const float qc = u;
    float taus[2];
    int n = 0;
    if (fabsf(qa) < 1e-12f) {
        if (fabsf(qb) > 1e-12f) taus[n++] = -qc / qb;
    } else {
        const float disc = qb*qb - 4.0f*qa*qc;
        if (disc >= 0.0f) {
            const float r = sqrtf(disc);
            taus[n++] = (-qb + r) / (2.0f*qa);
            taus[n++] = (-qb - r) / (2.0f*qa);
        }
    }
    for (int i = 0; i < n; ++i) {
        const float tau = taus[i];
        if (tau <= 0.0f || tau >= 1.0f) continue;
        const float p = delta_rev * quinticS(tau) + u * quinticH(tau);
        if (p < lo) lo = p;
        if (p > hi) hi = p;
    }
    lo_out = lo;
    hi_out = hi;
}

/* =====================================================================
   Smooth point-to-point move — VELOCITY-CONTINUOUS
   ---------------------------------------------------------------------
   * start position  = live encoder reading  (current_hand_position)
   * start velocity  = live encoder reading  (current_hand_velocity)
   * end   position  = target_rev   (argument)
   * boundary cond.  = (x0, v0, a=0) at the start, (target, 0, 0) at the end
   * duration chosen such that  |a_max| ≤ MAX_SMOOTH_MOVE_HAND_ACCEL

   WHY v0 IS READ AT ALL
   ---------------------
   This function is prepended to EVERY hand command — a kind-3 prime / retract /
   SAFE_ABORT, and the prelude ahead of every kind-0/1/2 stroke (Teensy_code_platform.ino
   :470 and :522).  It used to seed the quintic v = a = 0 from
   current_hand_position alone, while current_hand_velocity sat declared two
   lines above and was never read.  So any command landing while the hand moved
   commanded a VELOCITY STEP.  Measured 2026-07-25: a catch arm landing 8–18 ms
   after ball release froze the setpoint at the live encoder value (6.20–7.78
   rev) with the hand travelling through it at ~120 rev/s; the position loop then
   coasted to 10.17–10.32 rev — as little as 0.775 rev from the 11.1 rev
   overextension guard — and yanked the hand 0.34–1.75 rev = 10.7–55.3 mm BELOW
   the stroke end.  That is the operator-visible post-throw dip.

   THE SHAPE
   ---------
   The quintic through (x0, v0, 0) → (x1, 0, 0) decomposes EXACTLY into two fixed
   shapes (u ≡ v0·T, δ ≡ x1 − x0, τ ≡ t/T):

       pos(τ) = x0 + δ·s(τ) + u·h(τ)
       s(τ)   = 10τ³ − 15τ⁴ + 6τ⁵                 (the historical rest-to-rest shape)
       h(τ)   = τ(1−τ)³(3τ+1)

   h(0) = h(1) = 0, so v0 never moves the endpoints — the profile still starts at
   x0 and lands on the target at rest.  h ≥ 0 with a single maximum QUINTIC_H_MAX
   = 16/81 at τ = 1/3, so v0's entire effect is to bulge the profile by
   u·16/81 in the direction of travel.  That bulge IS the overshoot; it is
   correct (a hand moving away from its target must overshoot to come back) and
   it is what must be checked against the stroke end stops.

   In coefficient form (A τ³ + B τ⁴ + C τ⁵ added to x0 + u τ):
       A = 10δ − 6u ,  B = −15δ + 8u ,  C = 6δ − 3u
   which at u = 0 is (10δ, −15δ, 6δ) — the historical polynomial exactly.

   DURATION
   --------
   a(τ)·T² = δ·s''(τ) + u·h''(τ), so the old T = √(|δ|·S2/A_max) — which assumed
   v0 = 0 — no longer bounds anything.  Bounding the sum by the triangle
   inequality gives the quadratic

       A_max·T² − |v0|·H2·T − |δ|·S2 ≥ 0

   whose positive root is used below.  At v0 = 0 it reduces bit-identically to
   the historical formula.  The bound is 98.6 % tight when v0 points AWAY from
   the target (the case that needs the room, because s'' peaks at τ = 0.2113 and
   h'' at τ = 0.2427) and conservative — i.e. gentler than necessary — when v0
   points toward it.  Probed over 20 000 random (δ, v0) pairs: zero violations of
   A_max.  A closed form was chosen over bisecting the analytic peak: one sqrtf
   and six flops, with no iteration count and no convergence failure mode, inside
   a CAN receive handler.

   WHEN THE BULGE WILL NOT FIT
   ---------------------------
   The profile falls back to the rest-to-rest quintic — today's exact behaviour,
   losing continuity but adding no commanded magnitude the firmware could not
   already produce.  Refusing the command instead was rejected: for a kind-3 that
   means not clobbering, and a kind-3 retract clobbering an armed kind-0 is the
   ONLY un-arm mechanism the Teensy offers (a pre-release SAFE_ABORT depends on
   it) — and refusing via an EMPTY trajectory is worse still, because
   Teensy_code_platform.ino:472-475 returns before packedMsgs.clear().  Braking harder was
   also rejected: near a target the bulge has no room to be absorbed by the
   s-shape's own travel, and the acceleration needed to squeeze it in is bounded
   by nothing the firmware declares — a SAFE_ABORT retract dispatched with the
   hand already essentially AT hand_retract_rev = 0 and descending at the measured
   −60 rev/s would need ~28 000 rev/s², 280× the declared limit.  (Far from the
   target the same descent is served fine: 5.0 → 0.0 rev at −7.5 rev/s is
   honoured with a trough 2.4e-6 rev below the endpoint.)  Raising the limit is an
   operator decision, not this function's.  The fallback is observable — it emits
   a from-rest quintic seed, which tools/probes/hand_stroke_timeline.py counts and
   the bench aborts on.

   There are TWO cannot-fit tests, not one: the bulge must fit inside the stroke
   (smoothMoveExcursion vs the end stops) AND the duration must fit inside
   smoothMoveMaxDuration(), because arresting v0 costs time as well as travel and
   the host's windows are sized on the durations this firmware could already
   command.  See that helper for the failure it closes.

   See plans/active/hand-command-continuity.md Phase 4, contract C-HAND-1, and
   ros_ws/docs/hand_command_continuity.md.
   ===================================================================== */
inline Trajectory makeSmoothMove(float target_rev)
{
    Trajectory tr;

    const float start_rev = current_hand_position;          // live encoder
    const float start_vel = current_hand_velocity;          // live encoder [rev/s]
    const float delta_rev = target_rev - start_rev;

    /* ----- is the hand at rest? -------------------------------------- *
     * Below the dead-band the reading is treated as gravity-hold dither on the
     * ODrive Vel_Estimate rather than as motion, because honouring dither would
     * command excursion nobody asked for on a stationary hand: 0.280 rev = 8.9 mm
     * at the dead-band edge, rising as v0² (0.641 rev = 20.3 mm at 9.07 rev/s,
     * the largest v0 the stroke top can absorb).  The anchor is a p99, not a
     * maximum — see config/hardware_config.yaml smooth_move_v0_deadband_rps for
     * the two measured populations it sits between and what is still unmeasured.
     */
    const bool at_rest = fabsf(start_vel) <= SMOOTH_MOVE_V0_DEADBAND;

    /* Already there AND at rest → empty traj.  The `at_rest` conjunct NARROWS
     * this branch: at the target but MOVING now yields a braking profile rather
     * than nothing.  Never widen it — Teensy_code_platform.ino:472-475 returns from the
     * kind-3 handler BEFORE packedMsgs.clear() when the move is empty, so a
     * wider empty case is a wider hole in the only un-arm mechanism available.
     */
    if (fabsf(delta_rev) < 1e-6f && at_rest)
        return tr;

    float v0 = at_rest ? 0.0f : start_vel;

    float duration = fmaxf(smoothMoveDuration(delta_rev, v0),
                           SMOOTH_MOVE_MIN_DURATION_S);

    if (v0 != 0.0f) {
        /* ----- will the bulge stay inside the stroke, and the move inside the
         * time a rest-to-rest move could already take? ------------------ *
         * Both position bounds are RELAXED to admit the endpoints: the live
         * position may already be outside the band (a mid-coast reading above the
         * ceiling is exactly the case that most needs a continuous profile) and a
         * legal target may sit below the floor, and neither must make the move
         * infeasible by definition.  EPS absorbs float32 rounding at those
         * endpoints; it is not headroom (see its declaration).
         */
        float lo, hi;
        smoothMoveExcursion(delta_rev, v0, duration, lo, hi);
        const float ceil_rev  = fmaxf(SMOOTH_MOVE_POS_CEIL_REV,
                                      fmaxf(start_rev, target_rev));
        const float floor_rev = fminf(SMOOTH_MOVE_POS_FLOOR_REV,
                                      fminf(start_rev, target_rev));
        if (start_rev + hi > ceil_rev + SMOOTH_MOVE_END_STOP_EPS_REV
            || start_rev + lo < floor_rev - SMOOTH_MOVE_END_STOP_EPS_REV
            || duration > smoothMoveMaxDuration()) {
            /* fall back to the rest-to-rest profile (today's behaviour) */
            v0 = 0.0f;
            duration = fmaxf(smoothMoveDuration(delta_rev, 0.0f),
                             SMOOTH_MOVE_MIN_DURATION_S);
        }
    }

    const float u    = v0 * duration;

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

        /*  pos = x0 + δ·s(τ) + u·h(τ)                     */
        const float pos_rev = start_rev + delta_rev * quinticS(tau) + u * quinticH(tau);
        /*  vel = (δ·s'(τ) + u·h'(τ)) / T                  */
        const float vel_rev = (delta_rev * quinticS1(tau) + u * quinticH1(tau)) * invT;
        /*  acc = (δ·s''(τ) + u·h''(τ)) / T²               */
        const float acc_rev = (delta_rev * quinticS2(tau) + u * quinticH2(tau)) * invT2;

        const float acc_lin = acc_rev / LINEAR_GAIN;        // [m/s²]
        const float tor     = accelToTorque(acc_lin);       // [N·m]

        tr.t  .push_back(t);
        tr.x  .push_back(pos_rev);
        tr.v  .push_back(vel_rev);
        tr.tor.push_back(tor);
    }

    return tr;
}

#endif /* TRAJECTORY_H */
