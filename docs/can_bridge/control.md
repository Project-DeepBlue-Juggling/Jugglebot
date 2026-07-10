# Control Flow — Legs and Hand

**Source files:**

- [`leg_interp.cpp`](../../ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp) / [`.h`](../../ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h) — the 500 Hz leg interpolator
- [`hand_ops.cpp`](../../ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.cpp) — the hand trajectory relay
- [`docs/teensy-udp-protocol.md`](../teensy-udp-protocol.md) — wire format for every message referenced here
- [ADR-0012: Hermite interpolator port](../adr/0012-hermite-interpolator-port.md) — why this scheme was ported verbatim rather than redesigned

## Legs: waypoint interpolation, not free planning

A common misreading of this system is that the Teensy plans its own
trajectory from "current leg state" to "target leg state." It doesn't. The
Jetson's MPC solves at 40 Hz and hands the Teensy three **already-computed**
waypoints per leg every cycle — `u0` (current), `u1` (next, 25 ms out),
`u2` (next-next, 50 ms out) — plus a forward-looking velocity `v0`,
acceleration, and torque feedforward. This is the `SETPOINT` message
(156 bytes; see the "Setpoint" section of the
[protocol reference](../teensy-udp-protocol.md)).

The Teensy's job is to evaluate a curve between `u0` and `u1` at 500 Hz —
12.5× oversampling of the 40 Hz input — and stream the result to the leg
ODrives over CAN3. It never has more than a 50 ms lookahead; there is no
"final target" in its view of the world.

### The interpolation is C1-continuous, not C2

The scheme is a genuine cubic Hermite spline
(`leg_interp.cpp` "Mode 1", using the standard `h00/h10/h01/h11` basis
functions), but it is **C1-continuous** (position and velocity match at
segment boundaries) — **not C2** (acceleration is not matched). This is
stated explicitly by the codebase's own authors in four independent
places:

- The commit that introduced the scheme:
  `5c1a984 fix: C1-continuous Hermite interpolation eliminates 40Hz pos_setpoint noise`
- `udp_protocol.h`: `u2` "valid iff flags bit1 **— C1 continuity**"
- [`docs/teensy-udp-protocol.md`](../teensy-udp-protocol.md): identical wording
- `controller/hardware_plant.py`: "Used with `cmd_next_mm` for **C1-continuous**
  Hermite interpolation across segment boundaries"

The reason it can't be C2: a cubic fit between two endpoints has exactly
four degrees of freedom (position and velocity at each end). Once those
four values are fixed, the acceleration at each endpoint is *determined*,
not independently specifiable — there is no free parameter left to also
match acceleration across a segment boundary. `u2` is used only to compute
a **finite-difference velocity estimate** at `u1` (`v1 = (u2 - u1) / T`),
not to constrain acceleration. The `accel`/jerk fields carried in the
packet are used only by the fallback modes (Taylor extrapolation,
velocity-decay-to-zero) when a waypoint goes stale — never by the Hermite
path itself, and production always supplies `u1`, so those fields are not
load-bearing for continuity in normal operation.

!!! note "A genuine C2 (quintic) scheme does exist — elsewhere"
    `controller/hermite.py`, `controller/scheduler.py`, and
    `controller/target.py` use quintic (C2) Hermite interpolation to splice
    the MPC's *reference* trajectory across catch/throw/return events,
    matching position, velocity, **and** acceleration at event boundaries.
    That is a separate, upstream layer on the Jetson generating what the
    MPC tracks. `leg_interp.cpp` is a downstream, unrelated component that
    just upsamples the MPC's own 40 Hz output for the wire — different
    math, different purpose, different continuity order. Conflating the
    two is the likely source of any "C2" assumption about the Teensy path.

### Continuity across a replan is the sender's responsibility

Each new `SETPOINT` packet **hard-overwrites** the interpolator's base
state (`latch_from_staging()` in `leg_interp.cpp`) — there is no blending
with whatever the previous segment was outputting. If the new `u0`/`v0`
don't exactly match what the ISR computed the previous tick, the result is
a real discontinuity. The Teensy trusts the sender; the "bumpless" invariant
is enforced upstream, in the Jetson's `setpoint_pump.py` (its module
docstring calls this "the load-bearing invariant"). Two backstops exist if
that invariant is ever violated: a max-step gate on the Jetson side
(rejects any `u0` jump `> 0.3 rev` from the prior accepted frame) and the
Teensy's own max-deviation E-STOP (see [Safety Mechanisms](safety.md)).

### Joint limits are enforced on the interpolated output, not the target

Every 500 Hz tick, after the Hermite/Taylor/decay math, two clamps run in
sequence on the *result*:

1. **Lead clamp** — the commanded position is never allowed to run more
   than `MAX_LEAD_REV` (0.10 rev) ahead of the live encoder feedback.
2. **Stroke clamp** — the commanded position is clamped to each leg's
   measured physical hard-stop range (`STROKE_MIN_REV`/`STROKE_MAX_REV`),
   with a NaN/Inf fallback to the last-good encoder position.

An out-of-range target is therefore clamped, not rejected. The only
outright rejection at the interpolator is a non-finite (NaN/Inf) frame,
which is dropped whole at ingest before it can reach either clamp.

### What actually drives the legs today

Because the 40 Hz MPC stream is currently gated off in production (see
[the overview's status section](index.md#current-status)), the Hermite
ladder above is not presently in the live control path. What *is* live:

- **`ACTIVATE`** — the ODrive's own native `TRAP_TRAJ` mode drives the rise
  to the active pose. This is the ODrive's built-in trapezoidal-velocity
  trajectory generator, not the Hermite/MPC path.
- **`DEACTIVATE`** — like `ACTIVATE`, delegates to the ODrive's own onboard
  `TRAP_TRAJ` planner (`leg_deactivate.cpp`: SETUP → COMMAND → MONITOR
  ladder, POSITION/TRAP_TRAJ mode) to descend to the retracted (stow) pose,
  then IDLEs each leg once it arrives and settles. This is a distinct
  mechanism from the CAN-bus-down deferred stow (`interp_begin_stow()` in
  `leg_interp.cpp`), which is a Teensy-side, ISR-driven velocity-ramped
  descent used only by the fault machine after a confirmed CAN3 reconnect
  (see [Safety Mechanisms](safety.md#can-bus-down-the-deferred-stow-safety-inversion)).
- **Homing** — a velocity-limited move-to-hardstop with current-spike
  detection (see [Safety Mechanisms](safety.md#homing)).

## Hand: not interpolated by the can-bridge at all

The hand's control loop lives entirely on the **platform Teensy** — a
separate microcontroller — not on the can-bridge. The Jetson computes a
complete ballistic hand trajectory, including an absolute wall-clock
deadline, and packs it into an **opaque 8-byte payload**. The can-bridge's
only role is to arm the hand ODrive (`CLOSED_LOOP` + `POSITION`/
`PASSTHROUGH`) and forward those 8 bytes verbatim onto CAN as `0x6D0`
(`hand_ops.cpp`).

The can-bridge never decodes, interpolates, or re-stamps the payload. This
is a deliberate design choice: if the firmware understood the payload's
fields, a future change could accidentally re-stamp the deadline —
reopening a temporal-accuracy hazard from earlier Ball Butler work. The
platform Teensy is the component that actually executes and interpolates
the hand's real-time trajectory.
