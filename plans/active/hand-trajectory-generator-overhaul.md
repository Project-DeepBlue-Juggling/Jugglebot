---
title: Hand trajectory generator overhaul — jerk-limited, time-budget parameterised
created: 2026-05-22
status: parked   # 2026-08-15 — untouched since creation, self-declared "a strict improvement, not a prerequisite". NOTE for resumption: the hand firmware changed twice underneath this plan (hand-command-continuity Phase 4 velocity-continuous makeSmoothMove seeding + Phase 7 throwDecelToTorque), so Phase 1's baseline characterisation must be re-taken against Platform FW 2, not FW 0.
---

# Hand trajectory generator overhaul

## 1. Context

### Problem / motivation

The platform hand axis (a linear actuator mounted colinear with the platform Z
axis, commanded over CAN `0x6D0` by the platform Teensy) generates its
throw and catch motion profiles on-board, in
`ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h` (`HandTrajGenerator`). Two
limitations make that generator unsuitable for smooth juggling:

1. **Unbounded jerk.** `makeThrow()` and `makeCatch()` build three-segment
   profiles with *piecewise-constant acceleration* (`throwA`/`throwD`,
   `catchA`/`catchD`). Acceleration *steps* at every segment boundary, so jerk
   is unbounded at each corner. The result is a mechanical snap: it shocks the
   hand motor, transmits a reaction impulse into the Stewart platform, and adds
   scatter to the ball release velocity. (`makeSmoothMove()` in the same file
   already uses a jerk-bounded quintic S-curve — the throw/catch builders do
   not.)

2. **Backwards velocity scaling.** The generator is parameterised by a single
   scalar throw velocity `v` and always consumes the full effective stroke
   (`0.315 m`). With stroke fixed, acceleration scales as ~`v²` and event
   duration as ~`1/v`. Peak acceleration is hardware-bounded, so this scaling
   is inverted: it cannot deliver arbitrarily high throws, and it cannot
   express the full range the robot should cover — fast low throws *and* slow
   high throws.

### What this change achieves

- A **jerk-limited** (jerk-bounded, acceleration-continuous) throw/catch
  profile family, removing the release-time and segment-corner snap.
- Correct, physical parameterisation: minimum event duration becomes
  `v / a_peak` — **linear in `v`** — and the generator accepts a **time
  budget** above that minimum, spending slack on lower jerk rather than always
  running the fastest profile.
- **Stroke portion as a free parameter:** the throw/catch need not consume the
  full stroke; start/end positions become inputs, chosen from the event
  geometry by the offline trajectory optimiser.
- A tempo-feasibility consequence for the juggling demo: with a peak-
  acceleration-bounded generator, both flight time (`2v/g`) and event duration
  (`v/a_peak`) scale linearly with `v`, making the two-ball tempo feasibility
  nearly scale-invariant — **this overhaul is what unlocks lower throw heights
  later** (see `plans/archived/2026-08-15 bb-led-two-ball-juggle-demo.md` §4 Phase 1).

### Scope and relationship to the juggling demo

This plan is a **parallel side-quest** to the BB-led two-ball juggling demo
(`plans/archived/2026-08-15 bb-led-two-ball-juggle-demo.md`). The two are independent:

- The demo is built against the *current* hand generator and does not depend
  on this overhaul.
- When this overhaul lands, it improves hand smoothness and lowers the
  achievable throw height for the demo — a strict improvement, not a
  prerequisite.

The work may proceed concurrently, including in a separate session.

### Non-goals

- **No move of trajectory generation off the Teensy.** Streaming pre-computed
  profiles from the Jetson was considered and rejected: the Teensy is the
  hard-real-time-capable component, the Jetson does not run PREEMPT-RT, and
  offloading generation onto it would be a regression. The overhaul changes the
  *generation algorithm*; it stays on the Teensy.
- No change to the CAN `0x6D0` command interface or the host-side
  `_send_hand_traj_cmd` encoding — unless Phase 2 establishes that the new
  parameters (time budget, stroke start/end) cannot be derived on the Teensy
  and must be transmitted. That is a decision gated on Phase 2 (see §6).

## 2. Architecture

### Current

```
host: _send_hand_traj_cmd(event_delay, event_vel, traj_type)
        → CAN 0x6D0 → platform Teensy
                         │
                         ▼
        HandTrajGenerator(throwVel)         ← single scalar input
          calcThrow(): 3 segments, piecewise-CONSTANT acceleration
          calcCatch(): 3 segments, piecewise-CONSTANT acceleration
          always consumes full effective stroke (0.315 m)
                         │
                         ▼  500 Hz samples → hand ODrive
        jerk: UNBOUNDED at every segment boundary
        duration ∝ 1/v ; acceleration ∝ v²
```

### Proposed

```
OFFLINE (host): trajectory optimiser chooses, per event:
        release velocity v, time budget T (≥ v/a_peak), stroke start/end
                         │  (parameters; transport TBD — see §6)
                         ▼
        platform Teensy:
        HandTrajGenerator(v, T, stroke_lo, stroke_hi)
          jerk-limited profile (7-segment S-curve OR quintic-segment)
          acceleration CONTINUOUS; jerk BOUNDED
          consumes only the requested stroke portion
                         │
                         ▼  500 Hz samples → hand ODrive
        Python port sim/hand/trajectory.py kept in lockstep
```

### What changes vs what stays the same

| Element | Status |
|---------|--------|
| `Trajectory.h` `calcThrow`/`calcCatch`/`buildThrow`/`buildCatch` | **Replaced** — jerk-limited profile family |
| `Trajectory.h` `makeSmoothMove` (already quintic) | Unchanged; reused as the design reference |
| `sim/hand/trajectory.py` (Python port) | **Replaced in lockstep** with the firmware change |
| CAN `0x6D0` command / `_send_hand_traj_cmd` encoding | Unchanged unless Phase 2 requires new fields (§6) |
| `hardware_config.yaml` `teensy_trajectory` constants | Extended — add a hand peak-acceleration limit; existing constants reviewed |
| Hand ODrive config, 500 Hz sample rate | Unchanged |

## 3. Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| 1 | Characterise the current generator empirically | NOT STARTED | | Low | Quantifies the acceleration steps and the `v²`/`1/v` scaling |
| 2 | Design the jerk-limited profile family & parameterisation | NOT STARTED | | Med | Closed-form profile, jerk-bounded, validated offline in Python |
| 3 | Reimplement `Trajectory.h` + Python port in lockstep | NOT STARTED | | Med | Firmware and port agree; jerk bounded; unit tests pass |
| 4 | Hardware bring-up & jerk measurement | NOT STARTED | | High | Real throw/catch work; measured jerk reduced |

Phases are incremental; no phase depends on a later one.

## 4. Implementation Phases (detailed)

### Phase 1: Characterise the current generator — NOT STARTED

**Scope:** an empirical probe that constructs the current throw and catch
profiles across the event-velocity range (`0.3`–`7.0 m/s`) and plots position,
velocity, acceleration, and jerk. Outputs quantify: the acceleration step
magnitude at each segment boundary, the realised peak acceleration as a
function of `v`, and the `duration ∝ 1/v` scaling.

**Files:** a reusable probe at `tools/probes/hand_profile_probe.py` (committed,
per the `tools/probes/` convention), outputs to `temp/probes/`.

**Dependencies:** none — uses the existing `sim/hand/trajectory.py` port.

### Phase 2: Design the jerk-limited profile family — NOT STARTED

**Scope:** select and specify the profile family — a 7-segment jerk-limited
S-curve, or piecewise-quintic segments — both closed-form and cheap enough for
on-Teensy generation. The generator is reparameterised to accept:

- release velocity `v`,
- a time budget `T` with `T ≥ v / a_peak` (when `T` exceeds the minimum, peak
  acceleration is reduced to lower jerk),
- stroke start and end positions (the consumed portion of the `0.315 m`
  effective stroke).

Add a hand peak-acceleration limit to `hardware_config.yaml`
(`teensy_trajectory`) — the design's binding bound. Validate the profile math
offline in Python: jerk bounded, boundary conditions met, release velocity
exact.

**Decision (gated here):** whether `T` and stroke start/end can be derived
on-Teensy from existing inputs, or must be added to the `0x6D0` payload. If new
fields are required, the CAN encoding and `_send_hand_traj_cmd` change is
scoped in this phase.

**Dependencies:** Phase 1.

### Phase 3: Reimplement firmware + Python port — NOT STARTED

**Scope:** replace `calcThrow`/`calcCatch`/`buildThrow`/`buildCatch` in
`ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h` with the Phase 2 profile, and
replace the corresponding logic in the `sim/hand/trajectory.py` port **in the
same change** so the two never diverge. Unit tests assert jerk is bounded, the
firmware and port agree sample-for-sample, and boundary conditions hold across
the parameter ranges.

**Critical detail:** the host-side juggle optimiser models the Teensy generator
via this Python port; a divergence between port and firmware corrupts the
offline-optimised trajectory. The lockstep update is mandatory, not optional.

**Dependencies:** Phase 2.

### Phase 4: Hardware bring-up & jerk measurement — NOT STARTED

**Scope:** flash the updated firmware; validate throw and catch on hardware
across the velocity range; measure the realised jerk reduction against the
Phase 1 baseline; confirm release-velocity accuracy and repeatability.

**Dependencies:** Phase 3. Real robot, E-stop ready.

## 5. Testing Plan

### Unit tests (offline, no hardware) — `tests/sim/`

| ID | Validates | Pass criteria |
|----|-----------|---------------|
| T-U1 | Jerk is bounded | peak \|jerk\| of every generated throw/catch profile ≤ the design jerk limit across `v ∈ [0.3, 7.0] m/s` |
| T-U2 | Acceleration continuity | acceleration continuous at every segment boundary to < 1e-6 |
| T-U3 | Boundary conditions | start/end position, velocity, acceleration match the requested values |
| T-U4 | Release velocity exact | profile reaches the commanded release velocity at the release sample within tolerance |
| T-U5 | Minimum-duration law | minimum event duration scales linearly with `v` (`v / a_peak`) within numerical tolerance |
| T-U6 | Time-budget slack | for `T` above the minimum, peak acceleration and peak jerk both decrease monotonically |
| T-U7 | Stroke-portion parameter | a profile confined to a stroke sub-range stays within `[stroke_lo, stroke_hi]` |
| T-U8 | Port/firmware agreement | the `sim/hand/trajectory.py` port reproduces the `Trajectory.h` algorithm sample-for-sample |

### Hardware tests (real actuator, E-stop ready) — `tests/hardware/`

| ID | Validates | Pass criteria |
|----|-----------|---------------|
| T-H1 | Throw still functions | a throw at nominal velocity ejects the ball to the expected apex |
| T-H2 | Catch still functions | a catch at nominal arrival velocity captures and arrests the ball |
| T-H3 | Jerk reduction | measured hand-axis jerk is materially below the Phase 1 baseline |
| T-H4 | Release repeatability | release velocity scatter across repeats is no worse than the current generator |

### Regression tests

| ID | Validates | Pass criteria |
|----|-----------|---------------|
| T-R1 | Existing hand tests | existing `sim/hand` tests pass against the new port |
| T-R2 | Full suite | `pytest tests/ -q` passes; count cited with the (date, command, result) triple |

## 6. Notes for Collaborators

### Open items / decisions required

- **`catch_vel_ratio` discrepancy.** `config/hardware_config.yaml` sets
  `catch_vel_ratio: 0.6`; the Python port `sim/hand/trajectory.py` reportedly
  uses `0.9`. The port is supposed to mirror the firmware — this divergence is
  exactly the failure mode this plan's lockstep rule guards against, and it
  must be reconciled (Phase 1). It is also tracked in
  `plans/archived/2026-08-15 bb-led-two-ball-juggle-demo.md` §6.
- **Profile family.** 7-segment S-curve vs piecewise-quintic — decided in
  Phase 2 against the Phase 1 characterisation.
- **CAN payload.** Whether the time budget and stroke start/end fit existing
  on-Teensy derivation or require new `0x6D0` fields — decided in Phase 2.

### Safety-critical invariants

- The hand has a finite stroke (`hand_stroke_m: 0.355`, effective `0.315 m`
  after margins). Any profile must stay within the stroke; over-extension is a
  hardware fault. `hand_stroke_mm` is used elsewhere for over-extension
  detection — keep it authoritative.
- The Python port and the Teensy firmware must be updated together. A silent
  divergence corrupts the offline juggle trajectory optimisation, which models
  the hand via the port.
- Generation must remain closed-form and cheap — the Teensy is hard-real-time
  and must not be given an iterative solve in the 500 Hz path.

### Files affected

| File | Action |
|------|--------|
| `ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h` | Modify (throw/catch builders) |
| `sim/hand/trajectory.py` | Modify (port, in lockstep) |
| `config/hardware_config.yaml` | Modify (add hand peak-accel limit) |
| `config/generated/*` | Regenerate |
| `tools/probes/hand_profile_probe.py` | Create |
| `tools/probes/README.md` | Modify (probe entry) |
| `tests/sim/test_hand_*.py` | Create / extend |
| `tests/hardware/hand_profile_test.py` | Create |
| CAN `0x6D0` path | Modify only if Phase 2 requires new fields |

### Rollback plan

The change is confined to the hand-axis profile generator. Reverting
`Trajectory.h` and `sim/hand/trajectory.py` to their prior revision and
re-flashing the Teensy restores the current generator; the juggling demo,
built against the current generator, is unaffected either way.
