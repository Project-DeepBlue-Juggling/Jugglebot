---
title: Kinematic calibration — fit the Stewart geometry to a mocap pose sweep
created: 2026-09-23
status: active
owner: Harrison
last_updated: 2026-09-23
related_plan: two-ball-skill-stack.md
related_config:
  - config/hardware_config.yaml → jugglebot_geometry.{base_nodes_mm, init_plat_nodes_mm, initial_height_mm, init_leg_lengths_mm, mm_to_rev} (written only at § 6 step 5, by a deliberate commit)
  - config/tilt_calibration.yaml (recaptured after the geometry lands, § 6 step 6)
  - config/generated/admissible_box.yaml (re-swept after the geometry lands — coordinate with the skill-stack session, § 7)
related_code:
  - tools/kincal_fit.py (the fit, § 4)
  - tests/hardware/kincal_capture.py (the capture, § 5 — not yet written)
  - ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py::pose_to_leg_lengths (the model being calibrated)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py::_frame_offset_check (the landing subtraction this plan retires)
---

# Kinematic calibration

Design settled with the owner in one grilling session, 2026-09-23. **Every decision
below is the owner's, recorded as agreed.** § 9 lists the questions and answers in order.

## 0. Why

At the 2026-09-22/23 sittings the mocap `Platform` body sat about 8.5 mm laterally from
the commanded pose. The error was stable to 0.03 mm and grew with commanded height:
the § 9 z-sweep measured 0.77°, a lever arm about a pivot about 170 mm above the base
plane. The QTM global frame **is** the Base frame (the owner transforms global onto the
`Base` body, which is defined from markers in CNC'd holes, with pairs aligned exactly on
x and y). So the frame is not the problem. The platform really is somewhere other than
where the model says. Two other facts point at the model's geometry:

- the mocap origin sat **7 mm above the commanded centre**, and that origin is trusted:
  it was placed at the leg-joint centroid with the owner's jig;
- the robot is hand-built from FDM parts.

The error model is therefore **(B) kinematic-model geometry error**, not a frame
definition error (A) and not dynamic error (C). § 3 is the pre-registered test that
separates B from C.

What already exists covers **attitude only**. `config/tilt_calibration.yaml` is a 5×5
residual-tilt map at z = 170 mm. It is applied once per plan at the seed pose
(`trajectory_node.py`, `correction_for_pose` → `levelling_correction`). No code has ever
corrected platform *position*. The session-start frame check (`skill_node._frame_offset_check`)
subtracts one measured offset from tracker landings. That is scaffolding: a single
constant measured at rest at one height, blind to z and to anything path-dependent. It
**stays until this plan's § 8 criteria are met on a flying sitting**, because it fails
closed and only moves the learner's aim.

This plan is a **prerequisite for giving the learner z authority**, which is when a
model correct only at z = 170 stops being tolerable. It does **not** block
two-ball-skill-stack R4.

## 1. Remedy — decided

**A parametric kinematic calibration, with no runtime controller.** Fit the model's
geometry to data and write it into `hardware_config.yaml`. The IK is then right
everywhere, and the feasibility gate, the admissible box and the flown path all use one
geometry.

Rejected, and why:

- **A pose-offset controller that servos mocap pose back to target** (the owner's
  opening idea). It would correct a static geometric error with a dynamic loop. A mocap
  loop around the 40 Hz leg path is too slow to act inside a throw stroke. It would also
  add a mocap-dependent feedback path (marker swaps, occlusion, latency) to the leg
  command, where the only safety authority is the Teensy `MAX_DEVIATION` guard. It comes
  back **only** under § 3's RANDOM verdict.
- **A bigger residual map over x, y, z and tilt.** A 3-D-plus workspace needs hundreds of
  nodes. Interpolation kinks in the map's gradient become leg jerk on a moving path, and
  `LIMIT_JERK` is already the dominant refusal. A correction applied outside the IK also
  means the gate checks leg lengths other than the ones commanded. A residual map is
  allowed only for what the parametric fit leaves above § 8.

## 2. Measurement model — decided

The model the fit adjusts is exactly the IK: `ℓ_i = |c + R·p_i − b_i|`, and extension
`ℓ_i − L0_i` equals motor revolutions divided by `mm_to_rev_i`.

- **Fit leg-space.** The IK of the *mocap-measured* pose must reproduce the
  *encoder-measured* leg revolutions. Measured revolutions, not commanded ones, so servo
  tracking error never enters the calibration, and no forward kinematics sits inside the
  fit.
- **Position = the mocap `Platform` body origin**, trusted as the leg-joint centroid (it
  was placed with the owner's jig). The jig itself can no longer be fitted, because later
  components block it, and it is not needed.
- **Attitude = the mocap body attitude × a fitted registration rotation** (3 parameters).
  The body's axes were set parallel to global at definition time, which baked in whatever
  physical attitude the platform had then.
- **Zero attitude is defined by the leg-joint pattern** (owner, Q14). The platform nodes'
  centroid is pinned at the origin and their net rotation at zero (six linear gauge
  constraints), so the registration rotation is identifiable. Gravity-level remains the
  job of `level` plus the tilt map.
- **Parameters (Q16):** 18 base-node coordinates, 18 platform-node coordinates, 6 leg zero
  lengths `L0_i`, 6 leg scales `mm_to_rev_i`, and 3 registration angles, all pulled weakly
  toward CAD. Any parameter the data cannot pin down (posterior sd > 1 mm, or the
  equivalent for scale and angle) is **frozen at CAD and refitted**, and the report names
  it and says why.

## 3. Path-dependence — thresholds pre-registered (owner, Q10)

About 10 poses are each approached from two opposite directions, twice per direction.
Spread is measured on the mocap position at arrival.

| Verdict | Condition | Consequence |
|---|---|---|
| STATIC | overall spread ≤ 1 mm | The parametric fit is the whole remedy |
| DIRECTIONAL | spread > 1 mm, within-direction ≤ 1 mm | Backlash: handled in path planning (for example, a final approach from a fixed direction) |
| RANDOM | within-direction spread > 1 mm | A slow mocap-trim outer loop becomes justified; the owner's controller design is reopened |

## 4. Homing — two-part calibration (owner, Q17 + Q20)

Homing repeatability has **never been measured** and may be worse than 1 mm. The
calibration therefore splits in two:

- **Once:** the full sweep fits the node coordinates and scales. These are fixed hardware.
- **Every session:** a short mocap check after homing (6–10 poses, under a minute) re-fits
  **only the six `L0_i`**, using the same fit code with everything else held
  (`--offsets-only`). It becomes a session-start row next to the frame check. If homing
  proves ≤ 1 mm, this step shrinks to a verification.

The full-sweep capture measures homing directly: a 10-pose subset before a re-home, the
same subset after it. Verdict threshold ≤ 1 mm, on both the mocap arrival spread and the
per-leg `ΔL0`.

## 5. Sweep — decided (owner, Q11 + Q12)

- **Region:** inside ±300 mm in x/y and z = 100–250 mm, **clipped to the reachable set**,
  filling about 90 % of the reachable radius at each z, with poses weighted toward the
  edge (legs near the ends of their stroke separate the parameters best). Reachable
  radius, level, with 5 mm stroke margin (current IK, probe 2026-09-23): 330–365 mm at
  z = 100, 245–275 mm at z = 170, 115–135 mm at z = 250. At 6° tilt: 260–340, 210–245,
  65–80 mm. The ±300 mm corners (424 mm) are out everywhere.
- **Content:** about 5 z-levels × about 25 poses, **every pose tilted** (±6°, up to
  ±10° where reachable; § 5.1 says why). About
  10 poses are held out and never fitted. About 10 poses get the § 3 two-direction
  repeats. A 10-pose subset gets the § 4 re-home repeat.
- **Budget:** up to 30 minutes of motion (owner). The hand is parked and disarmed.
  Profiled moves go through the normal path, with a dwell at each pose. No throwing.
- **Tooling:** capture and fit are separate tools, both scripted (owner, Q8). The capture
  follows the pattern of `tests/hardware/tilt_cal_grid.py`: a pose-list dry-run against
  the IK, and a rehearsal on the loaded Jetson before the robot moves (CLAUDE.md). Output
  goes to `temp/`. It writes the fit's input CSV (the format is the docstring of
  `tools/kincal_fit.py`).

### 5.1 What the synthetic proof says about the sweep (2026-09-23, `tests/sim/test_kincal_fit.py`)

The fit was tested on a known geometry error: every group perturbed, 0.1 mm mocap
noise, 64 tilted poses. Pose-space recovery is solid: hold-out error went from 10.6 mm
RMS under the config geometry to 0.15–0.19 mm RMS. Parameter-level recovery is **not**
exact, and the capture design has to allow for two weak directions:

- **Registration tilt vs the six L0.** A constant tilt of the joint pattern changes each
  leg's length by a near-constant amount across a stroke-limited sweep. Its posterior sd
  was 0.18° with half the poses tilted, 0.11° with every pose tilted ±6°, and 0.075° at
  ±10°. At the 0.1° threshold it usually **freezes at zero**, and the six L0 absorb it.
  **This departs from Q14 in one respect:** a frozen tilt means zero attitude, in tilt, is
  the mocap body's rather than the joint pattern's (yaw stays joint-pattern). It is
  benign: a constant attitude offset is exactly what `level` absorbs, and pose-space
  accuracy is unaffected. **The capture should tilt every pose, as far as reach allows.**
- **Each L0 vs its base node's z.** Both act along a near-vertical leg. The fit freezes
  some base z coordinates or trades them against L0 (errors of about 1 mm at parameter
  level). The posterior sd reports this honestly (errors stay within about 2.6σ).

## 6. Sequence

1. **Fit tool** (`tools/kincal_fit.py`): tested offline on synthetic data with a known
   geometry error. **Done 2026-09-23.**
2. **Capture tool** (`tests/hardware/kincal_capture.py`): pose generator, dry-run,
   rehearsal. **Done 2026-09-23:** seed 1 gives 185 dwells, 18.8 min, 0 refusals
   under the production move gate. A `--check` mode is the § 4 per-session check.
   `mocap_node` publishes the Platform body's z shifted down by `initial_height_mm`;
   the tool adds it back and stores the capture in the Base frame, because step 5
   changes that constant.
3. **Sitting:** the full sweep, the two-direction repeats and the re-home repeat.
4. **Fit** plus the report: the § 3 and § 4 verdicts, and § 8's criteria on the hold-out
   set. The tool proposes a geometry YAML. **It never writes `hardware_config.yaml`.**
5. **Apply** by a deliberate commit: geometry → `python config/generate_config.py` → box
   re-sweep (§ 7) → `colcon build`. No firmware flash: the firmware headers carry the
   geometry constants but no firmware code reads them (checked 2026-09-23).
   `init_leg_lengths_mm` stops being "derived at STOW" and becomes the fitted `L0_i`, and
   `initial_height_mm` becomes the FK of all-zero revolutions. That comment block
   changes with it.
6. **Recapture the tilt map.** The current map was measured against the current geometry,
   so part of it is the same error the fit removes; keeping it would correct that part
   twice. It should come back much smaller.
7. **Flying sitting:** the frame check at every z is the acceptance test (§ 8). Then
   retire the landing subtraction.

## 7. Coordination with the skill-stack session (agreed 2026-09-23 with the R4 session)

- The plan file and its `plans/active/INDEX.md` row land together, in one step
  (`tests/sim/test_plans_index.py`).
- A new test file lands only once it is green. The gate and the nightly run against the
  shared working tree.
- The `logbook/INDEX.md` row is added only in this plan's own commit, **below** any R4
  rows.
- **The `hardware_config.yaml` geometry edit waits until R4 has committed, and the R4
  session is told first.** It changes the admissible-box gate hash, and R4 re-sweeps at
  its phase end, so the two re-sweeps can be folded into one.

## 8. Done means (owner, Q15)

- Hold-out position residual **≤ 1 mm RMS and ≤ 2 mm max**; attitude **≤ 0.1°**.
- Every fitted node within about **5 mm of CAD** (physically plausible).
- § 3 and § 4 verdicts recorded.
- On the next flying sitting, the session-start frame check reads **≤ 1–2 mm at every z**.
  Then the landing subtraction is retired.

## 9. Decision record (grilling session, 2026-09-23)

| # | Question | Decision |
|---|---|---|
| Q1–Q3 | Frame vs plant; urgency; static vs drift | QTM = Base (CNC'd markers). Not urgent, but should be done properly: subtraction is scaffolding. The owner suspects pose/path dependence, so it is measured (§ 3) rather than assumed |
| Q4 | Inclinometer's 0.4° spread between reads | Out of scope (related, separate) |
| Q5 | Base body definition | CNC'd holes, x/y-aligned marker pairs. Frame-definition error (A) is unlikely |
| Q6 | A no-throw attribution sitting first | Yes; it became the § 5 sweep |
| Q7 | Landing subtraction meanwhile | Keep until § 8 |
| Q8 | Parametric calibration vs residual map | Parametric, automated, capture and fit as separate steps |
| Q9/Q13 | Platform body registration | Origin at the jig point (trusted); arbitrary markers for attitude; jig no longer fittable |
| Q10 | Path-dependence thresholds | § 3 table as proposed |
| Q11/Q12 | Sweep extent | ±300 mm x/y, z 100–250, clipped to reachable; up to 30 min |
| Q14 | Zero-attitude convention | The leg-joint pattern; registration fitted; tilt map recaptured after the geometry lands |
| Q15 | Pass criteria | § 8 |
| Q16 | Parameter set | Full set with a CAD prior, freeze-and-refit what the data cannot pin down |
| Q17/Q20 | Homing repeatability | Unmeasured; two-part calibration (§ 4) |
| Q18 | Plumb bob | Deferred. Only needed if the ball-flight fit's 0.38° z-vs-gravity bias still shows in landings after this plan |
| Q19 | Plan placement | This file; a prerequisite for z authority, not an R4 blocker |
