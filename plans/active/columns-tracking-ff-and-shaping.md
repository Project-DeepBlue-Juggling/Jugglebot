---
title: Columns tracking — inertial torque feedforward, then planner-integrated input shaping
created: 2026-10-05
status: active
owner: Harrison
last_updated: 2026-10-05
related_plan: two-ball-skill-stack.md
related_config:
  - config/hardware_config.yaml → dynamics.{torque_ff_platform_inertia, torque_ff_max_nm, torque_ff_firmware_clamp_wire_nm}
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/torque_ff.py::LegTorqueFeedforward
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/emitter.py::KnotEmitter
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/jugglebot/motion/skills/ (segments, schedule, admissible)
---

# Columns tracking: inertial FF, then input shaping

High-level plan only. Detail is written per phase when that phase starts. It **adopts
and supersedes the Phase 1–2 intent of the parked `accel-ff-inertia.md`**, which
stays parked as the record of the original design (unpark/archive is the owner's call,
see § Open questions).

## 0. Why, and what the evidence says (2026-10-05)

Owner observes the platform wobbling during short, aggressive columns hops. A rigid-body
model puts the lateral platform mode near 3 Hz (leg PD stiffness against platform plus
2.15 kg/leg of reflected rotor mass), close to the content of a ~0.25–0.4 s hop. Lowering the
active height was examined and rejected: its effect on leg force, current and stiffness is
small, and it worsens the leg-space acceleration limit (logged in the session record).

First measurements, from a healthy-plant bag (`2026-10-05_16-42-57`, 35 hops):

- **Platform x tracks well.** RMS error ~1.3 mm at 10–20 ms lag; mocap minus encoder FK 1.4 mm.
  No large unobserved wobble in x is *yet demonstrated*.
- **The 3 Hz mode is NOT confirmed.** The spectral peaks are harmonics of the 0.87 Hz hop train.
  Free-decay evidence is thin and looks lag-like.
- **Leg 4 is the real anomaly.** It sits on the 10 A / lead clamp every hop (lag 70 ms, RMS
  3.7 mm), though its modelled accel torque is the smallest of the six. Inertial FF will not
  cure it; its cause is unidentified.
- **Sizing:** with inertial FF on, the real columns plan needs up to ~0.26 Nm per leg (legs 1, 2),
  likely an upper bound. The 0.15 Nm Jetson clamp would clip 27 % of knots; the firmware 0.25 wire-Nm clamp
  is almost enough. Both clamps need to rise to ~0.27 TRUE Nm (hard ceiling 0.30).
- **Plumbing exists.** The live skill path already carries gravity torque_ff through `KnotEmitter`;
  only the inertia flag is off. Torque is one value per 25 ms knot, held flat by firmware
  (a staircase for an accel-proportional term).
- **No input shaping exists** anywhere. `trajectory/shaping.py` is the cup-lean shaper, not spectral.

**Consequence for the plan:** the premise (wobble caused by missing inertial FF) is unproven, so
the plan is gated on measurement before any firmware or planner work.

## 1. Success criteria (fixed in Phase 0, owner-ratified)

Defined on the matched isolated-hop recording: peak platform x error, post-hop settle time to
within a tolerance, and RMS over a 1.5 s hold; plus leg-current headroom and zero
`MAX_DEVIATION` / lead-clamp regressions. The end state is columns running at the current
operating point with measurably less wobble, **or** the same wobble at a shorter hop (higher
accel) if that is the better trade. Phase 0 picks which.

## 2. Phases

### Phase 0 — Measure the target (no hardware changes, one short operator recording)
- Operator records one isolated hop plus a 1.5 s hold, a few repeats, existing topics only. Add a
  small hop-train-free capture so a ring-down is visible.
- Estimate the platform's actual lateral response (ring frequency and damping, or its absence).
- Diagnose leg 4's persistent saturation separately (mechanical, per-leg gain/Kt, cable or
  friction). Fix or explain it before judging FF, since it contaminates every A/B.
- **Decision gate:** wobble is real and lag/ring-like → continue. Wobble is not measurable in the
  platform → stop and re-scope with the owner (the visible wobble may be tilt or hand reaction,
  which this plan does not address).

### Phase 1 — Firmware prerequisites (one bundled flash, bench-verified)
- Stale-link torque decay alongside velocity (the original hard gate).
- Stroke-clamp torque handling without a square pulse.
- Raise the firmware ingest clamp to match the new sizing.
- Decide torque time-sampling: mid-knot sampling on the Jetson (cheap) vs. firmware interpolation.
- Gate: bench demonstrations of both behaviours on the existing runbook pattern, before any platform run.

### Phase 2 — Jetson enablement and offline gate
- Raise the pump clamp, sample accel at mid-knot, flip the flag in config and regenerate.
- Re-sweep the admissible box (config/generated gate hash changes with `hardware_config.py`),
  update the pinned tests, run the full suite.
- Offline replay gate: real columns plans through the emitter; peak torque, clip count and per-knot step
  inside the agreed bounds. The sim cannot validate leg dynamics, so this is a *bounds* check only.
- Dress-rehearse against the live robot state on the loaded Jetson before powering the robot.

### Phase 3 — Hardware A/B of inertial FF (operator-run)
- Matched hops, FF off vs. on, pre-registered metrics and abort signatures (the 2026-05-08
  limit-cycle class, current rail, guard latches). Ramp inertia scale from a fraction up to 1×.
- Gate: criteria from § 1 improved with no abort signature. Otherwise ship it OFF, record why, and
  stop here with an honest negative result.

### Phase 4 — Characterise the residual
- With FF settled, re-measure the platform's lateral response. This tells us the residual
  mode frequency, damping, and how much a shaper could still buy. If the residual is negligible,
  input shaping is not built (explicitly allowed outcome).

### Phase 5 — Input shaping, integrated into the planner (only if Phase 4 justifies it)
- **Design stance:** shaping lives in the planner and feasibility layer, not as a post-hoc convolution
  on the output. A classic ZV/ZVD shaper delays every move by about half the mode period, which
  conflicts with the CAN-clock schedule and ball flight; the planner must account for it in segment timing.
  Candidate forms: a spectral/notch constraint on lateral accel-jerk content, or a shaped
  lateral profile that keeps each segment rest-terminal and timing-exact.
- Extend `validate_cycle` so shaped plans stay inside the leg limits; re-sweep the box; sim columns gate
  (several seeds) must hold; learner behaviour re-checked since the hop time changes.
- Hardware A/B against the Phase 3 baseline, same metrics and aborts.

### Phase 6 — Closeout
- Logbook entry with the (date, command, result) triples and a real Discussion section (a hypothesis
  was reframed). Update the skill-stack plan, `controller/` and `motion/` docs, and the parked
  accel-ff plan's status. Run `./run_tests.sh --full` at closure. Archive via `/archive-plan`.

## 3. Cross-cutting rules
- **Control implications first** for every phase touching feedforward or timing: walk one 40 Hz
  cycle with the change (torque staircase, stale window, lead clamp, hand FF interaction).
- One flash sitting, one box re-sweep per phase boundary, not per experiment. No gated-file edits during a sweep.
- The hand has its own FW 22 torque feedforward; this plan touches legs only and checks the hand
  reaction is not double-counted.
- One `/audit --unstaged` per phase end. Sonnet by default; Opus only for the shaping design (Phase 5).
- Physical-intuition pushback invited at every hardware gate.

## 4. Risks
1. Wobble is not in the platform's x or is hop-train aliasing → Phase 0 gate stops the plan early.
2. Leg 4's saturation masks any FF benefit → diagnosed first.
3. Clamp raise eats current budget (4.5 A of 10 A at peak) and the loop loses authority → ramped scale, abort signatures.
4. Torque staircase injects its own 40 Hz content → mid-knot sampling or interpolation, measured in Phase 3.
5. Shaping delay breaks schedule timing or the learner → planner-integrated design, sim gate before hardware.
6. Config regen invalidates the pinned admissible box → planned re-sweep, not a surprise.

## 5. Open questions for the owner
1. Is a *shorter* hop (same wobble, faster) as good a win as less wobble at the current hop?
2. Unpark and fold in, or leave `accel-ff-inertia.md` parked as reference?
3. Is leg 4 known to differ mechanically (cable, spool, ball joint, encoder) from the others?
4. Is the visible wobble lateral, a tilt rocking, or a vertical bounce? (Decides what Phase 0 looks at.)
