# Wayfinder ticket 03 — reload CATCH refused CATCH_AXIS (diagnosis, read-only)

## Verdict

**Root cause found and reproduced offline.** It is not the schedule's geometry
(the PRE-TILT REST's *target* is correctly on the held-axis line) and not a
levelling correction or a stale re-aim. It is a live-pose sampling bug in
`trajectory_node.py`: the reload CATCH's fresh-origin seed is read from the
machine's live encoder state **up to `LEAD_S` (0.225 s / 9 knots) before the
preceding PRE-TILT REST has actually finished moving**, because
`schedule.compile_reload` places the CATCH's earliest possible dispatch
instant *exactly* `LEAD_S` before the REST's own target completion "by
construction" (its own docstring's phrase), and `_svc_install_segment`'s
fresh-vs-splice test uses that same `LEAD_S` as slack in the *other*
direction — declaring "fresh" (and therefore safe to sample-and-trust the
live pose) before the REST is actually done. Because `_cycle_start_state`
is always asked for `kind=uc.SETTLE`, whose `_KIND_SHAPE` entry has
`post_release=True`, it takes the branch that does **not** refuse a moving
machine — it just hands back whatever the REST's S-curve happens to be doing
at that instant, which is generically off the held-axis line (only the
REST's own endpoint is guaranteed to sit on it).

## 1. How the reload schedule is compiled

`schedule.compile_reload` (`ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py:1088-1225`):

- **PRE-TILT REST** (`skills[0]`): `rest_site_mm = pretilt_site =
  segments.hold_axis_site(landing_mm, tilt, REST_CUP_Z_MM)` (line 1170) — this
  IS the axis point `site_xy + kappa·(z − site_z)` at `z = REST_CUP_Z_MM`
  (`= unified_cycle.SETTLE_CUP_Z_MM`, the parked/settle height, ≈689.6 mm —
  see `unified_cycle.py:213`, "the parked height"). `kappa` comes from
  `receive_hold_tilt(landing_vel_mm_s)` — the announced BB throw's own
  arrival direction, clamped at 12°. Window = `pretilt_period =
  max(floor_lift_s, PRETILT_S)` (≈1.5–1.7 s in practice).
- **CATCH** (`skills[1]`): `t_abs_s = t_land_rel` (the announced touch-down,
  fixed), `window_s = catch_window = t_land_rel − pretilt_end_rel` (line
  1176), `rest_site_mm = pretilt_site` (same point). Its dispatch instant
  (`Skill.dispatch_s()`, `schedule.py:331-334`) is `t_abs_s − window_s −
  lead_s = pretilt_end_rel − LEAD_S` **exactly**.

`compile_reload`'s own docstring (lines 1122-1131) states the design intent
explicitly: *"Setting the CATCH's `window_s` to exactly `t_land −
pretilt_rest.t_abs_s` … puts its splice base EXACTLY on that event, so
`install_segment`'s fresh test (`t_now + lead >= record.end_s`) reads true by
construction at the catch's own dispatch instant, for ANY window size."*
`test_compile_reload_catch_is_a_fresh_origin_from_the_pretilt_rest`
(`tests/motion/test_skills_schedule.py:939-950`) only checks this **algebraic
identity** on the compiled `Skill.t_abs_s`/`dispatch_s()` fields — it never
drives the real install chain, so it cannot see what "fresh, by construction"
actually samples on the machine.

So: the REST's *target* is on the line, and the schedule's arithmetic is
self-consistent — the seed being off the line is not a geometry bug.

## 2. Why the seed is off the line

None of the ticket's first three candidates hold up:

- **Levelling correction** — not involved; `hold_axis_site` doesn't touch it,
  and the "frame check"/pre-level lines in the log run well before the reload
  schedule swap.
- **Stale re-aim moving the touch-down** — ruled out by the log itself: the
  refusal fires at `install_segment CATCH … splice_k`/fresh-origin time
  (`trajectory_node`), **before** any `CATCH-AIM`/`RESEND` line is ever
  printed for this skill. The landing used is the schedule's own
  `landing_prior` (occurrence 1, log line 267/269), unmoved.
- **"REST is on the vertical, CATCH's axis is tilted"** — false; both are
  built from the *same* `hold_axis_site(landing_mm, tilt, …)` call
  (`schedule.py:1170` and `1181` share one `pretilt_site`), so a pure z
  difference at the *target* cannot be the cause.

The real candidate, confirmed: **"the seed coming from the live pose rather
than the planned REST[-endpoint]"** — but more precisely than the ticket's
phrasing, it's the live pose sampled *before the REST has physically arrived*,
not after some unrelated levelling or re-aim event.

**Mechanism, traced through the real install path** (`trajectory_node.py`):

- `_svc_install_segment` (`ros_ws/src/jugglebot/jugglebot/trajectory_node.py:4017-4133`)
  computes `t_now_s = time.perf_counter()` (line 4036) and
  `fresh = record is None or (t_now_s + self._segment_lead_s) >= record.end_s`
  (line 4100), where `self._segment_lead_s = sk_exec.LEAD_S` (line 832, a
  fixed 0.225 s = 9 knots, set once at init — not the schedule's own
  per-skill `Skill.lead_s`, though for this particular CATCH they agree).
- If `fresh`, it calls `self._cycle_start_state(uc.SETTLE)` (line 4102) to
  build the seed from the **live machine state**, not from the previous
  plan's mathematically-declared endpoint.
- `_cycle_start_state` (`trajectory_node.py:~3690-3847`) computes an
  `at_rest` predicate from the *live* twist/hand-rate and logs exactly the
  line seen in the sitting: `"cycle seed for %s: %s (platform %.4f mm/s, …)"`
  (line 3831-3835, `'AT REST' if at_rest else 'IN MOTION'`). Crucially, the
  refusal-on-motion branch (`if not post_release: if not at_rest: return
  None, STALE_STATE, …`, line 3838-3845) is **only reached when
  `post_release` is False** — and `_KIND_SHAPE[uc.SETTLE].post_release` is
  **True** (per the node's own 2026-09-16 comment at line ~3778-3783,
  explaining why `uc.SETTLE` was chosen so the hand-seed reconciliation runs
  on every fresh install). So `_cycle_start_state(uc.SETTLE)` **never
  refuses on a moving machine** — it always takes the branch that just
  reports whatever the live pose/twist currently is.
- Because `compile_reload` places the CATCH's dispatch instant at *exactly*
  `pretilt_end_rel − LEAD_S` (§1 above), the very first tick where
  `SkillExecutor.tick()` even attempts this CATCH's install is *also* the
  earliest instant `_svc_install_segment`'s own `fresh` predicate can read
  true — which is `LEAD_S` (225 ms) **before** the PRE-TILT REST's S-curve
  has actually reached its target and stopped. The live sample at that
  instant is mid-slew: still translating/tilting toward the held-axis point,
  generically off it by whatever lateral distance remains.

This also explains why occurrence 1 (5.681 mm) and occurrence 2 (1.065 mm)
differ: the exact wall-clock instant `_svc_install_segment` processes the
CATCH's install RPC varies with 40 Hz tick quantisation and ROS2/DDS
service-call scheduling, so each attempt samples the REST's profile at a
slightly different point within that last 225 ms window — not a fixed
number of knots early, hence not a fixed residual.

(REBASE and clock-offset-drift were both checked and ruled out for this
sitting: neither REST install shows a "REBASED" log line — both solved in
tens of ms, well under the 75 ms wire-read margin — and the "clock offset
refreshed" lines bracketing this window all show "+0.0 µs step".)

## 3. Reproduction (probe)

`/tmp/claude-1000/-home-jetson-Desktop-Jugglebot-skills/157654d5-f8cc-42f2-b835-1fa13ecee62a/scratchpad/probe_t03.py`
(scratchpad, not committed). Builds the real `schedule.compile_reload`
schedule (BB arrival approximating sitting-3's occurrence 1: yaw 17.7°,
pitch 69.7°, speed 3.34 m/s → clamped to the 12° receive tilt), installs the
PRE-TILT REST for real through `motion.skills.executor.install_segment`
with the R4 launch limits, then:

- **Scenarios A/B/C** (calling `executor.install_segment` directly with a
  hand-built at-rest `seed_rest`, sweeping the REST's own dispatch instant
  by up to one 40 Hz tick): all plan fine. This confirms the pure-Python
  `install_segment` layer is *not* where the bug lives — its fresh branch
  assumes the caller's `seed_rest` is genuinely at rest, an assumption this
  layer alone never violates.
- **Scenario E (the real bug)**: samples the REST's own solved trajectory
  (`unified_cycle.state_at_knot`) at `t = record.end_s − LEAD_S` — the exact
  instant `_svc_install_segment`'s fresh check and `compile_reload`'s
  dispatch-instant identity jointly produce. Result:
  ```
  sampling at t=record.end_s-LEAD_S=1003.1160 (knot 60 of 69):
    pos=[-34.19  5.05  170.0  0.0625 -0.1959  0.0]
    vel=[  8.27  2.64    0.0  0.0143 -0.0447  0.0]
  CATCH refused: CycleInfeasible:
    REJECTED_CYCLE_INFEASIBLE(CATCH_AXIS: the platform isn't lined up for
    this catch: its resting position is 1.2 mm off the line …)
  ```
  Same refusal code, same message template, and a residual (1.2 mm) in the
  same range as the sitting's own 5.681 mm / 1.065 mm.
- **Scenario F (the fix, demonstrated)**: sampling at `t = record.end_s`
  (the REST's true last knot, knot 69 of 69) gives `vel ≈ (0.024, 0.008, 0)`
  mm/s (settled) and the CATCH plans OK.

Run: `source ~/Desktop/PDJ_venv/venv/bin/activate && PYTHONPATH=ros_ws/src/jugglebot python <probe path>`.

## 4. Proposed minimal fix

**File:** `ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py`, inside
`compile_reload`, around line 1176.

```python
# before
catch_window = t_land_rel - pretilt_end_rel
_check_window('the reload CATCH (pre-tilt REST end to touch-down)',
             catch_window)
```
```python
# after
# Give the CATCH's own dispatch a full LEAD_S of margin PAST the PRE-TILT
# REST's target completion, not AT it: `_svc_install_segment` (trajectory_
# node.py) declares an install "fresh" once `t_now + LEAD_S >= record.end_s`
# and then seeds knot 0 from the machine's LIVE pose (`_cycle_start_state`,
# kind=uc.SETTLE, which never refuses a moving machine — see its
# `post_release=True` branch). Placing the CATCH's dispatch_s() AT
# `pretilt_end_rel - LEAD_S` (the old arithmetic) makes the CATCH's very
# first dispatch attempt coincide with the earliest instant that "fresh"
# check can fire, which is LEAD_S BEFORE the REST has actually, physically
# reached its held-axis target -- the live sample lands mid-slew and fails
# CATCH_AXIS (empirically reproduced, probe_t03.py, scenario E). Subtracting
# LEAD_S here pushes catch.dispatch_s() out to `pretilt_end_rel` exactly, so
# the earliest dispatch attempt cannot occur until the REST's OWN completion
# instant has passed.
catch_window = t_land_rel - pretilt_end_rel - LEAD_S
_check_window('the reload CATCH (pre-tilt REST end to touch-down)',
             catch_window)
```

**Trade-off to flag to the owner:** this eats `LEAD_S` (0.225 s) from the
CATCH's own solve window, which `RELOAD_CATCH_WINDOW_S`'s own comment
(line ~666-70) says was sized *for* the heavier held-axis QP's solve budget
(0.5 s total today; 0.275 s after this fix — still comfortably above the
3-knot `MIN_WINDOW_S` floor, but a smaller margin for that specific QP).
If that margin needs preserving, bump `RELOAD_CATCH_WINDOW_S` by `LEAD_S`
(0.5 → 0.725 s) in the same commit — that shifts `pretilt_end_rel` earlier
instead, which only tightens the "no room to plan" `ValueError` fence
(lines 1158-1165), not the QP's own solve budget.

**Proposed test** (new, in `tests/motion/test_skills_executor.py` near the
other `_reload_schedule`/held-axis tests, ~line 3844): install the PRE-TILT
REST for real through `ex.install_segment` (as `probe_t03.py` does), then
assert that sampling the resulting `record` at `record.end_s - sc.LEAD_S`
(the pre-fix `catch.dispatch_s()`) is off the held-axis line by more than
`segments.HELD_AXIS_SEED_TOL`'s position bound, and that after the fix
(`catch.dispatch_s() == record.end_s` by construction), sampling at
`record.end_s` is on it. Concretely, fails before / passes after on:
`sg.plan_segment(sg.CATCH, uc.state_at_knot(record.plan, record.meta, k), …)`
raising vs. not raising `CATCH_AXIS`, for `k` computed from `t =
record.end_s - sc.LEAD_S` vs. `t = record.end_s`, using the R4 launch limits
(`leg 300/5000/150000`, `hand_acc 3500`) and the same BB-arrival recipe as
`tests/motion/test_skills_schedule.py::_bb_arrival_vel_mm_s` (25°/4.8 m/s, or
sitting-3's 20.3°/3.34 m/s — either clamps to the same 12° tilt).

## 5. What the hand does today when a CATCH is refused; is there a fallback?

**No fallback exists.** A refused install (`res.accepted is False`) is
handled identically for every skill kind in
`executor.py:2042-2047`: the attempt is simply ended
(`self.attempt_ended = True; self.end_code = res.code`) and the line
`'%.3f END %s at skill %d (%s): %s'` is logged — nothing else is installed.
The machine is left holding whatever the **last successfully installed**
segment's terminal state is.

For this reload, that's the PRE-TILT REST's own terminal hold:
`rest_site_mm = pretilt_site` at `z = REST_CUP_Z_MM` (=
`unified_cycle.SETTLE_CUP_Z_MM`, the parked/settle clamp, ≈689.6 mm — see
`unified_cycle.py:200-215`, "the settle is the parked height"), tilted at the
receive attitude, with the **hand parked at `sites.REST_HAND_REV`**
(`sites.py:51`, `= uc.hand_rev_for_cup_z(REST_CUP_Z_MM)` — i.e. the
bottom-of-stroke park position, by construction the same one every
between-attempt REST uses). That matches the symptom exactly: the hand isn't
lowered by the bug, it's simply *parked* there because that's what the
PRE-TILT REST's own target already was, and no compliant receiving stroke
ever gets installed for the ball that's still inbound.

The codebase's general safety principle — "a ball already released can still
be in flight… the rest tail already streaming is the safe end" (`executor.py`
`tick()` docstring, ~line 1886-1890) — assumes the last installed REST is an
acceptable place to sit indefinitely. That's a reasonable default for most
refusals, but it is a real gap for *this* refusal specifically: the schedule
already knows a ball is inbound (`Skill.landing_prior`) at a known time, so
"safe" here plausibly means "a receive-height REST that at least lets the
hand's stroke absorb the impact," not "hold the idle park pose." No such
fallback is coded anywhere in `executor.py` or `trajectory_node.py` today —
this is a design gap worth raising with the owner, separate from the timing
fix in §4 (which prevents the refusal outright and so is the higher-leverage
change).

## Files referenced (all read-only, no edits made)

- `ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py` (`compile_reload`
  1088-1225, `_assign_leads` 483-513, `dispatch_s()` 331-334, `LEAD_S`/
  `LEAD_KNOTS` 77-91, `REST_FRESH_MARGIN_S` 409-410, `RELOAD_CATCH_WINDOW_S`
  1048, `PRETILT_S`/`DECAY_S` 1035-1036)
- `ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py` (`hold_axis_site`
  265-282, `receive_hold_tilt` 251-262, `_axis_offsets`/`HELD_AXIS_SEED_TOL`
  usage in `cup_cycle.py`)
- `ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py` (CATCH_AXIS
  raises: 865, 953, 959, 975, 1165)
- `ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py`
  (`install_segment` 792-1021, `PlanRecord.end_s` 596-599, `_dispatch`
  1957-2063, refusal handling 2042-2047, `_catch_terminal` 1440-1537,
  `_rest_terminal` 1539-1548)
- `ros_ws/src/jugglebot/jugglebot/trajectory_node.py` (`_svc_install_segment`
  4017-4166, fresh/splice + `_segment_lead_s` 4095-4133, `_cycle_start_state`
  seed logging ~3690-3847, `post_release`-gated at-rest refusal 3836-3845)
- `ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py` (`SETTLE_CUP_Z_MM`
  200-215)
- `ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py` (`REST_HAND_REV` 51)
- `temp/logs/skills_r4_20260928_2355.log` (lines 259-277 occurrence 1,
  368-386 occurrence 2)
- `tests/motion/test_skills_schedule.py` (`compile_reload` tests 871-1056,
  esp. `test_compile_reload_catch_is_a_fresh_origin_from_the_pretilt_rest`
  939-950)
- `tests/motion/test_skills_executor.py` (`_reload_schedule` 3783-3803 and
  the held-axis tests around it, 3778-3921)
- Probe: `/tmp/claude-1000/-home-jetson-Desktop-Jugglebot-skills/157654d5-f8cc-42f2-b835-1fa13ecee62a/scratchpad/probe_t03.py`
