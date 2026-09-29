---
title: "R4 hardware gate MET: 25 consecutive alternating hop catches and three one-button Ball Butler reloads; the pre-release hold removed the hop overshoot; the reload's Juggle result raced its own schedule swap (fixed); the carried items that shape R5"
type: investigation
date: 2026-09-29
status: tuned
phase: "two-ball-skill-stack — R4"
related_plan: two-ball-skill-stack.md
sessions:
  - temp/logs/skills_r4_20260929_1911.log
  - temp/learn/r4-20260929/memory.csv
  - ~/Desktop/rosbags/2026-09-29_19-11-49 (mcap — not in the repo)
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_teensy_bridge_node_install_skew.py
  - tests/hardware/session_skills_r4.md
  - tests/hardware/session_skills_r4_diag.md
  - plans/active/two-ball-skill-stack.md
subsystem:
  - motion
  - ros
---

# R4 hardware gate MET

## Symptom

The owner flew `tests/hardware/session_skills_r4.md` on 2026-09-29, 19:11–19:24, from one launch
(bag `~/Desktop/rosbags/2026-09-29_19-11-49`). The diagnostic runsheet `session_skills_r4_diag.md`
was not flown. The owner reported: "everything passed with flying colours", "at least 10–15
consecutive catches of the hop", and the reload "semi-dodgy": "the hand could afford to raise
while the platform is tilting to not need to rocket up in anticipation of the catch". This entry
checks those claims against the log and the learner memory, and records what the sitting says
about R5.

## Diagnosis

### 1. The gate (plan § R4: 10 consecutive catches alternating P1/P2, and a one-button BB reload → catch → throw → catch)

| Pattern | Throws | Caught | Landing x (mean ± 1σ) | Landing y | Apex |
|---|---|---|---|---|---|
| hop P1→P2 | 45 | 40 | +12.6 ± 15.8 mm | −4.4 ± 20.0 mm | 0.953 ± 0.014 m |
| hop P2→P1 | 37 | 35 | +2.3 ± 21.4 mm | +4.1 ± 16.5 mm | 0.953 ± 0.013 m |
| self_toss | 24 | 24 | +5.6 ± 17.7 mm | +7.1 ± 15.8 mm | 0.918 ± 0.036 m (cold start included) |

- **Consecutive alternating catches: MET.** The 30-throw hop attempt (19:21:51) caught 14 in a
  row by the node's own verdict. Throw 15 logged `caught=False` (landing +42/+27 mm). But the
  next throw left the same cup with a normal flight (apex 0.977 m, landing +14/−20 mm, caught).
  So the ball stayed in the cup, and that verdict was a slow seat, not a drop. Ten more catches
  followed. The run ended on a real drop: a P2→P1 throw landed 60 mm long, beyond P1. So the
  run was 25 consecutive catches physically, and 14 by the strict verdict. Either count clears 10.
- **One-button reload chain: MET, 3 of 3.** One reload went into self_toss (19:15:13) and two
  into hop (19:20:38 and 19:23:21). Ball Butler already held a ball each time, so `bb/reload`
  was skipped by design. The reload CATCH installed from rest with no refusal; the sitting-3
  `CATCH_AXIS` refusal is gone. Each reload was followed by 4 caught throws.
- No guard latch, E-stop, `MAX_DEVIATION` or `CAN_BUS_DOWN` in the log.

### 2. The hop overshoot is gone, and the hold is what removed it

Before, the hop landed +88/+110 mm long (sitting 3, 2026-09-28). Now P1→P2 lands +12.6 mm, which
includes a −10 mm learner command, so the raw plant bias is +22.3 mm. P2→P1 lands +2.3 mm (raw
+11.4 mm). The only change between the two sittings that touches a hop THROW is the pre-release
hold (`0fbd7e66`); the box re-sweep changed what is admissible, not how a throw flies. This
sitting had no hold-off arm, so it is not the diagnostic runsheet's A/B. But a ~80 mm shift that
coincides with the one throw-side change is the answer the A/B was designed to give.

### 3. The reload's Juggle result raced its own schedule swap (fixed)

All three reloads reported `Juggle OK: COMPLETED (0/0 caught)` in the orchestrator ~20 ms after the
button, while each attempt went on to catch and throw four times.

The mechanism: `_on_announcement` popped `_reload_ctx` before it compiled the reload schedule and
swapped the executor in. For those few ms the node had neither an executor nor a reload context,
the idle state. The tick's `_maybe_signal_goal_done`, running on another thread of the
`ReentrantCallbackGroup`, set `_goal_done_event`. The goal returned with no end code, i.e.
COMPLETED, and zero OUTCOME lines.

The same window had a second consequence: a Stop landing in it found "no attempt was running",
and the swap then installed the reload schedule anyway.

### 4. Re-aims do not survive on the hop (owner asked for this to be documented clearly)

| | Re-aim installed | Refused | Skipped |
|---|---|---|---|
| self_toss | 24 | 7 | 49 |
| hop | **0** | **80** | 99 |

On the hop every re-aim the tracker asked for was refused by the planner:

| Refusal | Count |
|---|---|
| `LIMIT_JERK` | 51 |
| `INFEASIBLE` "unbounded dual step" | 24 |
| `HAND_LIMIT_C2` | 7 |
| `UNVERIFIED` | 2 |
| `HAND_LIMIT_ACC` | 1 |
| `LIMIT_ACC` | 1 |
| `SINGULAR` | 1 |

The counts cover both patterns; the hop accounts for 80 of the 87. The platform is translating
250 mm between the sites while the ball flies, so a re-aim late in the flight re-plans the tail of
a fast move. That is the sitting-1 finding (2026-09-24 outcome item 2): refusals within ~0.5 s of
touch-down, the TAIL not the seam.

Yet 75 of 82 hop throws were caught. **The hop catches worked open-loop, aimed at the scheduled
landing.** Each refused re-aim cost a 60–160 ms solve and a log line and changed nothing.

Conclusion, agreed with the owner: fly two-site patterns (the hop, and R5's columns) with re-aim
off (`catch_resend_max 0`). Keep it for self_toss, where it works. The skipped rows are mostly
`NO-CONVERGED-FIT` (114 overall), i.e. the fit had not converged when the re-aim window closed.

### 5. The learner and the hop apex: the learner IS correcting; the box limits it (owner question)

I first read the hop apex (0.953 m) against the 0.9 m default and told the owner the learner was
not correcting it. **That was wrong.** An offline replay of `learner.command` over this sitting's
own memory, row by row, reproduces 74 of the 82 hop commands to within 2 mm only at a **0.95 m
target**. The command-line hop goals asked for 0.95 m, and the throws converged there (0.953).

The other 8 rows are the two reload hops, which take the node's `apex_m` (0.9). There the learner
wanted 0.821–0.826 m, and the box floored it at 0.85, so those throws flew 0.935–0.941. The plant
still throws about 10 % high, so a 0.9 m target needs a command near 0.82 m.

Laterally, the learner wanted −15 to −19 mm on P1→P2 throws, and the hop box clipped it at
−10 mm. That clip is the +12.6 mm mean residual in the table. The learner's own authority
parameter (20 mm) was never the limit.

Why the box stops at ±10 mm: in the 2026-09-29 sweep (`temp/logs/admissible_sweep_r4d_run1_20260929.log`)
every ±20 mm hop cell passes at the 0.85 and 0.90 m flights. It fails at 0.95 m
(`HAND_LIMIT_ACC`, 17/17 per edge), where a lateral offset on the hardest throw exceeds the hand's
3500 rev/s². The box is one rectangle over the whole 0.85–0.95 m band, so the 0.95 m limit binds
at every apex. The 0.85 m floor is the grid's lowest apex (`APEXES_M`), not a feasibility edge.

### 6. Slow seats: half is aim, half is the catch (owner: "many throws were still quite messy")

55 of 99 catches seated more than 0.15 s after the scheduled landing (the rest at +0.07–0.10 s).
The late fraction rises with landing error:

| Landing error | n | Late fraction |
|---|---|---|
| 0–10 mm | 22 | 0.41 |
| 10–20 mm | 23 | 0.48 |
| 20–30 mm | 33 | 0.58 |
| 30–60 mm | 21 | 0.76 |

Better aim (§ 5) buys part of it. But four in ten well-aimed catches still seat late, which puts
the rest in the catch itself: the ball bounces in the cup. The cup-contact contract recorded a
stroke-limited touch-down closing speed (τ = 0.125 s, 2026-09-20), and that is a catch-planner
lever, not a learner one.

### 7. The stop after a drop was refused every time (owner question)

All three attempts that ended on a drop logged `hold rejected: LIMIT_JERK: peak leg jerk
157462–158140 mm/s³ > 150000` (19:20:28, 19:22:26 and 19:23:50). Two mechanisms produced this:

- **The stop is a pre-skill-stack move.** `trajectory/hold` is `planner.build_hold`: a
  platform-only quintic from the current state back to the pose the robot held when the stop was
  asked for, at rest, over `min_move_duration_s` = 0.2 s. It carries no hand track, so the
  firmware simply holds the hand where it is. From mid-swing, "come back to where you were, in
  0.2 s" needs more jerk than the limit. When it is refused, the already-installed plan runs out:
  it is validated and rest-terminal, so it is safe, but it includes the next carried throw.
- **The first empty throw passed the release check.** `_advance_release_evidence` accepts any
  tracker landing estimate after the release. The ball dropped at 19:22:24 bounced near the cup,
  the tracker produced an estimate, and the empty throw at ~19:22:24.7 counted as released.

The attempt ended only on the NEXT empty throw (`ABORTED_NO_RELEASE` at t_release + 0.5 s,
19:22:26.3). Then the hold was refused, and the throw after that ran too: up to three empty throws
after one drop.

Owner's rule: always make the first throw after a catch, because slow-settling balls were often
thrown well while not registering as held. Then stop before the second empty throw. Design
proposed in the Discussion; not implemented in this unit.

### 8. Leg heartbeat dropouts (owner question): the known leg-bus frame drops, harmless this sitting

`teensy_bridge_node` logged 13 heartbeat-dropout episodes: all six legs, 0.5–1.4 s each, legs 1 and 4
three times each. A bag analysis covered `/link_status`, `/robot_state`, `/cache_diag` and
`/ring_diag` (84,760 messages, an inline scratch analysis):

- **Not only the heartbeat.** The bridge's encoder-frame counter for the same leg also comes up
  short: 0–119 frames missing per episode, one episode with none missing. There is one
  inconsistency. In the one episode on a leg that was moving (leg 4 at 19:16:10, "0 of ~50
  frames"), `/robot_state.pos_estimate` still took a new value about every 100 ms. So some frames
  landed at about a tenth of the rate, or the WARN's counter and the position path disagree. The
  bag cannot say which. The other episodes hit legs at rest, where a frozen position is
  expected either way.
- **Nothing else moved.** `crc_errors`, `decode_errors` and `seq_gaps` were flat, and the
  ring/cache counters (`leak_jb`, FIFO overflows and warns, `rx_cap_hits_jb`, `decode_short`) read
  0 before and after every episode. The leg bus health stayed OK and its load held steady at
  61–62 %. The ODrives showed no error, state change or reboot. **`lead_clamp_mask` and
  `torque_clamp_mask` were 0 throughout, so no leg motion was disturbed this sitting.**
- **Not new, not rising.** The same WARN fired at 2.00/min on 2026-09-27, 1.06 and 1.99/min in
  the two 2026-09-28 sittings, and 1.04/min here, each time spread over 5–6 legs.
- **Episodes cluster across legs.** Three pairs on different legs came within 3–10 s of each
  other.

This matches `plans/active/leg-bus-frame-drops.md` § 2 (2026-08-15) in every exonerating
signature: the frames are never put on the wire. Its leading hypothesis stands: an ODrive
discards its own cyclic telemetry when its TX mailbox is still busy under the bus load of the
bridge's setpoint stream.

One thing is new relative to that plan's "0 of 232 idle windows": here 10 of 13 episodes hit a
leg at rest. The skill stack streams its rest tail continuously, so the bus is loaded the same
(~62 %) whether the platform moves or not. "Idle" in the 2026-08-15 sense (no stream) no longer
occurs during a sitting.

The cheapest discriminator is still that plan's § 2.6 one: read each ODrive's own CAN TX-drop
statistic over SDO after a session, which convicts or clears the hypothesis with no firmware
change.

## Discussion

**Declaring the gate met without the diagnostic A/B.** The diagnostic runsheet existed because
the owner contested the hop mechanism (2026-09-29: "the pre-throw movement is in the −x
direction, while the overshoot is reliably in +x"). This sitting ran the hold on throughout and
never off, so strictly it shows only that the overshoot vanished when the hold arrived. The
alternative the owner named (orientation, or post-release motion bleeding in) would have to
have changed by ~80 mm between two sittings in which neither the tilt plan nor the post-release
hold changed. That is not credible, so the A/B is dropped. The diagnostic sheet's self-toss spread
(Block B) and dwell (Block C) questions are still open and matter more for columns; they fold into
R5's first sitting.

**A retracted claim.** In the first reply to the owner I said the hop apex ran 0.95 against a 0.9
target and the learner was not correcting it. The replay (§ 5) shows the target was 0.95 m. The
real finding is narrower: at a 0.9 m target the box's apex floor, and at any target the box's
lateral width, bind before the learner does. The owner's question "ought we expand the learner's
authority?" therefore has the answer "expand the box, not the authority parameter". That is a
re-sweep with each apex as its own band and a 0.80 m row added. It is R5's to do anyway: columns
flies at 0.9 m and its box is empty under the hold.

**The race fix: claim-then-swap, not a busy flag.** Two shapes close the window:
- a `_reload_swapping` flag that `_maybe_signal_goal_done` and `_refuse_if_running` both honour;
- keeping `_reload_ctx` set until the executor is installed.

The second is taken. It needs no new predicate, both readers keep their existing check, and it
fixes the Stop overwrite for free: installation re-checks `_reload_ctx is ctx` under the same
lock, so a Stop, the timeout or a refused hand lane that ended the attempt during the compile wins.

The pop used to be the dedupe against a second announcement. That job now falls to a claim
(`ctx.announced`) set under the lock, and a regression test pins it.

A compile failure now ends the attempt as `REJECTED_RELOAD_UNSCHEDULABLE`. The alternative was to
leave the claimed context to time out as `ABORTED_NO_ANNOUNCEMENT`, which would name the wrong
fact.

**The stop-after-drop design (proposed, owner to confirm):**
- (a) Replace the legacy hold on the skill path with a REST spliced into the running plan by
  `install_segment`, the same planner every chain's final REST uses. It already plans platform
  and hand to rest from a moving seed (19:20:48: seed "IN MOTION … hand −7.07 rev/s", REST
  planned in 32.7 ms). The legacy hold stays as the fallback.
- (b) Release confirmation needs a converged ballistic fit whose launch matches the command, not
  any landing estimate. Its grace must be measured against how fast real throws' fits converge
  before it is tightened, or a good chain gets stopped.

Where the hand stops (the owner's "ideally at the top, ready for the next feed") is linked to the
reload's hand-rise item. Every attempt's opening REST homes the hand to the bottom today.

## Fix

- `skill_node._on_announcement` claims the reload context (`ctx.announced`) instead of popping it.
  It builds the executor outside the locks, then installs it and clears the context in one
  `_reload_lock` section, under `_tick_lock`, only if the claim is still current. A superseded
  compile is logged and dropped.
- A `compile_reload` refusal ends the attempt `REJECTED_RELOAD_UNSCHEDULABLE` through
  `_end_attempt`.
- `_start_reload` initialises `announced=False`.
- Phase-end audit, one WARNING, adjudicated and taken. The claim leaves `_reload_ctx` set through
  the compile, so `_check_reload_timeout` could still time a claimed context out: a wrong
  `ABORTED_NO_ANNOUNCEMENT`, with the compiled schedule dropped. The timeout now skips a claimed
  context, the same exclusion `'firing'` has, so `_on_announcement` alone resolves a claim. That
  removes the timeout as a backstop, so the body after the claim moved into
  `_install_announced_reload`. Any unexpected error there ends the attempt
  `ABORTED_RELOAD_ERROR(<type>)` with the traceback logged, rather than leaving the attempt
  claimed until a Stop.
- Tests, all in `tests/ros/test_skill_node.py`:
  - `test_the_juggle_goal_is_not_done_while_the_announced_reload_compiles` (failed before);
  - `test_a_stop_during_the_reload_compile_is_not_overwritten_by_the_swap` (failed before);
  - `test_a_second_announcement_during_the_compile_is_ignored` (regression guard; passed before);
  - `test_an_unschedulable_announcement_ends_the_attempt_with_its_own_code` (new behaviour).
  - `test_the_reload_timeout_leaves_a_claimed_announcement_to_its_compile` (audit; failed before);
  - `test_an_unexpected_error_in_the_compile_ends_the_attempt_by_name` (audit; failed before).

## Verification

- 2026-09-29, `python -m pytest tests/ros/test_skill_node.py -q -k "announced_reload_compiles or
  stop_during_the_reload_compile or second_announcement_during"` before the fix: **2 failed, 1
  passed** (the goal-done and Stop tests; the Stop test read "no attempt was running").
- 2026-09-29, `python -m pytest tests/ros/test_skill_node.py -q` after the fix: **138 passed in
  4.08 s**.
- 2026-09-29, the two audit tests before the audit fix: **2 failed**. After it,
  `python -m pytest tests/ros/test_skill_node.py -q`: **140 passed in 3.38 s**.
- Gates, all `./run_tests.sh --full` on 2026-09-29:
  - **20:01–20:07, before the audit fix: PASS.** 5579 passed, 9 skipped, 1 xfailed in 281.76 s;
    serial 6 passed (`temp/logs/gate_full_r4_close_20260929.log`). Against the morning's 5572/8:
    +4 are this entry's first tests, and +3 passed / +1 skipped are
    `tests/ros/test_gui_robot_assets.py` from the GUI commit `4446c9ac`, which landed after that
    gate (its skip: CAD exports kept offline).
  - **20:18–20:23, after the audit fix: FAIL, 1 failed.**
    `test_teensy_bridge_node_install_skew.py::test_the_fw_check_fail_line_also_names_the_host_half`
    read 0 `BRIDGE_FW_CHECK: FAIL` lines (`temp/logs/gate_full_r4_close2_20260929.log`). This was
    a pre-existing test race, untouched by this entry. The node's RX thread stores the identity
    and then logs the verdict (`_on_bridge_identity` → `_record_bridge_fw_version`), but the
    test's `_send_identity` waited only for the stored identity. It passed 5/5 scoped. Fixed:
    the helper now also waits for the logged `BRIDGE_FW_CHECK` line (three callers), 17/17 ×3.
  - **20:25–20:30, the final tree: PASS.** 5581 passed, 9 skipped, 1 xfailed in 280.75 s; serial 6
    passed in 20.10 s (`temp/logs/gate_full_r4_close3_20260929.log`).
- Replays, both 2026-09-29, from this sitting's `temp/learn/r4-20260929/memory.csv`:
  - learner (§ 5): inline scratch script, `learner.command` over the rows in order. 74/82 hop
    commands within 2 mm at a 0.95 m target; the 8 others are the reload hops (0.9 m target,
    box-floored).
  - seat vs landing error (§ 6): parsed from the log's OUTCOME lines.

## Withdrawn claims

- [2026-09-29 19:45] Told the owner the hop apex ran 0.95 m against a 0.9 m target and the learner
  was not correcting it.
  WITHDRAWN: the command-line hop goals asked for 0.95 m. A replay of `learner.command` over the
  sitting's own memory reproduces 74/82 hop commands only at a 0.95 m target, and those throws
  landed 0.953 m (converged).
  Superseded by: Diagnosis § 5. The box's 0.85 m floor binds at a 0.9 m target (the two reload
  hops), and the box's ±10 mm hop width binds laterally.
- [2026-09-29 19:45] Told the owner the 25-catch run ended on a drop "60 mm short at P2".
  WITHDRAWN: that row is a P2→P1 throw (`x0 = +0.125`), and `y0 = −60.5 mm` from the P1 target
  lies beyond P1, away from P2.
  Superseded by: Diagnosis § 1 ("60 mm long, beyond P1").

## Open Questions

- The reload-result race fix is verified by tests only. The next reload from the GUI should
  read the attempt's real result (throws, caught), not COMPLETED 0/0 at the button.

- The stop-after-drop design (§ 7, Discussion): owner to confirm (a) and (b), and where the hand
  stops.
- The box for R5: each apex as its own band, a 0.80 m row, and columns under the pre-release hold.
- Catch quality: the stroke-limited touch-down closing speed (§ 6).
- Leg heartbeat dropouts (§ 8):
  - read the ODrives' CAN TX-drop statistic over SDO;
  - reconcile the WARN's encoder-frame count with `pos_estimate`, which kept updating at ~10 Hz
    in the one moving-leg episode.
