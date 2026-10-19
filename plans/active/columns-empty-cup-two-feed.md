---
title: Columns accepts an EMPTY cup (two Ball Butler feeds), cup-keyed accept rule, 'Reload first' retired
created: 2026-10-17
status: active
owner: harrison
last_updated: 2026-10-17
related_plan:
  - two-ball-skill-stack.md
related_logbook:
  - 2026-10-14-two-ball-columns-sessions-why-attempts-end.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py::SkillNode._start_pattern
  - ros_ws/src/jugglebot/jugglebot/skill_node.py::SkillNode._run_columns
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/gui/js/juggle-panel.js
---

# Columns accepts an EMPTY cup (two BB feeds); accept rule cup-keyed; 'Reload first' retired

Paths: SN = `ros_ws/src/jugglebot/jugglebot/skill_node.py`, SCHED = `.../motion/skills/schedule.py`,
INV = `.../motion/skills/INVARIANTS.md`. Line cites are the 2026-10-17 survey's; re-grep before editing.
Hardware-bearing: **if your physical intuition disagrees with § 1 timing or the § 5 refill assumption, that is load-bearing signal; say so.**

## 0. Decisions (owner, 2026-10-17; normative for this plan)

- **D1** Two-ball `columns` with an EMPTY cup is ACCEPTED. BB feeds ball A as a catch-only reload (PRE-TILT REST,
  held-tilt CATCH, DECAY REST, ends seated + level), then BB feeds ball B and the normal columns entry runs (bridge
  REST holding A at the feed site, FEED WAIT, `compile_columns` hop entry). Both feeds fire automatically after
  Start; no operator action. SEATED cup unchanged (B fed straight away). UNKNOWN cup still refused
  (`REJECTED_BALL_UNKNOWN`).
- **D2** The columns HUMAN-LOB feed path (`reload=False` -> `await_feed`, `_maybe_resolve_columns_feed`,
  `COLUMNS_TRACKER_FEED_DEADLINE_S`) is RETIRED. Columns is BB-fed only.
- **D3** The accept rule is CUP-KEYED for every pattern that can reload (`self_toss`, `hop`, `columns_1ball`,
  `columns`): EMPTY -> BB reloads A first, then the pattern; SEATED -> throw first; UNKNOWN -> refused. `reload`
  (needs EMPTY), `columns_1ball_fed` (A phantom, B fed, needs EMPTY) and `magazine` (needs SEATED) keep fixed
  requirements. Operators can no longer force 'reload first'.
- **D4** `bool reload` is REMOVED from `Juggle.action` and from the GUI request string (interface rebuild + FULL launch
  restart accepted). The bare `reload` PATTERN token (GUI 'Reload' button) stays. The 'Reload first' checkbox is deleted.
- **O1** BB REFILLS ON ITS OWN after a throw (~5-10 s). The node does not command the fetch but MUST wait for BB ready
  (heartbeat fresh, IDLE, ball_in_hand) before commanding the second feed throw. The gate deadline is a NEW constant
  `COLUMNS_CHAIN_BB_REFILL_TIMEOUT_S` (proposal 15 s), probed on hardware (§ 5) before a test pins its value.
- **O2** Ball A's held-tilt reload catch is AT THE FEED SITE (x = -125 at the live sep 250; the earlier "-62.5 at sep 125" was the wrong number), so the second bridge REST is a
  pure hold. The sim probe (§ 5) is the FIRST P3 task; if it refuses, fall back to the proven reload site (-50, 0) +
  held-ball bridge transit and tell the owner.
- **O3** P1 and P2 land as TWO commits, same session, ONE `/audit --unstaged` at the end of P2. P3 is its own session.
- **Assumptions confirmed:** A1 start event keeps `reload='1'/'0'`, now DERIVED from the cup decision (and `_attempt_label`); A2 GUI shows the decision as a PREVIEW; A3 SEATED path and the EMPTY path's tail are ONE code path (`_start_columns_feed`); A4 INV rows, `Juggle.action` comment, rehearsal rows 18-22 and the rule-table test are amended in the phase that changes the rule.
- **Logbook note:** bags BEFORE this change carry columns `reload='1'` meaning "B fed by BB", not "A reloaded".

## 1. Design
### 1.1 Two installs, one shared tail

An EMPTY-cup `columns` attempt runs the `reload` pattern's own flow for A; on A's clean retire a CHAIN ctx waits at a
gate and calls the SAME function the SEATED path calls to feed B. One up-front schedule is impossible: B's timing is
known only from B's announcement. Refactor `_run_columns` (SN:3344):
```
def _run_columns(self, apex_m, separation_mm, dwell_s, n_throws, reload_a: bool, phantom_a: bool = False):
    # unchanged no-motion ladder SN:3409-3493 (box, columns boxes, memory, lift_s, frame check)
    plan = SimpleNamespace(site=bridge_site, lift_s=lift_s, boxes=boxes, memory=memory,
                           aim_site=feed_aim_site, columns_pattern=pattern,
                           holds_ball=not phantom_a, entry_tag=entry_tag)
    if not reload_a:
        return self._start_columns_feed(plan)                      # SEATED path and columns_1ball_fed
    prelevel_err = self._prelevel()                                # mirror _run_reload_only SN:4509
    a = OneBallPattern(sites=(bridge_site,), apex_m=apex_m, dwell_s=dwell_s, n_throws=0, floor_lift_s=lift_s)
    return self._start_reload(resp, site=bridge_site, lift_s=lift_s, pattern=a, boxes=boxes,
                              memory=memory, kind='reload', then_columns=plan)

def _start_columns_feed(self, plan, *, chain_ctx=None):
    return self._start_reload(resp, site=plan.site, lift_s=plan.lift_s, boxes=plan.boxes, memory=plan.memory,
        kind='columns', aim_site=plan.aim_site, columns_pattern=plan.columns_pattern,
        holds_ball=plan.holds_ball, chain_ctx=chain_ctx)
```
Delete the lob branch SN:3521-3588. `_start_reload` (SN:4519) gains `then_columns=None, chain_ctx=None`; `then_columns` is
stored on the ctx. When `chain_ctx` is set: install the bridge executor under `_tick_lock` + `_reload_lock` ONLY if
`self._reload_ctx is chain_ctx and not self._ended_code_for_swap()`; same check before arming B's ctx at SN:4686; on
failure drop the result. With `chain_ctx=None` the accept path is byte-identical to today. (Also a latent-bug fix:
today `_start_reload` writes `_executor` / `_reload_ctx` unconditionally, so a Stop during a mid-attempt call would be
resurrected.)
### 1.2 Chain ctx phases (`kind='columns_chain'`)

- `await_reload_a`: set at A's executor swap (`_install_announced_reload` plain branch SN:5034-5046) in place of
  `self._reload_ctx = None` when `ctx.then_columns`: `SimpleNamespace(kind='columns_chain', phase='await_reload_a',
  announced=True, then_columns=plan, decay_end_abs=schedule.skills[-1].t_abs_s, deadline_mono=math.inf)`. A live ctx
  keeps `_maybe_signal_goal_done`, `_log_attempt_end`, `_refuse_if_running` treating the attempt as running;
  `_on_announcement` cannot claim it. Never times out.
- `await_chain_gate`: set in `_on_tick`'s executor.done block (SN:3019-3041) by a new `_resolve_chain_on_retire(executor)`
  called BEFORE `_log_attempt_end`. end_code set -> clear ctx (normal end). end_code '' -> phase +
  `deadline_mono = perf_counter() + COLUMNS_CHAIN_BB_REFILL_TIMEOUT_S`.
- `chaining`: set under `_reload_lock` by a new `_maybe_advance_chain()` (next to `_maybe_fire_reload_throw`, called each
  tick) before it calls `_start_columns_feed(ctx.then_columns, chain_ctx=ctx)` OUTSIDE the lock (same pattern as
  'firing' SN:2759).
- **Gate predicate (all must hold):** A's executor retired with end_code '' AND `now >= decay_end_abs` AND debounced cup
  SEATED AND BB heartbeat fresh + connected + IDLE + ball_in_hand.
- `_check_reload_timeout` (SN:2672): skip 'chaining' (as 'firing'); `await_chain_gate` branch: cup UNKNOWN ->
  `REJECTED_BALL_UNKNOWN`, cup EMPTY -> `REJECTED_NO_BALL`, else `ABORTED_BB_NOT_READY(state, ball_in_hand)`.
- Per-feed guards: `_start_reload` already resets `_reload_throw_inflight`, `_bb_throw_dispatched`, a fresh ctx
  (bb_left_idle, announced, not_settled_retried), pops correlation ids 0 and 1 (SN:4594), overwrites
  `_bb_pending_release_mono`. `_on_tick` clears `_bb_throw_dispatched` only when ctx is None (SN:3040); harmless while
  chained (BY_HOST needs `_reload_throw_inflight`, False after A retires).
- Correlation: A's BB latch in `_correlation[0]` is popped at chain start; B's goes to `_correlation[1]` (SN:4923-4934);
  latches are keyed on (thrower, throw_time) so A and B cannot cross.
- Tighten `_on_announcement` (SN:4757) to claim only when `ctx.phase == 'await_announcement'` (today any unannounced ctx:
  a late A announcement during B's `await_bb_idle` would be taken). Phase is set before the throw call (SN:2880), so the
  C2 race still holds.
### 1.3 Timing after the gate

`_start_reload` skips `bb/reload` because BB has a ball; READY 3 s (`RELOAD_BB_READY_TIMEOUT_S`); announcement deadline
delay + 1.5 + 1.0 s; B's throw delay = `_fire_reload_throw` floor 2.5 s (`_BB_THROW_DELAY_FLOOR_S`, SN:2810-2860).
Expected B landing >= A landing + ~1.3 s + refill (5-10 s) + 2.5 s + ~0.7 s ToF. `RELOAD_BB_FETCH_TIMEOUT_S` does not
bound the inter-feed wait (the node sends no fetch); the new refill constant does.
### 1.4 Failure table (no new end code)

A steps = existing (`ABORTED_BB_*`, `REJECTED_BB(msg)`, `ABORTED_NO_ANNOUNCEMENT`, `REJECTED_RELOAD_UNSCHEDULABLE`, `MISSED_CATCH`, `LANE_REFUSED`); gate = `REJECTED_NO_BALL` / `REJECTED_BALL_UNKNOWN` / `ABORTED_BB_NOT_READY`; B steps = existing SEATED-path codes. A missed catch of A ends the attempt WITHOUT asking BB again.
### 1.5 Control-cycle walk

1. A's CATCH then DECAY REST (fresh origin, base clears the runway by `REST_FRESH_MARGIN_S`, SCHED ~1700) = proven reload
   streaming, one continuous plan on the Teensy.
2. A's executor done while the DECAY REST streams; tick nulls `_executor`, ctx -> `await_chain_gate`; Teensy follows the
   DECAY REST to its level terminal and holds.
3. Gate tick: `compile_reload_wait(bridge_site, lift_s, t0 = now + deficit + 1/TICK + lookahead, holds_ball=True)` at the
   SAME site as the DECAY REST = a pure hold (level, hand home), fresh-origin REST from the held state, no discontinuity.
4. B's announcement: existing columns branch, `compile_columns(feed, hop_entry=True)` swaps in from the bridge REST = same
   physics as the SEATED path.
5. I-PLAN-8 (INV:58) holds: the goal's first skill is A's bridge REST.
### 1.6 Stop / cancel semantics

During A: identical to the `reload` pattern (`_stop_attempt` clears ctx SN:5326, `_maybe_cancel_bb_throw`). In
`await_reload_a` / `await_chain_gate`: ctx cleared, goal STOPPED, BB never asked for B. During `chaining`: the chain_ctx
guard drops the bridge executor and B's ctx. During B's wait: existing path. BB BY_HOST cancel: ignored between feeds,
ends the attempt during B's `await_announcement` or live executor. Lane refusal: `_check_reload_lane_refused` runs in
every phase (SN:2720).
### 1.7 Cup-keyed decision (interface sketch, D3)
```
RELOAD_FIRST, THROW_FIRST = 'reload_first', 'throw_first'
_CUP_RULE = {  # evidence -> decision; UNKNOWN -> REJECTED_BALL_UNKNOWN for every pattern
  'self_toss': {E: RELOAD_FIRST, S: THROW_FIRST}, 'hop': same, 'columns_1ball': same, 'columns': same,
  'columns_1ball_fed': {E: RELOAD_FIRST, S: REJECTED_BALL_IN_HAND},
  'reload':            {E: RELOAD_FIRST, S: REJECTED_BALL_IN_HAND},
  'magazine':          {E: REJECTED_NO_BALL, S: THROW_FIRST}}
def accept_cup_decision(pattern, evidence) -> Tuple[Optional[str], bool]   # (code|None, reload)
SkillNode._accept_cup_decision(pattern) -> Tuple[Optional[str], bool]       # replaces _accept_possession_refusal SN:3831
```
`_start_pattern`: delete SN:3237-3260 (flag read + the `columns_1ball_fed` reload=False refusal); run the decision at the
SN:3278 slot; thread the derived `reload` into `_attempt_label` (SN:3302), `_run_columns(reload_a=reload)`,
`_run_columns_1ball(reload)`, `_run_one_ball(reload)`, `result.attempt['reload']`, the start event (SN:5395); optionally add
`cup=EMPTY|SEATED` to the start event. Delete `_ACCEPT_EMPTY_ALWAYS` / `_ACCEPT_SEATED_ALWAYS` (SN:475-483) and
`accept_cup_requirement` (SN:485-503); rewrite the `REJECTED_NO_BALL` message clause (SN:3858-3861). Cup source is
unchanged: `_cup_state()` SN:3808-3829 (debounced `_possession_evidence_stable`; UNKNOWN if never received, age >=
`_HAND_STATE_STALE_S` 0.5 s, or invalid). The launch-time ladder `precondition_refusals` (executor.py ~724-793) stays as the
second line. The sim gate (`run_columns_1ball_seed(reload=)`) is consistent and unchanged (bypasses accept).
### 1.8 GUI preview
Panel shows the cup from `/hand_telemetry` (panels.js:323-377) as a PREVIEW ("cup EMPTY: Ball Butler loads first" / "SEATED: throw first" / "UNKNOWN: will be refused"); skill_node is the authority, confirmed by the start event's `reload` field (juggle-panel.js:558). The GUI does not call `skills/check`. Diagram `feed`: columns SEATED = B; EMPTY = A then B ("BB x2"); `columns_1ball_fed` = B; reload-first one-ball = ball 0.

## 2. Accept rule table (EMPTY / SEATED; UNKNOWN is `REJECTED_BALL_UNKNOWN` everywhere)

| Pattern | EMPTY | SEATED | Notes |
|---|---|---|---|
| `self_toss`, `hop`, `columns_1ball` | reload first | throw first | was `EMPTY if reload else SEATED` |
| `columns` | reload A, then feed B (P3); **RELOAD_FIRST since P3 (was `REJECTED_NO_BALL` in P1/P2)** | B fed at once | was SEATED always |
| `columns_1ball_fed` | reload first (A phantom, B fed) | `REJECTED_BALL_IN_HAND` | reload=False refusal deleted |
| `reload` | BB catch only | `REJECTED_BALL_IN_HAND` | unchanged |
| `magazine` | `REJECTED_NO_BALL` | throw first | unchanged |

## 3. Retirement lists (each grep must reach 0; `rg` is not installed, use `grep -rn`)

**D2 (SN):** `COLUMNS_FEED_BOUND_X/Y_MM` (382-383), `COLUMNS_TRACKER_FEED_DEADLINE_S` (398), the `_on_balls` columns-wait
block (~1300-1310), `_maybe_resolve_columns_feed` (1317-1400), the `await_feed` timeout branch (2690-2696) + docstring
(2659-2667), the lob branch (3521-3588), `_stop_attempt` comment (5322), `_install_columns_schedule` docstring "both feed
sources", `_bb_throw_awaited_live` "tracker-fed" text.
Zero-count greps: `await_feed` (SN 3 today), `COLUMNS_TRACKER_FEED_DEADLINE_S` (SN 5), `_maybe_resolve_columns_feed` (SN 6,
panel 1), `COLUMNS_FEED_BOUND_` (SN 4), `ABORTED_NO_COLUMNS_FEED` (SN 3).

**D4:** `Juggle.action:29-31` (and the `columns_1ball_fed` comment -> "needs an EMPTY cup"); orchestrator_node.py
`_parse_juggle_request` (339-377) returns (pattern, values), a `reload` token AFTER the pattern = unknown-token refusal,
delete :405 and the reload text at :411/:415/:148, the reload PATTERN (parts[0]) untouched; juggle-panel.js: checkbox
(1007-1018, dom 1084), `effectiveReload` (320-322), `reloadRequired` (99, 120), `parts.push('reload')` (343), header doc
(19, 31-33), sync code (1202-1218), cross-check reload (451, 598), diagram `m.lob` branch (675-681) + lob drawing
(776-782, 920); keep the running `· reload` label (1179) fed from the event field; state-minimap.js:1192 stays;
test_juggle_panel.html 161, 187-189, 211, 290; skills_plan_bench.py `--reload` + :1523; tests/ros/conftest.py:687.
Zero-count greps: `getattr(goal, 'reload'` (SN); `reload_` in orchestrator_node.py; `bool reload` in Juggle.action; in
juggle-panel.js `userReload|effectiveReload|reloadRequired|juggle-reload"|m\.lob|jp-arrow-lob|hand lob`.
`RELOAD_REQUEST` and `reloadBtn` STAY (the reload PATTERN button). Sweep `grep -rn` over the whole tree before and after; record counts.
tests/ and docs/) before and after; record counts in the logbook.

## 4. Phases
### P1 - cup-keyed accept + lob retirement (node only)
- **Scope:** `accept_cup_decision` / `_accept_cup_decision`, `_start_pattern` re-plumb, D2 deletions (§ 3), `Juggle.action`
  keeps `bool reload` but the node ignores it. Columns SEATED always BB-fed; columns EMPTY temporarily
  `REJECTED_NO_BALL`.
- **Tests** (tests/ros/test_skill_node.py): rewrite `_columns_fed` (:315) onto `_columns_bb_started` (:4483) + a fake
  announcement (users :358, :813, :1926, :5675; delete :5669 tracker BY_HOST); delete lob tests :4717, :4754, :4774, :4799,
  :4843 and :3969 (lifted-at-R5); rewrite :766 (hop bridge via BB) and the reload=False docstrings :4015, :4135, :4322;
  replace :5148 with "`columns_1ball_fed` SEATED gives BALL_IN_HAND"; fix :691; accept section :5846-6080: replace
  `test_accept_possession_rule_table` (:5971) + `..._covers_every_pattern` (:5976) with a (pattern, evidence) -> decision
  table over 7 patterns x {EMPTY, SEATED, UNKNOWN}; delete :5905, :5928; add
  `test_accept_columns_seated_feeds_b_from_bb_without_a_flag`; parametrise :5953 over magazine only;
  `test_every_pattern_entry_opens_on_a_rest` (:4960) over (pattern, cup); add
  `test_a_reload_token_after_the_pattern_is_ignored_by_the_node` (P1 only, removed at P2);
  tests/ros/test_skill_node_reload_only.py:33-46.
- **Docs:** INV:119/120 clauses -> "keyed on (pattern, cup evidence)"; INV:58 I-PLAN-8 tests clause; rehearsal doc
  `tests/hardware/session_skill_stack_rehearsals.md` rows 21-22; logbook (reload field semantics, old-bag caveat).
- **Gate:** `./run_tests.sh --full` (change under `ros_ws` skill stack; full tier per CLAUDE.md for hardware sittings).
- **Risk:** low-medium (lob gone; SEATED columns always calls BB). **Status: DONE 2026-10-17 (9eed91c1; full gate 6716 passed + serial 7, exit 0).**
### P2 - D4 interface removal
- **Scope:** `Juggle.action`, orchestrator parse, GUI checkbox + cup preview + diagram, bench, conftest, panel harness.
  Interface rebuild + FULL launch restart.
- **Tests:** tests/ros/test_orchestrator_node.py:1118-1169 (`columns,reload` refused as unknown token; bare `reload` still
  the pattern); tests/ros/test_gui_juggle_defaults.py:70-79; tests/ros/test_skills_plan_bench.py:905, :913; the panel
  harness; remove the P1-only ignored-token test.
- **Docs:** `Juggle.action` comments; rehearsal rows 18-22 drop `reload: true` from goal literals (row 22 "SEATED for
  columns (either reload)" rewritten). One `/audit --unstaged` at the end of P2 covers P1+P2 (O3).
- **Gate:** `./run_tests.sh --full`; `colcon build` of `jugglebot_interfaces` + `jugglebot`.
- **Risk:** low. **Status: DONE 2026-10-17 (4cccece2; interfaces rebuilt, FULL launch restart + GUI hard-refresh owed; `TWO_FEED_LANDED = false` in juggle-panel.js until P3 (flipped true at P3); audit P1+P2 applied).**
### P3 - the two-feed chain
- **Scope:** sim probe (§ 5 i) FIRST; then `_run_columns` split, `_start_columns_feed`, `_start_reload(then_columns,
  chain_ctx)`, the three phases, `_resolve_chain_on_retire`, `_maybe_advance_chain`, the `_on_announcement` claim
  tightening, the timeout branch, `COLUMNS_CHAIN_BB_REFILL_TIMEOUT_S` (after § 5 ii); lift the P1 `REJECTED_NO_BALL`
  stop-gap for `columns` EMPTY.
- **Tests** (new, test_skill_node.py): `test_columns_empty_cup_opens_with_the_reload_of_a_at_the_feed_site`,
  `..._a_swap_arms_the_chain_ctx_not_none`, `..._chain_waits_for_decay_end_cup_seated_and_bb_idle_with_ball`,
  `..._chain_runs_the_seated_columns_tail` (asserts `_start_columns_feed` gets the same plan the SEATED path builds),
  `..._second_announcement_latches_ball_1_and_installs_compile_columns`, `..._a_missed_catch_ends_without_asking_bb_again`,
  `..._an_empty_cup_at_the_gate_is_rejected_no_ball`, `..._gate_timeout_names_bb_state`,
  `..._stop_during_chaining_does_not_resurrect`, `..._late_a_announcement_during_b_await_bb_idle_is_not_claimed`,
  `..._by_host_between_feeds_is_ignored`, `test_every_pattern_entry_opens_on_a_rest[columns-EMPTY]`.
- **Docs:** INV REJECTED_BB / `ABORTED_BB_NOT_READY` enforcement columns add `_maybe_advance_chain`; INV:119 (columns EMPTY
  accepted); `_reload_ctx` phase comment (~SN:806); new rehearsal rows (columns EMPTY; Stop between feeds); logbook
  (full form, Discussion on two installs vs one up-front schedule)
- **Gate:** `./run_tests.sh --full`. Coverage caveat: the sim gate has no `_start_reload` and bypasses accept, so it
  certifies each half, not the join; the join, phases and Stop races are mocked-ROS only (blind to choreography). The first
  hardware sitting is the join's real gate: columns, cup EMPTY, `num_cycles` 1-2, bag recorded, dress-rehearsed on the
  loaded Jetson first.
- **Risk:** HIGH (live hardware path, unmeasured refill, new catch xy, new Stop races). **Status: code landed 2026-10-17/18 (audit W1 exception wrapper + W2 guarded latch pops applied, 18 chain tests), NOT FLOWN: first sitting = rehearsal rows 32-34, measure the BB refill time** (first sitting = rehearsal rows 32-34; refill constant still provisional).

## 5. Probes

- **(i) Sim reload at the feed site:** `SkillsGate.run_reload_attempt(site=columns_sites(125)[0])`
  (sim/skills_gate.py:2888), seeds 0-4, before any P3 test asserts the reload at the feed site. Refusal -> O2 fallback
  (reload site (-50, 0) + held-ball bridge transit), tell the owner. One-off in `/tmp/probe_*.py` unless reused, then
  `tools/probes/`. **RESULT 2026-10-17: accepted and caught 5/5 seeds at x = -50, -62.5 and -125 (the live feed site is x = -125 at sep 250, not -62.5); the site has no effect, O2 holds, no fallback. Output `temp/probes/reload_at_feed_site.json`.**
- **(ii) Hardware BB refill time:** empty BB hand after a throw -> time from throw to heartbeat IDLE + ball_in_hand, N >= 10
  throws, log max and p95; pin `COLUMNS_CHAIN_BB_REFILL_TIMEOUT_S` (proposal 15 s) with margin over the max, then write the
  test. Decision rule: if max refill > 12 s, raise the constant rather than shorten the gate; if refill is unbounded, stop
  and tell the owner.

## 6. Open residue

- **Sim reload gate `LIMIT_JERK` regression (found 2026-10-17, BISECTED 2026-10-17 to `08ec3fe6` 2026-09-30, the R5 columns feed-catch commit; good at `8364808c` 2026-09-24, 5/5 makes 2 per seed, dwell 0.30 s; after it the scenario gets 1 make, dwell 0.27 s and `LIMIT_JERK` in the continuation throws, catch itself fine; no config/limit change in that commit; hardware flew R5 cleanly after it, so this reads as sim-scenario drift, owner to decide whether to fix or re-baseline):** the stock `python sim/skills_gate.py --reload --seeds 0 1 2 3 4` is FAIL at HEAD 4cd33254 (seeds 0, 2, 3, 4 `LIMIT_JERK` in the continuation throws AFTER the catch; seed 1 PASS; docstring says it once passed 5/5; not pinned by any test; predates P1/P2). Not a P3 blocker: A's reload is catch-only (`n_throws=0`). 2026-10-19 update: the capture-move sim check (`tools/probes/capture_move_sim.py`, logbook `2026-10-19-held-axis-reload-catch-refuses-tracker-reaim`) found the refusal comes from the UNCLAMPED 41-45 mm re-aim on the second catch, because the sim gate leaves lateral authority unset; with the live 20 mm clamp the chain passes. Candidate fix: set the authorities in the gate (not done).
- Refill constant `COLUMNS_CHAIN_BB_REFILL_TIMEOUT_S = 15.0` provisional until rehearsal row 32 measures it.
- **Latent 'firing'-phase hang (pre-existing, audit 2026-10-18):** `_maybe_fire_reload_throw` sets `ctx.phase='firing'` (skipped by `_check_reload_timeout`) and calls the BB throw outside any try, the same shape W1 fixed for 'chaining'. Not new to this plan; fix when the reload path is next opened.
