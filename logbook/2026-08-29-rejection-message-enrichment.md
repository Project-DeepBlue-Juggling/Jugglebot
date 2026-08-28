---
title: "Every limit-bearing refusal now names the value, the limit and the knob"
type: feature
date: 2026-08-29
status: resolved
phase: "toss-pipelined-preamble — 2026-08-28 evening sitting debrief, fix 3 of 3"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/outcome_detail.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - tests/ros/test_outcome_detail.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_session.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_continuous_node.py
  - tests/ros/test_reload_sequencer.py
  - tests/hardware/toss_trace_recorder.py
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_cadence_ladder.md
subsystem:
  - ros
tags:
  - toss
  - reload
  - docs
  - testing
---

# Every limit-bearing refusal now names the value, the limit and the knob

## Summary

Operator debrief after the 2026-08-28 evening sitting, request 3 of 3: *a
rejection must tell me what I asked for and what the limit was.* Until today a
refusal was a bare code — `REJECTED_WORKSPACE`, `REJECTED_DWELL`,
`REJECTED_BAD_GOAL(throw_delay_s)` — and the numbers behind it lived in a node
log line the operator had to know to go and find, in a console a goal storm had
often already scrolled away. The outcome **string** is the only artefact that
reaches every channel at once: the action result, the session's
`per_cycle_outcomes[]`, the toss record's `outcome` column and the node's
authoritative log line.

New pure module `ros_ws/src/jugglebot/jugglebot/outcome_detail.py` — `bound_msg`,
`range_msg` for minting, `base_outcome` / `outcome_subcode` for **matching** —
and every limit-bearing refusal in the three FSMs and the node now reads like
this (shape, with illustrative values):

```
REJECTED_WORKSPACE(|B.y| = 178.0 mm > 160.0 mm [toss_workspace_xy_mm])
REJECTED_DWELL(dwell 0.900 s < floor 1.041 s; max(throw_delay 0.550 + handoff 0.120, hand floor 0.490) — raise dwell_time_s or lower throw_delay_s toward 0.361)
```

*Built under the standing orchestration pattern: a read-only Opus diagnosis, a
build agent, and one independent audit over the whole three-fix wave (verdict: no
blocking findings; 5 findings, 3 applied). The owner was AFK and the work was
pre-authorised.*

## What changed

**Enriched** (each names the requested value, the nearest limit and the knob):
`DISPLACEMENT`, `WORKSPACE` (×4 mint sites — `|B.x|`, `|B.y|`, the z band, and
the live throw site `|A|`), `TILT_CLAMP`, `EVENT_VEL`, `CANT_MAKE_LEAD` (the
debounce site, which now says *debounce* explicitly — the code is minted at three
sites and the other two are physical lead floors), `POSITION(CODE: …)` — the
`go_to_pose` service **message**, previously discarded at the node —
`REACH_CENTER_DRIFT`, `NUM_THROWS`, `THROW_DELAY`, `DWELL` (×2, branch-aware:
the pipelined branch quotes the commit budget, not the delay), `CHAIN_UNREACHABLE`,
`NOT_CENTERED` (×2 — a finite offset is a geometry verdict; a NaN one means the
pose was never read, a different subsystem entirely), the reload FSM's
`CANT_MAKE_LEAD`, and `BAD_GOAL`, which now carries the offending **value** and
not just the field name.

**Deliberately left bare**: structural and freshness verdicts with no numeric
knob — `NOT_LEVELLED`, `WRONG_MODE`, `NO_BALL`, `HAND_*`, `MOCAP_STALE`, `TIER`,
`POSE_UNKNOWN`, the `BB_*` family. There is nothing to quote, and inventing a
number would be worse than saying none.

**Three production exact-match guards re-keyed** — this is the silent-death class
an enrichment creates, and it is the reason the module ships `base_outcome` in
the same commit as `bound_msg`:

| guard | was | now |
| --- | --- | --- |
| the auto-reload trigger (`toss_session`) | `outcome == 'REJECTED_NO_BALL'` | `base_outcome(outcome) == …` |
| the `ABORTED_NO_RELEASE` retry matcher | `outcome == NO_RELEASE_OUTCOME` | `base_outcome(outcome) == …` |
| `_TOSS_POSITION_UNKNOWN_TERMINALS` (the zombie-move superseder) | whole-string set membership | `(base_outcome, outcome_subcode)` tuples |

Each failure would have been **silent**: a guard that stops matching simply does
nothing. The first would end a session on an empty cup that could have reloaded;
the third would skip the best-effort `go_home` while an ack-timed-out move is
still executing — the one failure here that ends with the platform traversing
under a torn-down catch. All three are now pinned by regression tests, including
`test_the_zombie_superseder_survives_an_enriched_position_refusal`.

**`tests/hardware/toss_trace_recorder.py`**: `OUTCOME_RE` tightened to
`^Toss ([A-Z][A-Za-z0-9_]*)` — the old class included `()` and so captured a
dangling `REJECTED_WORKSPACE(` on an enriched line, or half of a
`REJECTED_THROW_ENVELOPE(END_STOP:…)` payload; the RJ-1 scrape matches on the base
code; `REJECTED_POSITION` joins `REJECT_WIRE_MAP` with its subcode ladder. The
full line is still the third element of every `outcome_lines` tuple, so nothing
is lost: **identity from the code, detail from the message.**

**Runbooks**: `session_anomaly_fixes.md` and `session_cadence_ladder.md` now
state the scoring rule once, as a rule rather than a list — *score every row on
the CODE, i.e. the text before the first `(`; the parenthetical is diagnosis,
never identity* — because a closed enumeration of "which codes are enriched" goes
stale silently.

**67 test assertions** were converted from whole-string equality to the
split-pin style already used at `tests/ros/test_toss_coordinator.py` (the
`REJECTED_THROW_ENVELOPE` row): the code pinned by
`outcome == expected or outcome.startswith(expected + '(')`, the content pinned
by a separate content assertion. The doctrine is that a test asserting the code
should not fail when the machine gets *more* informative.

## Discussion

### The fork: a parenthetical, or a new string field on the action?

There is in-repo precedent for the other choice — `BallButlerThrowCmd.Result`
carries `message` alongside `outcome` — so this was a real fork, not a
formality. The parenthetical won on four counts, and only one of them is effort:

1. **`per_cycle_outcomes[]` already carries per-cycle detail, for free.** A
   session returns N per-cycle outcome strings; a message field on the result
   returns *one* message for the whole session, and the interesting refusal is
   usually cycle 3 of 5.
2. **Composition propagates the detail upward for free.** The composers prefix,
   they never wrap: `ABORTED_CYCLE_` + the cycle's outcome,
   `STOPPED_RELOAD_` + the reload's. A detail minted in the innermost FSM arrives
   at the operator through two layers with no plumbing at all — and
   `base_outcome`'s split-at-first-paren is still correct for every composed
   form.
3. **A message field reaches NO consumer until ~6 are updated** — the two action
   IDLs, the node's two result fillers, the record writer and the trace recorder
   — and until the last of them lands, half the channels still show a bare code.
4. **Zero test cost was zero TEST cost, not zero work.** Worth saying plainly:
   the parenthetical's bill was 67 assertion conversions, and it was paid here.
   The message field's bill would have been an IDL change plus a `colcon build`
   in every consumer's path, which is a different kind of debt and a louder one.

### The provenance verdict on `REJECTED_DISPLACEMENT`

The operator's fourth question was sharper than a formatting request: *is the
displacement limit physics or arbitrary — and if it is arbitrary, remove it.*
Answering it required reading the gate rather than the code that reports it, and
the answer is that **it is two limits wearing one name**:

* **the closed-form reach bound** — physics, and it follows the **live** session
  limits (`reach_displacement_bound` now also returns *which* peak binds). Below
  roughly a 0.6 m apex the **jerk** term binds and it scales as **T³**: 93 mm at
  0.4 m, 60 mm at 0.3 m. That steepness is exactly why the refusal now names the
  binding term — the remedy for a jerk-bound refusal (a longer flight) is cheap,
  and the remedy for a velocity-bound one is not.
* **the 150 mm cap** (`toss_max_displacement_mm`) — **chosen**, and the honest
  provenance is that it is *the operator's own ordered working range*, set
  2026-07-28 (`plans/active/single-ball-toss.md` Phase E).

So: chosen, but **not arbitrary, and not removed**. It is kept for two reasons
that the closed-form bound cannot cover. The bound is **measured-optimistic above
T ≈ 0.75 s**, so past that flight it is the wrong side of the truth to be a
safety limit; and the cap is the **only pre-throw stop** standing between a
too-far goal and the mid-flight-infeasibility class that contract **C-REACH-1**
was written to kill — a `TOO_FAST` at `t_release` with the ball already airborne.
Hardware evidence, meanwhile, stops at **70 mm**: everything above that is
modelled, on both gates.

What the enrichment changes is that a refusal now says **which** of the two
limits bound, quotes the other one for context (they move under different knobs,
so reporting only the binder sent half these refusals to the wrong one), and
labels the limit triple `default` / `live` / `mixed` so an operator cannot ramp
`set_limits` on a term the session never reported.

**Offered to the operator, not taken unilaterally**: raising the cap is a
two-key YAML edit — `toss_max_displacement_mm` **and** `toss_workspace_xy_mm ≥
cap × 1.03`, the relational invariant pinned by
`test_local_constants_match_generated_config`; below that ratio the
centroid-vs-cup chain divergence re-binds at the cap edge and
`REJECTED_CHAIN_UNREACHABLE` comes back at the working range. A **true
per-direction physics gate** that would retire the cap altogether is banked as a
follow-on plan idea, not attempted here.

## Verification

* ROS sweep (`pytest tests/ros/ -q -n 4 --dist loadfile`, run 2026-08-29):
  **2617 passed, 1 skipped in 95.62 s**.
* Motion + toss gate (`pytest tests/motion/ tests/sim/test_toss_gate.py -q -n 4
  --dist loadfile`, run 2026-08-29): **1972 passed, 3 skipped in 149.55 s**.
* Trace-recorder synthesis matrix (`tools/probes/toss_trace_synth.py --all
  --verify`, run 2026-08-29): **matrix CLEAN** — the tightened `OUTCOME_RE` and
  the base-code RJ-1 scrape read enriched lines correctly.
* THE GATE: full gate (`./run_tests.sh --full`, run 2026-08-29): parallel **6648 passed, 4 skipped, 3 xfailed in 535.28 s**; serial **9 passed in 42.17 s**; total 584 s — RESULT: PASS.
* New: `tests/ros/test_outcome_detail.py` (8 tests over the four functions,
  including the subcode/prose discrimination and the composed-outcome forms),
  plus the enrichment regressions
  `test_the_displacement_refusal_names_the_binding_bound_and_its_remedy`,
  `test_the_reach_bound_names_which_peak_bound_it`,
  `test_tilt_clamp_names_the_aim_and_the_ceiling_when_the_node_supplies_them`,
  `test_chain_unreachable_quotes_the_predicted_centroid_and_the_box`,
  `test_the_pipelined_dwell_refusal_names_the_commit_budget_not_the_delay`,
  `test_workspace_precheck_rejected` (parametrised over the four sites),
  `test_not_centered_says_UNKNOWN_when_the_pose_was_never_read`,
  `test_the_position_refusal_carries_the_services_message_after_its_subcode`,
  `test_a_named_prepare_refusal_carries_the_nodes_numbers_to_the_terminal`, and
  the two matcher pins
  `test_the_two_terminal_matchers_key_on_the_code_not_the_whole_string` and
  `test_the_zombie_superseder_survives_an_enriched_position_refusal`.

## Outcome

A refusal is now self-diagnosing from the action result alone. The rule that
makes it safe to keep going is written down in the module docstring and enforced
by two functions: **mint with `bound_msg`, match with `base_outcome`** — so the
next enrichment is a non-event rather than a regression nobody sees until a
sitting.

⚠ **Deploy**: `cd ros_ws && colcon build --packages-select jugglebot` before the
next sitting — the launch runs the install space, so this and its two siblings
are inert until it is rebuilt (one build covers the whole wave;
`jugglebot_interfaces` is untouched — the enrichment deliberately needed no IDL
change, see the Discussion).

Sibling fixes from the same debrief: [[2026-08-29-bb-reload-busy-patience]] and
[[2026-08-29-position-busy-repoll]] — the latter's `REJECTED_POSITION(BUSY)` is
the refusal this entry taught to carry `trajectory_node`'s own sentence.

## Open questions / follow-ups

* **the cap raise is the operator's call** — two YAML keys, the ratio invariant
  above, and the reminder that hardware evidence stops at 70 mm;
* **a per-direction physics gate** to replace the chosen cap: banked as a plan
  idea, and the thing that would make it real is measurement above 70 mm;
* **the enriched set will keep growing.** The runbooks now state the scoring rule
  rather than an enumeration for exactly that reason — anything that adds a
  parenthetical must go through `bound_msg` / `range_msg`, and anything that
  matches an outcome must go through `base_outcome`.
