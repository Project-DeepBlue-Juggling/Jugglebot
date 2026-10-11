---
title: Outward catch re-aims capped at 10 mm (2 of 3 sitting-7b LIMIT refusals); the missed-catch rule's silent abstentions now logged
type: investigation
date: 2026-10-12
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / two-ball columns"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-12-skill-stack-r5-learner-support-collapse.md
  - 2026-10-11-skill-stack-r5-skill-node-response-starvation.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_skill_node.py
  - logbook/2026-10-12-skill-stack-r5-outward-reaim-and-catch-evidence.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
tags:
  - limits
  - catch-verdict
---

# Outward catch re-aim authority; missed-catch abstentions logged

Two units from the sitting-7b review (bag `2026-10-11_22-15-28`, launch.log
`~/.ros/log/2026-10-11-22-15-28-601295-jetson-3213327`, L = line). They share `executor.py` and its test file, so
they land in one commit.

## What happened

- **LIMIT refusals.** Only three actual refusals, each logged on 4-5 lines. Two followed a P2 catch re-aimed
  +20 mm OUTWARD in x, away from P1, with the tracker asking for +21 mm and +44.6 mm:
  - L760: the next P2->P1 transit was 145 mm instead of 125 (+16 %), and the next catch was refused on leg 1
    (vel 352.7 against 350).
  - L1745: same pattern, leg 1 acc 5222 against 5000.
  - Overall, 7 of 56 tracker-aimed catches were re-aimed +20 mm outward. The third refusal (LIMIT_JERK) is the
    learner support collapse; see the sibling entry.
- **L1990, ABORTED_NO_RELEASE.** Throw 8 of Ball 2 was a physical miss, 42 mm off the cup. The MISSED_CATCH stop
  rule did not end the pattern, and the NO_RELEASE guard stopped it one throw later.

## Discussion

- **Outward is the only harmful direction.** An outward re-aim lengthens the transit to the next catch site; an
  inward one shortens it.
  - Chosen: an asymmetric clamp on the CATCH re-aim only. Inward keeps the full 20 mm; the outward component is
    capped by a new parameter.
  - Rejected: re-certifying the box against +/-20 mm aim dimensions. That needs a re-sweep, buys margin only for
    the outward side, and leaves the 10 % margin consumed.
  - Unchanged: the learner's THROW lateral clip and the hop command, which share `lateral_authority_m`.
- **Pre-registered value: 10 mm.** A replay ran the real executor and `install_segment` on the `columns_1ball`
  schedule with the logged tracker offsets, outward cap 0/5/10/15/20 mm, at 100 % and 98 % of the launch limits
  (scratchpad `q_drive.py`, `q_sweep.txt`).
  - The symmetric clamp refuses the same skill on the same leg class (acc 5153 against 5000).
  - 10 mm passes every case with >= 2 % margin. 15 mm fails at 98 % in two cases.
  - Cost: an outward ball far beyond the authority is caught further off-centre. The typical +21 mm ask is now
    about 11 mm off the cup instead of 1 mm. The sitting's extreme +44.6 mm ask was already 25 mm off at the
    20 mm cap and is now about 35 mm off, against a capture radius of roughly 35-40 mm (that radius is a memory
    note, not a code constant).
  - `catch_outward_authority_mm:=5` is the conservative launch override.
- **L1990: the review's hypothesis is withdrawn.** It proposed a BLIND abstention under a 701 ms hand heartbeat gap
  (L1976). That gap was the 10 Hz heartbeat mask: the same line counts 97 encoder frames inside it. The bag's
  `/hand_telemetry` is healthy over the window (max gap 19 ms, cup empty 811.624-812.468), so the window
  [811.673, 812.26] should have fired.
- **What remains is the seated veto from late-stamped samples.**
  - `_on_hand_telemetry` stamps each sample at callback time; the message has no header.
  - All of skill_node's subscriptions share one MutuallyExclusive group (`_sub_cbgroup`), so the 100 Hz hand
    callback queues behind the 300 Hz mocap one.
  - Measured over 84 caught throws: skill_node's first-SEATED stamp lags the bag's raw rising edge by a median of
    about 50 ms.
  - The previous ball's last SEATED sample sits only 57-69 ms before the window opens. In the window replay, a
    60 ms lag still fires; an 80 ms lag stamps 2 SEATED samples inside the window and silently vetoes the miss.
  - This is MEDIUM confidence: the lag at that moment was never logged. What IS measured is the systematic lag
    and the margin. c5241d2e moved only the service clients, so it does not change this.
- **This unit logs the abstention instead of fixing the cause.** The three silent `continue`s in
  `_advance_catch_evidence` (history missing, window not live, seated veto) now each log a line, so the next
  sitting can confirm or refute the veto. The root fix is a separate decision:
  - **A hand_telemetry callback group of its own.** NOT just two lines: the shared group is what currently
    serialises the subscription callbacks' shared state (`skill_node.py` ~L635). The comment at ~L1015, "every
    subscription gets its OWN callback group", no longer matches the code.
  - **A bridge-side stamp on HandTelemetryMessage.**
  - **A wider `MISSED_CATCH_OPEN_AFTER_RELEASE_S`.** It is a normative constant.

  The safe direction holds in the meantime: a veto can delay a stop but never causes a false miss.
- **Heartbeat dropouts.** 26 episodes over 200 ms (hand 3, legs 23) are real bus/ODrive heartbeat loss reported
  by the can-bridge firmware's stale mask, not consumer starvation.
  - The Jetson link is clean in the bag: 0 seq gaps, 0 decode errors, heartbeat_age p99 12 ms.
  - 11 of the 26 lost >= 80 % of their encoder frames, which matches `plans/active/leg-bus-frame-drops.md`.
  - `/hand_telemetry` is republished from a cache at 100 Hz, so it cannot detect them.

## Fix

- **`executor.py`.**
  - `SkillExecutor(catch_outward_authority_m=...)`, where `None` keeps the symmetric clamp.
  - `_outward_unit` points from the pattern's other site(s) toward this site; a one-site pattern has none and stays
    symmetric.
  - `_clamp_lateral_to_schedule` caps the outward component at `min(outward, lateral)`. The AIM-LATERAL-CLAMPED
    line names the OUTWARD side when it bit.
  - `_advance_catch_evidence` logs `CATCH-EVIDENCE-BLIND` and `CATCH-EVIDENCE-ABSTAIN`, with valid/expected
    counts, the SEATED count, the max gap and the first SEATED offset. Neither contains `MISSED_CATCH`, the
    substring that means "the rule fired". Executor lines go to the DEBUG log, which reaches launch.log.
- **`skill_node.py`.** New parameter `catch_outward_authority_mm` (default 10.0), passed at all five executor
  construction sites.
- `executor.py` is not in the box gate's `_GATED_FILES`, so there is no hash change and no re-sweep.

## Verification

- **New tests:**
  - six `test_catch_*outward*/inward*/one_site/none` tests in `tests/motion/test_skills_executor.py`;
  - `test_missed_catch_gappy_window_abstains_with_a_logged_reason`, and the seated-veto and blind tests now
    assert their lines;
  - `test_the_catch_outward_authority_parameter_defaults_to_ten_and_plumbs` in `tests/ros/test_skill_node.py`.
- **Scoped run**, 2026-10-12, `pytest tests/motion/test_skills_executor.py tests/ros/test_skill_node.py -q -p
  no:cacheprovider`: 433 passed.
- **Gate:** in the commit message.
- **Control implications:** only the catch's x/y aim changes, not its time or velocity. The worst P2->P1
  transit is now about 135 mm (+8 % over nominal) instead of 145 mm.
- **Next sitting:** grep launch.log for `OUTWARD side` and `CATCH-EVIDENCE-ABSTAIN`. A "first SEATED at +0.00x s"
  next to a cup that the bag shows empty confirms the veto.
