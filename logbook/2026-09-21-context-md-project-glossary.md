---
title: "CONTEXT.md — the project glossary lands as a normative doc (documentation layer 9)"
type: feature
date: 2026-09-21
status: resolved
phase: "docs layer — domain glossary"
files_changed:
  - CONTEXT.md
  - DOCUMENTATION_GUIDE.md
  - CLAUDE.md
  - docs/agents/domain.md
  - logbook/INDEX.md
---

# CONTEXT.md — the project glossary

## What

A root `CONTEXT.md`: 60-odd terms in nine clusters (the machine, electronics and
links, states and poses, safety, juggling, skills and motion, ball tracking,
learning, process), each a one-or-two-sentence definition plus an `_Avoid_`
line naming the synonyms it displaces. Built in a `/grill-with-docs` session
with the owner; every term below was put to the owner as a question and
answered, none was assumed.

It is **normative** (owner decision): `DOCUMENTATION_GUIDE.md` gains layer 9
and § 2.9, § 6 classes the file as a normative spec rather than an exception to
the no-new-`ALLCAPS.md` rule, and `CLAUDE.md` adds it to the `/audit` trigger
list. `docs/agents/domain.md` drops its "doesn't exist yet" note.

## Why

The vocabulary had drifted far enough to cost sittings. "leg 6" in a Guard
message was read as a Leg fault on 2026-09-13 when it was the Hand; "armed"
named three different state machines; "the bridge" named a board, a ROS node
and a websocket; "session" meant both a powered period at the robot and a
Claude conversation; "gate" meant the test suite, a rung's acceptance
criterion and the feasibility check.

## Decisions worth keeping

- **Single context**, on `mvp-trajectory-bringup` — not in the `skill-stack`
  worktree, which held another session's uncommitted `feasibility.py` work. The
  skill-stack vocabulary (Skill, Schedule, Segment, Beat, REST) was read from
  that tree and defined here; skill-stack picks the file up at its next merge.
- **Inclusion bar**: project coinages, plus juggling/robotics words only where
  the project narrows them. Superseded vocabulary (MPC chain, FSM toss/reload,
  ILC) appears only on `_Avoid_` lines.
- **New words the owner chose**: **Feed** (a Ball Butler Throw to Jugglebot;
  retires "reload"), **Sitting limits** (retires "session limits"),
  **Soft E-stop** vs **E-stop button** (retires bare "E-STOP" and
  "guard E-STOP"), **Active pose** (retires "home" as a pose — Homing is only
  the zero-finding procedure).
- **The glossary may run ahead of the code.** Renames are their own
  grep-counted commits; historical entries are never rewritten.

## Discussion — two recommendations that did not survive the evidence

**Runsheet → Runbook.** I recommended "Runsheet" as canonical (it is the word
in CLAUDE.md's dress-rehearsal rule) and the owner agreed. Counting afterwards:
"runbook" 792 hits (321 outside the logbook), "runsheet" 5. The recommendation
was reversed and the owner agreed again; the one live "runsheet" in CLAUDE.md
was changed. Lesson: count before recommending a canonical word, not after.

**Active and the Hand.** The owner defined the Active pose as having the Hand's
motor controller Idle. The code on skill-stack does the opposite since R1:
ACTIVATE Parks the Hand at 0.0 rev, Closed-loop, and the opening REST is built
around that. The owner confirmed this is a *wanted behaviour change*, not a
description. A glossary describes what is true, so **Active** says nothing
about the Hand until the change lands (issue #19).

**"At home" was Stow.** CLAUDE.md's "condition number ~3-8 at home, not the raw
~450" had to be reworded once "home" stopped being a pose. I guessed the Active
pose; the owner guessed Stow. Computed with the repo's own
`compute_jacobian` / `compute_condition_number` (2026-09-21): Stow (offset
0,0,0, z = 574.3 mm) raw 428.78 / normalised 2.793; Active pose (offset
0,0,170) raw 555.70 / normalised 3.605. The raw figure pins it to Stow, and
"3-8" is the workspace spread (`test_singularity_map` observes a max of ~4.3).
The owner was right. Side finding, not fixed here: `ik_solver.py`'s frame
docstring calls that same origin the "initial (active) pose", while
`hardware_config.yaml` calls it STOW and the state machine's Active pose is
170 mm above it — the IK docstring is the odd one out against the glossary.

## Follow-ups (GitHub issues)

- #16 — decide on an Avoid-word enforcement test, on or after 2026-10-19. No
  test landed deliberately: prose-grepping has false positives
  (`teensy_bridge_node`, historical entries) that need design.
- #17 — rename `session limits` → `sitting limits` on skill-stack, bundled with
  the next forced admissible-box re-sweep (the rename touches `feasibility.py`).
- #18 — the can-bridge Guard reports the Hand as "leg 6".
- #19 — design change: Hand Idle, not Parked Closed-loop, while Active.

## Verification

Suite gate (`./run_tests.sh`, run 2026-09-21, after the audit's fixes and with
no edit between the run and the commit other than this paragraph): **PASS —
parallel 6897 passed / 4 skipped in 278.89 s, serial 1 passed in 15.23 s, total
300 s.** Docs-only is not an exemption here, so the tests that read the touched
paths, and what they assert: `tests/sim/test_logbook_front_matter.py` (this
entry's `title` / `type` / `date` / `status`, re-run scoped after this
paragraph was written), `tests/sim/test_docs_links.py` (links out of `docs/`;
it skips `docs/agents/`, and the edit there added no link),
`tests/sim/test_plans_index.py` (no plan was added or moved). Nothing asserts
on `CONTEXT.md` itself — that is issue #16.

One `/audit --unstaged` pass (2026-09-21): five findings, all narrative, none
behaviour-affecting; four applied, one (Leg defined as "one of the six linear
actuators" beside its own `_Avoid_: actuator`) left as a correct genus-species
definition.
