---
title: The launch/reload THROW's fresh-origin lead - 9 -> 13 knots (150 -> 250 ms of solve budget), one per-kind lead at both ends
created: 2026-10-16
status: completed
completed: 2026-10-16
archived: 2026-10-16
owner: harrison
last_updated: 2026-10-16
related_logbook:
  - 2026-10-16-throw-fresh-origin-lead.md
  - 2026-10-16-handoff-lead-flown-zero-splice-too-late-tracker-fit-late.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py::install_lead_s
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py::install_segment
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py::_svc_install_segment
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py::_stream_plan_locked
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/stream_handoff.py::StreamHandoff
---

# THROW fresh-origin lead

## 1. Why

The 2026-10-16 10:50 sitting lost its first fed-columns attempt to `ORIGIN_TOO_LATE` at THROW 0. The solve took 188 ms against the 150 ms that `LEAD_S` (9 knots, less the 3-knot wire read) allows, on a box loaded by a parallel session.

This is the class the handoff lead closed for the CATCH. The release is an absolute instant, so a late solve cannot be absorbed by moving the origin. It ends the attempt.

## 2. Decision

- **New lead.** `FRESH_THROW_LEAD_EXTRA_KNOTS = 4` gives `FRESH_THROW_LEAD_KNOTS = 13`, which is 0.325 s of lead and **250 ms of solve budget**.
- **One number at both ends.** `schedule.install_lead_s(kind)` returns the THROW lead for a THROW and `LEAD_S` for every other kind. It is read by:
  - `_assign_leads`
  - the `Skill.lead_s` default
  - the `install_segment` default (`lead_s=None`), which keeps the sim gate in step
  - `trajectory_node`, which takes the kind from the request.

  If the schedule's dispatch lead and the reserved origin differ, the difference becomes extra window (the 2026-10-02 class). No interfaces change and no `jugglebot_interfaces` rebuild.
- **Not changed.** A THROW that follows a release keeps `HANDOFF_LEAD_S`. `LEAD_S`, `CATCH_FREEZE_S` and the re-aim fences are untouched.

## 3. Numbers

**Solve replay.** `tools/probes/throw_fresh_lead_replay.py` read 62 fresh THROWs from five logs, 2026-10-13 to 10-16, measuring `t_inst - t_now`:

| Lead | Budget | Refused | Worst margin | Opening-lift leg step at install: before / after I-INSTALL-1 |
|---|---|---|---|---|
| 9 knots | 150 ms | 1/62 | -38 ms | ~0.040 rev (67 % of the 0.06 guard) / 0 |
| 12 knots | 225 ms | 0/62 | +37 ms | ~0.051 rev (85 %) / 0 |
| **13 knots** | **250 ms** | **0/62** | **+62 ms** | **~0.055 rev (92 %) / 0** |

**Why 13.** The pre-registered rule (budget >= 188 + 25 ms) gives 12. 13 is taken for +25 ms of solve margin. With the tail step removed (§ 4) it costs nothing at the wire.

**Dispatch gaps.**
- THROW 0 to the first CATCH: `0.500 - 0.025*EXTRA`, which is 0.300 s at 8 and covers THROW 0's whole 250 ms budget.
- REST to THROW 0 shrinks by 0.1 s: lift - 0.1 s; 0.9 s after DECAY.

**Columns feed.** The feed gate now needs `tau + launch_s + FRESH_THROW_LEAD_S + tick`. The 2026-10-16 fed attempt had about 2.2 s of slack.

## 4. Decision: the running record plays to `t0` (I-INSTALL-1)

The REST-tail step (THROW 0 installs while the REST it follows still streams; the old `_install` swapped the wire onto knot 0, the REST's terminal rest, at install) existed at every lead and only the 0.06 rev `_install_continuity_ok` bounded it. Owner, 2026-10-16: fix it, keep 13 knots. The emitter now plays the running record to `min(its end, t0)`, holds its terminal rest to `t0`, and switches there (`StreamHandoff`, one read `_stream_plan_locked`). The new record is the planning head from install, so the first CATCH (dispatched before THROW 0's `t0`) splices into it. The continuity guard is asked at `t0`. Stops and holds retire the handoff. Measured step at the switch: 0 at 9/12/13 knots. The lead is now a free parameter at the wire. A DECAY REST rides the same rule.

## 5. Done means

- DONE 2026-10-16: `./run_tests.sh --full` green (6666 passed, 11 skipped; serial 7), landed 205c527c. It covers `sim/skills_gate.py` through the `install_segment` default.
- DONE 2026-10-16: row 30 read from log + bag over two sittings, n = 10 fresh THROWs at origin +0.325 s (16:10: one, solve 0.028 s; 16:23: nine, solves 0.062-0.167 s of 0.250 s — the 0.167 s one would have been refused at the old 150 ms budget), 9 of 9 and 1 of 1 accepted, zero ORIGIN_TOO_LATE / STALE_STATE; bag leg step 0.0000 rev dispatch -> install -> t0 and across t0 on all ten (details in the entry).
- DONE: the entry carries both flown reads; archived 2026-10-16.

## Archival note (2026-10-16)

**Shipped (205c527c):** `FRESH_THROW_LEAD_EXTRA_KNOTS = 4` (THROW lead 13 knots, 250 ms of solve budget) selected by one function `schedule.install_lead_s(kind)` at both ends (dispatch lead and reserved origin), and I-INSTALL-1: a fresh-origin install streams the running record to its origin t0 (`TrajectoryNode._stream_plan_locked` / `StreamHandoff`), so the wire no longer steps at install (0.040 rev at 9 knots before; 0 at any lead after). **Why it closed:** full gate green at 205c527c; flown 2026-10-16 16:10 and 16:23 — 10 of 10 fresh THROWs accepted (solves 0.028-0.167 s of 0.250 s), zero ORIGIN_TOO_LATE / STALE_STATE, zero leg step at install or t0 in both bags. **Residue:** none of this plan's. The second sitting's other ends (ABORTED_NO_RELEASE x4, MISSED_CATCH, LIMIT_ACC, ABORTED_MODE_CHANGED) are hand- and catch-side and belong to the late re-aim and hand-release work, not here. Owner decision 2026-10-16: emitter fix + 13 knots over 12 or 13 knots alone. Entry `logbook/2026-10-16-throw-fresh-origin-lead.md`.
