---
title: REBOOT_ODRIVES did not clear encoder_search_complete → orchestrator skipped encoder-search after a reboot → homing ODRIVE_FATAL loop
type: bugfix
date: 2026-07-02
status: resolved
related_plan: canbridge-foundation-coldstart-parity.md
related_entries:
  - 2026-07-02-canbridge-phase4-orchestrator-wiring
  - 2026-06-30-canbridge-phase6-reboot-latch
  - 2026-06-29-canbridge-phase2-coldstart-relay-state
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_coldstart.py
commits:
  - TBD
subsystem:
  - ros
  - can
  - cold-start
tags:
  - cold-start
  - reboot
  - encoder-search
  - orchestrator
  - hardware-bug
---

## Summary

A hardware bug found while validating the Phase-4 `REBOOT_ODRIVES` cold-start clear
(residual validation V1, the follow-up to the 2026-07-02 orchestrator-wiring sitting):
**`REBOOT_ODRIVES` cleared `is_homed`/`levelling_complete`/`pose_offset` but not the
in-session encoder-search bit** (`_encoder_search_done_session`), so the derived
`encoder_search_complete = is_homed OR _encoder_search_done_session` stayed **True**
after a reboot. The orchestrator's `HomingHandler` then **skipped encoder-search** and
homed on an **un-indexed encoder** (the ODrive MCUs lose their incremental-encoder
index on reboot), producing `ODRIVE_FATAL` and a `FAULT→BOOT→HOMING→FAULT` loop.

**Fix:** clear `_encoder_search_done_session` in the `REBOOT_ODRIVES` shared hook
(`_clear_cold_start_state_on_reboot`) alongside the reference fields, so
`encoder_search_complete` drops to False after a reboot and the orchestrator re-runs
encoder-search before homing — exactly as a fresh launch does.

## Symptom

At the 2026-07-02 Phase-4 residual sitting, with the robot already homed+levelled
(persisted), the operator called `/reboot_odrives`. The read-only probe
(`/tmp/phase4_coldstart_probe.py`) recorded:

```
robot_state flags: fw_validated=1 enc_search=1 is_homed=1 levelling=1   (already homed → IDLE)
robot_state flags: fw_validated=1 enc_search=1 is_homed=0 levelling=0   (reboot: is_homed/levelling cleared, enc_search STILL 1)
robot_state.error -> ['Fatal ODrive issue (Teensy fault_state=NONE).']
orchestrator_state -> FAULT ; control_mode -> 'ERROR'
robot_state.error -> ['Fatal ODrive issue (Teensy fault_state=ODRIVE_FATAL).']
orchestrator_state -> BOOT -> HOMING -> (…~20 s…) -> FAULT -> BOOT -> HOMING -> FAULT   (loop)
```

The tell is `enc_search=1` **persisting** through the reboot while `is_homed`/
`levelling` correctly went to 0.

## Diagnosis

1. `_clear_cold_start_state_on_reboot` cleared `is_homed`/`levelling_complete`/
   `pose_offset` (both on the wire via relay STATE_WRITE and in the cache) but left
   `_encoder_search_done_session` set — a **deliberate** choice at the time,
   documented as "exact can_node parity" (can_node.py:1552-1566 clears
   is_homed/levelling/pose but never `encoder_search_complete`).
2. `encoder_search_complete` is DERIVED: `is_homed OR _encoder_search_done_session`.
   After the reboot: `is_homed=False` but `_encoder_search_done_session=True` →
   `encoder_search_complete=True`.
3. `HomingHandler.on_enter` (`state_machine.py:252`) skips encoder-search when
   `encoder_search_complete` is True → phase jumps straight to `home`.
4. **Operator-confirmed physical fact (2026-07-02):** a reboot loses the ODrive
   encoder calibration (the incremental-encoder index is *not* `pre_calibrated` to
   flash — which is why a *fresh* boot runs encoder-search, `enc_search 0→1`). So
   homing entered CLOSED_LOOP on an un-indexed encoder → `ODRIVE_FATAL`.
5. **Confirmation test:** a plain relaunch cold-starts cleanly (a fresh bridge process
   resets `_encoder_search_done_session=False`; `is_homed` was persisted as 0 → the
   orchestrator runs encoder-search → homes → IDLE). The only thing the relaunch
   changes is that encoder-search runs again — pinning the root cause to the stale
   `_encoder_search_done_session` after a reboot.

**This is a latent bug in can_node too** — can_node left `encoder_search_complete` set
after a reboot. It never bit there because a reboot-then-*orchestrator-auto-rehome*
path was rare (operators power-cycled the whole robot, resetting the bridge process,
or re-homed manually). Phase 4 made the orchestrator drive homing automatically after
a reboot, surfacing it.

## Fix

`_clear_cold_start_state_on_reboot` now also sets `self._encoder_search_done_session
= False` (under `self._lock`, alongside the reference-field clear). Rationale, stated
at the call site: **the reboot must clear everything the ODrive loses on reboot —
references (is_homed/levelling/pose) AND the encoder index.** This is a deliberate
divergence from can_node's literal behaviour (which had the latent bug). No orchestrator
/ state-machine change (locked-decision #1) — the orchestrator just sees the correct
`encoder_search_complete=False` and re-runs encoder-search.

## Verification

- `pytest tests/ros/test_teensy_bridge_node_coldstart.py -q` (2026-07-02): **17
  passed**. Extended `test_reboot_clears_all_cold_start_fields` to assert
  `_encoder_search_done_session` is cleared; **flipped**
  `test_encoder_search_complete_monotonic_across_reboot` (which asserted the old buggy
  "monotonic across reboot" behaviour) → `test_reboot_clears_encoder_search_complete`
  (asserts the derived `encoder_search_complete` drops to False after a reboot — the
  regression guard).
- `pytest tests/ -q` (2026-07-02): **1994 passed, 1 failed, 1 xfailed in 473.20 s** —
  the lone failure is the documented order-flaky `test_hot_loop_allocation_contract`
  (the twin of the t3b_h4 flake; they move between runs), confirmed **passing
  isolated** (1 passed in 7.49 s) — not a regression (memory
  `project_hot_loop_alloc_test_flaky`). No new/removed tests (the coldstart test was
  extended + one flipped, count unchanged at 17).
- **Hardware re-validation (V1):** PENDING — re-home, `/reboot_odrives`, confirm the
  orchestrator now auto-recovers BOOT→HOMING(**encoder-search**→home)→IDLE with no
  ODRIVE_FATAL loop.

## Discussion

### Why diverge from can_node parity

The can-bridge's cold-start relay design (locked-decision #2/#3) rests on "the store
forgets exactly what the ODrives forget." A reboot makes the ODrives forget their
**encoder index** as well as their homed/levelled references — so a faithful store
must forget the encoder-search state too. Keeping it (can_node's literal behaviour)
violates the very invariant the design is built on; the "parity" here was parity with
a *bug*. Climbing one level: the reboot hook clears the *complete* set of state the
ODrive loses, not a subset — the same discipline as the REBOOT shared hook clearing
is_homed+levelling+pose *together* rather than piecemeal.

### Why it was safe-but-stuck, not dangerous

Throughout the loop the legs were never armed (homing was *failing*, so no CLOSED_LOOP
hold established), setpoint output was off, and `run_mpc.py` was not running — so the
`control_mode='ERROR'` flapping issued an inert motor_guard estop (consistent with the
Phase-4 ESTOP analysis). The robot was stuck, not unsafe. The fix restores automatic
recovery.

## Related

- Surfaced during: [[2026-07-02-canbridge-phase4-orchestrator-wiring]] (residual
  validation V1, the `REBOOT_ODRIVES` cold-start clear).
- Reboot hook: [[2026-06-30-canbridge-phase6-reboot-latch]] (step 1, the watchdog latch)
  + [[2026-06-29-canbridge-phase2-coldstart-relay-state]] (step 2, the STATE_WRITE clear
  this bug is in).
- Plan: [`canbridge-foundation-coldstart-parity.md`](../plans/active/canbridge-foundation-coldstart-parity.md).
