---
title: "`levelling_complete` ownership resolved — Platform Teensy RAM, on the ODrive rail; a can-bridge reset cannot clear it"
type: bugfix
date: 2026-08-16
status: resolved
phase: "record correction"
related_plan: levelling-frame-contract.md
files_changed:
  - ros_ws/docs/levelling_frame.md
  - tests/hardware/session_anomaly_fixes.md
  - ros_ws/src/jugglebot_interfaces/msg/RobotState.msg
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - logbook/INDEX.md
subsystem:
  - docs
  - ros
  - can
tags:
  - docs
  - safety
---

# `levelling_complete` ownership — resolved from source

## Summary

The record disagreed with itself about which board owns `levelling_complete`.
`tests/hardware/session_anomaly_fixes.md` standing rule 2 and pre-brief item 2
said the flag is **Platform**-Teensy-held and that a **can-bridge** power-cycle
does not clear it; pre-brief item 1 and `ros_ws/docs/levelling_frame.md` said a
can-bridge power-cycle is what made it false at every launch. Both prior sessions
flagged the contradiction (2026-08-15) rather than resolving it. It went
load-bearing when the reboot-before-every-session rule was retired
(`2026-08-15-fw14-validated-arc-closed.md`), because it decides whether an
operator must re-`level` after a bridge reset.

**Verdict: the operator's model is CONFIRMED. A can-bridge power-cycle does not
clear `levelling_complete`, and no re-`level` is required for one.** The
refinement is that "persisted" means RAM-under-power, not NVM.

## Evidence (all read from source, 2026-08-16)

- **Storage.** `ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino:139-145`
  — `struct RobotState {...}; RobotState state = { false, false, 0.0f, 0.0f };`,
  a file-scope global zero-initialised at every boot of *that* board.
  `grep -rn EEPROM ros_ws/src/jugglebot/Teensy_code_platform/` returns nothing
  (exit 1), so there is no NVM anywhere in the sketch. Written only by
  `decodeStateCANMessage` on a dlc-8 `0x6E0` frame (`:460-470`, dispatched
  `:512-523`); read out by `createStateCANMessage` (`:441-455`).
- **The can-bridge holds no copy.**
  `Teensy_code_canbridge/platform_relay.cpp:39-75` only builds the `0x6E0`
  trigger/write frames; `can_buses.h:132-135` forwards the reply verbatim. The
  only `levelling_complete` in that firmware is the UDP RPC argument
  (`udp_protocol.h:562`), i.e. a wire field in transit, not storage.
- **Power domains.** Platform Teensy on Jugglebot's 12 V / ODrive supply —
  stated in code at `teensy_bridge_node.py:3426-3428` and `:5171-5176` ("the
  Platform Teensy shares the ODrive supply"); can-bridge on the Jetson's 5 V rail
  — `logbook/2026-06-29-canbridge-phase2-coldstart-relay-state.md:262-266`,
  `tests/hardware/mvp_bench_runbook.md:195-197`. Different boards, different
  rails.
- **What actually clears it** — three storage events and one impostor:
  (1) the Platform Teensy losing power or being reset (the 12 V/ODrive supply
  dropping, or a flash), which clears `is_homed`, `levelling_complete` and
  `pose_offset` together; (2) `REBOOT_ODRIVES` →
  `_clear_cold_start_state_on_reboot` (`teensy_bridge_node.py:5304-5347`);
  (3) an explicit level/home write (`:5247-5302`). The impostor is a **read**
  failure: `_boot_read_cold_start_state` → `_read_cold_start_state_conservative`
  (`:5154-5203`) retries 3× and on total failure seeds the Jetson cache with
  `is_homed=False, levelling_complete=False, pose=0`, which `/robot_state`
  publishes verbatim (`:3231-3233`). **The `is_homed` boot-read transient covers
  `levelling_complete` identically** — same fallback tuple, same line. Racing
  that synchronous boot read (`__init__`, `:1928`) with a bridge that has just
  been power-cycled is the most plausible origin of the "cleared every launch"
  belief; the plainer confound is that a sitting which power-cycles Jugglebot
  itself drops the Platform Teensy's rail anyway.
- **Hardware corroboration.** On 2026-07-27 the auto-push won the discovery race
  on all **7** `/gravity_offset` publishes, of which the persisted **boot-push
  subset scored 5/5** (`2026-07-28-anomaly-fixes-validation-sitting.md`). That
  boot push is gated on `ctx.levelling_complete`
  (`orchestrator_node.py:327-334`), so the flag was **true** at five separate
  post-`level` relaunches in one sitting. The launch that came up `levelling=0`
  was the one following a Platform Teensy **flash** — a genuine reset of the
  owning board.

## LG-3 consequence — the 2026-07-28 disagreement is settled

`2026-07-28-anomaly-fixes-validation-sitting.md` § "`LG-3` — an unresolved
disagreement" recorded two analysts split on whether
`levelling_complete: true ∧ gravity_correction_loaded: false` is reachable.
Source settles it, and **both were partly wrong**:

- Analyst 1's recipe ("power-cycle the **Platform** Teensy, then relaunch") is
  **refuted**: that cycle zeroes `levelling_complete` too (`:145`), producing
  LG-1's state.
- Analyst 2 is **right** that no power-cycle of any board reaches it — the
  can-bridge stores nothing, the Platform clears everything.
- Analyst 2's conclusion, "the honest closure is a unit test, not robot time", is
  **too strong**. The state is reachable on hardware, deterministically, with no
  power-cycle: the orchestrator's persisted push is **one-shot per orchestrator
  boot** (`orchestrator_node.py:130` sets `_startup_offset_sent = False`, `:328-334`
  latches it True on the first IDLE entry and nothing resets it), `/gravity_offset`
  is VOLATILE with no re-request path, and `trajectory_node` starts with
  `_gravity_correction_loaded = False` (`trajectory_node.py:362`). So `level`, then
  restart **`trajectory_node` alone** with the orchestrator left up: the Platform
  Teensy still says true, no re-push happens, the new process holds identity.
  No race. The runbook's original whole-graph relaunch reaches the same state only
  by *losing* the discovery race, which it lost 0/7.

The check is now written with that as Route A and the relaunch as Route B, with
one bench caveat flagged in the runbook: `trajectory_node` is the sole binder of
:5557, so the replacement process's re-bind after the old one exits is unexercised
and must be confirmed once.

## Files corrected

- `ros_ws/docs/levelling_frame.md` — the 2026-08-15 "contradiction is
  unreconciled" amendment replaced by a § "Which board owns `levelling_complete`
  — RESOLVED 2026-08-16, from source" box carrying the mechanism and the four
  clearing paths; the "Teensy-persisted per-boot flag" sentence now names the
  Platform board; the stale unconditional "level manually after every launch or
  relaunch" re-pointed at standing rule 2's check-then-level (itself corrected
  2026-07-27).
- `tests/hardware/session_anomaly_fixes.md` — pre-brief item 1's wrong half
  struck through and resolved; the ⚠ box that offered ONE recipe for LG-1 and
  LG-3 split into two (the Platform power-cycle is right for LG-1, wrong for
  LG-3), with the re-home cost of that cycle stated; CHECK LG-1's variant row
  gains the same re-home note; CHECK LG-3 gains the reachability verdict and
  Route A/Route B.
- `ros_ws/src/jugglebot_interfaces/msg/RobotState.msg` — comment-only header over
  `is_homed`/`levelling_complete` defining *which* board's "bootup" the fields
  mean. **No field, type or order change**, so no `jugglebot_interfaces` rebuild
  is required for correctness. The un-scoped "since the last bootup" here is the
  root ambiguity the whole contradiction grew from.
- `ros_ws/src/jugglebot/jugglebot/orchestrator_node.py` — "(persisted across
  reboots)" was the most misleading line in the tree (it reads as NVM, and is
  false for the owning board's own reboots); now names the board and the actual
  scope. Kept to one line so the `orchestrator_node.py:165-167` citations in two
  runbooks stay valid.
- `ros_ws/src/jugglebot/jugglebot/state_machine.py` — "# Persisted on Teensy" now
  names the Platform board. One line, no shift.

**Deliberately NOT changed**: `Teensy_code_platform.ino`. The storage comment
belongs at the declaration, but ~30 committed line-number citations into that file
(tests, `ros_ws/docs/hand_command_continuity.md`, `sim/hand/trajectory.py`, and
plans this session must not touch) all point past line 145, so any insertion there
invalidates them for a comment. The statement lives in the contract doc and the
`.msg` instead. Also left alone because they were already **right**:
`trajectory_node.py:356`, `toss_sequencer.py:39`, `reload_coordinator_node.py:1578`,
`session_phase8_toss_hardware.md:93-104`, `session_phase8_toss_trace.md:89`,
`session_tilt_calibration.md:134`, and standing rule 2 itself.

## Verification

All four changed source/doc surfaces are comment- or prose-only; no behaviour
changes. The tests that actually read them:

- `python -m pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q`,
  run 2026-08-16: **34 passed in 0.45 s** (this entry's front matter + the
  loader's INDEX-independent scan).
- `python -m pytest tests/ros/test_levelling_frame.py tests/ros/test_orchestrator_node.py tests/ros/test_orchestrator_conduit_contract.py tests/ros/test_orchestrator_conduit_integration.py tests/ros/test_state_machine.py tests/ros/test_teensy_bridge_node_robot_state_freshness.py tests/ros/test_teensy_bridge_node_coldstart.py -q`,
  run 2026-08-16: **293 passed in 23.17 s** — the C-LEVEL-1 bypass guard plus
  every suite over the two edited modules and over the cold-start read path the
  verdict rests on.
- `python -m pytest tests/firmware/ -q`, run 2026-08-16: **399 passed in 19.12 s**
  — includes `test_platform_fw_version_xref.py` (10 passed), the suite that parses
  `Teensy_code_platform.ino` and whose line-number-sensitive xrefs are exactly what
  the decision not to edit that file protects.

Nothing in the suite asserts the *text* of `levelling_frame.md` or
`session_anomaly_fixes.md` (checked: every test referencing them cites them in a
docstring only), so the doc corrections carry no automated coverage — the guard on
them is the source citations they now carry.

Gate (`./run_tests.sh`, run 2026-08-16, box confirmed quiet — a concurrent `pytest tests/sim` had corrupted an allocation baseline during scoped verification): **5241 passed in 241.24 s, RESULT: PASS.**
