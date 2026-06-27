---
title: can_node → Teensy can-bridge feature-parity audit — 117-capability matrix + prioritized gap list (foundation-first)
type: investigation
date: 2026-06-27
status: resolved
phase: "11"
related_plan: teensy-can-offload.md
related_entries:
  - 2026-06-26-phase11-u5-six-leg-cutover
  - 2026-05-19-can-loss-fault-response-safety-inversion
  - 2026-06-25-phase11-u4-production-cutover
  - 2026-06-24-phase11-bench-cutover
files_changed:
  - ros_ws/docs/can-node-teensy-parity.md
commits:
  - 305f856
subsystem:
  - can
  - ros
tags:
  - safety
  - docs
---

# can_node → Teensy can-bridge feature-parity audit

## Summary

A read-only, desk-only audit enumerating **every** capability the retired
Jetson `can_node` provided and mapping each to its can-bridge (Teensy 4.1
firmware + `teensy_bridge_node.py` UDP bridge) equivalent, to make
"the legacy foundation is fully and robustly ported" **falsifiable** ahead of
any MPC re-architecture (the replanner rides the same Teensy leg-command
substrate regardless — so the port must be complete first).

**Deliverable:** [`ros_ws/docs/can-node-teensy-parity.md`](../ros_ws/docs/can-node-teensy-parity.md)
— a living parity matrix of **117 capabilities** across 9 domains:
**53 ported+validated · 7 ported+unvalidated · 37 PARTIAL · 20 GAP**, with a
severity-ranked gap list (64 actionable items).

**Headline finding:** the **automated orchestrator/state-machine cold-start
path is non-functional against the bridge** — and this is *masked* because the
entire U1–U5 cutover was operator-driven via direct bridge services, which
bypass the orchestrator. The **safety-critical fault behaviours** (CAN-loss
deferred-stow, soft-reset bounce-loop limiter, undervoltage gating) are by
contrast **faithfully ported with explicit invariant comments and equivalence
tests** — the reassuring half.

No code or firmware was changed (audit-only, per operator scope).

## Scope & method

Operator-chosen scope (pre-work sync): **audit-only, desk-only, maximum rigour**
— no gap-closing, nothing armed. The matrix lives in `ros_ws/docs/` (beside the
CAN-subsystem peers `safety.md` / `control_modes.md`), not top-level `docs/`,
because it is subsystem-coupled and edited as items flip validated (per
`DOCUMENTATION_GUIDE.md` §2.4).

Nine capability domains were each **mapped** by an independent reader against
`can_node.py` (1725 lines), `teensy_bridge_node.py` (1917 lines), the firmware
(`Teensy_code_canbridge/`), and the `controller/teensy_link/` host observers,
then **adversarially re-verified** by a second independent reader (refute-the-
mapping pass), with a final completeness pass. Grep-driven, not eyeballed;
`file:line` on both sides; `ported+validated` requires a quoted logbook/test/CSV.

**Process honesty — interrupted passes.** A session usage limit killed the
adversarial-verify pass for two domains (`telemetry-state-pubs`,
`levelling-tilt-persist-timesync`) and the completeness pass. The **high-severity
claims in those two domains were independently re-confirmed by direct grep/read**
during synthesis (see Verification); lower-severity rows there are marked
**map-only** in the matrix. 7 of 9 domains carry a full adversarial-verify pass
(3 status revisions applied).

## Key findings

**The headline (high confidence, self-confirmed).** Production
`jugglebot_launch.py` launches **both** `orchestrator_node` (line 208) and
`teensy_bridge_node` (line 216). The orchestrator drives cold-start through six
interfaces (the six rows of the matrix's headline table) the bridge does not
provide or always reports negative — grouped below:

- `BootHandler` gates on `firmware_validated` (`state_machine.py:232`); the
  bridge hardcodes `firmware_validated=False` (`teensy_bridge_node.py:682`) and
  the firmware `encode_get_version` is **defined but never called** → BOOT can
  never advance → FAULT after `BOOT_TIMEOUT_S`.
- `home_motors` **action** client (`orchestrator_node.py:61`) vs the bridge's
  `home` **Trigger service** → `server_is_ready()==False` → homing fails.
- `is_homed` / `encoder_search_complete` hardcoded False → skip-ahead never fires.
- `get_platform_tilt` client + `set_level_state` publisher have no bridge
  service/subscriber → levelling faults / gravity-offset persist is discarded.
- `activate_or_deactivate` client never rewired to the split `activate`/
  `deactivate`/`configure` Trigger services.

The U1–U5 cold-start validated because it was **operator-driven** (manual
`ros2 service call`), bypassing the orchestrator state machine entirely.

**The reassuring half.** The CAN-loss deferred-stow safety inversion
(`logbook/2026-05-19`) is preserved *exactly* in `fault_machine.cpp:160-208`
(header enumerates the 5 invariants), HW-validated on a single bench leg. The
soft-reset attempt limiter (`MAX_SOFT_RESET_ATTEMPTS=1`, the bounce-loop guard)
is ported verbatim with the "do NOT simplify" comment and an executable-spec
test. Undervoltage gating + uniform-UV-benign distinction, homing hardstop
arithmetic (1:1 xref test), and the leg CAN encoding (`'<fhh'`, codegen scales)
are all faithful.

**Top gap themes** (full severity-ranked list in the matrix):

1. **Orchestrator integration broken** (HIGH) — the six signals above.
2. **Robust `clear_errors`** (HIGH, known) — `CLEAR_ERRORS`/`REBOOT` RPCs are
   gated on `jugglebot_commands_allowed()` and return `ERR_BUS_DOWN` when the
   bus reads WARN/BUS_OFF; since `health_of` derives WARN from heartbeat
   staleness, a just-repowered bus blocks the very clear needed for recovery
   (the 2026-06-27 stale-UV-trips-`/activate` incident). `can_node` never gated
   the clear.
3. **`reboot_odrives` dropped two side-effects** (HIGH) — no watchdog/heartbeat
   suppression (so a reboot's ~10 s heartbeat silence **falsely trips the
   CAN-loss deferred-stow machinery**) and no `is_homed` invalidation.
4. **Entire hand command surface absent** (GAP, **real regression** — see
   Discussion §6) — `set_hand_*` / `smooth_move_hand` are GAPs; the can-bridge
   rejects `HAND_AXIS` ("platform Teensy owns the hand") and does not forward the
   `0x6D0 PLATFORM_TRAJ_CMD` channel. `can_node` was the Jetson→CAN conduit that
   forwarded these onto the bus; the can-bridge owns that CAN3 bus now and must
   replicate the conduit. `catch_coordinator` clients fail **silently**
   (service-not-ready → catch not armed). Fix direction: un-reject `HAND_AXIS`
   for hand-ODrive op-codes (axis 6 state/gains) + add a `0x6D0` forward RPC.
5. **Levelling / tilt / cold-start persistence GAP** (HIGH) — `get_platform_tilt`,
   `set_level_state`, and persisted-state read are all gone; the **gravity
   offset (`pose_offset_rad`) is lost across reboot and feeds every MPC target**
   (`mpc_bridge_node.py:144`).
6. **Leg-command path system-level unvalidated** (HIGH) — the 40 Hz
   run_mpc→bridge→UDP→Teensy→CAN stream has **never moved a leg under MPC** on
   hardware; the interp math is bit-for-bit cross-checked but the end-to-end
   path is component-validated only (the deprioritized compute-marginal hold).

## Discussion

CLAUDE.md makes the Discussion non-negotiable here: a plan-stated intent was
found violated, a non-obvious masking effect was the key insight, and a
documentation-placement contract was applied against the operator's first-pass
choice.

### 1. The plan's "interfaces stay identical" intent was not achieved — and that is mostly fine

`teensy-can-offload.md` ("Kept") states *"All ROS2 service signatures and topic
schemas — interfaces stay identical."* The audit found the bridge diverged
substantially: `home_motors` action → `home` service; `activate_or_deactivate`
→ three Trigger services; three subscriptions dropped; `can_traffic` /
`platform_target_reached` publishers removed. Most divergences are **defensible
re-architecture** (operator-driven cold-start, firmware-owned moves, ZMQ
setpoint ingress) — but the plan never reconciled them, so the **orchestrator,
which still speaks the old interface, was silently orphaned**. The audit's value
is exactly to convert "interfaces stay identical (assumed)" into a row-by-row
ledger of where they didn't and what consumes the difference.

### 2. The masking insight — why a broken cold-start path passed five bring-up units

The orchestrator-integration breakage is invisible in the U1–U5 logbooks because
the operator drove every step by hand (`ros2 service call /encoder_search`,
`/home`, `/configure`, `/activate`, `/deactivate`). Manual operation is the
*correct* posture for powered bring-up, but it means the automated path accreted
six independent regressions with zero test or hardware signal. This is the
single most important thing the desk audit surfaced that a hardware sitting
could not: the failure is in a path **nobody exercised**, not in a path that
failed. "Foundation is solid" is true for manual cold-start and false for
automated cold-start — and the replanner direction will need *one of* those
resolved (retire the orchestrator state machine, or rewire + re-expose the six
signals) before it can rely on autonomous arming.

### 3. The reboot → false-deferred-stow interaction is the subtlest gap

`can_node._reboot_odrives_steps` suppressed heartbeat tracking during the ~10 s
reboot window precisely so the watchdog would not interpret the deliberate
silence as a CAN loss. The bridge reboot path dropped that. On the new
architecture the cost compounds: the silence now trips `fault_machine`'s
`s_fatal_can_error` **and** arms the hard-won deferred-stow latch, so a routine
reboot would fire the 2026-05-19 safety-inversion machinery on reconnect. This
is a clean example of a gap that is invisible per-capability (reboot "works",
watchdog "works") but dangerous in interaction — caught only because the audit
mapped both behaviours and asked how they compose. (Recorded as a HIGH gap, not
fixed — audit-only.)

### 4. Why `ros_ws/docs/`, not top-level `docs/`

The operator chose "new docs/ reference doc." `DOCUMENTATION_GUIDE.md` reserves
top-level `docs/` for the general-audience MkDocs site (nav-registered, stable
subsystem knowledge) and routes subsystem-coupled normative references to the
subtree (`<subsystem>/docs/`, e.g. the existing `ros_ws/docs/safety.md`). A
parity matrix that is edited as items flip validated, and is tightly coupled to
the `ros_ws` CAN stack, is subtree material. Placed at
`ros_ws/docs/can-node-teensy-parity.md`; flagged for the operator to redirect to
top-level `docs/` + nav if preferred.

### 5. The standing validation risk worth repeating — no compiled-firmware test

The firmware fault machine (the most safety-critical ported code) has **no
compiled-C++ test**. Its correctness rests on two Python transcriptions
(`tests/firmware/test_fault_logic.py`, `controller/teensy_link/fault_logic.py`)
kept in three-way agreement with `fault_machine.cpp` by **manual** transcription,
plus on-hardware replays. A C++ edit that diverges from the mirrors passes the
suite. This is acceptable today (the logic is stable and HW-validated) but is the
highest-leverage place a future regression could hide.

### 6. The hand-conduit reframe (operator correction, 2026-06-27)

The audit's first-pass framing rated the hand command GAPs as "architectural /
possibly intentional / out of this audit's scope," on the reading that a separate
platform Teensy owns the hand. The operator corrected this: the hand has
**always** been a Platform-Teensy device (it predates the can-bridge), and
`can_node`'s job was the **Jetson→CAN conduit** for hand commands — it forwarded
them onto the bus it owned, directly to the hand ODrive (axis 6: `set_state`,
`set_pos_gain`/`set_vel_gains`) and to the Platform Teensy (CAN id `0x6D0`
`PLATFORM_TRAJ_CMD`: `set_hand_traj_cmd`, `smooth_move_hand`, verified at
`can_node.py:782-830,1626-1661`). The can-bridge now owns that same CAN3 bus, so
"the hand must be processed the same way it was in `can_node`" means the bridge
**must replicate the conduit**, not defer it. The `rpc.cpp:85-86` "platform
Teensy owns the hand" rejection conflates *"the Platform Teensy is the hand's
real-time controller"* (true, unchanged) with *"the can-bridge needn't relay
Jetson hand commands"* (false) — so the four `set_hand_*`/`smooth_move_hand`
GAPs and the cold-start hand-gain GAP are **real porting regressions** that
silently break the catch pipeline. The matrix's hand-subsystem section and the
"GAP" note were corrected with this framing; the genuinely cross-Teensy items
(inclinometer, persisted home/level state physically on the Platform Teensy)
remain for a *platform-Teensy* parity audit. Lesson for future sessions: do not
infer "off-scope" from "a different device runs it" — check whether `can_node`
held a *conduit* responsibility for it. Captured in memory
(`project_hand_platform_teensy_conduit`).

## Deliverable

[`ros_ws/docs/can-node-teensy-parity.md`](../ros_ws/docs/can-node-teensy-parity.md):
header + method, the headline finding, a "what is solid" section, status
summary, the severity-ranked gap list (64 actionable rows), the full per-domain
matrix (117 rows with `file:line`, differences, and validation evidence),
cross-cutting caveats, and the known-follow-ups crosswalk. It is the
definition-of-done artifact — update rows + counts as items flip.

## Verification

- **Baseline suite** (CLAUDE.md date/command/result triple): `pytest tests/ -q`
  (run 2026-06-27) = **1882 passed, 1 failed, 1 xfailed in 437.41 s**; the lone
  failure (`tests/sim/test_mpc_time_pathologies.py::…allocates_within_budget`,
  a wall-clock-budget assertion) was load-induced — it ran concurrently with
  git/grep commands. Isolation re-run (`pytest …::test_t3b_h4_on_post_solve_…
  -q`, 2026-06-27) = **1 passed in 7.36 s**. Effective baseline **1883 pass, 1
  xfailed**. No code changed this session, so the gate is informational.
- **Lynchpin self-confirmation** (direct grep/read, 2026-06-27, for the two
  unverified domains): launch runs `orchestrator_node`+`teensy_bridge_node`
  together (`jugglebot_launch.py:208,216`); `firmware_validated`/`is_homed`/
  `encoder_search_complete`/`levelling_complete`/`pose_offset_rad` hardcoded
  (`teensy_bridge_node.py:682,696-702`); `encode_get_version` defined but with
  no call site (only `can_buses.cpp:95` "handled elsewhere … Ignore here");
  orchestrator consumers as cited (`state_machine.py:232,239,252`;
  `orchestrator_node.py:52,58,61,77,104-106`).

## Open questions / recommended next steps

Ranked for a follow-up gap-closing session (each mirrors a proven pattern;
none done this session):

1. **Runtime check (read-only):** does the wedged orchestrator (stuck
   BOOT→FAULT) actively interfere with operator-driven operation — e.g. publish
   `control_mode='ERROR'` that `motion_bridge_node` turns into an ESTOP — or is
   it dormant? Use a dedicated **rclpy subscriber probe** on `/control_mode` +
   orchestrator state, **not** `ros2 topic echo` (flaky for high-rate RELIABLE
   topics on this Foxy box). This decides whether the orchestrator gaps are
   *latent* or *live*.
2. **Robust `clear_errors`** (HIGH, known): carve `CLEAR_ERRORS` out of the
   `jugglebot_commands_allowed()` bus-health gate (a single non-motion clear to
   a recovering bus is benign). Smallest well-scoped fix; cost bench time.
3. **`reboot_odrives` side-effects** (HIGH): reboot-in-progress latch that
   suppresses the CAN3 watchdog + deferred-stow arming for the reboot window;
   `is_homed` invalidation.
4. **Decide the orchestrator's fate** (HIGH, design): retire the state machine,
   or rewire it (`home` service shim or action; split-service activate client;
   re-expose `firmware_validated`/`is_homed`/tilt on the link).
5. **Powered re-validation:** six-leg lockstep deferred-stow reconnect descent
   (only single-leg validated); the end-to-end 40 Hz MPC leg-command stream
   moving the legs (tied to the run_mpc-on-β / replanner direction).
6. **Cross-Teensy:** a *platform-Teensy* parity audit for the hand command
   surface + inclinometer + persisted state (off this firmware's scope).

## Related

- Deliverable: [`ros_ws/docs/can-node-teensy-parity.md`](../ros_ws/docs/can-node-teensy-parity.md)
- [2026-06-26-phase11-u5-six-leg-cutover.md](2026-06-26-phase11-u5-six-leg-cutover.md)
  — the foundation-first re-scope that motivated this audit; the `clear_errors`
  and orchestrator-integration gaps it flagged.
- [2026-05-19-can-loss-fault-response-safety-inversion.md](2026-05-19-can-loss-fault-response-safety-inversion.md)
  — the deferred-stow contract the firmware preserves (and that a bridge reboot
  would falsely trigger).
- [2026-06-25-phase11-u4-production-cutover.md](2026-06-25-phase11-u4-production-cutover.md),
  [2026-06-24-phase11-bench-cutover.md](2026-06-24-phase11-bench-cutover.md)
  — leg-path + fault-scenario hardware validation evidence cited in the matrix.
- Plan: `plans/active/teensy-can-offload.md` ("Jetson-side code impact",
  "U5b operator next-steps").
