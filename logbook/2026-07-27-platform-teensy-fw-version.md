---
title: Platform Teensy FW_VERSION — the only deployment in the stack that failed silently
type: feature
date: 2026-07-27
status: resolved
phase: "Self-toss anomaly fixes — hand-command-continuity Phase 6"
related_plan: "hand-command-continuity.md"
files_changed:
  - ros_ws/src/jugglebot/Teensy_code/Teensy_code.ino
  - ros_ws/src/jugglebot/Teensy_code/platformio.ini
  - ros_ws/src/jugglebot/Teensy_code/extra_script.py
  - controller/teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot_interfaces/msg/RobotState.msg
  - ros_ws/docs/platform_fw_version.md
  - ros_ws/docs/hand_command_continuity.md
  - tests/firmware/test_platform_fw_version_xref.py
  - tests/ros/test_teensy_bridge_node_coldstart.py
  - tests/ros/test_teensy_bridge_node_relay.py
  - tests/ros/conftest.py
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_phase8_toss_hardware.md
  - plans/archived/hand-command-continuity.md
  - plans/active/PROMPT-anomaly-fixes-orchestration.md
  - logbook/2026-07-27-velocity-continuous-prelude.md
  - .gitignore
commits:
  - bb15d9b
subsystem:
  - can
  - ros
  - controller
tags:
  - safety
  - testing
  - docs
---

# Platform Teensy FW_VERSION — the only deployment in the stack that failed silently

## Summary

The Platform Teensy carried no version of any kind, so an un-flashed board was
indistinguishable from a flashed one from the Jetson — the one deployment in this
stack that failed **silently** (a stale `colcon` install trips a `STALE` grep; a
stale `jugglebot_interfaces` build exits `trajectory_node` ~200 ms after launch).
It now declares `FW_VERSION` (currently `1`) and reports it in **bytes 5-6 of the
0x6E0 RobotState reply it already sends** — bytes every firmware ever built
zero-filled unconditionally at the same dlc 8, so a **pre-versioning board answers
with 0** instead of going silent. The verdict surfaces on `robot_state`, on
`link_status`, and as a `PLATFORM_FW_CHECK: OK|FAIL|UNKNOWN` log line. It **warns
and never refuses**. `Teensy_code/` also gets its first compile gate — the whole
sketch now builds for the Teensy 4.0, which nothing in the repository had ever
done. **NOT FLASHED**; the flash is the one Phase 4 already required, now
confirmable.

## Motivation

Phase 4 (`5369fc2`) shipped a firmware change to `Trajectory.h` that the operator
must flash to the Platform Teensy. Nothing anywhere could confirm they had. The
run's closing review named this as its main residual operational risk, and the
runbook's answer was a **four-link circumstantial chain** — right source in the
tree → the header compiles → you uploaded → the board rebooted. Every link is an
inference about the operator's own actions; none is an observation of the board.

The cost of getting it wrong is not cosmetic. § CHECK HAND-4's fix is a **no-op on
the clean path by design** (`smoothMoveDuration`'s `v0 == 0` branch is bit-identical
to the historical form), so an un-flashed board produces plausible-looking rows
that silently re-measure the pre-fix baseline. A working fix gets scored as broken,
and a powered sitting burns.

Phase 4 step 5 said to bump the firmware version "**if** the Platform Teensy
carries one", and correctly declined to invent one — inventing a wire protocol
unasked is not an implementer's call. The operator has now asked.

## Design

### The report path — the one real decision

The can-bridge announces its identity over USB serial and over UDP. The Platform
Teensy has neither: it is reachable only through the can-bridge over CAN3, and
there is no serial console attached during a launch. So the version had to come
back over that conduit, and the candidates were judged on **one property — what an
un-flashed board does**, not on elegance:

| candidate | verdict |
|---|---|
| **bytes 5-6 of the existing 0x6E0 RobotState reply** (chosen) | Every firmware ever built executed `m.buf[5] = m.buf[6] = m.buf[7] = 0;` unconditionally at dlc 8, so a pre-versioning board **ANSWERS, with 0** |
| a dedicated query frame / new RPC | An old board would not answer at all. The un-flashed signature becomes an **absence** — indistinguishable from a CAN3 hiccup, an unpowered board, or a bridge not forwarding a new id. Also needs a new `PlatformCanId`, relay trigger, reply-id predicate, `udp_protocol` regen and **a can-bridge flash**: a second silent deployment introduced to detect a silent deployment |
| the 0x7DF traffic report | No host consumer, not in the bridge's relay-reply set ⇒ can-bridge change + flash again |
| the 0x7DE tilt frame | Two float32s; full |
| serial banner only | Shipped as well, and useful at flash time — but "invisible from the Jetson" *is* the defect |

Properties, all deliberate: **no new CAN frame** (nothing added to the duty cycle
of the bus the 0x6D0 hand conduit shares, so nothing can preempt, delay, reorder or
drop a hand command), **dlc unchanged at 8** (the bridge's `(can_id, dlc)`
correlator and the host's `expected_dlc=8` await are untouched), **no can-bridge
change and no can-bridge flash**, and **direction-safe** (the bridge's
`state_write` zero-fills 5-7 and `decodeStateCANMessage` reads only bytes 0-4, so a
host write cannot poison the reported version).

## Discussion

### Why a detector and not a gate

The tempting change here — "a stale board must not be commanded" — is actively
dangerous, and the reasons are concrete rather than cautious:

1. **A refusal on the hand dispatch path destroys the abort.** `SetHandTrajCmd`
   carries the kind-3 retract, and a kind-3 clobbering an armed kind-0 is the
   **only un-arm mechanism the Teensy offers**. A gate there converts a skipped
   flash into "the abort stopped working with a ball about to launch".
2. **Refusing kind-1 after kind-0 has flown drops a ball** the pre-fix stack was
   catching 15/19 of. The throw and the catch-arm are separate dispatches from
   different nodes; refusing the second turns a cosmetic skew into a ball on the
   floor.
3. **The detector's own input can be legitimately unknown.** The version rides a
   cached relay read whose failure is a *documented benign transient* on this
   robot. `is_homed` has a harmless conservative fallback ("force a re-home");
   there is no harmless fallback for "refuse all hand commands".
4. **A stale board is today's behaviour, not a new hazard.** The cost of the skew
   is a mis-scored bench row — a diagnostic cost, which a loud warning fully
   addresses.

Plus the asymmetry that settles it: **a detector is monotonic, a refusal is not.**
Enforcement can be added later on evidence; a blind refusal cannot be un-shipped
from a board mid-sitting. This also matches the pattern being mirrored — the
can-bridge's `FW_VERSION` is documented as "a human-facing identity marker only",
with wire compatibility enforced separately by `PROTOCOL_VERSION`.

### Why the host's expected value is a second, hand-written constant

`rpc_args.PLATFORM_FW_VERSION_EXPECTED` is authored independently of the firmware's
`FW_VERSION` rather than generated from a shared source. The skew being detected is
**board vs tree**; a single codegen'd value would move in the source tree without
the board ever being flashed — i.e. it would agree with itself in exactly the
situation the check exists to catch. The drift risk this creates is closed by
`test_platform_fw_version_xref.py`, which fails in **both** directions.

By the same logic `0` is reserved for "pre-versioning" and can never be a release.
That is not a host convention imposed on the wire; it is what an un-flashed board
**physically transmits**, which is why it is trustworthy. A release numbered 0
would report every stale board as current — the detector switched off while still
looking switched on.

### Why the version is not a field of `RelayRobotState`

That namedtuple is rebuilt from scratch at three conservative-fallback sites whose
semantics are "the ODrive references may be gone" (boot default, total-read-failure
fallback, `REBOOT_ODRIVES` clear). The Platform Teensy stays powered through all
three and its firmware cannot change without a flash. Folding the version in would
make a `REBOOT_ODRIVES` silently erase a known-good version and raise a false skew
alarm — training the operator to ignore the check, which is how a detector dies.

### FINALIZE — the reviewers' convergent findings, and what changed

Three independent reviewers ran on this phase. Four findings survived my own
verification; each is recorded with the fork it forced.

**1. A claim in the code that was false — and the fix was to make it true.**
All three lenses landed on the same comment: it said that assigning the two new
`RobotState` fields as plain attributes means "on a stale `jugglebot_interfaces`
build this raises inside the 100 Hz timer and the node **dies loudly**". It does
not. The assignment sits inside `_publish_robot_state`'s own
`try/except Exception` with `throttle_duration_sec=5.0`. The invoked precedent is
genuinely different: `trajectory_node._publish_status` has **no** handler, so rclpy
re-raises out of `spin()` and the process exits — impossible to miss. Here the same
stale build degrades to **one throttled ERROR per 5 s and a silently-dead
`/robot_state`**, while the node stays in `ros2 node list` looking healthy.

The failure this enables is sharp, and it is *worse* after this phase than before,
because `/robot_state` is what the orchestrator waits on: it stalls in BOOT and
reports "Check power and CAN connections", routing the operator to the CAN bus for
a pure deployment fault. Worse still on a *partly* stale tree — if
`jugglebot_interfaces` already carries `gravity_correction_loaded` from an earlier
sitting, `trajectory_node` does **not** exit and the loud row-B signature the
runbook promises never appears at all.

Two forks here. **First**: correct the comment, or make the claim true? Deleting
the claim leaves a real silent-deployment hole in a phase whose entire thesis is
that a skipped deployment step must name itself — the same defect class, one layer
up. So the claim was made true: `_warn_if_robot_state_msg_is_stale()` runs once at
construction and logs a named, greppable `INTERFACES_STALE:` error listing the
missing fields and the exact rebuild command. **Second**: raise, or log? Raising
would be genuinely loud, but it converts a deployment mistake into a node that
refuses to start — a refusal shape, in the one phase that argues refusals must be
earned on evidence. It logs. The comment was also rewritten to state the traced
mechanism and explicitly not to repeat the claim it used to make, and runbook row
B was corrected (it said "floods"; it is one line per 5 s) with a new run-sheet row
**FW-2** (`grep INTERFACES_STALE`).

*Implementation detail worth recording*: the probe is `hasattr` on a constructed
instance, not class introspection. It is the only form correct for **both** a real
rosidl message (a missing field is absent from `__slots__`) and the dataclass
stand-in in `tests/ros/conftest.py`; `get_fields_and_field_types()` exists on only
one of the two and would have crashed every ROS test at construction. Found by
running it, not by reasoning about it.

**2. The instrument mis-scored a healthy board — the two-sided check.** All three
lenses converged on the `UNKNOWN` verdict. Its routing was "the relay/CAN3 read is
broken — **fix that first**", i.e. a hard ABORT into a bus investigation. But
`UNKNOWN` is exactly what this robot's **documented benign boot-read transient**
produces on a launch-only restart — the same miss that gives an unexpected re-home.
So a correctly-flashed board would have sent the operator hunting a fault that does
not exist. That is precisely the failure mode a bench instrument must not have: it
scores correct work as broken. The contract document was internally inconsistent
about this too — it cites the transient as a *reason not to enforce* while its own
verdict table treated the same reading as a hard fault.

I verified the no-recovery half myself rather than taking it: `_platform_fw_version`
is written only in `_record_platform_fw_version`, called only from
`relay_read_robot_state`, whose only caller is `_refresh_cold_start_state` —
dispatched at boot and thereafter **only on edges** (UDP link lost→restored, or
CAN3 `bus1_health` WARN/BUS_OFF→OK, with UNKNOWN→OK explicitly excluded). A clean
launch that merely missed its boot read produces neither edge, so the verdict stays
`unknown` for the whole session.

**The fork: doc fix or code fix.** All three reviewers offered the same code
option — retry the read from the 1 Hz health timer while the version is `None`. I
declined it, and the reason is not scope protection. That retry cannot read the
version without also reading the cold-start state, so a late success would flip
`is_homed` False→True mid-session, after the boot path already committed to the
conservative fallback and the orchestrator may already have acted on it. That is a
new discontinuity on the homing path, introduced to improve a diagnostic, in a
phase whose whole argument is "add a detector, not enforcement". The doc fix
removes the false abort completely; the code option is recorded as open.

**3. The test that pinned "warn, never refuse" did not pin it.** Two lenses
independently found — and both proved by mutation — that
`test_a_skew_does_not_gate_the_hand_dispatch_path` entered *below* the funnel a
refusal would be written into. It called `node._call_rpc(...)` directly rather than
the service handlers, and its `inspect.getsource` tripwire inspected only
`_svc_set_hand_traj` / `_svc_smooth_move_hand`. Both of those route through
`teensy_hand_traj_cmd`, which is the **single funnel** and therefore exactly where
this repo's own "one enforcement point" convention would put a gate. A gate there
was invisible to the test that claims to forbid it.

I reproduced this rather than trusting it, and validated the repair two-sided:
the corrected test now drives both real service handlers and extends the tripwire
to `teensy_hand_traj_cmd`. On the clean tree: **26 passed**. With a version gate
inserted at `teensy_hand_traj_cmd`: **`test_a_skew_does_not_gate_the_hand_dispatch_path`
FAILS** on the kind-3 retract assertion — the behavioural assertion fires first,
which is the stronger of the two guards. Mutation reverted and re-verified green.
The new `INTERFACES_STALE` test got the same treatment: deleting the constructor
call makes it fail, restoring it makes it pass.

**4. Refuted: the SRX_DIS hardening.** One lens filed (at NOT-PROVEN) that if CAN3
self-reception were enabled, the bridge's own dlc-8 `0x6E0` `STATE_WRITE` echo
could be consumed as a read reply and decode to version 0 — a false
`FAIL — PRE-VERSIONING` on a healthy board. A second lens actively refuted it from
the FlexCAN configuration; a third declined to file it, reasoning that it requires
a pre-existing invariant to already be broken, in which case `is_homed` is already
being read from the wrong frame.

My adjudication: the *consequence* is real and newly-acquired (before this change,
consuming a write echo was near-harmless), but it is conditional on an assumption
this phase does not weaken, and it is not a defect introduced here. It is now
recorded in the contract document so it is not forgotten. **The suggested code
hardening is refused, and that matters more than the finding**: "ignore a reply
whose bytes match the last write" would discard *correct* reads — immediately after
a legitimate `_write_is_homed`, the very next honest read matches the written bytes
exactly, by construction. A fix that breaks the healthy path to defend an unproven
one is worse than the risk. If this ever needs closing, close it at the source by
verifying `SRX_DIS` on the bench.

**5. The logbook's navigational surface still carried the defect as open.** Two
lenses flagged that `logbook/INDEX.md` and the Phase-4 entry still assert the
Platform Teensy has no `FW_VERSION` and that "row H4.0 is the only guard". The
implementer left both deliberately, on the principle that the logbook is a
historical record. That principle is right for the dated entry body and wrong for
`INDEX.md`, which is a *navigational summary* — a future session reading it
top-down would re-derive the retired four-link chain and never learn FW-1 exists.
Both now carry a supersession marker pointing here; the Phase-4 entry's own claim
is left in place beneath it, because what it recorded was true when written.

### The compile gate, and why it does not flash

`Teensy_code/` had no `platformio.ini`, so **nothing in the repository compiled
this sketch**. Phase 4 rewrote `makeSmoothMove` and could not build-check it at
all — a syntax or type error would first have surfaced at the bench with the robot
powered. There is one now, and the shipped sketch compiles and links for the
Teensy 4.0.

It deliberately has **no `upload_command`**. The pio image is not the image this
board has been running: linking the sketch's `std::vector` needs `-fno-exceptions`
plus a patched linker script (libgcc's `pr-support.o` emits a bare `.ARM.extab`
that falls outside the stock script's ITCM pattern and blows its `R_ARM_PREL31`
relocation). Switching the flash toolchain in the same sitting that validates
Phase 4 would confound the two — a misbehaving § CHECK HAND-4 could then be
Phase 4 *or* the toolchain. Keep flashing from the Arduino IDE; switch
deliberately, later, with its own powered re-validation. As a bonus, without
`upload_command` a stray `pio run -t upload` fails on PlatformIO's glibc-2.34
loader helpers rather than flashing the live board.

## Implementation

**Firmware** (`Teensy_code.ino`): a `FW_VERSION` / `FW_NAME` identity block with
its bump history inline, mirroring the can-bridge's `canbridge_config.h:44`;
`createStateCANMessage` writes the version little-endian into bytes 5-6 (byte 7
stays reserved and zero, dlc still 8); one boot banner above the existing
`Teensy platform MCU ready.` line, which is kept verbatim because a runbook greps
it.

**Host**: `rpc_args.PLATFORM_FW_VERSION_EXPECTED` / `_UNVERSIONED` and
`decode_platform_fw_version`; `teensy_bridge_node._record_platform_fw_version` as
the single capture-and-verdict point, called from inside `relay_read_robot_state`
— the only place holding the raw bytes, so no future caller can bypass it (the
same reasoning that puts the C-HAND-1 stroke-window gate inside `_arm_hand_catch`
rather than at its call site). Three surfaces, each doing a job the others cannot:
typed `robot_state` fields (machine-readable, land in the bag), `link_status` at
10 Hz (the surface an operator can read live, since `ros2 topic echo` gives false
negatives for high-rate RELIABLE topics on this Foxy box), and the
`PLATFORM_FW_CHECK` log line (what a runbook greps out of `launch.log` after the
fact). Two message fields, not one: `platform_fw_version` alone is ambiguous at 0
("never read" vs "un-flashed"), and separating those is the whole point of the
UNKNOWN verdict.

Added at finalize: `_warn_if_robot_state_msg_is_stale()`, and the corrected
`teensy_hand_traj_cmd` coverage described above.

**Contract**: `ros_ws/docs/platform_fw_version.md` (**C-PLATFW-1**).

## Verification

`pytest tests/ -q`, run 2026-07-27 on the Jetson (venv
`~/Desktop/PDJ_venv/venv`): **3966 passed, 3 xfailed in 1411.32 s (23:31)**,
exit 0.

**The delta, accounted exactly.** The immediate pre-phase baseline was `2b0f3f0`
at **3947 passed, 3 xfailed** (itself `+4` over the run brief's `1e78f3f`
baseline of 3943, from four `seat_rate` parametrisations). `3966 − 3947 = +19`,
which is precisely the tests this phase adds: **10** in the new
`tests/firmware/test_platform_fw_version_xref.py`, **8** added by the implementer
to `tests/ros/test_teensy_bridge_node_coldstart.py`, and **1** added at finalize
(`test_a_stale_interfaces_build_names_itself_at_construction`). Confirmed by
collection: those two files now hold 36 tests. **The xfail count is unchanged at
3** — no test was weakened, skipped or xfailed.

*A prediction in this entry's own drafting was wrong and is corrected rather than
quietly reconciled*: before running, I wrote `+2` over the baseline (3949),
having mis-tallied the finalize additions against the implementer's 18. The
measured figure is `+19`. The number above is the measured one.

Neither allocation-budget flake (`test_hot_loop_allocation_contract`,
`test_t3b_h4_on_post_solve_allocates_within_budget`) failed in this run, so no
isolated re-run was needed.

Mutation evidence for the two instruments added at finalize is in the Discussion
above: the dispatch test fails on a gate at `teensy_hand_traj_cmd` and passes
clean; the `INTERFACES_STALE` test fails when the constructor call is removed.

**Not flashed, and hardware deferred.** Bench rows are in
`tests/hardware/session_anomaly_fixes.md` § Section FW — INST-4 and INST-5 at the
desk, **FW-1** and **FW-2** at stage 3, **H4.0d** in the § CHECK HAND-4 pre-flight.

## Deployment

1. `colcon build --packages-select jugglebot_interfaces jugglebot` +
   `source install/setup.bash` + **relaunch**. **Both** packages —
   `RobotState.msg` gained two fields. This was already mandatory since
   `TrajectoryStatus.msg` gained `gravity_correction_loaded` in `e36d60d`.
2. **Platform Teensy flash** of `Teensy_code/Teensy_code.ino`, **from the Arduino
   IDE as before** — not the can-bridge, not the CatchingCone, and *not* via
   `pio run -t upload`. This is the same flash Phase 4 already required; this
   phase adds no second flash, it makes the existing one confirmable.
3. **No codegen run** — no YAML changed.

## Open Questions

- **Should a TOSS START refuse on a definite `version == 0`?** It is the one
  refusal shape without any of the four failure modes above: it fires before
  anything is armed and before the ball leaves, exactly like the existing
  `REJECTED_NOT_LEVELLED`. Not implemented because it is a cross-subsystem
  contract change (a new `Toss.action` rejection reason + a new `TossObservations`
  input) and because it still needs an answer for the UNKNOWN case — refusing on
  UNKNOWN blocks all tossing on a healthy robot after a transient boot-read
  failure; not refusing leaves a hole.
- **Should the `UNKNOWN` verdict be recoverable within a launch?** Today it is
  not (see Discussion #2). The obvious retry also refreshes the cold-start cache
  and would flip `is_homed` mid-session; doing it safely means separating the
  version read from the state read, which is a real design change.
- **Should the Platform flash move to PlatformIO?** The gate now produces a
  working `firmware.hex`, so `pio run -t upload` is one line away — but its image
  differs from every prior flash of this board. Worth doing later, on its own,
  with its own powered re-validation.
- **`Teensy_code/hardware_config.h` and `protocol_config.h` are generated copies**
  and `pio run` compiles whatever is in the tree. INST-5 proves the sketch
  *builds*, not that those headers are *current* — a gap covered today by PF-3 and
  by `test_hand_smooth_move_xref.py` parsing the shipped header.

## Related

- Plan: `plans/archived/hand-command-continuity.md` § Phase 6
- Contract: `ros_ws/docs/platform_fw_version.md` (C-PLATFW-1)
- Phase 4 (the flash this confirms): `2026-07-27-velocity-continuous-prelude.md`
- Bench: `tests/hardware/session_anomaly_fixes.md` § Section FW
## Close-out — 2026-08-21

**Status `in-progress` → `resolved`.** Gated on the operator's build + flash +
relaunch, then `FW-1` / `FW-2` / `H4.0d`. All ran at the 2026-07-27 sitting:
`FW-1` and `FW-2` PASSED on **all six launches**, with
`PLATFORM_FW_CHECK: OK — Platform Teensy reports v1`. The check has since done
the job it was built for twice more — the board is on **FW 3** and the runbook's
FW rows are read against `teensy_link/rpc_args.py::PLATFORM_FW_VERSION_EXPECTED`
rather than a number restated in prose.

**Numbering since this entry**: `2` (2026-07-28, the C-HAND-2 decel feedforward)
and `3` (2026-08-18, the hand end-stop correction). The history lives inline in
`Teensy_code_platform.ino`'s `FW_VERSION` comment — obligation **D** — and is
deliberately not duplicated here or in `ros_ws/docs/platform_fw_version.md`.

**One thing this entry got right that cost a session to re-learn**: prose copies
of the expected version rot, and a runbook expecting a superseded version sends
the operator to re-flash a *correctly flashed* board. That is exactly what
`session_anomaly_fixes.md` rows **FW-1** and **H4.0d** did between 2026-08-18 and
2026-08-21, when they still expected `v2` on a `v3` board.

**Both operator questions remain open, unchanged**: whether a *toss start* should
refuse on a definite `version == 0`, and whether the `UNKNOWN` verdict should
become recoverable within a launch. Both are recorded in
`ros_ws/docs/platform_fw_version.md`; neither blocks anything.
