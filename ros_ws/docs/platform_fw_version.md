# Platform Teensy firmware identity — contract C-PLATFW-1

**Normative.** This document specifies how the Platform Teensy's firmware version
reaches the Jetson, and what the host may and may not do with it. It is the
written half of the repo's contract pattern (normative statement + one canonical
enforcement point + a test that fails on the omission); the other two halves are
`teensy_bridge_node._record_platform_fw_version` and
`tests/firmware/test_platform_fw_version_xref.py` /
`tests/ros/test_teensy_bridge_node_coldstart.py`.

Scope: the Platform Teensy sketch `ros_ws/src/jugglebot/Teensy_code/` and the host
that reads it. It says nothing about the can-bridge Teensy, which has had its own
`FW_VERSION` since 2026-07-16 and its own wire-compat mechanism
(`JbUdp::PROTOCOL_VERSION`, a hard reject in `decode`).

## The defect this closes

Until 2026-07-27 the Platform Teensy carried **no version of any kind**, so an
un-flashed board was indistinguishable from a flashed one from the Jetson: no log
line, no field, no warning.

That mattered more than it sounds, because **every other deployment step in this
stack fails loudly when skipped**:

| deployment | how a skip announces itself |
|---|---|
| `colcon build --packages-select jugglebot` | the runbook's pre-flight greps the *installed* copy and prints `STALE` |
| `colcon build --packages-select jugglebot_interfaces` | `trajectory_node` assigns a field the generated message lacks and **exits ~200 ms after launch** |
| a **Platform Teensy flash** | *(before this contract)* **nothing at all** |

Worse, the change the operator most recently had to flash — the velocity-continuous
`makeSmoothMove`, `plans/active/hand-command-continuity.md` Phase 4 — is by
construction a **no-op on the clean path**: its `v0 == 0` branch is bit-identical
to the historical expression. So "the capture looked exactly like the pre-fix
baseline" is simultaneously the designed PASS and the signature of a skipped
flash, and the two were not distinguishable from any observation available.

## The invariant

> **C-PLATFW-1.** The Platform Teensy declares a firmware version, reports it to
> the host over the existing conduit, and the host surfaces the verdict on every
> boot. A board that has not been flashed since the version was introduced must
> produce a POSITIVE, distinct signal — never silence, and never a value
> indistinguishable from a healthy board's.

Three obligations follow.

| # | Obligation | Owner |
|---|---|---|
| **D** | The firmware **declares** `FW_VERSION` with its bump history inline. | `Teensy_code/Teensy_code.ino` |
| **R** | The firmware **reports** it in bytes 5-6 of the 0x6E0 RobotState reply. | `Teensy_code.ino::createStateCANMessage` |
| **S** | The host **surfaces** the verdict — log, `robot_state`, `link_status` — on every read. | `teensy_bridge_node._record_platform_fw_version` |

## The report path, and why this one

The Platform Teensy is not reachable from the Jetson directly. It sits on CAN3
behind the can-bridge, and the host already talks to it through exactly one
request/response exchange: `STATE_READ` → 0x6E0 dlc-1 request → an 8-byte
RobotState reply relayed verbatim as a `PlatformFrame` and correlated by
`(can_id, dlc)`.

**Bytes 5-6 of that reply, uint16 LE. Byte 7 stays reserved-zero.**

The decisive property is not economy, it is what a *stale* board does:

> Every firmware built before 2026-07-27 executed
> `m.buf[5] = m.buf[6] = m.buf[7] = 0;` unconditionally, at the same dlc 8. **A
> pre-versioning board therefore ANSWERS — with 0 — rather than staying silent.**

That is what makes the check usable. The case that matters most produces a
*definite wire value*, not a timeout that would be indistinguishable from a CAN3
hiccup, an unpowered Platform Teensy, a wedged relay, or a can-bridge that does
not forward a new arbitration id.

Alternatives considered and rejected, on that same criterion:

- **A dedicated version query (new CAN id or new RPC).** An old board would not
  answer at all, so the un-flashed signature would be *absence* — the very
  ambiguity being removed. It would also need a new `PlatformCanId`, a new relay
  trigger, a new entry in the bridge's reply-id predicate, a `udp_protocol`
  regeneration and **a can-bridge flash**: a second silent deployment introduced
  to detect a silent deployment.
- **The 0x7DF traffic report.** Periodic and already on the bus, but no host
  consumer and not in the bridge's relay-reply set — so, again, a can-bridge
  change and flash.
- **The 0x7DE tilt frame.** Two float32s; no spare bytes.
- **Serial banner only.** The board does print `[boot] jugglebot-platform v1`, and
  that is genuinely useful at the moment of flashing — but there is no serial
  console attached during a launch, and "invisible from the Jetson" is the whole
  defect.

Consequences of the chosen path, all deliberate:

- **No new CAN frame.** Not one extra byte of duty cycle on the bus the 0x6D0 hand
  conduit shares. Nothing can preempt, delay, reorder or drop a hand command.
- **dlc stays 8** in both directions, so the bridge's `(can_id, dlc)` correlator
  and the host's `expected_dlc=8` await are untouched.
- **No can-bridge change and no can-bridge flash.**
- **Direction matters.** Bytes 5-6 are meaningful *Teensy→host only*. The
  can-bridge's `state_write` zero-fills 5-7 and `decodeStateCANMessage` never
  reads them, so a host write cannot poison the reported version — pinned by
  `test_a_host_write_cannot_poison_the_reported_version`.

### One-shot per read, not polled

The version is captured wherever a RobotState read already happens: the boot read,
the UDP-reconnect re-read, and the CAN3-recovery re-read. There is no dedicated
poll. A firmware version can only change by a flash, and a flash reboots the
board; a periodic query would add CAN3 traffic for a value that is constant
between reboots. The continuous surface is the cached field on `link_status`, not
a repeated question on the wire.

## Numbering

`FW_VERSION` is a **hand-authored** constant in `Teensy_code.ino` carrying its
bump history inline — the can-bridge's pattern (`canbridge_config.h`), for the
same reason: a version number with no record of what each bump meant degrades to
a changed/unchanged bit, and the operator cannot tell an urgent re-flash from a
cosmetic one.

- **`0` is reserved for "pre-versioning" and is never a release.** It is not a
  convention the host imposes; it is what an un-flashed board physically sends. A
  release numbered 0 would report every stale board as current — the detector
  switched off while still looking switched on. Pinned by
  `test_zero_is_reserved_for_the_unversioned_board`.
- **`1` (2026-07-27)** is the first numbered release: the velocity-continuous
  `makeSmoothMove` (commit `5369fc2`) plus this identity block.

The host's expected value, `rpc_args.PLATFORM_FW_VERSION_EXPECTED`, is a **second,
independently-authored constant**, not a shared generated one. The skew being
detected is *board vs tree*: a single codegen'd value would move in the source
tree without the board ever being flashed, i.e. it would agree with itself in
exactly the situation the check exists to catch. The two are pinned together by
`tests/firmware/test_platform_fw_version_xref.py`, which fails in both drift
directions.

## The three verdicts, never collapsed

Read `platform_fw_version` together with `platform_fw_version_read`:

| verdict | wire state | `link_status` | log |
|---|---|---|---|
| **OK** | read landed, version == expected | `1` | `PLATFORM_FW_CHECK: OK` (INFO, on change only) |
| **FAIL — pre-versioning** | read landed, version == 0 | `0 (PRE-VERSIONING)` | `PLATFORM_FW_CHECK: FAIL` (ERROR) |
| **FAIL — other release** | read landed, version != expected | the number | `PLATFORM_FW_CHECK: FAIL` (ERROR) |
| **UNKNOWN** | no authoritative read has landed | `unknown` | `PLATFORM_FW_CHECK: UNKNOWN` (ERROR) |

UNKNOWN and FAIL–pre-versioning must not be conflated: the first means *no read
landed* (the same read sources `is_homed`, so it independently forces a re-home),
the second is a board that needs flashing. Collapsing them would send an operator
to re-flash a healthy board while the real fault went undiagnosed. They share the
`PLATFORM_FW_CHECK` grep token so one search of `launch.log` returns all of them,
and they say different things so the search is decidable.

> ### UNKNOWN is not, by itself, a fault — relaunch before you investigate
>
> **The single most likely cause of UNKNOWN is a documented benign transient, not
> a broken bus.** The boot `STATE_READ` missing on a launch-only restart is a
> known, previously-investigated behaviour of this stack (bridge boot read →
> conservative fallback; it is also why you sometimes get an unexpected re-home).
> Treating UNKNOWN as "the relay/CAN3 read is broken, fix that first" sends the
> operator hunting a fault that does not exist **on a correctly-flashed board** —
> the instrument scoring correct work as a failure, which is worse than having no
> instrument.
>
> **Procedure on UNKNOWN: relaunch once and re-read.** Escalate to a CAN3/relay
> investigation only if it repeats. Corroborating co-signature in the same log:
> `cold-start boot read failed after N attempts` and `cold_start_authoritative`
> reading `0` on `link_status`.
>
> **The verdict does not self-heal within a launch.** `_platform_fw_version` is
> captured only in `relay_read_robot_state`, whose only caller is the cold-start
> refresh — dispatched at boot and thereafter **only on edges**: a UDP link
> lost→restored transition, or a CAN3 `bus1_health` WARN/BUS_OFF→OK recovery. A
> clean launch that simply missed its boot read produces neither edge, so the
> verdict stays `unknown` for the whole session. This is deliberate for now: the
> obvious fix (retry the read from the 1 Hz health timer) would also refresh the
> cold-start cache, flipping `is_homed` False→True mid-session — a new
> discontinuity on the homing path, in a phase whose whole thesis is "add a
> detector, not enforcement". Recorded as an open question rather than fixed
> blind.

A failed re-read **keeps the last known version** rather than downgrading it to
UNKNOWN — same policy as the cold-start cache's keep-stale branch, and for a
stronger reason: a CAN3 hiccup is no evidence that the firmware moved.

**A fourth reading exists that is not a verdict at all: no `PLATFORM_FW_CHECK`
line, and no `platform_fw_version` key on `link_status`.** That is not a board
state — it means the running `teensy_bridge_node` predates this check, i.e. the
`colcon build --packages-select jugglebot` step was skipped. Do NOT score an
absent `FAIL` as a pass. Rebuild both packages, source, relaunch, re-read.

**One inherited assumption, stated so it is not forgotten.** The host correlates a
relay reply by `(can_id, dlc)`, which is sound only while CAN3 `SRX_DIS`
suppresses self-reception of the bridge's own dlc-8 `0x6E0` `STATE_WRITE` — a
caveat that predates this contract and is carried in
`Teensy_code_canbridge/can_buses.h` and the generated `udp_protocol` headers,
flagged there as bench-unverified. It is asserted by the FlexCAN configuration and
by three independent comments in `can_buses.cpp`, and this phase does not weaken
it. But note what this phase *adds* to the consequence: were that assumption to
fail, a consumed write-echo would now also decode a version of 0 and report
`FAIL — PRE-VERSIONING` on a correctly-flashed board. The tempting hardening —
"ignore a reply whose bytes match the last write" — is **wrong and must not be
added**: after a legitimate `_write_is_homed`, the very next honest read matches
the written bytes exactly, so that rule would discard correct reads. If this ever
needs closing, close it at the source by verifying `SRX_DIS` on the bench.

## Warn, never refuse

> **The version is REPORTED and never ENFORCED. No code path refuses a command,
> a service call, a toss, or a state transition on a version skew.**

This is not caution for its own sake; each refusal shape has a concrete failure
mode that is worse than the thing it would prevent.

1. **Refusing on the hand dispatch path destroys the abort.** `SetHandTrajCmd`
   carries the kind-3 retract, and a kind-3 clobbering an armed kind-0 is the
   **only un-arm mechanism the Teensy offers** — a pre-release `SAFE_ABORT`
   depends on it (`toss_sequencer`'s ORDERING PRINCIPLE). A version gate there
   converts a skipped flash into "the abort button stopped working with a ball
   about to launch".
2. **Refusing kind-1 after kind-0 has flown drops a ball.** The throw and the
   catch arm are separate dispatches from different nodes. A gate that refuses the
   second turns a cosmetic skew into a ball on the floor, against a pre-fix stack
   that was catching 15/19.
3. **The detector's input can be legitimately unknown.** The version rides a
   cached relay read whose failure is a *documented benign transient* on this
   robot (a launch-only restart's boot `STATE_READ` falling back conservatively).
   The safe fallback for `is_homed` is "force a re-home" — wasteful but harmless.
   There is no equivalently harmless fallback for "refuse all hand commands".
4. **A stale board is today's behaviour, not a new hazard.** `smoothMoveDuration`'s
   `v0 == 0` branch is bit-identical to the historical form, so an un-flashed board
   differs from a flashed one only on a hand command that lands while the hand is
   moving — which the host-side gate (contract C-HAND-1 obligation H,
   `ros_ws/docs/hand_command_continuity.md`) already prevents on the toss path. The
   cost of the skew is a **mis-scored bench row and a wasted powered sitting**: a
   diagnostic cost, which a loud warning fully addresses.
5. **A detector is monotonic; a refusal is not.** Adding enforcement later is a
   one-line policy change the operator can authorise on evidence. A refusal
   shipped blind cannot be un-shipped from a board mid-sitting.

The can-bridge's own `FW_VERSION` is documented the same way — "a human-facing
identity marker only; it has no runtime/handshake effect"
(`tests/hardware/mvp_bench_runbook.md`) — with wire compatibility enforced by a
separate, wire-shaped mechanism. Adding a refusal here would diverge from that
pattern in the direction of higher risk.

**Enforcement is the operator's, and it is procedural**:
`tests/hardware/session_anomaly_fixes.md` § DEPLOYMENT MATRIX row C is a hard
ABORT on a skew. The software reports; the human refuses.

Whether a *toss start* (as opposed to a hand dispatch) should refuse on a
definite `version == 0` — the shape `REJECTED_NOT_LEVELLED` already has — is a
live question for the operator, deliberately left open. It is the only refusal
shape that does not have one of the failure modes above, and it still needs an
answer to "what does the sequencer do when the version is UNKNOWN".

## Build gate

`Teensy_code/platformio.ini` compiles the whole sketch (`pio run`) against the
real Teensy 4.0 toolchain, FlexCAN_T4, SCL3300 and the generated headers. It is a
**compile gate, not a flash path** — it deliberately has no `upload_command`; see
its header for why. Before flashing, the sketch must compile there.
