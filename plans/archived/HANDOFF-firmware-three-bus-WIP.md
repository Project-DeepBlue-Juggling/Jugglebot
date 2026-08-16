---
title: Teensy CAN Offload — Three-Bus Firmware Refactor Handoff
created: 2026-06-03
status: active
branch: teensy-can-offload-firmware-three-bus-wip
parent_plan: teensy-can-offload.md
predecessor_handoff: HANDOFF-teensy-can-offload-firmware-wip.md
archived: 2026-07-05
---

# Firmware Three-Bus Refactor — Overnight WIP

## TL;DR

**Complete.** All six BLOCKING-INPUT items addressed across 5 work commits
(`6b4846c`, `4c0f67f`, `917a4e0`, `8ea119d`, `fd9aff9`) on
`teensy-can-offload-firmware-three-bus-wip` (base `07a895b`, nothing pushed; this
HANDOFF is the 6th commit).
Three subsystem-named CAN buses (bb=CAN1, cone=CAN2, jugglebot=CAN3) with a
third FlexCAN instance, ODrive RX decode moved to CAN3-only (incl. the hand),
0x7DD time-sync fanned out partial-failure-tolerantly on all three buses, and a
cone-absent TX gate on CAN2. Every firmware commit was `pio run`-gated before and
after; a final **clean** build (32.39 s, all TUs) is green (`dec 346656`). Zero
`FIXME(OQ-N)` were needed — §6a/§6b/§6c all resolved from ground truth. **Next
session's first action:** the morning reviewer runs `pio run` to confirm green,
then decides whether to correct the ADR-0013 / parent-plan reversed CAN2/CAN3 pin
tables (D1, flagged below) before any bench bring-up. No hardware was touched;
the cone-gate + per-bus 0x7DD + CAN3 ODrive cycle still need bench validation.

Unattended overnight pass bringing the can-bridge Teensy firmware from the old
two-bus assumption (CAN1 shared aux / CAN2 private leg) to the three
subsystem-isolated buses decided in [ADR-0013](../../docs/adr/0013-three-can-buses.md):
CAN1 = Ball Butler, CAN2 = catching cone, CAN3 = Jugglebot core (legs + hand +
platform Teensy). Firmware-only; no hardware touched; `pio run` is the only gate.

**Biggest finding (read this first):** the §6a CAN2 TX/RX disagreement resolves
*opposite* to the prompt author's guess. The FlexCAN_T4 library default pin map
(silicon-fixed IOMUXC mux, the §6a-designated top authority) is CAN2 **TX=1 /
RX=0** and CAN3 **TX=31 / RX=30**. The current firmware constant for CAN2 is
already correct; **ADR-0013, the parent plan, and this prompt's §4 table have
CAN2 and CAN3 TX/RX reversed.** Firmware + firmware-docs were made to match the
library; the ADR/plan pin tables are out of scope and are flagged for the human
under "Files NOT touched but probably should be". See decision D1.

## Parallel-session signal observed

None. At session start `git status -sb` showed only the one expected
pre-existing unstaged file (`ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h`),
matching the session-start snapshot. `origin/teensy-can-offload-firmware-wip`
was at the **same** SHA as local HEAD (`07a895b`), not ahead.

**Note on §5's "two pre-existing unstaged files":** §5 expected a second
unstaged file `ros_ws/src/jugglebot/Teensy_code_legbridge/platformio.ini`. That
directory **does not exist** — commit `07a895b` ("leg-bridge → can-bridge
rename") renamed `Teensy_code_legbridge/` to `Teensy_code_canbridge/`. So only
one pre-existing unstaged file is present. §5's reference is stale, not a
divergence signal. Left untouched.

## Plan (written before any code changes — frozen at session start)

Read-only recon established: pin constants are documentation-only (`can_buses.cpp`
uses the FlexCAN_T4 default pin map, never the `CAN*_*_PIN` constants); §6c
callsite count = 16, all in-firmware, zero external blast radius; FlexCAN_T4
`CAN3` template is valid (no §7 hard-stop); no one-shot-TX API exists.

Commit order (each firmware commit gated by `pio run` before AND after):

- **C1 (items 4, 6) — `canbridge_config.h`:** three-bus pin map with
  library-correct directions (CAN3_TX=31/RX=30 added; CAN2 kept TX=1/RX=0),
  rewritten bus-wiring comment block, cone-presence staleness constant, PRIO
  comments. Const/comment only.
- **C2 (items 1, 3, 6) — three-bus restructure + subsystem rename:**
  `can_buses.{h,cpp}` gains three subsystem-named instances (bb=CAN1
  counters-only, cone=CAN2 counters-only + gated TX, jugglebot=CAN3 full ODrive
  decode incl. hand), three send fns, three-bus `CanStats`, cone-absent gate
  (candidate 3). Atomic API rename rippled through `time_sync_master.cpp`
  (rename only, no fan-out yet), `leg_interp.{cpp,h}`, `rpc.cpp`,
  `fault_machine.{cpp,h}`, `axis_state.h`, `Teensy_code_canbridge.ino`
  (bus1_health←jugglebot, bus2_health←bb, cone deferred TODO),
  `profiling.{cpp,h}` (CanStats remap: jugglebot→wire-can1, bb→wire-can2, cone
  deferred TODO).
- **C3 (item 2) — `time_sync_master.{cpp,h}`:** fan out 0x7DD on all three
  buses, partial-failure-tolerant.
- **C4 (items 1, 4) — docs:** `README.md` + `BRINGUP.md` three-bus topology
  (library-correct pins) + bus-labelling note. Doc-only, no build gate.
- **C5 — this HANDOFF doc.** Doc-only.

Item 5 (BOM) is informational — no firmware change.

## BLOCKING-INPUT items addressed (HANDOFF L116-149)

- **Item 1 (CAN1/CAN2/CAN3 renames):** **done.** Subsystem-named API
  (`can_bb_send`/`can_cone_send`/`can_jugglebot_send`, CanStats bb/cone/jugglebot),
  leg setpoint + RPC + fault-machine TX moved to CAN3, ODrive RX decode (incl.
  hand) on CAN3 only, all narrative comments updated. Commits `4c0f67f` (code),
  `8ea119d` + `fd9aff9` (docs). See D3.
- **Item 2 (multi-bus 0x7DD time-sync fan-out):** **done.** `broadcast_0x7dd()`
  TXes on bb+cone+jugglebot, independent and partial-failure-tolerant. Commit
  `917a4e0`.
- **Item 3 (CAN2 cone-absent tolerance):** **done.** Candidate 3 (gated
  broadcast), encapsulated in `can_cone_send()` (gate on `CONE_PRESENT_STALENESS_US`
  = 5 s). Commit `4c0f67f`. Backout: revert the `can_cone_send` gate to an
  unconditional `send_on(can_cone, f)` (the bus init is otherwise standard). See D2.
- **Item 4 (pin assignments + CAN2 TX/RX direction resolution):** **done.**
  Resolved against the FlexCAN_T4 library default pin map (CAN1 22/23, CAN2
  TX1/RX0, CAN3 TX31/RX30); the firmware CAN2 constant was already right, the
  ADR/plan are reversed. CAN3 constants added. Commits `6b4846c` (config) +
  `8ea119d` (docs). See D1.
- **Item 5 (BOM / hardware notes):** informational; no firmware change. (The
  three-transceiver / six-termination BOM is already in ADR-0013 + the parent plan.)
- **Item 6 (CAN3 FD-capable peripheral, run classical):** **done.**
  `FlexCAN_T4<CAN3, ...>` confirmed valid in the pinned library; CAN3 carries the
  Jugglebot core bus at classical 1 Mbps, documented in `canbridge_config.h`.
  Commits `6b4846c` + `4c0f67f`.

## Commit log

- **C1 `6b4846c`** — `firmware(canbridge): three-bus CAN pin map + cone-presence
  constant (items 4, 6)`. `canbridge_config.h` only. Build gated OK before +
  after. Resolves item-4 CAN2 TX/RX against the FlexCAN_T4 library (D1). No
  behavioural change (pin constants are doc-only).
- **C2 `4c0f67f`** — `firmware(canbridge): three subsystem-isolated CAN buses +
  cone-absent TX gate (items 1, 3, 6)`. 12 files. The structural heart: three
  FlexCAN instances, RX restructure (CAN3 decodes legs+hand; CAN1/CAN2 count
  only), subsystem-named API + CanStats, cone gate, bus1/bus2 health + PROFILE
  remap. Mostly mechanical rename (16 callsites). Build gated OK before + after.
- **C3 `917a4e0`** — `firmware(canbridge): fan out 0x7DD time-sync on all three
  buses (item 2)`. `time_sync_master.{cpp,h}`. Build gated OK before + after.
- **C4 `8ea119d`** — `docs(canbridge): three-bus topology in README + BRINGUP
  (items 1, 4)`. Doc-only (no build gate). `/audit` skipped per the no-skills
  rule (flagged for the human).
- **C4b `fd9aff9`** — `docs(canbridge): fix stale CAN1-only time-sync note in
  BRINGUP (item 1)`. One line, caught by the finish-early stale-identifier sweep.
- **C5** — this HANDOFF doc (`docs(plans): three-bus-WIP HANDOFF`).

## Adversarial review (2026-06-03) + post-review fixes

After the refactor landed, a 7-dimension adversarial review (17 agents: one
audit-reporter per failure surface, each finding independently verified by a
skeptic prompted to refute it) was run over the three-bus diff. **10 findings,
all 10 survived verification.** Triage + actions:

- **#1/#2 (HIGH) — FlexCAN&lt;CAN3&gt; TX race.** The time-sync fan-out (C3) put
  the 0x7DD broadcast and the 500 Hz leg-setpoint ISR on the SAME non-reentrant
  FlexCAN object → a preempting writer could corrupt/drop a leg setpoint
  (jerky-motion class). **FIXED `8c8eb4f`:** IRQ-off (PRIMASK) guard on all three
  `can_*_send` (also closes the #8/#9 counter RMW race). Bench-validate interp
  jitter with the fan-out active.
- **#4/#5/#7 (LOW) — 64-bit RX-timestamp atomicity.** Asymmetric atomic-read /
  plain-write of `s_*_last_rx_us`. **FIXED `b77d6ab`:** `atomic_write_u64` in the
  RX callbacks + `atomic_read_u64` in `can_buses_stats`.
- **#3 (MEDIUM, latent) / #6 (LOW) — Jetson consumers stale vs the D4 wire remap.**
  `teensy_bridge_node` `has_fatal_can_error` read `bus2_health` (now Ball Butler)
  for the leg-fatal signal; `profile_monitor` legends mislabeled. **FIXED
  `7e5187c`:** consumer reads `bus1_health` (core bus) + test updated + a negative
  test added; profile_monitor relabeled. Full suite (`python -m pytest tests/ -q`,
  2026-06-03): **1685 passed, 1 xfailed, 455.70 s**.
- **#8/#9 (LOW)** — counter RMW race: resolved for free by #1's IRQ-off guard.
- **#10 (LOW, PRE-EXISTING) — CAN3 RX-drain throughput. RESOLVED 2026-06-04
  (commit e2b4cfb).** `events()` drained one frame per 1 ms tick (~1000 fps).
  Quantified: the *received* CAN3 load is ~2,240 fps steady (not the 5,340 fps
  bus total — SRX_DIS keeps our own ~3,100 fps TX off the RX path), so the
  256-deep rxBuffer overflowed and dropped ~55% of telemetry, feeding the cache
  ~112 ms-stale frames. The 2.0 s heartbeat watchdog / deferred stow stayed
  robust (real silence still trips it); the harm was to the stale-feedback
  500 Hz lead-clamp + E-stop backstops. Fix: bounded per-tick drain
  (`CAN_RX_DRAIN_BUDGET = 32`) in `can_buses_service()`, decode kept in the
  priority-5 task below the interp ISR. NOT introduced by the refactor (the base
  had the same one-frame drain). Full analysis + rejected alternatives +
  bench-validation TODO in
  `plans/archived/PROMPT-canbridge-rx-drain-throughput.md` (Resolution section).

## Decisions made autonomously

### D1 — CAN2/CAN3 TX/RX pin direction resolved against FlexCAN_T4 library (§6a, item 4)

**Chosen:** CAN1 TX=22/RX=23, CAN2 **TX=1/RX=0**, CAN3 **TX=31/RX=30**.

**Rejected:** the ADR-0013 / parent-plan / prompt-§4 values (CAN2 TX=0/RX=1,
CAN3 TX=30/RX=31) — they are reversed.

**Authority + evidence:** §6a authority order is "FlexCAN_T4 library source
(pin direction is fixed in silicon and the library hardcodes it) > datasheet >
docs > current firmware." The library's `FlexCAN_T4.tpp` `setTX()`/`setRX()`
DEF branch (invoked by `begin()` at `.tpp:155`, with no `ALT` override anywhere
in `can_buses.cpp`) hardwires:
- CAN3 TX → pin 31 (`GPIO_EMC_36`), RX → pin 30 (`GPIO_EMC_37`)
- CAN2 TX → pin 1 (`GPIO_AD_B0_02`), RX → pin 0 (`GPIO_AD_B0_03`)
- CAN1 TX → pin 22 (`GPIO_AD_B1_08`), RX → pin 23 (`GPIO_AD_B1_09`)

**Concrete failure mode the choice prevents:** wrong-direction CAN pins do not
communicate at all (§6a). Because the `CAN*_*_PIN` constants are
documentation-only (grep confirms they are referenced nowhere but their own
definitions; `can_buses.cpp` uses the library default mux), the hardware runs
on the library pins regardless of the constants. Writing the constants to match
the library means a future human who adds explicit `setTX`/`setRX` overrides, or
who reads the constants to wire transceivers, is not misled into a
non-communicating bus. The prompt author guessed "likely the firmware constant"
was the wrong one (§6a step 3); the evidence shows the opposite — the firmware
constant for CAN2 was already right and it is the ADR/plan that are reversed.
Per CLAUDE.md "push back when evidence contradicts a hypothesis," I followed
ground truth. **The ADR-0013 and parent-plan pin tables remain wrong and are
out of scope to edit — flagged for the human below.**

### D2 — CAN2 cone-absent tolerance = candidate 3 (gated broadcast), encapsulated in `can_cone_send()` (§6b, item 3)

**Chosen:** gated broadcast. `can_cone_send()` transmits only when a cone
heartbeat has been seen within `CONE_PRESENT_STALENESS_US` (5 s); otherwise it
silently skips the TX and returns false. The time-sync master calls
`can_cone_send()` unconditionally — the gate lives in the TX accessor, not the
master.

**Rejected:**
- Candidate 1 (one-shot TX / NACK-tolerant mailbox) — the pinned FlexCAN_T4
  exposes **no** one-shot-TX or abort-on-error API (verified by reading the full
  `FlexCAN_T4` class in `FlexCAN_T4.h` and grepping `FlexCAN_T4.tpp`: `write()`
  uses auto-retransmit mailboxes; `begin()`'s MCR config enables AEN/LPRIO but
  no one-shot bit; no public bus-off-recovery setter). Implementing it would
  require raw register pokes — not "compiles cleanly against the pinned
  FlexCAN_T4 API."
- Candidate 2 (bounded auto-recovery) — FlexCAN's *default* bus-off recovery is
  the auto-recovery loop the ADR explicitly says to avoid; bounding it requires
  raw ESR1/ECR monitoring + re-init with no clean library support — "guessing
  at a candidate-2 implementation," which §6b warns against.

**Concrete failure modes the choice prevents:** with the cone Teensy unpowered/
disconnected, an unconditional 100 Hz TX on CAN2 gets no ACK → the FlexCAN TEC
climbs 8 per failed retransmit → bus-off at TEC>255 → auto-recovery loop that
thrashes the error counters (ADR-0013 §Consequences). Gating the TX on observed
cone presence means we never transmit into the void, so TEC never climbs and the
bus never goes off — it *prevents* the failure rather than recovering from it.
Encapsulating the gate in `can_cone_send()` (the can_buses layer legitimately
owns bus state) keeps the time-sync master bus-agnostic about slave presence,
addressing the architectural objection §6b raised against candidate 3. Tradeoff
accepted: the gate adds a brief startup window where CAN2 0x7DD is withheld until
the first cone heartbeat arrives (~one cone-heartbeat period); harmless because
slaves hold their last IIR offset (same as the existing `time_synced()` gate).
**Bench validation required** — see "Needs hardware validation."

### D3 — subsystem-named API (`bb`/`cone`/`jugglebot`) applied (§6c, item 1)

**Chosen:** the §6c two-layer naming — peripheral names (`CAN1/CAN2/CAN3`) only
as FlexCAN template params; subsystem names (`bb`/`cone`/`jugglebot`) in the
public API, stats fields, and comments.

**Evidence for the gate:** the reproducible callsite count
(`grep -rn 'can[12]_send(\|CAN[12]_TX_PIN\|CAN[12]_RX_PIN' Teensy_code_canbridge/ | wc -l`)
= **16**, all inside `Teensy_code_canbridge/`; zero `can1_send`/`can2_send`
references anywhere else in the repo. Well within the 50-callsite downgrade
threshold, so the rename proceeds.

**Concrete failure mode the choice prevents:** the bus-name == subsystem-name
conflation is exactly what produced the two-bus→three-bus churn this refactor
exists to fix. With subsystem-named accessors, a future redesign that moves a
subsystem to a different peripheral changes only the one wiring line in
`can_buses.cpp`; every call site stays correct.

### D4 — UDP wire-name mappings repurposed, names unchanged (§5, items 1, 3)

**HeartbeatT2JPayload** (per §5's explicit policy): `bus1_health ← CAN3
(jugglebot)`, `bus2_health ← CAN1 (bb)`. Cone health deferred with a
`TODO(phase-10b)`. **ProfilePayload** (same forced 2-slots-for-3-buses situation,
resolved consistently with the health mapping): wire `can1_* ← jugglebot (CAN3)`,
wire `can2_* ← bb (CAN1)`, cone util deferred with a `TODO(phase-10b)`. The
on-wire field NAMES (`bus1_health`/`bus2_health`, `can1_*`/`can2_*`) are
**unchanged** — `udp_protocol.h` is out of scope and untouched. Rationale: CAN3
is the bus whose health most impacts hardware safety, so it takes the
more-prominent first slot in both payloads; keeping the two payloads' slot
assignments consistent minimises surprise for the Jetson consumer and the
reviewer.

### D5 — co-author trailer uses the actual model (Opus 4.8)

The prompt's illustrative HEREDOC shows `Co-Authored-By: Claude Opus 4.7`. The
actual session model is Opus 4.8 (the harness standing directive specifies the
4.8 trailer). Commits use the accurate `Co-Authored-By: Claude Opus 4.8 (1M
context) <noreply@anthropic.com>`. Trivial; noted for transparency.

## Deferred items needing human input

**Zero `FIXME(OQ-N)` in the diff** — every open question (§6a/§6b/§6c) and the §5
wire mappings resolved from ground truth (the FlexCAN_T4 library + the explicit
§5 policy), so nothing was silently guessed and nothing is blocked. The
following are non-blocking follow-ups (each cross-referenced elsewhere in this
doc), not decisions that gate the next session:

1. **Correct the ADR-0013 / parent-plan pin tables** — ✅ DONE 2026-06-03 (user
   confirmed the pins, then asked for the ADR fix; corrected ADR-0013 + the
   parent plan in lockstep, cross-doc consistency verified). The predecessor
   HANDOFF is left as a historical artifact (see "Files NOT touched but probably
   should be").
2. **Phase-10b protocol bump for cone telemetry** — the cone (CAN2) health and
   bus utilisation are not on the UDP uplink (only two wire slots exist;
   `udp_protocol.h` is out of scope). Marked with `TODO(phase-10b)` in
   `Teensy_code_canbridge.ino` (heartbeat health) and `profiling.cpp` (PROFILE
   util). A future codegen update adds a third slot. See D4.

## Build verification record

_(one line per `pio run`: timestamp + result + commit context)_

- 2026-06-03 20:24 — inherited tree (untouched, `07a895b` content) — **SUCCESS**
  (4.96 s). Baseline green confirmed before any edit.
- 2026-06-03 20:27 — C1 pre-stage — **SUCCESS** (7.28 s).
- 2026-06-03 20:28 — C1 post-commit (`6b4846c`) paranoia rebuild — **SUCCESS** (4.94 s).
- 2026-06-03 20:38 — C2 pre-stage — **SUCCESS** (7.10 s).
- 2026-06-03 20:39 — C2 post-commit (`4c0f67f`) paranoia rebuild — **SUCCESS** (4.87 s).
- 2026-06-03 20:42 — C3 pre-stage — **SUCCESS** (5.03 s).
- 2026-06-03 20:43 — C3 post-commit (`917a4e0`) paranoia rebuild — **SUCCESS** (4.86 s).
- C4 / C4b: doc-only, no build gate.
- **Final (date, command, result) triple:** 2026-06-03 ~20:50, clean build
  `pio run -t clean && pio run` (in
  `ros_ws/src/jugglebot/Teensy_code_canbridge`, venv active) — **SUCCESS**,
  32.39 s, every TU compiled from scratch, `text 210240 / data 34496 / bss
  101920 / dec 346656`. This is the authoritative final firmware state at the
  firmware tip `fd9aff9`. (Only warning: the pre-existing
  `actuators_intact_and_holding` unused-function — see Incidental findings.)
- 2026-06-03 ~20:55 — HEAD `bb7d3ea` (this HANDOFF commit, doc-only — firmware
  byte-identical to `fd9aff9`) incremental `pio run` re-confirmed — **SUCCESS**
  (5.03 s). The reviewer's first `pio run` reproduces this.

## Build failures

_(none so far)_

## Needs hardware validation (bench bring-up)

- **Per-bus 0x7DD sniff** on CAN1, CAN2, CAN3 — confirm 100 Hz frames in
  `pack('<II', sec, usec)` on each, BB synced on CAN1, cone on CAN2 (when
  present), platform Teensy on CAN3.
- **Cone-absent / D2 gate** — with the cone Teensy disconnected from CAN2,
  confirm: (a) the can-bridge does NOT enter bus-off on CAN2 (the gate withholds
  TX); (b) CAN1 and CAN3 0x7DD are unaffected; (c) when a cone IS connected, CAN2
  0x7DD resumes within ~5 s of the first cone heartbeat. Validate the 5 s
  `CONE_PRESENT_STALENESS_US` is appropriate against the real cone heartbeat
  cadence.
- **CAN3 ODrive cycle** — drive one bench ODrive on CAN3 IDLE → CLOSED_LOOP →
  position → IDLE; confirm telemetry decodes (the ODrive cache is now populated
  from CAN3 only).
- **Hand-axis telemetry on CAN3** — the hand ODrive decode moved from CAN1 to
  CAN3; confirm hand pos/vel still populates the cache.
- (Plus everything in the predecessor handoff's "Needs hardware validation",
  unaffected by this refactor.)

## Files touched

All under `ros_ws/src/jugglebot/Teensy_code_canbridge/` (verified by
`git diff 07a895b..HEAD --stat` — 16 files, nothing outside this dir), plus this
HANDOFF in `plans/active/`:

- `canbridge_config.h` — three-bus pin map + CAN3 constants + cone-staleness const + PRIO comments.
- `can_buses.h` / `can_buses.cpp` — three subsystem-named FlexCAN instances, RX restructure, cone gate, three-bus CanStats.
- `time_sync_master.h` / `time_sync_master.cpp` — multi-bus 0x7DD fan-out.
- `leg_interp.cpp` / `leg_interp.h` — leg setpoint TX → CAN3 (`can_jugglebot_send`) + comments.
- `rpc.cpp` — RPC ODrive TX → CAN3; `jugglebot_commands_allowed` reads jugglebot health.
- `fault_machine.cpp` / `fault_machine.h` — clear/IDLE TX → CAN3; watchdog narrative → CAN3.
- `axis_state.h` — cache-populated-by-CAN3 comments.
- `Teensy_code_canbridge.ino` — topology block, `bus1_health←jugglebot`/`bus2_health←bb`, task comments.
- `profiling.cpp` / `profiling.h` — CanStats→PROFILE wire-slot remap (slot1←jugglebot, slot2←bb).
- `README.md` / `BRINGUP.md` — three-bus pin map + bus-labelling note + pin-direction note.

**NOT touched (confirmed):** `Teensy_code/` (platform Teensy 4.0), any production
Jetson code under `ros_ws/.../jugglebot/`, `controller/`, `sim/`, `tools/`,
`tests/`, `udp_protocol.h` and the other out-of-scope headers, and the two
pre-existing working-tree items — `CatchingCone_code/hardware_config.h` (still
unstaged, unmodified by this session) and the (non-existent) legbridge file.

## Files NOT touched but probably should be (next-session recommendations)

- **`docs/adr/0013-three-can-buses.md`** — ✅ **FIXED 2026-06-03** (pin-correction
  commit on this branch, after the user confirmed the pins): CAN2 → TX 1 / RX 0,
  CAN3 → TX 31 / RX 30, plus a dated pin-direction correction note in the
  Decision section. (Was reversed; see D1.)
- **`plans/archived/teensy-can-offload.md`** — ✅ **FIXED** in the same commit: all
  topology diagram / network-setup / BOM / task-table pin references corrected to
  the silicon mux (8 occurrences).
- **`plans/archived/HANDOFF-teensy-can-offload-firmware-wip.md`** — left as a
  historical artifact. Its L110-114 / L135-144 tables still show the reversed
  CAN2/CAN3 directions, but that section is a "BLOCKING INPUT — resolve this"
  banner that explicitly flagged the disagreement for resolution; this HANDOFF +
  the corrected ADR record the resolution, so the predecessor is superseded
  rather than rewritten.

## What was deliberately NOT done

- **Did not edit ADR-0013 / parent plan / predecessor HANDOFF** despite finding
  their pin tables wrong (D1) — out of scope per §5; flagged above instead.
- **Did not rename the UDP wire field names** (`bus1_health`, `bus2_health`,
  `ProfilePayload.can1_*`/`can2_*`) — `udp_protocol.h` out of scope (D4).
- **Did not expose cone (CAN2) health/util on the uplink** — only two wire slots
  exist; deferred via `TODO(phase-10b)` (D4).
- **Did not regenerate any config codegen** — `hardware_config.yaml` / UDP
  protocol unchanged.

## Incidental findings

- **Pre-existing unused-function warning** (NOT introduced here): `fault_machine.cpp`
  `static bool actuators_intact_and_holding()` is defined but never called
  (`-Wall` warns). It was present on `07a895b` before this session (the predecessor
  ported it from `can_node._actuators_intact_and_holding` but the Teensy
  fault-machine never wired it in). Left as-is — out of scope; flagged so a future
  cleanup either wires it into the fault evaluation or removes it.
- **`PRIO_CAN_TX`** in `canbridge_config.h` appears unused (there is no separate
  CAN-TX task — TX happens inline / in the CAN-RX task). Pre-existing; left as-is.
- **`bus.py:108`** slave-list docstring (omits the cone) and the Jetson-side
  `bus.broadcast_time()` disable are Phase-5/10b production-side changes already
  tracked in the predecessor HANDOFF — not re-flagged here.

## How to verify (morning reviewer's first commands)

```
git log teensy-can-offload-firmware-three-bus-wip --oneline
source /home/jetson/Desktop/PDJ_venv/venv/bin/activate
cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run
git log teensy-can-offload-firmware-wip --oneline -1   # should still tip at 07a895b
git ls-remote origin teensy-can-offload-firmware-three-bus-wip   # should return nothing (no remote branch)
```

To discard everything (nothing was pushed):
`git checkout teensy-can-offload-firmware-wip && git branch -D teensy-can-offload-firmware-three-bus-wip`.
