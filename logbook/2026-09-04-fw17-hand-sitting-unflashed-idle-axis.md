---
title: "First FW 17 hand bring-up sitting — a build read as a flash, then a whole ladder run against a de-energised hand: two literally-correct artifacts, both trap-shaped"
type: investigation
date: 2026-09-04
status: resolved
phase: "unified-7dof-planner — Phase 3 (first hand bring-up sitting)"
related_plan: unified-7dof-planner.md
files_changed:
  - tests/hardware/hand_stream_bench.py
  - tests/hardware/session_unified7_hand_bringup.md
  - plans/active/unified-7dof-planner.md
  - logbook/2026-09-04-fw17-hand-sitting-unflashed-idle-axis.md
  - logbook/INDEX.md
subsystem:
  - can
  - ros
  - tools
tags:
  - safety
  - testing
  - docs
---

# First FW 17 hand bring-up sitting — the unflashed board and the idle axis

## Summary

The evening of **2026-09-03** carried the first operator sitting of
[`plans/active/unified-7dof-planner.md`](../plans/active/unified-7dof-planner.md)
Phase 3 — the lockstep FW 17 flash plus the T-H1..T-H4 hand ladder (runbook
`tests/hardware/session_unified7_hand_bringup.md`). It produced **two
independent failures, neither of them a defect in FW 17 or in the v6 wire**:

1. **The flash that wasn't.** Runbook row 4 ran `pio run -e teensy41`, which
   **builds only**. The hex md5 matched the committed reproducible FW 17 build,
   and that match was reasonably read as "flashed". No `-t upload` was issued
   that evening. The board stayed on FW ≤ 16 / `PROTOCOL_VERSION 5` while the
   host at HEAD `408303e` spoke `PROTOCOL_VERSION 6` — producing the **designed,
   correct, total link darkness**, which then cost a debugging session because
   its operator-visible symptom is `NO_HEARTBEAT`, which reads like a cable or
   power fault.

2. **The ladder ran against a de-energised hand.** After a correct flash the
   legacy `smooth_move` path worked perfectly, but the streamed stages aborted.
   The hand ODrive was in **IDLE** (`axis_state=1`) for the whole ladder. An
   idle ODrive ignores every `set_input_pos` the streamed lane puts on the wire
   and is backdrivable. **The `hold` stage PASSED** — a 30 s run, the driver's
   `--duration` default, **not** row 13's 600 s T-H1 soak, which never ran (the
   only hold CSV spans t = 0.000 → 29.975 s, 1200 rows). That pass is the
   structural hole: a hold stage of *any* length cannot distinguish "holding in
   CLOSED_LOOP" from "unpowered and undisturbed", and T-H1's pass criterion was
   literally *"ZERO hand motion (by eye + encoder flat in the CSV)"*, which a
   dead axis satisfies perfectly. **The first gate certified the condition that
   broke every later stage.**

The firmware is **exonerated in detail** (below) and needs **no change and no
reflash**. The defect is in the actuating driver and the runbook, and both are
fixed here: `tests/hardware/hand_stream_bench.py` gains the leg bench's
fail-closed precondition block and a post-bring-up verification;
`tests/hardware/session_unified7_hand_bringup.md` relabels rows 4/5 BUILD-ONLY
vs THIS IS THE FLASH and adds precondition row **11b**. The ladder resumes at
row 12 with the flashed board.

## Symptoms

### Failure 1 — total link darkness, identical before and after the "flash"

Three mcap bags from the evening, in `/home/jetson/Desktop/rosbags/`:
`2026-09-03_22-32-01` (pre), `2026-09-03_22-36-09` (post-"flash"),
`2026-09-03_22-44-10` (post-reboot of both the Jetson *and* the Teensy).

| Observable | 22:32:01 (pre) | 22:36:09 (post-"flash") | 22:44:10 (post-reboot) |
|---|---|---|---|
| `/link_status` `decode_errors == rx_frames` | every sample | every sample | every sample |
| samples with an exception to that | **0 / 779 across all three bags** | | |
| RX datagram rate | 374.04 Hz | 373.96 Hz | 375.17 Hz |
| `crc_errors` / `seq_gaps` | 0 / 0 | 0 / 0 | 0 / 0 |
| `bridge_fw_version` | `unknown (never seen)` | `unknown (never seen)` | `unknown (never seen)` |
| link / fault | `NO_HEARTBEAT` / `UNKNOWN` (level 2) | same | same |
| node-start → bridge-up latency | 5.03 s | 5.03 s | 5.05 s |

Two readings matter here.

**Datagrams were arriving the whole time.** The bridge transmitted fine at
~374 Hz; the *host* rejected 100 % of frames at the header version check
(`config/generated/udp_protocol.py:191`). That check precedes the CRC check by
four lines, so `crc_errors=0` and `seq_gaps=0` are **pinned at 0 by
construction** and carry no diagnostic information whatsoever — a fact worth
recording, because both look reassuring on the panel.

**The signature was identical before and after the "flash", down to the
node-start-to-bridge-up latency (5.03 / 5.03 / 5.05 s).** That is exactly what
"the firmware never changed" predicts, and exactly what a *bad* flash does not:
a partially-written or wrong image changes the boot behaviour, and this did not
change at all.

The host side was verified spotless in all three bags: the install tree
byte-identical to source (`INSTALL_SKEW: OK`, same sha256 in every bag),
`teensy_link` resolving to the live repo root, codegen fresh.

### Failure 2 — every streaming stage aborts, encoder dead flat

After a correct flash, `smooth_move` (the legacy hand path) worked perfectly.
The bench ladder did not:

| Stage | Result | Abort arithmetic |
|---|---|---|
| `hold` (30 s, the driver default — **not** row 13's 600 s T-H1 soak) | **PASSED** | — |
| T-H2a `triangle` | ABORTED | `\|0.498 − (−0.014 + 0.0·2 ms)\| = 0.512 rev > 0.5` |
| T-H2b `stroke` | ABORTED | `\|2.344 − (−0.076 + −0.0·1 ms)\| = 2.420 rev > 1.5` |

CSV evidence (`temp/logs/hand_stream_{triangle_20260903_232553,
stroke_20260903_232724,hold_20260903_232432}.csv`):

- `cmd_rev` ramps **exactly as designed** (0.5 rev/s on the triangle).
- `echo_rev` tracks `cmd_rev` to one knot of lag; the stroke's time-aligned
  reconstruction reads **0.52–0.97 mm against a 3.25 mm bar**. The commanded
  lane is, by this measure, working beautifully.
- `enc_rev` is **DEAD FLAT across a full second in both aborting runs** — and
  the two runs must not be conflated, because their baselines differ:
  - **triangle** (41 rows, t = 0.000 → 1.000 s): `enc_rev` spans
    **−0.01458..−0.01441** under a command ramping to **+0.486 rev** (the last
    knot written to CSV; the aborting knot is +0.498, one tick later — the belt
    breaks before that row is written).
  - **stroke** (43 rows, t = 0.000 → 1.050 s): `enc_rev` spans
    **−0.07637..−0.07610** under a command ramping to **+2.344 rev**.
- `vel_rps` is noise about zero. `fault=0`. `lead_mask=0`.

The `hold` stage's own numbers are the tell in hindsight: `cmd_rev` constant at
−0.01444, `enc_rev` ranging −0.01458..−0.01432 — **noise only**. Nothing in
that stage's pass criteria could tell a held axis from a dead one.

Independently, the printed baseline wandered between runs: **−0.108 / +0.209 /
−0.076 rev**. A powered ODrive in POSITION/PASSTHROUGH does not drift by a fifth
of a revolution between runs; a backdrivable idle one does, every time the
operator touches the slider.

## Diagnosis

### The flash that wasn't

`pio run -e teensy41` **compiles**. The flash is `pio run -e teensy41 -t
upload` — runbook row 5, and `platformio.ini:5` documents it. The operator ran
row 4's `rm -rf .pio/build && pio run -e teensy41`, verified the hex md5 against
the committed reproducible build (`2e90c43d5b6889e1059982d2e7aaf961`,
769,144 B), and read the green build plus the matching md5 as a completed
flash. No `-t upload` was issued that evening — confirmed from the operator's
shell history.

The board therefore stayed on FW ≤ 16 (`PROTOCOL_VERSION 5`) while the host at
HEAD `408303e` spoke `PROTOCOL_VERSION 6`. Total darkness in both directions is
the **designed** consequence of the v6 bump — Phase 2 chose it deliberately so a
half-rollback would be loud rather than silent — and the darkness itself is
therefore not a bug. What cost the session is how the darkness *presents*.

### Why the mismatch can never report itself — a structural note

`BridgeIdentity` is the **only** frame carrying `fw_version` / `protocol_version`,
and a version mismatch makes `decode_frame` reject **every** frame in both
directions — including that one. So the frame that appears to be *about*
version skew can never report the skew it is about.
`config/generated/udp_protocol.py:590` already documents this and names a prior
occurrence ("the 24608bb total-darkness failure").

The operator-visible symptom is therefore `NO_HEARTBEAT` with
`bridge_fw_version: unknown (never seen)` — which reads like a cable, a dongle
or a power fault, and sends you at the hardware. It sent us at the hardware:
the 22:44:10 bag is a reboot of *both* ends, which is what you do when you
believe the link is physically broken.

### The de-energised hand

The hand ODrive was in **IDLE** (`axis_state=1`; CLOSED_LOOP = 8, see
`teensy_link/activate.py:44`) for the whole ladder. An idle ODrive:

- **ignores `set_input_pos`** — so the streamed lane's commands landed on the
  wire and did nothing; and
- **is backdrivable** — which independently explains both the wandering
  baselines and the stroke stage's settle-band refusals.

Everything the abort arithmetic reported was true: `cmd` really was 0.498 rev
away from `enc`, because `enc` never moved. The deviation belt did its job
precisely; the axis it was measuring was not powered.

### The firmware is exonerated — in detail

This matters for the next sitting, so it is recorded fully rather than
asserted. A second defect hiding behind the idle axis — the streamed lane
failing to *transmit* — was explicitly hunted and refuted:

- **The lane really transmits.** `leg_interp.cpp:1019` calls
  `can_jugglebot_tx(encode_leg_setpoint(HAND_AXIS, ...), TxCls::LEGS)` → arb id
  **0x0CC** (node 6 << 5 | 0x0C `Set_Input_Pos`), payload `<fhh>`, scales
  100/100, **no sign flip** (axis 6 is outside `is_leg()`), clip [0, 10.8].
  That is the correct frame for the hand.
- **That send provably executed every tick.** `s_hand_sent` is incremented at
  `leg_interp.cpp:1020`, immediately after the send, and it is the
  `HAND_CMD_ECHO` freshness key (`telemetry.cpp:260-281`). A *tracking echo
  stream* therefore proves the dispatch ran — which the CSVs show it did, on
  every knot.
- **One nuance, stated precisely so it is not over-read.** The echo's
  *contents* are re-encoded from the bridge's own `axes[6].target_*` cache, not
  sniffed off the bus. The echo would track perfectly **with the hand ODrive
  unplugged**. It proves the lane computed and dispatched; only the encoder is
  downstream evidence. (That is exactly why the flat `enc_rev` against a
  beautiful `echo_rev` is the whole diagnosis in one plot.)
- **Observe-first gates only the E-STOP latch, never the TX.**
  `interp_hand_dev_guard_armed()` appears only in `fault_machine.cpp:418` (its
  sole call site; `:412` is the comment above it, `:419` the continuation line)
  and in the accessors (`leg_interp.h:163` decl, `leg_interp.cpp:1185` def) —
  nowhere in the output path. Observe-first silently
  suppressing the hand lane was the named worst-case risk of the Phase 3
  design, and **it is closed**.

**Verdict: with axis 6 in CLOSED_LOOP + POSITION/PASSTHROUGH, the lane moves
the hand correctly. No firmware change. NO REFLASH.**

### Three driver defects, one root cause

`tests/hardware/hand_stream_bench.py` states in its own docstring that it is
*"Modeled on `teensy_setpoint_bench.py` (the leg equivalent)"*. The leg bench
has refused to arm an axis that does not read CLOSED_LOOP since it was written
(`tests/hardware/teensy_setpoint_bench.py:337-356`), fails **closed** when no
diagnostic arrived, names `--close-loop` as the remedy, and verifies that the
bring-up took (`:326-335`). The hand bench dropped all of that — on the **one
axis that must be energised**:

1. **No `axis_state` gate.** The driver *read* `axis_state` and *printed* it at
   the top of every failing run, and never compared it to anything. The value
   `1` was on screen the entire evening.
2. **The health check failed OPEN.** `if d is not None and int(d.active_errors)
   != 0` — when no `DIAGNOSTIC` had arrived, the only hand-ODrive health check
   was **silently skipped**. The run that printed `axis_state=?` armed knowing
   nothing at all about the axis.
3. **`--close-loop` was unverified.** It ended in a bare `time.sleep(0.5)` with
   no check that the axis reached CLOSED_LOOP, and the baseline print is a
   **pre**-bring-up snapshot that is never refreshed.

And the reason defect 2 is not a rare corner: **the diagnostic's absence is a
coin flip by construction.** The startup wait loop waits only for `TELEMETRY` +
`BRIDGE_IDENTITY`, while the axis-6 `DIAGNOSTIC` is emitted on-change or on a
staggered ~1 Hz forced refresh (`telemetry.cpp:671-700`, send-slot 84) — and a
quiescent IDLE hand *changes nothing*. That is precisely why one run printed
`axis_state=?` and the next printed `axis_state=1`.

## Discussion

### 1. Assert vs check — the reframe

The first fix shape was the obvious one: **gate on `axis_state` before
arming**. That was reframed part-way through, on discovering that the *mode* is
**unverifiable from telemetry**. `AxisState::controller_mode` and
`AxisState::input_mode` (`axis_state.h:42-43`) are **never written by any
firmware code** — only read — so the `DIAGNOSTIC` reports `0` for both,
forever. A gate can confirm CLOSED_LOOP; it can never confirm
POSITION/PASSTHROUGH. **Only sending the frame asserts the mode.**

This is not a hypothetical distinction: the production arm path already asserts
unconditionally rather than checking. `teensy_bridge_node.py:4218` sends
`Set_Axis_State(CLOSED_LOOP)` and `Set_Controller_Mode(POSITION, PASSTHROUGH)`
every time it arms the streamed hand, and the comment there names this exact
failure — *"a hand left IDLE would silently swallow the streamed lane"*. The
production path was right about this before the sitting proved it; the bench
driver was the outlier.

The landed fix therefore does **both**, and knows why: it *checks*
`axis_state` (a cheap, honest, telemetry-backed refusal that catches the
common case with a clear message) and it *asserts* the full pair via
`--close-loop` (the only thing that can establish the mode at all).

### 2. Why `--close-loop` was left opt-in rather than made the default

Making `--close-loop` the default would close the **residual hole**: a hand
that reads CLOSED_LOOP but sits in the *wrong input mode* still swallows the
stream, silently, and the state-only gate passes it. Defaulting the assert
would fix that outright.

It was not done, because it changes **what a bench tool energises by default on
a 48 V motor**. Every streaming invocation would then, as a side effect of
being run, power the hand — including a run the operator started to *look* at
something. That is an owner decision about actuating defaults, not a bug fix,
and it is deliberately surfaced rather than taken (carried below). The residual
hole is stated plainly so a future session knows it is open: **a hand in
CLOSED_LOOP with the wrong input mode is not caught by the landed gate.**

### 3. Why fail-closed on a missing diagnostic

Not by precedent ("the leg bench does it"), but by root cause. This driver
commands a **3 m/s stroke** with the ROS launch down and the deviation guard
**observe-first** — i.e. with the firmware's E-STOP latch deliberately
disarmed for the first sitting. An *unknown* axis state is exactly the state
that produced this sitting's aborts. Proceeding on the **absence** of evidence
leaves the gate blind in precisely the situation it exists to see, and the
absence is common (see the coin-flip mechanism above) rather than exotic. So a
missing `DIAGNOSTIC` is now an abort, after a bounded **2.5 s** wait. The wait
is 2.5 s and not one refresh period because the forced refresh is an **AND of
two independent clocks** (`telemetry.cpp:676-678`): `(now − last_sent_us >=
DIAG_FORCE_PERIOD_US) && (slot == i·slots_per_axis)`. When the elapsed
measurement lands just under 1 000 000 µs at axis 6's slot 84 the send is
**vetoed** and the next opportunity is a whole second later — so the worst-case
quiescent interval is ~2 s, and a 1.5 s wait would have failed closed on a
*healthy* link. 2.5 s covers two opportunities plus margin.

The abort is also scoped: it stands down for a `--close-loop` run, because there
the **post-bring-up verification is the authority**. The IDLE→CLOSED_LOOP (1→8)
transition is itself a `diag_changed()` trigger (`telemetry.cpp:61`), so a
`--close-loop` bring-up *produces* a diagnostic, and that gate's own
`dcl is None` arm already fails closed for the same shape. Keeping the entry
abort for `--close-loop` would have made the runbook's **only** invocation shape
the likeliest spurious abort of the sitting.

### 4. The homing trap — a real residual, invisible to any telemetry gate

There is a second way to reach a CLOSED_LOOP hand that still swallows the
stream, and it is on a path the sitting actually walks.
`leg_homing.cpp:195-197` puts axis 6 into **CLOSED_LOOP + VELOCITY/VEL_RAMP**,
and `:225` returns it to IDLE at the end. In `VEL_RAMP` the ODrive controller
consumes `input_vel`, **not** `input_pos` — so a hand that reads CLOSED_LOOP on
the wire will ignore the streamed position lane with a symptom **identical to
this sitting's**: perfect echo, flat encoder, deviation abort.

This is why both frames are required — `Set_Axis_State` **and**
`Set_Controller_Mode` — and why a state-only gate is **necessary but not
sufficient**. It is also the concrete cost of leaving `--close-loop` opt-in
(item 2): the gate cannot see this state, so only the operator remembering the
flag closes it.

### 5. Why the new runbook row is `11b` and not a renumber

Rows **1** and **18–19** of `session_unified7_hand_bringup.md` are cited **by
number** from `logbook/2026-09-02-unified-7dof-planner-phase3-fw17-hand-lane.md`
— row 1 at `:223`, rows 18–19 at `:156`. (The inbound set is exactly that: the
plan cited **zero** hand-runbook rows before this change, and the only other
"row 16" in the tree belongs to a *different* runbook,
`session_unified7_bus_headroom.md`, via `plans/active/unified-7dof-planner.md:369`.)
Rows 18–19 fall **inside the renumber range** (12–21, everything after the
insertion point), so renumbering would have shifted them and silently broken
those inbound references: the same failure class as the plan-filename convention in CLAUDE.md,
where twelve entries pointed at nothing within a day of one archival. `11b`
keeps every existing number stable at the cost of one ugly label, which is the
right trade.

### 6. What was ruled out

The possibility of a **second defect hiding behind the idle axis** — the
streamed lane failing to transmit at all — was explicitly hunted and refuted
with the `s_hand_sent` / echo-freshness argument in the Diagnosis. This
mattered operationally, not academically: fixing only the arming and returning
to the bench with a lane that also did not transmit would have burned the next
sitting too. The refutation is recorded in enough detail (arb id, payload,
scales, the freshness key, the observe-first gate's actual call sites) that the
next session can re-check it in minutes rather than re-derive it.

### 7. The class both failures share

Neither artifact was *wrong*. Runbook row 4 said `pio run -e teensy41`, which
is exactly the command it should have said, and row 5 immediately below said
`-t upload`. T-H1's pass criteria described exactly the evidence a healthy hold
produces. **Both were literally correct and operationally trap-shaped:**

- Row 4 ran a command that **looks like a flash** — it churns, it prints a
  green `SUCCESS`, it produces an artifact you then verify by md5 — and it sat
  directly above the real flash row, so satisfying it *felt* terminal.
- The `hold` stage certified a dead axis because T-H1's criteria — the ones it
  was read against — are all **negative evidence** (no motion, flat encoder,
  zero counters), and negative evidence is produced equally well by "nothing is
  wrong" and "nothing is connected". Duration is not the lever: the 600 s soak
  would have certified it just as confidently as the 30 s run that actually
  happened.

**Correctness of instructions is not the same as resistance to being
followed.** The fixes in both files are aimed at that, not at correctness: row
4 now says BUILD-ONLY and names this failure, row 5 says THIS IS THE FLASH, and
T-H1's criteria now **lead with positive energisation evidence** and state the
blind spot in the row itself.

## Fix

**`tests/hardware/hand_stream_bench.py`**

- `_hand_diag()` — the cached axis-6 `DIAGNOSTIC` read **under `_lock`**,
  matching the leg bench's `axis_diag()`. The caches are written from the
  client's RX thread, and the old lock-free `_cache["diag"].get(...)` was the
  odd one out in the file.
- `_wait_hand_diag()` — a bounded **2.5 s** poll (**two** forced-refresh
  opportunities plus margin — the refresh is an AND of two independent clocks,
  `telemetry.cpp:676-678`, so a vetoed slot pushes the next send a whole second
  out and the worst-case quiescent interval is ~2 s), documenting the coin-flip
  mechanism at the point of use.
- A **leg-bench-shaped precondition block** that accumulates *all* unmet
  preconditions so one run shows the operator every one of them, and which:
  **fails CLOSED** on a missing diagnostic *for a run without `--close-loop`*
  (with `--close-loop` the post-bring-up gate is the authority — Discussion 3);
  gates `axis_state == CLOSED_LOOP` when `--close-loop` was not passed; fixes
  the fail-open `active_errors` check; and keeps the existing `fault_state`
  check. Its missing-diagnostic message names the likely cause — a stale or
  silent axis-6 DIAGNOSTIC slot — and says re-run, rather than prescribing a
  bridge power-cycle.
- A **post-bring-up verification** after `--close-loop`: a 3 s poll for
  CLOSED_LOOP, then a check of **both** `axis_state` **and** `active_errors`
  before the line `post-bring-up: … the hand is ENERGISED and holding` may
  print — that line is what runbook row 13 tells the operator to read as the
  sitting's positive evidence, so it must never appear with a live error
  latched; the abort names whichever half failed. An RPC ack is the *bridge*
  acking the CAN send, not the ODrive entering closed loop — an axis with an
  active error, a missing encoder index or a live undervoltage acks and stays
  IDLE.
- A **mid-stage energisation check** in the streaming loop, beside the fault
  belt: both gates above are **entry-only**, and nothing re-read `axis_state`
  afterwards — not across the unbounded interactive ARM prompt, not during the
  run. A hand that drops to IDLE mid-stage makes a `hold` produce zero
  deviation, so the belt never fires and the stage passes on a dead axis again,
  one step later. The loop now aborts (`hand left CLOSED_LOOP mid-stage`), and
  the stage CSV carries an `axis_state` column (empty until a diagnostic has
  arrived, the same convention as `echo_rev` / `recon_mm`) so the energisation
  record is auditable after the fact.
- **The `HAND_SOURCE_SET` refusal is now actionable.** That switch runs *before*
  the `--close-loop` bring-up and returns hard, so a de-energised hand that has
  drifted outside the firmware settle band could never reach the bring-up row
  11b calls "the only route" — and the operator got no on-screen way out. (Not
  hypothetical: the sitting's baselines were −0.108 / +0.209 / −0.076 rev, and
  +0.209 is outside the retract band.) The failure branch now says the hand is
  backdrivable with the launch down, to park it inside the retract band by hand
  and re-run, and that `--close-loop` cannot run until the switch succeeds. The
  bring-up is deliberately **not** reordered ahead of the switch: that changes
  when a 48 V motor energises relative to the latch switch, which is an owner
  decision rather than a bug fix.
- **`--source-only` is exempted from the energisation half** — it is a latch
  verb that commands no motion and never arms; its error/fault gates are
  unchanged.
- Docstring PRECONDITIONS and Usage updated (`--close-loop` on every streaming
  example).

**`tests/hardware/session_unified7_hand_bringup.md`**

- Rows **4 / 5** relabelled **BUILD ONLY — this row does NOT touch the board**
  vs **THIS IS THE FLASH — the only row that changes the firmware aboard**,
  each naming what the other is not, and row 4 naming this sitting's cost.
- New precondition row **11b**: the hand must be energised for every streaming
  row; `--close-loop` is the only clean route with the launch down; and the two
  **non-routes** named so nobody re-derives them at the bench — (a) arm via the
  launch then shut it down does *not* work, because shutdown's stow
  `DEACTIVATE` idles the hand explicitly (`teensy_bridge_node.py:6327` and
  `:6365`), and (b) the legacy `hand_ops` bring-up preamble is structurally
  unavailable once the latch is STREAMED (`hand_ops.cpp:84-88` returns
  `ERR_HAND_SOURCE` before any CAN side-effect, by design).
- `--close-loop` added to rows **14–19**; row 13 already carried it at HEAD —
  every streaming command now has it, including the post-E-STOP re-arm in row 18
  (a guard trip leaves the hand de-energised).
- **T-H1's pass criteria now lead with the positive energisation evidence** and
  state the blind spot; row 19's (b) half notes that "hand does NOT move" only
  means something on an energised hand.

No firmware change. No reflash. No host-node change.

## Verification

Re-run 2026-09-04 after the audit-fix wave (the 2026-09-03 triples were taken
against the pre-audit file and are superseded):

- (2026-09-04, `python -m py_compile tests/hardware/hand_stream_bench.py`,
  **exit 0**).
- (2026-09-04, `python -m pytest tests/sim/test_plans_index.py
  tests/sim/test_logbook_search.py tests/sim/test_logbook_front_matter.py -q`,
  **108 passed in 0.68 s**).
- (2026-09-04, `python -m pytest tests/teensy_link -q`, **297 passed in
  7.33 s**).
- Pre-commit gate, after the audit fixes (6 WARNING + 8 NOTE, all adjudicated
  and applied bar one deliberate part-approval — the `--close-loop`/latch
  reorder, left as an owner decision): (2026-09-04, `./run_tests.sh`,
  **6392 passed / 4 skipped in 256.35 s parallel + empty serial phase,
  total 269 s — PASS**). `--full` deliberately not run: the owner directive
  scopes it to `controller/`/`sim/` diffs and this change touches neither
  (`tests/hardware/` + narrative only). Runbook row 1's `./run_tests.sh --full`
  before the resumed sitting stands separately and is the operator's.

**Test-coverage honesty.** **No pytest test reads `hand_stream_bench.py` or the
runbook** — `grep -rln "hand_stream\|close_loop" tests/ --include="test_*.py"`
returns nothing. The triples above establish that the file still imports and
that nothing adjacent regressed; they do **not** exercise the new gate. **The
real gate for these two files is the bench sitting itself**, which is why the
fix is written to fail loudly and legibly rather than cleverly.

## Outcome

Both failures are understood, neither implicates FW 17 or the v6 wire, and the
two artifacts that produced them are fixed. **The ladder resumes at row 12 with
the flashed board**; rows 1–11 are discharged (the board is now genuinely on FW
17, `smooth_move` regression clean). Row **11b** is neither discharged nor
discharge*able*: it is a **standing requirement for every streaming row**, not a
one-time step, and it is carried by the `--close-loop` now on rows 13–19 plus
the driver's own entry, post-bring-up and mid-stage gates.

Carried to the operator / owner:

1. **Should `--close-loop` become the default?** It would close the wrong-input-
   mode residual (Discussion 2 and 4), at the cost of a bench tool that
   energises a 48 V motor by default. Owner call.
2. **`--close-loop` does not send `SET_VEL_CURR_LIMITS`**, though the leg bench
   does (`teensy_setpoint_bench.py:315-316`). The hand therefore arms with
   whatever limits the ODrive happens to hold, rather than the configured
   `ODRIVE_HAND_*` values.
3. **The driver has no IDLE-on-exit**, unlike the leg bench (`:578-583`). A
   hand armed by `--close-loop` **stays energised after the driver exits**.
4. **The homing trap** (Discussion 4): `leg_homing.cpp` leaves axis 6 in
   CLOSED_LOOP + VELOCITY/VEL_RAMP mid-sequence, a state no telemetry gate can
   distinguish from a stream-ready hand.

## Withdrawn claims

- [2026-09-03 22:36] Claimed the board was running FW 17 and that the link
  darkness was therefore a transport, cable or power fault — the reading that
  motivated the 22:44:10 reboot of both ends.
  **WITHDRAWN:** `pio run` builds only; no `-t upload` was issued (operator
  shell history). The failure signature was byte-for-byte identical before and
  after the supposed flash, including node-start-to-bridge-up latency
  5.03/5.03/5.05 s and `decode_errors == rx_frames` on 779/779 samples — which
  is what "the firmware never changed" predicts and what a bad flash does not.
  **Superseded by:** Diagnosis § "The flash that wasn't".

- [2026-09-04] Claimed that gating on `axis_state == CLOSED_LOOP` before arming
  is sufficient to guarantee the streamed lane will move the hand.
  **WITHDRAWN:** the *mode* is unverifiable from telemetry —
  `AxisState::controller_mode` / `input_mode` (`axis_state.h:42-43`) are never
  written by any firmware code, so the `DIAGNOSTIC` reports 0 forever. A hand in
  CLOSED_LOOP + VELOCITY/VEL_RAMP (the homing exit state) passes the gate and
  still swallows the stream.
  **Superseded by:** Discussion 1 (assert vs check) and Discussion 4 (the
  homing trap); the landed fix checks the state *and* asserts the pair via
  `--close-loop`.

## Open Questions

1. **The wrong-input-mode residual is OPEN.** A hand reading CLOSED_LOOP in the
   wrong input mode still swallows the stream silently, and no telemetry gate
   can see it. Closing it means making the assert unconditional — the owner
   decision in Outcome 1.
2. **Hand arming limits** — Outcome 2: should `--close-loop` push
   `ODRIVE_HAND_*` vel/curr limits the way the leg bench pushes the leg ones?
3. **Energised-on-exit** — Outcome 3: should the driver idle the hand on exit,
   or is a hand left holding the safer end state for a multi-stage ladder? (The
   leg bench idles; the ladder runs many stages back to back.)
4. Whether any *other* runbook row is trap-shaped in the same way as row 4 —
   i.e. a row whose command succeeds loudly while doing less than the row's
   prose implies. Rows 4/5 were found by being burned; nothing has swept for
   siblings.
5. **CLOSED — recorded because the shape recurs:** the first cut of this fix
   gated energisation at **entry only** (the precondition block plus the
   post-`--close-loop` verification), with nothing re-reading `axis_state`
   across the unbounded ARM prompt or during the run. That is the 2026-09-03
   failure displaced by one step: a hand dropping to IDLE mid-stage makes a
   `hold` produce zero deviation, the belt never fires, and the stage passes on
   a dead axis. Closed by the mid-stage check in the streaming loop (Fix), whose
   verdict is now also written per-tick to the stage CSV's `axis_state` column.
   The residual is the cache's own latency: the check reads the last DIAGNOSTIC,
   so a drop-out is caught within a refresh, not on the tick it happens.
