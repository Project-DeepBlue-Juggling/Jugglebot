---
title: "ERR_TIMEOUT hand-path: sequences are random (state bug excluded), write() rejection is deferral not drop, and the attribution counters land"
type: investigation
date: 2026-08-02
status: in-progress
phase: "refactor-2026-07 Phase 7 / PROMPT-err-timeout-hand-path Steps 1-2"
related_plan: refactor-2026-07.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - plans/active/INDEX.md
  - plans/active/refactor-2026-07.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/extra_script.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/__init__.py
  - teensy_link/protocol.py
  - teensy_link/rpc_args.py
  - tests/firmware/native/test_hand_ops.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_firmware_build_pins.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/hardware/toss_trace_recorder.py
  - tests/hardware/session_err_timeout_bench.md
  - tests/ros/test_teensy_bridge_node_bridge_diag.py
  - tests/ros/test_teensy_bridge_node_hand_acks.py
  - tools/probes/hand_dispatch_ladder.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - logbook/2026-08-01-err-timeout-recount.md
  - logbook/2026-08-02-err-timeout-attribution-instrumentation.md
  - logbook/INDEX.md
subsystem:
  - canbridge
  - ros
tags:
  - err-timeout
  - can3
  - observability
  - forensics
---

# ERR_TIMEOUT hand-path: Step 1 sequence verdict + mechanism reframe + attribution instrumentation

Continues `2026-08-01-err-timeout-recount` (read it first). Three results this
session, in evidential order:

## 1. Step 1 — the 399 recorded outcomes are RANDOM, not structured

Ordered per-dispatch outcome sequences were reconstructed for all 16 sessions
with hand dispatches (2026-07-23 → 07-31) from `~/.ros/log/*/launch.log`,
validated cell-for-cell against the recount table (pooled prime 133/63 fail,
arm 266/139 fail — 202/399 = 50.6 %). Arm-path ordering uses a
single-outstanding-channel reconstruction justified from
`catch_coordinator_node.py:1171-1172` (a same-ball re-arm can only follow the
previous arm's failure callback) and validated empirically: in the codes-visible
era, 97/97 arm dispatches have visible responses with 0 interleavings, and 167
measured response→next-dispatch margins are all positive (min +3.5 ms).

Verdict (independently re-derived, adversarially attacked, pairing-invariant):

- **No alternation** (arm z = +0.54, prime z = +0.19), **no parity effect**,
  **no lag-1 state signature** (P(F|F) vs P(F|O): OR 0.82 arm / 0.76 prime,
  both ≈ p 0.45; retry-vs-first OR ≈ 0.9) — the stale-pending-slot / state-bug
  class is EXCLUDED at the effect sizes that matter (caveat: n=84 short-lag
  attempts cannot exclude modest effects, OR ≤ 1.7).
- **No genuine time-clustering** once within-retry-chain pairs are fully
  removed (0 excess FF pairs at W=1 s), and **session rates are homogeneous**
  (X² 11.03/15 df, dispersion 0.74) across fresh-to-30 h bridge uptimes.
- Prime (47.4 %) and arm (52.3 %) rates are statistically compatible
  (p = 0.40); under the code-verified 3-send structure (`hand_ops.cpp:55/59/73`)
  independence implies ~21 %/frame. (The exponent check k=3 is circular from
  rates alone — k=3 is verified in source, not fitted.)
- Random ⇒ **congestion-consistent**: exactly what per-dispatch-independent
  TX-mailbox pressure would produce, and what a state machine would not.

Per-session ordered sequences (dispatch order, O=ok F=fail) — durable copy so
re-analysis never needs the extraction again:

| session | prime seq | arm seq |
|---|---|---|
| 07-23 09:13 | OOOFFOFFFOOOFF | FFOFOOOOFO |
| 07-23 11:37 | OFOFFOOOFFOOOFOOFOOF | FFFOOOFOOFFFO |
| 07-23 17:51 | FOOOFFOOOFOFOOOOFFFO | FOOFFOOFFOFOOOOFFFOOFFO |
| 07-24 09:07 | FOOFOFOOFOOOOFOFFOFFFFFFOFOFOFOFF | FOFOFOFOFOFFOOFFOFOFFFFOFOFOOFFOFOOOFOOFFFOFFFFFFFO |
| 07-25 15:04 | - | FF |
| 07-25 15:17 | OFO | FOFOOFOOOFO |
| 07-25 15:22 | - | OO |
| 07-27 15:23 | - | O |
| 07-27 15:37 | - | FO |
| 07-27 15:39 | FFOOOOOFOOFFOOFFOOFOFOOFF | FFOFOOOOFFOOFFFOOFOOFFFFOFOFOFOFFFOOOOFFOOFFOFFOFFOFF |
| 07-27 15:51 | - | FOOOOFFFOFFFFOFFOOOOFOOOOOFF |
| 07-27 16:00 | - | OOFFOFFOOFFFOFFFO |
| 07-27 16:07 | - | OOOFF |
| 07-29 11:10 | OOOFFFF | FOOFFFFFFOFOOFFOFOOOFOFFOOFOOFOFFFFFFO |
| 07-31 13:56 | FFO | OO |
| 07-31 14:51 (post-swap) | FOFOOFFO | FOFFOFOO |

Two secondary signals, recorded honestly:
- **A real, unexplained within-session drift on the PRIME path only**
  (+0.62–0.77 log-odds/100 s, p 0.006–0.015, wall-clock-shaped not
  call-count-shaped, robust to retry removal and leave-one-session-out) while
  the arm path is flat and session MEANS stay homogeneous. Unresolved — the
  bench A/A' arms below are the cheap test.
- A boundary-significant same-reload-cycle prime↔arm association (MH OR 2.79,
  Fisher p 0.05–0.09 depending on cycle definition) — consistent with shared
  time-local pressure, under-determined at this n.

## 2. Mechanism reframe — `bus.write() <= 0` is DEFERRAL, not drop [VERIFIED IN SOURCE]

The recount narrowed the mechanism to `bus.write() <= 0` and inferred
"write rejected ⇒ frame never enqueued ⇒ hand NOT armed", placing it in open
tension with the catch latch's lying-ack premise. Reading the vendored
FlexCAN_T4 (`~/.platformio/.../FlexCAN_T4.tpp`, orchestrator-verified line by
line):

- `write(msg)` returns ONLY 1 (mailbox written) or **-1 = pushed into the
  64-slot software txBuffer** (`struct2queueTx`, :1024-1048). Never a drop at
  the call site. `send_on()`'s `> 0` maps -1 to false.
- The TX-complete ISR drains the ring correctly, one frame per freed mailbox,
  FIFO order (:1243-1256) — TX mailbox interrupts are enabled (8 TX mailboxes,
  MB8-15, `setMaxMB(16)` + FIFO). A deferred frame goes out ~0.1–1 ms later.
- Two genuine loss paths exist and are narrower: ring overflow silently
  overwrites the oldest entry (`circular_buffer.h:340-349`), and the `events()`
  TX drain has a real defect (peeks one frame, writes it to EVERY free mailbox,
  pops one entry per write → duplicates the head, discards the rest;
  :1084-1102) — it only runs when the 1 kHz tick beats the ISR to a non-empty
  ring.

**Consequence: the recount's "hand NOT armed" inference is withdrawn** (addendum
added there). A failed ack with the frame transmitted ~1 ms later is exactly
the lying-ack field observation — the two premises are COMPATIBLE. This is
source-reading, not measurement: **the latch fence stays up** until the
counters confirm on hardware. It also shifts the epidemic's likely operational
cost from "lost stroke" to "the FSM's reaction to a false-failure ack"
(aborted reloads, ladder churn) — a Step 4 question, explicitly not acted on.

Open gap the counters must close: naive phase-uniform burst arithmetic with 8
TX mailboxes predicts well under 50 % per-call failure — either arrivals are
phase-correlated with the leg tick, effective TX capacity is lower than
modeled, or something else adds pressure. Do not paper over this.

## 3. What landed

- Host-side (commit 1 of this entry's arc) — `hand_traj_acks` row (calls/ok/fail_teensy/fail_host)
  at the `_call_rpc` choke point; splits Teensy-returned from host-synthesised
  ERR_TIMEOUT (the pooling the recount unpicked by hand); closes the 20d01e9
  forensics blind spot against the CURRENT bridge. `/link_status` 37→38 keys.
- Build guard (commit 2) — pio staleness guard: sha256 of `udp_protocol.h` vs a BUILD_DIR
  marker; wipes project objects + link products on mismatch (measured: full
  BUILD_DIR rmtree breaks the run — library Dir nodes already materialised).
  Closes the 24608bb class for ADDITIVE changes, where it is silent.
- Wire + firmware (commit 3, lands with this entry) — BRIDGE_TX_DIAG (0x8D) + BRIDGE_IDENTITY (0x8E), FW_VERSION
  8→9, no PROTOCOL_VERSION bump: per-bus `tx_deferred` + `tx_q_hwm` (sampled at
  send instants inside the PRIMASK section), per-stage hand counters
  (calls/rej_homing/bus_down/pre1/pre2/traj), wire-visible bridge firmware
  identity with warn-never-refuse BRIDGE_FW_CHECK. `/link_status` 38→40.
  Independently audited pre-commit: COMMIT-READY, zero blocking findings;
  the six polish notes were applied before the commit was written.
- Step 3 prep (commit 4) — the operator bench runbook
  `tests/hardware/session_err_timeout_bench.md` + the
  `tools/probes/hand_dispatch_ladder.py` driver: dispatches ride the REAL
  stack (smooth_move_hand to the hand's current position — mechanically null,
  full 3-send firmware path); A-B-A-prime arms with the refutation cell
  pre-registered and tx_q_hwm's boot-monotonicity pinned in the method.

## Discussion

**Why the counter is `tx_deferred`, not the planned `tx_write_fail`.** The name
in the recount/plan encoded the "rejected ⇒ lost" model this session refuted.
A wire-format field name outlives its investigation; naming it for the verified
mechanism (deferral into the software ring) prevents the next reader from
re-deriving the wrong operational story. The two REAL loss paths (ring
overflow; the events() drain defect) are named in the field's comment instead.

**Why per-stage cumulative counters, not new RpcStatus codes.** Distinct codes
(ERR_HAND_PRE1/...) would give per-call attribution in the launch log — but
they mutate the semantics of an existing wire field consumed by live logic
(catch ladders key on failure acks; old-host `== ERR_TIMEOUT` comparisons stop
matching under skew; the recount's own regexes break). The additive contract
exists precisely to keep cross-version semantics frozen. Rates per stage are
what Steps 3-4 decide on; per-call joins are deferred (0x8F remains free).

**Why two MsgTypes, not one.** Identity and diagnostics have different
lifecycles: the diag payload will plausibly evolve (each evolution = a new
MsgType under the never-append rule), while identity is a stable contract with
its own consumer (the xref-pinned expected-version check). Bundling would
couple the stable contract to the evolving one's id churn.

**Why the discriminator drives the REAL stack** (Step 3 runbook): a synthetic
UDP client would need hand-crafted ArgHandTraj payloads (deadline semantics =
stroke risk), fights the single-owner UDP constraint, and validates a path
production never takes. `smooth_move_hand` to the current position is a
mechanically-null dispatch through the identical 3-send firmware path, with
the counters read off `/link_status`.

## Open questions

1. The prime-path within-session drift (above) — bench A/A' arms test it.
   **NOT REPRODUCED on the bench** (2026-08-09): both idle arms are 0/40, so
   there is no within-session drift to fit and armA vs armA2 cannot separate.
   The signal remains open for CATCHING sessions, where the prime path is
   exercised under load; the bench simply has no purchase on it.
2. The ~50 % vs naive-burst-math gap — tx_q_hwm + Step 3 arms discriminate.
   **RESOLVED 2026-08-09** — see the addendum § A3: phase-locked dispatch
   quantisation (dominant), a congested window of 1–2 stretched slots
   (second-order), bus load alone refuted by ~50×. One INFERENCE link remains
   flagged there, with its falsifiable test.
3. tx_q_hwm is boot-monotone with no reset RPC — bench arms must run
   low-load-first and read per-arm increments, or reboot between arms; a
   pinned 64 from an earlier phase is NOT evidence about the current one.
   (Honoured on 2026-08-09: A → B → A2 on one boot, per-arm increments read;
   the watermark reached 2, nowhere near the 64 overflow proxy.)
4. txBuffer loss paths are un-instrumentable without touching the vendored
   library — tx_q_hwm==64 is the overflow proxy; the events() drain defect is
   a candidate Step 4 item ONLY if attribution shows real frame loss.
   **TRACKED RESIDUAL** (addendum § A6): the `mb == -1` refill loop's missing
   `break` (`FlexCAN_T4.tpp:1084-1102`) makes "deferral is not a drop" only
   CONDITIONALLY true and can duplicate a deferred 0x6D0 on the wire today.
   Exposure is rare (the TX-complete ISR normally wins the race) and it
   becomes unreachable at the observed occupancy once the TX mailbox count is
   raised (peak pending 8 against 16 mailboxes; the ring is only re-entered if
   a future TX producer doubles the burst), so it is not a fix in its own
   right — but it must not be forgotten when the Step 4 shape is
   chosen. Probe: count duplicate 0x6D0 arrivals Platform-Teensy-side.
5. One ordinary post-swap reload sitting (n up from 4/8) — unchanged from the
   recount; now doubles as the counters' first field data. Still open after
   the 2026-08-09 bench (a null-move ladder is not a reload).

## Verification

- Step 1: extraction validated 16/16 sessions cell-for-cell vs the recount
  table; independent re-derivation 0/16 mismatch; raw-log spot-check on
  07-23 17:51 + 07-31 14:51 exact incl. µs timestamps. Scripts (one-off, probe
  policy): session scratchpad `seq_extract.py`, `seq_analysis_*.py`,
  `seq_verify.py`; durable numbers are in this entry.
- Commit 1: ./run_tests.sh, run 2026-08-02: 3988 passed, 195.40 s parallel,
  serial phase empty, RESULT PASS. New 5-test file drives the real RpcClient
  against the FakeTeensy; mutation-checked.
- Commit 2: pio run -e teensy41 and -e teensy41_bench_sysid each ×2, run
  2026-08-02: guard fires once then stays quiet, both SUCCESS; ./run_tests.sh,
  run 2026-08-02: 3989 passed, 189.34 s parallel, RESULT PASS.
- Commit 3: `pio run -e teensy41` and `pio run -e teensy41_bench_sysid`, run
  2026-08-02: both **SUCCESS** (7.31 s / 7.40 s); the pio staleness guard fired
  once per env on each post-regen build (`udp_protocol.h CHANGED
  2198631f6813 -> ... -> 81db5186babd`, wiping `$BUILD_DIR/src` + link
  products) and stayed quiet on rebuilds with an unchanged header — commit 2
  exercised on its first real trigger, twice, since a late wording fix to a
  payload summary regenerated the header again.
  `pytest tests/firmware -q`, re-run 2026-08-02 post-close-out audit: **399
  passed in 19.36 s** on a warm native-build cache (the original post-change
  run rebuilt the native harness and took ~3 min; its wall-clock was cited
  inconsistently across two documents — 192.91 vs 192.42 s — so the fresh
  re-run replaces both) (389
  before this arc). `./run_tests.sh`, run 2026-08-02: **4009 passed, parallel
  phase; serial phase empty (4444 deselected); RESULT: PASS**.
  Independent pre-commit audit: COMMIT-READY, zero blocking
  findings; six polish items applied before the commit: (1) `getTXQueueCount`
  is a single member load, not two; (2) the HWM sample means different things on
  the deferral vs mailbox paths; (3) the torn-read note on a transient `ok=-1`
  (left unclamped — a PERSISTENT negative is real signal); (4) the RX-thread
  logging deviation recorded explicitly; (5) `proto` given a read site in the
  row; (6) the announce-on-change test re-synchronised on `rx_count_by_type`
  (its old wait predicate was already satisfied by frame 1, so the repeats were
  never witnessed as arrived and the assertion proved nothing).
  A SECOND, narrative-only audit of this entry then caught three more, all
  mine: a suite count of 4020 that had been written from expectation rather
  than measurement (the real figure is 4009 — the run had not happened yet when
  the sentence was drafted), a wrong pre-arc firmware-tier baseline, and an
  INDEX row with six columns against a five-column header, which silently drops
  the entry link under GFM. Recorded because the first two are the failure mode
  the (date, command, result) rule exists to prevent, and it still got past a
  first pass.
- Session closure: `./run_tests.sh --full`, run 2026-08-02: **parallel 4432
  passed, 3 xfailed in 448.40 s; serial 9 passed in 40.22 s; RESULT: PASS** —
  every tier, `nightly` included. (One earlier default-gate run this session
  flaked on the KNOWN flaky
  `tests/motion/test_motor_guard.py::test_decay_boundary_continuity` — passes
  3/3 in isolation, same tree green immediately before and after. At least the
  FIFTH documented occurrence (2026-04-18, 2026-05-10 ×2, the 2026-07-07 run
  cited in the 2026-07-06 entry, and today); the three 2026-04/05 entries
  called it heap-state contamination, but the test samples
  `time.perf_counter()` twice across real work
  (`test_motor_guard.py:1352,1359`) so wall-clock load is sufficient to
  explain it — a deterministic-clock rewrite is now overdue.)

## Addendum — 2026-08-09: bench discriminator run — congestion confirmed, mechanism quantified, rate anomaly resolved

The operator ran the pre-registered discriminator
(`tests/hardware/session_err_timeout_bench.md`) on FW 9. It fired the top row of
the interpretation table, and the per-stage counters bought more than the
verdict: they read out the mailbox occupancy directly, which is what closes the
Open question 2 gap.

### A1. The three-arm result

| arm | leg stream | fails | rate | 95 % Clopper-Pearson |
|---|---|---|---|---|
| A (idle) | off | 0/40 | 0.0 % | [0.0 %, 8.8 %] |
| B (500 Hz stream, platform holding) | on | **15/40** | **37.5 %** | [22.7 %, 54.2 %] |
| A2 (idle) | off | 0/40 | 0.0 % | [0.0 %, 8.8 %] |
| A + A2 pooled | off | 0/80 | 0.0 % | [0.0 %, 4.5 %] |

Fisher exact, armB vs pooled idle: **p = 8.5e-09** (vs armA alone, p = 1.21e-05).
The leg stream is the discriminator, and it is not close.

The A-B-A design earns its cost here: armA2 returns to **0/40 at a HIGHER bridge
uptime** (582 s) than armA (369 s), which kills the warm-up and
uptime-degradation confounds by construction rather than by argument. The whole
run sat 6.2–11.0 min after a bridge reboot, i.e. on a deliberately FRESH plant —
so 15/40 is a *lower* bound on what a long-uptime session would show, and any
field comparison inherits that caveat.

The validity condition held: **all 15 `can3_errors` fields were zero in all 127
logged rows** — no wire errors, so the congestion read is not contaminated by bus
health. (127 = the three arms' 3 × 40 dispatches **plus** the 7 rows of the
aborted run below. Those 7 are pooled here and ONLY here: a wire-error count is a
property of the bus during the logging window regardless of whether a dispatch
reached the Teensy, whereas pooling them into any *rate* would manufacture 7
phantom failures in an idle arm. Every rate in this addendum is 3 × 40.) (`rej = 0`, `busdown = 0` throughout too; `defer bb`
and `defer cone` both 0, so the contention is confined to the jb loom. The
converse is NOT available: the cone bus was idle, so zeroes there say nothing
about the CAN3 hardware fault either way.)

Against the field epidemic: the armB CI **contains the entire 47–52 % band**
(exact binomial p = 0.27 / 0.15 / 0.081 at 0.47 / 0.50 / 0.52). The bench
reproduces the field rate at a statistically indistinguishable level; n = 40
cannot resolve a difference below ~17 pp (n ≈ 124 for 0.375 vs 0.50 at 80 %
power). Do not read 37.5 % vs ~50 % as a discrepancy — this data cannot see one.

Cross-layer arithmetic: host `fail_teensy = 15`, `fail_host = 0` agrees exactly
with the firmware's own counters, and 26/26 cross-layer identities pass
(`calls == dispatches`, `pre1+pre2+traj == fail_teensy`, `defer_jb` reconciled
per-dispatch). `fail_host = 0` is the load-bearing one: **all 15 failures are
genuinely Teensy-side**, not host-synthesised — exactly the split the
`hand_traj_acks` row was added to make visible, working on its first real use.

### A2. The structural signature — `pre1 = 0` is the informative digit

Per-stage split of the 15: **pre1 = 0, pre2 = 7, traj = 8**.

`pre1 = 0/40` refutes per-frame independence outright. Three tests, increasingly
specific:

- vs a per-frame-uniform null (each of the three deficit states equally likely,
  0.375/3 = 12.5 %): expected pre1 = 5.0, observed 0, one-sided **p = 0.0048**.
- vs the full uniform stage split: exact multinomial **p = 0.0098**.
- vs iid-per-frame deferral at the fitted q = 0.145 (the decisive one): expected
  (5.8, 5.0, 4.2), observed (0, 7, 8), exact multinomial **p = 0.0011**. The
  direction of the rejection is the whole story — too FEW pre1, too MANY traj,
  with stage-conditional deferral rates rising monotonically (0.000 → 0.175 →
  0.242). That is a *shared resource being consumed by the dispatch itself*, not
  three independent Bernoulli trials.

Which is exactly the mailbox model, and the counters now read out the pending
count **P** at the dispatch instant. The TX mailbox count is no longer assumed:
`can_jugglebot` is `setMaxMB(16)` + FIFO with **RFFN = 0** (no `setRFFN`,
`setFIFOFilter` or `setMBFilter` call exists anywhere in the tree, and
`begin()`'s only CTRL2 write is an OR), so `mailboxOffset() = 6 + (RFFN+1)*2 = 8`
and TX mailboxes are **MB8…MB15 = 8, exactly**. `CAN_message_t::seq` defaults to
0, so `send_on` takes `write()`'s scan-all-mailboxes branch — all 8 are used, and
`write() == -1` means, without ambiguity, all 8 held un-transmitted frames. The
interp ISR's leg burst is **6 frames back-to-back in one pass**, no yield
(`leg_interp.cpp:533-541`, `NUM_LEGS = 6`), all six resident simultaneously.

| P at dispatch | fails at | predicted by | observed |
|---|---|---|---|
| ≤ 5 | — (ok) | — | 25/40 |
| 6 | traj | the leg burst alone | 8/40 |
| 7 | pre2 | burst + **one** non-leg frame | 7/40 |
| 8 | pre1 | burst + **two** non-leg frames | **0/40** |

The zero is the fingerprint: **at most ONE non-leg frame is ever co-resident with
a full burst** — which is what a census of ~150 fps of non-leg TX (0x7DD at
100 Hz, the 0x0C4 SDO poll at 50 Hz) predicts. At n = 40 the split alone cannot
discriminate a 7-mailbox world from an 8-mailbox one; the library source can, and
does. (Expected pre2 share given those two candidate frames is ~30 % of 15 ≈ 4–5;
observed 7 is +1.4 σ on Binomial(15, 0.3) — noise, not a discrepancy.) The 97.5 %
upper bound on P(mailboxes full) is 8.8 %.

`tx_q_hwm_jb` peaked at **2**, and `defer_jb` came to 16 = the 15 hand deferrals
+ one non-hand jb deferral localised to the 2 s interval of dispatch 25 — 0-based
CSV `i`, i.e. **1-based #26**, since A4's sequence indices are 1-based and this
one is not — the event that pushed the watermark 1 → 2. `defer_jb` is a global
counter with no finer
attribution, so *which* frame is not recoverable — a leg frame is the only
high-rate candidate. Read `tx_q_hwm` as the watermark it is: it says depth 2
happened at least once, never when or how often.

### A3. Open question 2 RESOLVED — why 37.5 % and not ~6 %

The naive arithmetic made three errors, in descending order of magnitude.

**(1) DOMINANT — it assumed uniform dispatch phase. The phase is two-valued.**
`task_net` (which runs the hand dispatch) wakes ONLY on FreeRTOS tick boundaries
(`vTaskDelayUntil`, `configTICK_RATE_HZ = 1000`), so an RPC arriving
asynchronously from the Jetson is dispatched at `tick_k + δ`, never at a
uniformly random instant. And the two timebases are exactly commensurate: SysTick
at `configCPU_CLOCK_HZ` is **600000 core cycles** per 1 kHz tick, exact; the
interp `IntervalTimer` PIT is **48000 counts at 24 MHz** per 2000 µs, exact; both
trees derive from the single 24 MHz crystal, so there is zero relative drift and
a **boot-fixed** offset. A dispatch can therefore occupy only **two** phases mod
the 2 ms interp period, `δ` and `δ + 1000 µs`. The observable is bimodal with a
hard **ceiling of 50 %** (one tick parity lands in the congested window), and
15/40 = 37.5 % is 3/4 of that ceiling. δ is itself biased late and repeatable —
`task_can_rx` (prio 5) runs first on the tick, then lwIP pump → stream drain →
**RPC drain last** (`udp_link.cpp:159-161`).

⚠️ **One unverified link, flagged as INFERENCE:** `CCM_CSCMR1[PERCLK_CLK_SEL]`
was not read on this build to prove PERCLK is the 24 MHz OSC, and δ was not
measured. The commensurability claim rests on both timers deriving from the one
crystal — standard for this part, but the one link in the chain not verified from
source. **The cheap falsifiable test:** stamp `micros64() - s_last_tick_us` at
each hand dispatch (`leg_interp.cpp:272,279` already keeps `s_last_tick_us`).
Clustered ⇒ model confirmed and the mailbox-count fix is the right shape. Spread
uniformly over 0–2000 µs ⇒ **this verdict is wrong** and the occupancy story
re-opens. **This probe ships WITH the Step 4 fix** (owner's decision, 2026-08-09
— see A5), as a console-only diagnostic, so the fix and its own falsification
test land together rather than the test being deferred behind a green result.

**(2) It used one drain slot of 115 µs. The real congested window is 1–2 slots of
~150 µs.** Bus census on the jb loom: our TX ≈ **3150 fps** (6 × 500 Hz leg
setpoints = 3000, 0x7DD time-sync 100, hand SDO poll 50 — `teensy_bridge_node`'s
12 timers are all ROS uplink publishers, so zero steady-state CAN TX from the
node) against ODrive RX ≈ **1904 fps** (7 axes × 272 fps from the committed
ODrive configs) plus SDO replies and relay traffic. At 111–130 bit times per
standard-ID DLC-8 frame that is ≈ **62 % utilisation** — matching the firmware's
own census comment at `can_buses.cpp:478-480` (~5,340 fps total) independently. A
stretched slot plus a lingering lowest-priority co-resident (0x7DD or the 0x0C4
poll — both outranked by *every* leg id, so they linger through exactly the
post-burst window) raises the uniform-phase prediction from 5.8 % to ~10 %. Real,
second-order, and still 3.5–4× short — which is the residual factor (1) supplies.

**(3) Bus load ALONE is refuted by ~50×.** For `P ≥ 6` to last 750 µs by
arbitration delay, the highest-priority queued frame 0x00C would have to lose
~5.4 consecutive arbitrations. Only ids 0x001–0x00B outrank it, and on this bus
that set is exactly axis 0's heartbeat, error and encoder frames = **160 fps ⇒
1.9 % duty**. Getting 5.4 back-to-back from a 1.9 %-duty set needs that set at
~100 % duty, i.e. >99 % utilisation composed almost entirely of sub-0x00C
traffic. Rule it out as a primary cause, and don't let a future session
re-derive "the bus is just busy" as the answer.

### A4. Statistics honesty — what the sequence does NOT show

The armB sequence is **consistent with iid Bernoulli(0.375)**, and it would have
been easy to over-read:

- Exact runs test (conditional on 25 OK / 15 FAIL): R = 16 against E[R] = 19.75,
  two-sided **p = 0.27** (one-sided P(R ≤ 16) = 0.133). Not significant. Lag-1
  r₁ = +0.17, permutation p = 0.13. The omnibus test is the honest summary.
- The eye-catching **6-fail run** (dispatches 12–17, 1-based) has exact
  fixed-margin P(L ≥ 6) = 0.034 — but that statistic was chosen after seeing the
  data, and the search-corrected version (max enrichment over all 820 contiguous
  windows, permutation null) gives **p = 0.068**. It is selection-inflated, and
  the FOFOFOF stretch later in the arm supplies 8 extra runs that exactly offset
  the one long block.
- **The phase-drift hypothesis — that a slowly-drifting phase produces the runs —
  was tested and is INCONCLUSIVE, leaning refuted.** Its own precondition fails:
  the host inter-dispatch gap has SD 4.875 ms = **2.44 full interp ticks**, so
  the per-dispatch phase increment is re-randomised every dispatch rather than
  creeping, and the observed per-gap phase increments are indistinguishable from
  Uniform (Rayleigh R = 0.145, p = 0.44; KS D = 0.157, p = 0.26). The one
  pre-specifiable period (exactly 2.000 ms) gives an exact label-permutation
  p = 0.052 — marginal, and **not promoted**: "2.000 ms" is 2.000 ms of *Teensy*
  time, and host↔Teensy skew is bounded only by the 100 ms `uptime_ms` grain at
  ±687 ppm = **±27 full tick periods** of accumulated phase across the 78 s arm,
  so the host-clock 2.000 ms hypothesis is one arbitrary member of a ~54-tick-wide
  family. The honest search over that family returns p = 0.51 (±200 ppm) / 0.47
  (±3000 ppm). Unresolvable without a Teensy-side timestamp — the same probe A3
  already asks for.
- The overfitting guard worked, and that is worth recording: **the fitted-drift
  sweep maxima sat BELOW the permutation null means** (0.504 vs null mean 0.513;
  0.569 vs 0.571). A drift fit flexible enough to separate any labelling
  separates the null's labellings just as well; only the search-corrected
  p-value means anything.

Note the reconciliation, because it looks like a contradiction and is not: **iid
outcomes are exactly what the two-phase model predicts here.** With per-dispatch
phase effectively random (2.4 ticks of jitter), the two-phase quantisation sets
the *rate ceiling* without inducing any *serial structure*. A3's mechanism and
A4's iid finding agree; neither is evidence against the other.

### A5. What this means for the latch premise

The stage split is operationally load-bearing, not bookkeeping:

- **pre2 failures (7 of 15) abort BEFORE the 0x6D0 frame** — `hand_ops.cpp:59`
  returns on the failed `Set_Controller_Mode` send, so `:73` never runs. Those
  dispatches genuinely do not move the hand, and the failure ack is TRUTHFUL.
  Say it precisely: `pre1` succeeded, so the hand **is** in CLOSED_LOOP and
  energised — what is missing is the *stroke*, not the arming. "No stroke", not
  "hand never armed" and not "hand de-energised".
- **traj failures (8 of 15) have the 0x6D0 frame queued into the software ring**
  and, usually, transmitted ~0.1–1 ms later. The ack LIES.

So the lying-ack is **direction-split roughly 50/50 by stage** — and the split is
now measurable live off `BRIDGE_TX_DIAG` instead of inferred. That is precisely
the cell the catch latch's `_MAX_ARM_DISPATCHES` premise needs, and it says the
premise is *half* right, in a way neither the recount nor the source-reading
could have told apart.

Corroborating, from the same CSVs: a deferred dispatch is a **clean no-op** on
the hardware. armB hand position spans 0.0006 rev across the whole arm, and FAIL
vs OK positions are indistinguishable (Mann-Whitney p = 0.79) — no lurch, no
half-executed stroke. That is the safety-relevant result and it is unambiguous.

**The latch fence STILL holds.** Confirming a mechanism is not fixing it; the
fence comes down when a fix lands AND an ordinary sitting validates it.

**Step 4 fix — DECIDED 2026-08-09 (owner); IMPLEMENTED the same day as FW 10,
flash + bench validation pending — see "Addendum — 2026-08-09 (2)".** Two parts,
landing together: (1) raise `can_jugglebot`'s `setMaxMB(16 → 24)`, i.e. **8 → 16
TX mailboxes**; (2) a **console-only phase-stamp diagnostic** logging
`micros64() - s_last_tick_us` at each hand dispatch — A3's falsifiable test,
shipped WITH the fix rather than deferred behind a green result, so the
phase-quantisation verdict is checked on the same hardware run that is supposed
to benefit from it. Chosen over the other two assessed candidates because it is
the only one with **no 0x6D0 duplicate-or-invert hazard** — bounded retry
re-sends a frame `write() == -1` has already enqueued (idempotent for the two
preamble frames, *not* for a trajectory command), and continue-past-deferral lets
a directly-written frame 3 overtake a ring-queued frame 2, driving the hand
against an ODrive not yet in PASSTHROUGH — and because it also makes the A6
`events()` residual **unreachable at the observed occupancy** (peak pending 8
against 16 mailboxes, so the software ring is not entered at all). Honest
expected outcome, recorded now so it is not read as a regression later: the
ERR_TIMEOUTs go away, **the wire latency does not** — 0x0C7, 0x0CB and 0x6D0 all
rank below every leg id, so they still transmit after the burst drains.

### A6. Two side-findings

**(i) The `events()` refill loop's missing `break` — sharpened from §2, not new.**
§2 named the defect; the bench read turns it into a standing caveat. The
`frame.mb == -1` branch (`FlexCAN_T4.tpp:1084-1102`) iterates every TX mailbox and
calls `writeTxMailbox(i, frame)` + `pop_front()` on each free one, with **no
break** — so one deferred frame is written into EVERY free mailbox while popping
(and thereby discarding) other queued frames. Our deferral path always sets
`mb = -1`, and `can_buses_service()` calls `events()` at 1 kHz from
`task_can_rx`. Two consequences: **"deferral is not a drop" is only CONDITIONALLY
true**, and a deferred 0x6D0 may occasionally duplicate on the wire *today*,
independent of any fix. Exposure is genuinely rare — the FlexCAN TX-complete ISR
normally refills within µs and only loses the race when it was masked by
`interp_isr` or a PRIMASK region — and under the decided fix it **becomes
unreachable at the observed occupancy** (peak pending 8 against 16 mailboxes, so
the software ring is never entered); the ring is only re-entered if a future TX
producer doubles the burst, which is exactly when this residual would come back.
Tracked as a residual (Open question 4); the cheap probe is counting duplicate
0x6D0 arrivals Platform-Teensy-side during a failing dispatch.

**(ii) Stale comment: `can_buses.cpp:491-492` states the interp ISR's NVIC
priority as 32. It is 16** (`leg_interp.cpp:608`). What makes this worth a fix is
that the comment's own *conclusion* — that the ISR sits above the FreeRTOS
syscall ceiling and so preempts `task_net` at any instruction, including inside
`NetLock` — is correct, but **not at the number it cites**. The ceiling
(`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`) is itself 32, so an ISR at 32
would sit *at* the ceiling and be maskable by FreeRTOS critical sections; the
stated number and the stated consequence contradict each other. At the real
priority of 16 both hold, which is why the wrong number never produced a wrong
inference — and why a future reader who trusts the number instead of the
consequence would reason correctly about masking and land on the opposite answer.
Comment-only fix, no behaviour change.

### Addendum verification

- Bench CSVs (operator, 2026-08-09):
  `temp/probes/hand_dispatch_ladder_armA_2026-08-09_13-33-28.csv`,
  `temp/probes/hand_dispatch_ladder_armB_2026-08-09_13-35-19.csv`,
  `temp/probes/hand_dispatch_ladder_armA2_2026-08-09_13-37-01.csv`.
  A fourth file, `hand_dispatch_ladder_armA_2026-08-09_13-33-08.csv`, is an
  ABORTED 7-dispatch run (`Invalid target: -0.140 rev`) — the HOST rejected the
  target, nothing reached the Teensy, and `calls`/`ok`/`fail_teensy` stayed 0
  across all 7 rows. **Do not pool it with armA**: doing so manufactures 7
  phantom failures in the idle arm and destroys the discriminator.
- Statistics: every tally re-derived from the CSVs, plus the runs / phase /
  mailbox-model tests, run 2026-08-09 — **26/26 cross-layer identities PASS**.
  Script and write-up in the session scratchpad (`bench_analysis.py` →
  `bench_analysis.md`, one-off per the probe policy; the durable numbers are in
  this addendum).
- Firmware ground truth: source-anchored read of `Teensy_code_canbridge/` (FW 9),
  the vendored FlexCAN_T4 and the Teensy 4 core, run 2026-08-09 (scratchpad
  `bench_fw_groundtruth.md`) — mailbox count, burst shape, task priorities, bus
  census and fix-candidate hazards, with every non-source claim tagged INFERENCE.
  `bridge_fw_version` read `9 (proto 5)` in all 127 logged CSV rows (3 × 40
  dispatches + the aborted run's 7; the aborted rows are pooled only for
  invariants like this and the wire-error check, never for a rate).
- Docs gate for this addendum: `pytest tests/sim/test_plans_index.py
  tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q`,
  run 2026-08-09: **55 passed in 0.45 s**.

## Addendum — 2026-08-09 (2): the Step 4 fix is implemented (FW 10), flash + bench validation PENDING

Both halves of the decided fix are in the tree, in one commit, unflashed.

**(1) `can_jugglebot.setMaxMB(16 → 24)`** — 8 → 16 TX mailboxes, jugglebot bus
only (`can_buses.cpp`). `mailboxOffset()` is fixed at 8 regardless of MAXMB
(6 RX-FIFO engine MBs + 2 filter-table MBs, RFFN = 0), so TX count is `MAXMB − 8`.
Sizing: measured peak pending 8 (6-frame leg burst + ≤2 non-leg co-residents) plus
a worst-case 3-frame hand dispatch = 11 ≤ 16. Costs zero MCU RAM (peripheral RAM)
and does not lengthen the FlexCAN ISR (it breaks at the highest *flagged* mailbox).
bb/cone are unchanged: both deferred 0 frames across the whole bench arm and each
carries only the 100 Hz 0x7DD fan-out plus event-driven RPC relays, never a 500 Hz
burst, so a symmetric change is risk without need. The call order
`setMaxMB → enableFIFO → enableFIFOInterrupt` is load-bearing and preserved —
reversing it silently clears BUF5M and the bus goes RX-dark with no error.

One asymmetry to record before it is forgotten: parking the § A6 `events()`
missing-`break` defect is **one-directional in blast radius**. At the observed
occupancy the software ring is never entered, so the defect is unreachable — but if
anything ever re-opens the deferral path, that break-less loop now duplicates the
peeked frame into up to **16** mailboxes instead of 8, i.e. a deferred 0x6D0
duplicates twice as hard as it did on FW 9. Anything that re-opens deferral must fix
the vendored loop, not just re-size the mailboxes.

**(2) The `[handphase]` console diagnostic** — `hand_ops` stamps
`micros64() - interp_last_tick_us()` at HAND_TRAJ_CMD *entry* (before the gates, so
every dispatch is stamped at the same point and the OK/FAIL phase distributions are
comparable) and pushes `{phase_us, outcome}` into an 8-deep ring that `task_diag`
prints on change. `leg_interp` gained a torn-load-guarded `interp_last_tick_us()`
accessor; `s_last_tick_us` is now `volatile`. **No wire change** — no MsgType, no
payload, PROTOCOL_VERSION stays 5, and `test_wire_layout_frozen` passes unchanged
as the check that nothing leaked onto the wire. Also fixed: the stale
`can_buses.cpp` NVIC-priority comment from § A6(ii).

`FW_VERSION` 9 → 10 and `EXPECTED_BRIDGE_FW_VERSION` 9 → 10. Because the bump is
wire-invisible, an FW 9 board decodes identically — so a healthy link is **not**
evidence the fix is on the board; the `BRIDGE_FW_CHECK` line is.

**Validation PENDING (nothing below has been run).** Flash FW 10, then re-run the
same three-arm A-B-A ladder (procedure in `tests/hardware/session_err_timeout_bench.md`
§ "Post-fix validation"). Expected: 0/40 in every arm, `defer jb` increment 0, and
the `[handphase]` samples clustered at two values — a uniform 0–2000 spread refutes
§ A3's phase-quantisation verdict and re-opens the occupancy story, which is the
whole reason the probe ships with the fix. Also compare `interp_max_jitter_us` /
`interp_deadline_misses` before and after: the longer `write()` mailbox scan sits
inside the existing PRIMASK region.

**Entry stays in-progress.** The latch fence stays up until the flash lands AND an
ordinary reload sitting validates it — confirming a mechanism was not fixing it,
and implementing a fix is not validating it.

### Addendum 2 verification

- `pio run -e teensy41` and `pio run -e teensy41_bench_sysid`, run 2026-08-09:
  both **SUCCESS** (9.28 s / 9.34 s).
- `pytest tests/firmware -q`, run 2026-08-09: **399 passed in 193.61 s** —
  including `test_wire_layout_frozen` UNCHANGED, which is itself the check that
  nothing about this change reached the wire.
- `./run_tests.sh --full`, run 2026-08-09: **4649 passed, 3 xfailed** in the
  parallel phase + **9 passed** serial, **RESULT PASS**, total 499 s. Run because
  the next step for this change is a hardware sitting; the 3 xfails are the
  pre-existing `nightly`-tier ones, unrelated to this change.
- `./run_tests.sh`, run 2026-08-09 (final pre-commit gate): **4226 passed,
  RESULT PASS**, total 201 s.

An independent pre-commit audit returned **COMMIT-READY** with 4 warnings and
3 notes, all applied before the commit. The one that mattered was caught by
reading the disassembly, not the source: `micros64() - interp_last_tick_us()`
written as a single expression leaves the two calls only *indeterminately
sequenced*, and GCC emits `micros64()` first — so an interp tick landing between
the two samples underflows the u64 subtraction and surfaces as `phase_us` ≈
65480-65535, a value this ring's own contract calls impossible. A rare
self-inflicted refutation of the very model the probe exists to test. The tick is
now read into a local first, making the ordering a source-level fact.
