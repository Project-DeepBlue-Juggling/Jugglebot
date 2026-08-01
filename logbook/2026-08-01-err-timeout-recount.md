---
title: "ERR_TIMEOUT epidemic recount pre/post bus-role swap — it survived the swap; not the CAN3 drive-path fault"
type: investigation
date: 2026-08-01
status: resolved
phase: "refactor-2026-07 Phase 7a"
related_plan: refactor-2026-07.md
files_changed:
  - logbook/INDEX.md
subsystem:
  - canbridge
  - ros
tags:
  - can3
  - err-timeout
  - forensics
---

# ERR_TIMEOUT recount, pre vs post bus-role swap

Closes open item 4 of `2026-07-31-can3-drive-path-fault-jugglebot-to-can2`
("re-check the epidemic's incidence now that Jugglebot runs on CAN2, before
investigating it as a separate bug"). Read-only: no node, no hardware, no code
changed.

## What was found

**Recoverable offline — from `~/.ros/log/*/launch.log`, not from the bags.** No
recorded topic carries an RPC status string and `/rosout` is not in the record
list; the bags supply the *context* (bus health, firmware wire-error counters)
and the fingerprint that dates the swap. One era boundary must be respected:
commit `20d01e9` (2026-07-24 11:09) demoted the *arm*-ack failure from WARN to
DEBUG, so from then on the **arm path's error code is unrecoverable**. Arm
failures stay *countable* by difference (`Arming hand catch:` dispatches minus
`Hand catch trajectory armed on Teensy` successes — both INFO, both textually
stable since 2026-03-17); the *prime* path keeps its full WARN message
throughout, code included.

**The epidemic survived the swap at the same rate.** Pre-swap pooled (15
sessions, 2026-07-23 → 07-31 13:56): prime **59/125 = 47.2 %**, arm
**135/258 = 52.3 %**. Post-swap (2026-07-31 14:51, the only post-swap session
with hand dispatches): prime **4/8 = 50 %**, arm **4/8 = 50 %**.

**And every one of them is a TEENSY-returned code, not a host-side timeout.**
`controller/teensy_link/rpc.py:86` synthesises its own `RpcTimeout` as
`RpcError(method, ERR_TIMEOUT, "after N retries × Ts")`, which renders with the
*same* `METHOD: ERR_TIMEOUT` prefix the recount regex keys on — so the two
classes would pool silently, and a host-side timeout means the Teensy never
answered at all, which would leave `can_jugglebot_send` unimplicated. Checked:
across the whole `~/.ros/log` window only three files contain `ERR_TIMEOUT
after`, and every occurrence is `HOME` / `STATE_READ` / `STATE_WRITE` /
`GET_AXIS_VERSIONS` — **zero on `HAND_TRAJ_CMD`**, whose failures are all the
bare form. The distinction survives.

**Every recoverable HAND_TRAJ_CMD failure is `ERR_TIMEOUT`; not one is
`ERR_BUS_DOWN`.** In the four codes-visible sessions the identity
`prime_fail + arm_fail == ERR_TIMEOUT lines` holds exactly (11=7+4, 15=8+7,
19=8+11, 48=18+30; 93/93) — which validates the derived arm counter *and*
proves the split. The only 5 ERR_BUS_DOWNs in the whole 07-01→08-01 log window
are on ACTIVATE / DEACTIVATE / SET_* — never the hand path. These are distinct
firmware paths (`hand_ops.cpp:34` gate → ERR_BUS_DOWN; `:42/:45/:56` sends →
ERR_TIMEOUT) and only one ever fires.

**The decisive cell.** Post-swap bag `2026-07-31_14-51-38` (1051 s, n=10513
`/link_status`): `bus1/2/3_health` OK 100 %, `fault_state` NONE 100 %, and
`can3_errors` — the Jugglebot-role bus's own 1 kHz firmware counters — reads
`flt=0 sust=0 tec=0 rec=0 tecInc=0 recInc=0 ack=0 crc=0 form=0 stuff=0 bit0=0
bit1=0 txctx=0 rxctx=0 gated=0` on **every one of the 10513 samples**. Zero wire
errors of any class, zero TX-gate refusals — and still 4 of 8 prime acks
ERR_TIMEOUT.

**The epidemic era has no comparable cell, and never will.** Those firmware
counters did not exist then: `/link_status` carries **33 keys and no
`can3_errors` row at all** in `2026-07-27_15-39-38` (n=6022) and
`2026-07-29_11-10-47` (n=6389), and in the pre-swap `2026-07-31_13-56-35`
(36 keys, n=1594) the row exists but reads `unknown (never seen)` on every
sample. So the pre-swap half of this argument rests entirely on the coarse
10 Hz health classifier — `bus1_health` OK 100 % in every bag from 07-23 to
07-29 11:10 while the rate ran 40–59 %, and the famous 42.4 %-WARN bag
(07-29 22:37) had no hand dispatches at all. That is enough to say **the
epidemic and the CAN3 degradation never coincided**, and it is *not* enough for
a counter-level pre/post comparison — anyone planning one should know it is
impossible retroactively.

**Mechanism, narrowed.** `can_jugglebot_send` (`can_buses.cpp:756-763`) returns
false two ways: `!partner_recent()` (counted — it is the `gated=` field) or
`send_on() → bus.write() <= 0` (**counted nowhere**). With `gated=0` all session,
every post-swap ERR_TIMEOUT was a FlexCAN **write rejection on an error-free
bus**. ~50 % per call over three sends implies ~21 % per frame if independent —
high enough that the missing counter, not more offline inference, is the next
step.

**This narrowing is in open tension with a live production premise — do not
act on it yet.** Under `hand_ops.cpp`'s early-return structure a failed
`can_jugglebot_send` either aborts before the 0x6D0 trajectory frame or *is*
that frame, so "write rejection" means the frame was never enqueued and the
hand was **not** armed. But `catch_coordinator_node.py:159-166` encodes the
opposite as load-bearing safety reasoning — "the ack ... lies — frames were
observed transmitted after a failed ack" — which is exactly why
`_MAX_ARM_DISPATCHES` **keeps** the latch after the cap instead of
re-dispatching (memory `project_reload_action_catch_latch`: never blind-
re-dispatch a hand move). Both cannot be true. Either the lying-ack
observation is a different mechanism (a response lost on the return path, with
the frame genuinely sent), or some failures come from a path this data cannot
see. **The `tx_write_fail` counter below settles it; until then the latch
behaviour stays as it is** — a future reader must not read this paragraph as
licence to change it.

**A tempting correlation, killed.** 100 % of failures land inside `mpc_active=1`
windows that occupy only 17.4 % of the post-swap session (naive p ≈ 9e-4, looks
like contention with the 500 Hz leg ISR). It is selection: 100 % of the
*successes* are in armed windows too — hand dispatches only happen during
catches, and catches only happen armed. Zero discriminating power. Recorded so
nobody re-runs it.

## Swap dating (no guesswork)

`bus3_health` (cone health on `HeartbeatT2J.flags` bits 4-5) is the FW7 +
new-ROS-surface marker row. `/link_status` keys go **36 → 37** between bag
`2026-07-31_14-17-13` (no `bus3_health`) and `2026-07-31_14-51-38`
(`bus3_health` OK ×10513). Commit `1e5a8aa` is dated 15:29, i.e. written after
the hardware work — the deployed boundary is **14:17–14:51 on 2026-07-31**.

## What is still needed

A `tx_write_fail` counter split from `tx_gated` in `can_jugglebot_send` (the
single highest-value addition — it turns ERR_TIMEOUT from a two-cause aggregate
into an attributed count, visible in every future bag); per-call attribution of
*which* of the three sends failed (preamble vs trajectory frame are different
operational stories); a TX-queue high-water mark on the Jugglebot bus; a
`link_status` counter for hand-traj ack failures so the 07-24 console-noise
demotion stops costing forensics; and **one ordinary post-swap reload sitting** —
the point estimate sits exactly on the pre-swap rate, but the CI on 4/8 is
enormous.

Full per-session table, method, reproduction commands and the caveat list:
`/tmp/claude-1000/.../scratchpad/err_timeout_recount.md` (session-scratch; the
numbers above are the durable summary). No probe scripts promoted to
`tools/probes/` — the reusable half is already
`tools/probes/link_status_health_scan.py`, and the recount script is one-off
until a fresh sitting needs re-scoring.

## Verification

- `python3 recount.py` over `~/.ros/log` (4359 session dirs, 2026-07-20→08-01),
  run 2026-08-01: 16 sessions with hand dispatches; counts as tabulated above.
- `python tools/probes/link_status_health_scan.py --match 2026-07-31` and a
  marker-row scan over the 2026-07-23…07-31 bags, run 2026-08-01: bus-health
  duty and `can3_errors`/`bus3_health` values as quoted.
- `/link_status` key-presence scan over the four cited bags (`mcap_ros2`
  reader, venv), run 2026-08-01: `07-27_15-39-38` n=6022 / 33 keys /
  `can3_errors` **absent**; `07-29_11-10-47` n=6389 / 33 keys / **absent**;
  `07-31_13-56-35` n=1594 / 36 keys / present, `unknown (never seen)` ×1594;
  `07-31_14-51-38` n=10513 / 37 keys / present, all-zero ×10513.
- `grep -rh "ERR_TIMEOUT after" ~/.ros/log`, run 2026-08-01: 15 lines across 3
  files, all `HOME`/`STATE_READ`/`STATE_WRITE`/`GET_AXIS_VERSIONS` on
  `teensy_bridge_node` — none on `HAND_TRAJ_CMD` (the host-side-`RpcTimeout`
  contamination check).
- No code changed by this entry, so no suite is implicated; the gate cited by
  its commit is Phase 5's (`./run_tests.sh --full`, run 2026-08-01:
  **parallel 4339 passed, 3 xfailed in 439.28 s; serial 9 passed in 40.13 s;
  RESULT: PASS**).
- Firmware paths read at `hand_ops.cpp:22-56`, `can_buses.cpp:725-763`,
  `telemetry.cpp:295-320` (confirming `CanErrorsPayload` is filled from
  `h.jugglebot`, i.e. the `can3_errors` row is the Jugglebot-role bus).
