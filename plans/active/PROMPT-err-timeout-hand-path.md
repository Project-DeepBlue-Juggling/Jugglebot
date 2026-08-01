# PROMPT — ERR_TIMEOUT hand-path epidemic: attribute, then fix

Self-contained prompt for a fresh Claude (Fable 5) session. Paste verbatim.

---

You are investigating and fixing the **hand-path ERR_TIMEOUT epidemic**: ~50%
of `HAND_TRAJ_CMD` dispatches (prime and arm) fail with a **Teensy-returned**
`ERR_TIMEOUT`, and have since at least 2026-07-23. This is the single biggest
operational blocker to reliable catching (a failed prime/arm costs the
stroke).

## Read first, in this order

1. `logbook/2026-08-01-err-timeout-recount.md` — the foundation. Established
   there, do not re-derive: the epidemic **survived the 2026-07-31 bus-role
   swap at the same rate** (pre-swap prime 59/125, arm 135/258; post-swap 4/8
   + 4/8), so it is NOT the CAN3 drive-path hardware fault; every failure is
   Teensy-returned from `hand_ops.cpp:42/45/56` (never `ERR_BUS_DOWN`, never
   host-side); and in the decisive post-swap cell the 1 kHz firmware wire
   counters read **zero on all 10,513 samples** with `gated=0` — narrowing
   the mechanism to `bus.write() <= 0` in `can_jugglebot_send`
   (`can_buses.cpp:756-763`): a **FlexCAN write rejection on an error-free
   bus, currently counted nowhere**.
2. `logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md` §§ on
   `rpc.cpp:117/:118` (ERR_BUS_DOWN vs ERR_TIMEOUT are distinct paths) and
   the command-gate design.
3. Memory `project_reload_action_catch_latch` + `catch_coordinator_node.py`
   ~:159-166 — the never-blind-re-dispatch latch and the lying-ack premise.

## The load-bearing fence — do not cross it

The recount's "write rejected ⇒ frame never enqueued ⇒ hand NOT armed" is in
**open tension** with the field observation the catch latch is built on
("ack lies — frames were observed transmitted after a failed ack"). Both
cannot be true. **You must not change the latch / re-dispatch behaviour until
the `tx_write_fail` counter (Step 2) settles which story is real.** The
recount entry says this explicitly; it is not a suggestion.

## Working hypothesis to test (not assume)

Hand dispatches only occur while the 500 Hz leg-setpoint stream occupies the
same controller's TX path; a 3-frame hand burst hitting TX mailbox/queue
exhaustion would produce exactly "write rejected on a clean bus" at
~21%/frame. Alternatives you must be able to exclude: a stale-pending-slot /
state bug (would show sequence structure), and a return-path loss (would
vindicate the lying-ack premise).

## Staged plan

**Step 1 — free, offline, do it first.** Sequence-pattern analysis of the
399 recorded dispatch outcomes (383 pre-swap + 16 post-swap) in `~/.ros/log/*/launch.log` (the recount's
regex recipe is in its scratch notes; re-derive from the entry if expired).
Per session: success/failure sequence → test for alternation/parity
structure vs randomness vs burstiness. Alternation ⇒ state bug; random ⇒
congestion-consistent. Document either way.

**Step 2 — the attribution flash (bridge only; platform pio stays
SUSPENDED).** Additive firmware observability on the can-bridge Teensy,
FIX-B pattern (additive `/link_status` rows / MsgType, NO PROTOCOL_VERSION
bump, warn-never-refuse):
- `tx_write_fail` split from `tx_gated` in `can_jugglebot_send` — turns
  ERR_TIMEOUT from a two-cause aggregate into an attributed count;
- per-call attribution of WHICH of the three hand sends failed (preamble vs
  0x6D0 trajectory frame — different operational stories);
- a TX-queue high-water mark for the Jugglebot bus;
- a hand-traj ack-failure counter (the 2026-07-24 console demotion made arm
  failures log-invisible; give forensics a counter instead).
Consider bundling with refactor plan Phase 7's fw-identity item (also a
bridge flash) to amortize the flash window. Flash discipline: `pio run -e
teensy41 -t upload` (NEVER `-e`-less — the bench-sysid trap), clean rebuild +
on-wire verify after any protocol-adjacent change (the 24608bb lesson).
Native-harness tests for the new counters; `pytest tests/firmware -q` green.

**Step 3 — bench discriminator (operator runs actuating steps).** Hand
dispatches WITH vs WITHOUT a synthetic 500 Hz setpoint stream
(`tools/probes/teensy_link_profiling/jetson/setpoint_stub.py` is the
exerciser). Rate collapses without the stream ⇒ queue contention confirmed.
48 V off where possible; hand axis is energised whenever ACTIVE — safe-state
discipline between hand-in-cup actions; reboot the bridge Teensy before the
session; log `uptime_ms` with every measurement (standing rule).

**Step 4 — fix, contingent on attribution.** Candidates, in rough order of
likelihood-times-cheapness: bounded in-firmware retry on write-fail
(**bounded** — the unbounded-retransmit storm class of 2026-07-31 is the
cautionary tale); dedicated TX mailbox allocation for the hand conduit;
scheduling hand bursts into interp-stream gaps. Then reconcile the latch
premise per the Step 2 verdict and update
`catch_coordinator_node.py`'s comment + the memory. Validation = one
ordinary reload sitting: dispatch failure rate must drop far below the
47-52% baseline (the 4/8 post-swap CI is huge — get n up).

## Process constraints

- Operator runs all robot-actuating commands; you prep exact commands +
  PASS/ABORT criteria and verify read-only.
- Gates: `./run_tests.sh` per commit (mutex-queued), `--full` before any
  hardware sitting, at phase closure, and for `controller/`/`sim/` changes.
  `pio run -e teensy41` + `-e teensy41_bench_sysid` must both build for
  firmware edits.
- Logbook: short-form default; this investigation WILL hit the Discussion
  triggers when the tension resolves — write the full form then. No SHA
  backfill (trailer only). `/audit --unstaged` for multi-narrative commits.
- Stop-don't-guess at any fork touching the latch, arming, or firmware TX
  paths beyond the additive counters.
- Check `git status -sb` for parallel-session work before starting and
  before every push.

## Done means

`tx_write_fail` (or its refutation) attributes the failures with counter
evidence in a bag; the mechanism is named in a logbook entry with the
lying-ack tension explicitly resolved; a fix is landed + validated at a
sitting with the (date, command, result) triple; the refactor plan's Phase 7
table and `plans/active/INDEX.md` rows updated.
