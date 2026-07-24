---
title: "Single-ball toss Phase 3 (prep): real-ordering trace harness + runbook — PREPARED, operator execution pending"
type: feature
date: 2026-07-25
status: prepared-pending-hardware
phase: "MVP trajectory bringup — Phase 8 / single-ball toss Phase 3 prep (trace harness + runbook)"
related_plan: single-ball-toss.md
subsystem: hardware-test
tags: [testing, hardware, docs]
commits:
  - PENDING
files_changed:
  - tests/hardware/toss_trace_recorder.py
  - tests/hardware/session_phase8_toss_trace.md
  - plans/active/single-ball-toss.md
  - tools/probes/toss_trace_synth.py
  - tools/probes/README.md
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
---

# Single-ball toss Phase 3 (prep): trace harness + runbook

> **Phase 3 is PREPARED, not executed.** The live-launch trace captures are
> operator-run (this run's charter forbids hardware actuation). Everything
> below lands the tooling and the session script; the plan's Phase 3 flips
> to PREPARED.

## Summary

The real-ordering trace tooling for Reload → Toss on the live launch lands:
`tests/hardware/toss_trace_recorder.py` (an observe-only rclpy recorder +
pure-stdlib offline checker evaluating 14 dry-trace and 4 reject-trace
ordering invariants) and `tests/hardware/session_phase8_toss_trace.md` (the
operator runbook, mirroring `session_phase7_reload.md`'s conventions). The
design phase surfaced a plan error: the plan's "unpowered bench" dry trace
is unreachable — activation faults against unpowered ODrives and the 10 Hz
mode republish kill the goal `REJECTED_WRONG_MODE` before the ball-evidence
waiver ever matters — so the runbook specifies a **powered no-ball bench**,
and the plan's Phase 3 text is amended accordingly. Mocked-ROS tests are
blind to cross-process choreography (the Phase-7 audit found five BLOCKING
ordering bugs exactly there); this harness is the pre-hardware gate for the
Phase-1 orderings.

## Changes

- **`tests/hardware/toss_trace_recorder.py`** — `record`: observe-only
  rclpy node subscribing the 16 ordering-relevant wires (correct QoS incl.
  TRANSIENT_LOCAL for action status; `/rosout` as the service-evidence
  channel since `arm_catch`/`set_hand_traj` leave no topic trace;
  `link_status` capture for the `uptime_ms` discipline; JSONL rows with
  monotonic stamps to `temp/logs/`; a 1 Hz live confirmation line).
  `check`: pure stdlib; goal windows from action-status transitions; every
  invariant reports PASS / FAIL / AMBIGUOUS with a 5 ms same-wait-set
  epsilon band (sub-epsilon orderings are AMBIGUOUS, never silently PASS);
  exit 0 iff no FAIL. Deliberately NOT pytest-collected (name matches
  neither `test_*` nor `*_test` pattern).
- **Invariant checklist** (cross-referenced in the runbook): DT-1
  `prime_hold` before `catch/armed`; DT-2 armed before announcement; DT-5
  announcement before the THROWING transition (a ~1-tick dispatch-time
  proxy — the true `set_hand_traj_cmd` instant is unobservable) plus a
  negative pre-announcement cmd-echo scan; DT-6 no prime dispatch while
  hold is True;
  DT-8 no `dynamic_target` actuation pre-arm; DT-9 single throw dispatch
  (never re-sent); DT-11 `prime_hold` released last in the teardown; plus
  7 more DT and the 4 REJECTED_NO_BALL invariants (RJ-1 exact reject code,
  RJ-2 total choreography silence — the hand-cmd-echo-untouched check lives
  in this composite, RJ-3 no waiver WARN in the un-waived capture, RJ-4
  zero action-feedback publishes).
- **`tests/hardware/session_phase8_toss_trace.md`** — the runbook: colcon
  build (`--packages-select jugglebot_interfaces jugglebot`) + relaunch
  preflight with a stale-build check; tracker-liveness check (the Phase-1
  known limitation: a dead tracker passes every CHECKING gate); the
  un-waived `REJECTED_NO_BALL` short trace FIRST; then the dry trace with
  the waiver parameter set (and re-verified unset at session end — this
  runbook is the one sanctioned place the waiver is ever set; ball-flying
  runbooks never set it); PASS/ABORT criteria per capture; the
  `toss_mocap_body` bench task (resolve the platform QTM body name +
  platform_start frame, do NOT enable the parameter); `uptime_ms` logged
  via the recorder's link_status capture.
- **`plans/active/single-ball-toss.md`** — Phase 3 amended (unpowered →
  powered no-ball bench, with the grounded reason) and flipped to
  PREPARED (operator execution pending).

## Discussion

**Why powered, when the plan said unpowered.** The waiver exists because an
unpowered bench cannot show hand-telemetry ball evidence — but the grounded
walk shows the goal never gets that far: ACTIVE activation faults against
undervoltage ODrives, and the orchestrator republishes the resulting
non-TRAJECTORY mode at 10 Hz, so CHECKING dies `REJECTED_WRONG_MODE` before
the possession gate. On a powered no-ball bench every precondition passes
naturally except possession — exactly what the trace-only waiver waives
(hand park-band and freshness stay hard and pass, since hand telemetry
flows at 100 Hz when powered). The plan text is amended rather than worked
around: a future reader following the original wording would burn a bench
session discovering this.

**The dry trace is a real actuation — named loudly.** With power, the dry
capture fires the first-ever real kind-0 throw stroke on Jugglebot's hand
(empty cup, ≈3.93 m/s release-equivalent, flight_time 0.8 s — mid-band,
away from the hardware-marginal <0.7 s region; the trace is about ordering,
not ballistics). The runbook frames this explicitly ("dry ≠ motion-free")
and recommends folding the captures into the same sitting as Phase-5 T0
(same powered-bench no-catch discipline; the empty-cup stroke is strictly
milder than T0's `event_vel` ladder), leaving sequencing to the operator.
The checker's PASS on both captures stays a hard gate before any ball
flies.

**Recorder/checker honesty.** Same-wait-set observation limits mean the
recorder cannot always prove sub-5 ms orderings; the checker reports those
AMBIGUOUS rather than PASS (the never-silent-PASS principle — an ambiguous
DT-1 is a re-run, not a green light). The `ABORTED_NO_RELEASE` outcome
(the live ERR_TIMEOUT epidemic eating the dispatch) is a valid alternate
capture: the checker banners it and SKIPs exactly the two release-dependent
invariants — DT-9 (single stroke at the release) and DT-12 (the MISSED WARN
pair). DT-5 is NOT skipped on this variant post-fix: the FSM idles in
THROWING until the release deadline, so the THROWING-feedback dispatch
proxy exists and both DT-5 layers (proxy ordering + the pre-announcement
negative scan) evaluate normally. The runbook's re-run rule covers the
MISSED-path capture.
The checker was verified both ways on synthetic traces before landing:
happy-path dry and reject traces pass 14/14 and 4/4; an injected 22 ms
armed/announcement inversion and a duplicated dynamic_target fail exactly
DT-2 and DT-7 with no collateral; a wrong reject code + waiver WARN fails
exactly RJ-1/2/3.

**Deliberately not done** (the prep-only charter): no live captures, no
launch, no waiver ever set outside the runbook's own steps; the optional
synthetic-announcement injection capture was rejected outright (it seeds
phantom tracker expectations — the `REJECTED_TRACK_ACTIVE` class). Phase 4
note: the displaced-throw (Tier 8b) choreography will need an addendum
capture before T4 — flagged in the Phase-4 spec.

## Verification

- Checker end-to-end matrix vs the committed synthetic-trace probe
  `tools/probes/toss_trace_synth.py`, run 2026-07-25 (after the audit's
  DT-5 dispatch-proxy fix): `python tools/probes/toss_trace_synth.py --all
  --verify` (generates 21 traces to `temp/probes/toss_trace_synth/` and
  runs `python tests/hardware/toss_trace_recorder.py check <trace> --dry` /
  `--reject` on each) — dry happy-path 14/14 PASS exit 0; reject
  happy-path 4/4 PASS exit 0; the faithful `ABORTED_NO_RELEASE` variant
  (dispatch eaten, THROWING reached, no stroke) → banner + exactly DT-9 and
  DT-12 SKIP, DT-5 evaluated via the THROWING proxy with a clean
  pre-announcement negative scan, remaining 12 PASS, exit 0; each of the
  18 single-invariant violation traces (one per DT-1..14 / RJ-1..4) FAILs
  exactly its targeted invariant with all other invariants PASS (matrix
  CLEAN). Counterfactual, same
  date: the `viol_dt5` inversion trace (hand cmd-echo changed before the
  announcement) PASSes 14/14 exit 0 under the reconstructed pre-fix DT-5 —
  reproducing the audit finding the fix closes.
- `py_compile` clean under the venv Python and system Python 3.8; `check`
  imports zero ROS modules (verified via `sys.modules`).
- Not pytest-collected (verified against both collection patterns).
- Full suite (pre-commit gate) — `pytest tests/ -q`, run 2026-07-25
  against the final post-audit tree: **3330 passed, 3 xfailed, 198
  warnings in 1316.43 s** (same counts as the Phase-2 ci-deep baseline —
  the harness and probe are deliberately uncollected, so the suite count
  is unchanged).
- `/audit` (pre-commit, 2026-07-25): no BLOCKING. 3 WARNINGs, all
  fixed-and-landed: **DT-5 passed an inverted trace** (empirically probed
  by the auditor — the dispatch-evidence scan re-anchored at the
  announcement and read the inversion as a positive gap; deeper, the true
  `set_hand_traj_cmd` instant is unobservable on every channel; fixed with
  the THROWING-feedback dispatch-time proxy + a DT-8-style negative
  pre-announcement scan, and the self-verification's blind spot is why the
  synthetic-trace generator is now the committed 21-case probe); the
  waiver's code comment still carried the disproven "unpowered bench"
  rationale (reworded, comment-only); this entry mislabelled RJ-4. Plus
  2 NOTEs fixed: DT-2 now enforces the ≥1-tick floor its text claimed;
  the generator was promoted to `tools/probes/toss_trace_synth.py` with
  one violation case per invariant (the audit's own counterfactual: the
  pre-fix checker passes the inverted trace 14/14; the fixed checker
  fails it on exactly DT-5). The audit verified every runbook command
  against source (action/param/topic names, log-match strings verbatim),
  the recorder QoS against all 13 publishers, the collection-safety
  claim empirically, and the plan-amendment's technical claims
  (10 Hz mode republish at orchestrator_node `_tick`; the unpowered
  bench additionally fails `REJECTED_HAND_STALE`).
