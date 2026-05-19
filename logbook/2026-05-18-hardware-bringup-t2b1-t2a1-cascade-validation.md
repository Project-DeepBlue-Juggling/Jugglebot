---
title: Hardware bringup — T-H-T2b-1 + T-H-T2a-1 telemetry-stale cascade validation (+ 2 harness bugs, z=30 solve-failure)
type: investigation
date: 2026-05-18
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 5 hardware obligations"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-11-tier2b-hardware-plant-telemetry-ff
  - 2026-05-11-tier2b-set-pose-singular-ff-bugfix
  - 2026-05-11-tier2a-hardware-plant-fk-degradation
files_changed:
  - tests/hardware/_th_test_common.py
  - tests/hardware/th_t2b1_publisher_kill_test.py
  - tests/hardware/th_t2a1_can_unplug_test.py
  - tests/hardware/rescore_artifact.py
commits:
  - 1e3bcc8
  - 7e4f84e
subsystem:
  - controller
  - hardware-plant
  - safety
  - ipc
tags:
  - hardware-bringup
  - safety
  - watchdog
  - telemetry-stale
  - estop
  - test-harness
  - false-fail
  - open-question
---

# Hardware bringup — T-H-T2b-1 + T-H-T2a-1 telemetry-stale cascade validation

## Summary

Plan 2 Phase 5's two hardware-bringup obligations both **PASS**, with
evidence from the live MPC logs:

| Test | Date | Cascade | Evidence |
|------|------|---------|----------|
| **T-H-T2b-1** (encoder-publisher SIGSTOP) | 2026-05-18 19:50 | **PASS** | ESTOP at `telem_age=0.525 s` vs 0.5 s threshold (25 ms / 1 poll late) |
| **T-H-T2a-1** (CAN unplug 1.5 s) | 2026-05-18 20:02 | **PASS** | ESTOP at `telem_age=0.505 s` vs 0.5 s threshold (5 ms late); MPC clean-exit |

The Jetson-side telemetry-stale watchdog (`controller/hardware_plant.py`
`HardwarePlant.get_state()`, cascade at `:627–702`) fires
`estop(reason='telemetry_stale')` correctly under both fault-injection
modes. Plan 2's hardware-bringup gate is **cleared**.

The harness mishandled both runs — but differently, and neither
mishandling was a watchdog defect:

* **T-H-T2b-1** — harness final verdict **FAIL** (wall-clock
  criterion bug). Re-scored → PASS.
* **T-H-T2a-1** — harness pattern-match captured nothing
  (`log_hits: []`, internal logic computed FAIL), but the operator
  used the built-in Q5 verdict-override to record **PASS**
  (correctly, from physical observation). The artefact's final
  `verdict` field is therefore PASS-by-override; the forensic
  re-score independently *confirms* PASS from the recovered cascade.

Both were test-instrumentation bugs, root-caused and fixed this
session (commits `1e3bcc8`, `7e4f84e`). Both runs were re-scored
from captured data **without a hardware re-run**
(`tests/hardware/rescore_artifact.py`); original artefacts preserved
alongside `*_rescored.{json,md}`.

A **third, unrelated finding** surfaced incidentally and is **OPEN**:
the MPC could not solve at `z=30` (1769+ consecutive
`Maximum_CpuTime_Exceeded` from session start; static hold ran
entirely on the fallback path). See Open Questions — a dedicated
`/investigate` is recommended before relying on `z=30` as the
"low safe pose" for further hardware work.

## Motivation

T-H-T2b-1 and T-H-T2a-1 are the two in-scope hardware tests for
Plan 2 (`mpc-sadpath-coverage-tiers-1-3.md`). T-H-T2b-1 (Low hazard)
validates the Jetson-side telemetry-stale watchdog in isolation;
T-H-T2a-1 (Medium-high hazard — platform freewheels) validates the
cascaded chain on real CAN loss and is **gated on T-H-T2b-1 PASS**.
The plan's archival gate requires both run and be logged. This entry
is that record.

## Investigation

### Run 1 — T-H-T2b-1 19:32 (INVALID — two compounding defects)

First attempt reported FAIL with `log_hits: []`. Diagnosis:

1. **The MPC/watchdog was not running.** `jugglebot_launch.py`
   starts motor_guard + the ROS2 graph but **not** the MPC process.
   The telemetry-stale watchdog lives in `HardwarePlant.get_state()`,
   which only executes inside `run_mpc.py`. The operator commanded
   `z=170` via the GUI (orchestrator → motor_guard path; "GUI not
   hooked to MPC yet"). No `run_mpc.py` ⇒ no `HardwarePlant` ⇒ no
   watchdog polling. Definitive: no `temp/logs/mpc_*.log` from
   2026-05-18 existed (`run_mpc.py` auto-tees stdout there on
   start; absence ⇒ it never ran).
2. **Stale-log false-FAIL (harness bug #1).** `find_latest_mpc_log()`
   silently fell back to an 8-day-stale `mpc_20260510_151628.log`;
   the tailer watched a dead file and declared FAIL after 5 s.

Rosbag `2026-05-18_19-26-50` corroborated: all 6 leg motors armed
(`current_state=8`) throughout, no errors, no mode change —
consistent with motor_guard holding the GUI setpoint while nothing
watched. The run produced **no data on the watchdog**.

### Run 2 — T-H-T2b-1 19:50 (VALID — watchdog works)

With `run_mpc.py --pose 0,0,170,0,0,0` running (watchdog live;
fresh `mpc_20260518_195035.log`), the cascade fired correctly:

```
Telemetry aging (0.100s > 0.075s)                       # WARN, telem_age 0.100 vs 0.075
Telemetry stale (0.525s > 0.5s) — triggering e-stop     # ESTOP, telem_age 0.525 vs 0.5
HardwarePlant: sent E-STOP (telemetry_stale)            # correct reason
```

The harness still reported FAIL — **criterion bug (harness bug #2,
part A)**: it scored wall-clock-from-SIGSTOP (911 ms) against a
`[475, 525] ms` window. That window assumed SIGSTOP stops telemetry
instantly; in reality ~386 ms is ZMQ/OS socket-buffer drain (queued
frames keep refreshing `_last_telem_recv_time` until the buffer
empties). After staleness onset, wall-clock and `telem_age` track
1:1 (911−486 ms ≈ 0.525−0.100 s). The watchdog fired exactly one
40 Hz poll after the `telem_age` threshold crossing — textbook.

### Run 3 — T-H-T2a-1 20:02 (VALID — cascade works)

CAN cable unplugged 1.5 s, `z=30`, `run_mpc.py` live
(`mpc_20260518_200137.log`). The full CAN-loss cascade fired:

```
Telemetry aging (0.086s > 0.075s)
Telemetry stale (0.505s > 0.5s) — triggering e-stop
HardwarePlant: sent E-STOP (telemetry_stale)
MPC loop: plant estop_requested — exiting cleanly
Exit 3: stopped by HardwarePlant estop()
```

`telem_age=0.505 s` vs 0.5 s threshold — 5 ms / well within one poll.
Operator: platform "didn't move" (Q1), no manual E-stop needed (Q2),
no other anomalies (Q4). candump `th_t2a1_candump_20260518_200207.log`
(32 828 frames, t0±~12 s) captured the ODrive-side traffic; not
auto-parsed (ODrive-protocol decoding out of scope — available for
forensic disarm-timing analysis if needed).

The harness captured `log_hits: []` **despite a live log
(25 093 bytes grew)** and its internal logic computed FAIL
("cascade may be broken") — **harness bug #2, part B (exit-flush
race)**. The operator, seeing the platform safe and the system
clean, used the Q5 verdict-override prompt to record **PASS**, so
the artefact's final `verdict` is PASS (not FAIL). The override was
substantively correct, but for the right reason only in hindsight:
it is the *live-log evidence* above (`telem_age=0.505 s` vs 0.5 s),
not "no disaster", that confirms the cascade fired. Root cause of
the empty `log_hits`: T-H-T2a-1 *structurally always* ends with the
MPC exiting on `estop()`; its block-buffered stdout tee flushes the
cascade + clean-exit lines as one block racing the tailer stop, and
a torrent of `MPC solve failed` spam (see Finding 3) outpaced the
200 Hz readline loop. The dead-tail guard did not fire (bytes > 0).

## Fixes

| Commit | Fix |
|--------|-----|
| `1e3bcc8` | **Staleness guard** — abort before the test if the resolved log's mtime is > 120 s stale, with the ROS2-launch root cause + tee fix. **Dead-tail INDETERMINATE** — `bytes_observed() == 0` ⇒ INDETERMINATE (instrumentation), not FAIL (watchdog). |
| `7e4f84e` | **Contract criterion** — score on `telem_age` at ESTOP fire vs the documented threshold (± `CASCADE_DETECTION_SLACK_S = 0.10 s`), via `evaluate_estop_contract()`. Robust to buffer drain. **Final deterministic rescan** — `LogTailer.stop()` re-reads `[start_offset, EOF]` once, de-duped against live hits; immune to the exit-flush race and torrent lag. **`rescore_artifact.py`** — forensic re-score without a hardware re-run (scans the MPC log file when `log_hits` is empty); `write_artifacts(name_suffix=…)` never clobbers the original safety record. |

Harness fix #2's rescan was functionally verified against a
simulated exit-flush (2000-line spam burst + cascade flushed as one
block + immediate stop → cascade recovered).

## Verification

- **T-H-T2b-1**: `temp/reports/t_h_t2b_1_20260518_195054_rescored.{json,md}`
  — PASS (ESTOP `telem_age=0.525 s` vs 0.5 s). Original FAIL preserved
  at `t_h_t2b_1_20260518_195054.json`.
- **T-H-T2a-1**: `temp/reports/t_h_t2a_1_20260518_200219_rescored.{json,md}`
  — PASS (ESTOP `telem_age=0.505 s` vs 0.5 s, recovered by scanning
  `mpc_20260518_200137.log`). Original FAIL preserved at
  `t_h_t2a_1_20260518_200219.json`.
- `tests/hardware/` is pytest-ignored; `temp/reports/` is gitignored
  (this logbook entry is the durable record). No regression-suite
  impact (test-infra + standalone scripts only).

## Discussion

### Why "no disaster" was not sufficient

The operator's reasonable position was "no disaster in either test,
so both PASS". At `z=30` (near the mechanical rest) the platform
settles safely whether or not the watchdog fires — so "didn't move"
is consistent with both a working cascade and a *broken* one. The
load-bearing question was never "did anything break" but "did the
safety mechanism actually engage". Only the live-log evidence
(`telem_age` at fire vs threshold) answers that. It did engage,
correctly, both times — but verifying it required reading the logs,
not trusting the harness verdict or the absence of damage.

### Two hypotheses withdrawn

1. *"The watchdog may be broken"* (harness FAIL, Run 1) — withdrawn:
   the watchdog was never running (no `run_mpc.py`).
2. *"ESTOP fires ~400 ms late"* (harness FAIL, Run 2) — withdrawn:
   the lateness is fault-injection buffer drain, not watchdog
   latency; `telem_age` at fire is one poll past threshold.

The discipline that caught both: distrust a FAIL verdict on a
safety test until the production contract variable (`telem_age`,
stated verbatim in the cascade log line) is checked against the
documented threshold. Wall-clock-from-fault-injection is not the
contract.

### Tradeoff accepted — contract criterion over wall-clock

Scoring on `telem_age` (HardwarePlant's own staleness measurement)
rather than wall-clock makes the harness immune to IPC buffer drain
— a property of *every* telemetry-loss fault-injection method, not a
defect. The accepted cost: the harness no longer asserts an absolute
fault→estop wall-clock bound (kept as informational). That bound was
never the watchdog's contract; the contract is "fire when
`telem_age` crosses `_telem_stale_estop_s`", which is exactly what
is now scored.

### Why the exit-flush rescan is structural, not belt-and-braces

T-H-T2a-1's cascade *by definition* terminates the MPC (`estop()` →
clean exit). The cascade log lines therefore *always* land in the
process's final stdout flush, exactly where a live readline tailer
races the stop. Without the deterministic stop()-time rescan the
harness would false-FAIL T-H-T2a-1 **on every future run**. This was
not a flaky edge case; it was a guaranteed failure of the original
design for this specific test.

## Open Questions

### Finding 3 (RESOLVED — superseded) — MPC cannot solve at z=30

> **Superseded 2026-05-18 — root cause found.** See
> [2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md](2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md).
> The "numerically pathological / near-infeasible z=30" hypothesis
> below was **refuted** by an in-data control (the same session held
> z=30 cleanly for 16 s at 100 % solve success / 13.8 ms / 1.58 IPOPT
> iters). The latch was a CAN-loss telemetry dropout
> (`motor_pos=None` while frames stayed fresh) that evaded both
> staleness watchdogs — a real safety gap, now closed by
> `PLANT_INTERFACE_CONTRACT.md` P5. The original framing below is
> preserved as the decision arc, not as a current claim.

`mpc_20260518_200137.log` shows `MPC solve failed (N consecutive):
Maximum_CpuTime_Exceeded` from **line 11 (session start, right after
"MPC controller initialised")** through to the cascade — **1769+
consecutive failures**, every tick, during a *static `z=30` hold*.
`Solve time: mean=29.0 ms, max=101.6 ms` vs a ~25 ms budget. The
platform was held **entirely by the fallback path** the whole
session; it never produced a real optimized solution.

This does **not** invalidate the cascade results (the watchdog is
independent of solve success) and was invisible to the operator (a
static fallback hold holds position adequately — "didn't move").
But it means **`z=30` may be a numerically pathological /
near-infeasible operating pose** for this MPC — which matters
because `z=30` is the "low safe pose" the hardware-bringup plan
wants for the hazardous tests.

Candidate mechanisms (none confirmed — needs a dedicated probe):
- Ill-conditioned Jacobian near the lower workspace boundary →
  IPOPT iteration blow-up → CPU-time cap.
- Jetson load during the test (full `jugglebot_launch.py` + rosbag
  record + `run_mpc.py` + the test script + diagnostic shell all
  running) inflating per-solve wall time.
- A genuine reference-feasibility issue at low z (intersects the
  K1–K6 reference-feasibility contract and the just-landed Tier-3a
  `quintic T<=0` / `DIAG_SCHEMA_CONTRACT` feasibility work).

**Recommended:** a dedicated `/investigate` against the K1–K6
contract (`controller/REFERENCE_LAYER_CONTRACT.md`) and Tier-3a
feasibility fixes before `z=30` is used as a trusted operating
pose. Not a Plan 2 blocker — Plan 2's hardware gate is cleared by
the cascade PASSes; this is a separate control-feasibility concern.

## Related

- [plans/archived/2026-05-18 mpc-sadpath-coverage-tiers-1-3.md](../plans/archived/2026-05-18%20mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2; Phase 5 Outcome carries the hardware-test commitment
  this entry discharges.
- [logbook/2026-05-11-tier2b-hardware-plant-telemetry-ff.md](2026-05-11-tier2b-hardware-plant-telemetry-ff.md)
  — Phase 5 unit tests for this exact watchdog (T-U-T2b-1..8);
  hardware behaviour here matches the unit-test contract.
- [plans/active/hardware-bringup.md](../plans/active/hardware-bringup.md)
  — the broader hardware-bringup plan the operator continues next.
- `tests/hardware/{_th_test_common,th_t2b1_publisher_kill_test,th_t2a1_can_unplug_test,rescore_artifact}.py`
  — the harness + forensic re-score tool.
- Artefacts (gitignored runtime): `temp/reports/t_h_t2b_1_20260518_195054*`,
  `temp/reports/t_h_t2a_1_20260518_200219*`,
  `temp/reports/th_t2a1_candump_20260518_200207.log`.
