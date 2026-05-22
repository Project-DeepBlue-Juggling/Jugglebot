---
title: MPC is compute-bound on the Jetson Orin Nano — 17-40% of solves exceed the 40 Hz CPU-time budget; system-profiling harness built
type: investigation
date: 2026-05-22
status: in-progress
phase: "hardware diagnostics / compute-capacity assessment"
related_plan: ""
related_entries:
  - 2026-04-18-mpc-overhead-spikes-fallback-bursts
  - 2026-05-08-friction-ff-platform-limit-cycle
  - 2026-05-20-hold-extrap-positive-feedback-chaotic-motion
  - 2026-05-20-mpc-warmstart-deadlock-escape
sessions:
  - profile_20260521-180945
  - profile_20260521-181517
  - profile_20260521-182416
files_changed:
  - tools/probes/profile_session.py
  - tools/probes/README.md
  - logbook/INDEX.md
  - logbook/2026-05-22-mpc-compute-bound-jetson-profiling.md
commits:
  - 07fb47c
subsystem:
  - mpc
  - can
  - tools
tags:
  - performance
  - safety
---

# MPC is compute-bound on the Jetson Orin Nano

## Summary

The MPC motion planner produces jerky, poorly-controlled end-effector motion
on hardware — even for trivial moves — while the *same* controller runs
smoothly in simulation on both the Jetson and a Win10 PC. To find out
whether this is a hardware-capacity problem, we built a Jetson
system-profiling harness (`tools/probes/profile_session.py`) and ran three
instrumented hardware sessions.

The decisive finding: **17-40 % of MPC solves exceed the 40 Hz CPU-time
budget (25 ms)**. Mean solve time is 17-20 ms — already 70-80 % of budget —
and every over-budget solve raises `Maximum_CpuTime_Exceeded`, dropping the
controller into the `cold_hold` / `fallback_extrap` path. That fallback path
is precisely where this project's recently-documented motion-quality bugs
live (singleton-emission jerk, the hold-extrap positive-feedback chaos). So
the "algorithmic bugs" are, in large part, *downstream of compute-budget
violations*.

Conclusion: the MPC is genuinely compute-bound on this hardware. A compute
upgrade is well-justified. The recommended next step is a CPU-isolation
experiment that previews the upgrade for the price of an `apt install`,
before any hardware is purchased.

No production code was changed in this investigation. The deliverable is the
profiling harness plus this assessment.

## Symptoms

- MPC-driven motion on the physical robot is jerky and poorly controlled,
  including on moves that are geometrically trivial.
- Reproduced across three hardware sessions on 2026-05-21
  (`profile_20260521-180945`, `-181517`, `-182416`). Session 2 additionally
  had the 500 Hz hand-telemetry timer disabled; session 3 was a fresh boot.
  All three behaved essentially the same.
- The *same* MPC controller runs smoothly in `sim/` on both the Jetson and a
  Win10 desktop. The problem is hardware-deployment-specific, not algorithmic
  in the controller core.
- The last hardware session remembered as genuinely smooth was 2026-05-08
  (logged in `2026-05-08-friction-ff-platform-limit-cycle.md`, code state at
  commit `38d7415`).

## Diagnosis

### Profiling harness

Built `tools/probes/profile_session.py` — a *live-observer* probe (a new
category in `tools/probes/`, distinct from the existing offline-driver
probes). It records, for the duration of a session:

- `tegrastats` — per-core CPU %/frequency, EMC, GPU, thermal, power
- `pidstat` — per-process CPU %, context switches, IO wait
- `mpstat` — per-core utilisation
- `candump` — every CAN frame, for frame-rate / arbitration-ID analysis
- `py-spy` — flame graphs of named processes (wait-and-attach)
- a periodic `ps` snapshot — resolves generic `python` PIDs to command lines

It auto-analyses the recording into a `report.md` flagging CPU saturation,
DVFS throttling, thermal/EMC/memory pressure, top processes, and CAN traffic.

### Finding 1 — the system is oversubscribed

Summing mean %CPU across the top 15 processes in session 3: **~852 % of CPU
demand against 600 % capacity** (6 cores), before the ~15 smaller processes
below the top 15. Cores are not permanently pegged (mean ~61 % all-core) but
every core's p95 is 100 % — the system is **frequently maxed out in bursts**,
and those bursts coincide with the 40 Hz solve.

### Finding 2 — `can_node` pins a core

`can_node` consumed 82-99 % of one core across the three sessions. CAN
traffic was 2300-2800 frames/s (ODrives broadcasting heartbeat + encoder +
iq + temp + bus-voltage streams continuously). By the harness's own
per-frame formula (`mean_cpu/100 × 1e6 / frame_rate`) that works out to
~290-430 µs of CPU work per CAN frame (approximate — the bound depends on
which session's CPU%/rate pair is used). The cost is structural, not algorithmic: seven
concurrent ROS2 timers (1 kHz CAN poll, 500 Hz hand telemetry, 100 Hz state,
100 Hz time-sync, 10 Hz × 2, 1 Hz) = **1721 timer callbacks/s** on a single
Python executor thread, plus a per-frame dispatch chain that takes the
`MotorStateTracker` lock on every message. *(The earlier "busy-poll on
`recv(timeout=0)`" hypothesis was ruled out — the loop breaks on `None`; see
Withdrawn claims.)*

### Finding 3 — THE decisive finding: solves exceed budget

`solve_time_ms` and `solve_status` from the MPC telemetry CSVs
(`temp/logs/mpc_20260521_18*.csv`) for each session, against the 40 Hz /
25 ms budget:

| session | mean | p50 | p95 | p99 | max | % over 25 ms | % Maximum_CpuTime_Exceeded |
|---|---:|---:|---:|---:|---:|---:|---:|
| S1 (as-outlined)  | 20.1 | 20.3 | 31.7 | 38.5 | 56.0 | 33 % | 40 % |
| S2 (hand timer off) | 17.7 | 16.8 | 30.5 | 34.5 | 59.6 | 16 % | 17 % |
| S3 (post-reboot)  | 17.0 | 13.9 | 29.8 | 35.3 | 46.3 | 18 % | 22 % |

Every `Maximum_CpuTime_Exceeded` solve falls through to `cold_hold` or
`fallback_extrap`. The mean solve already eats 70-80 % of budget; p95 is over
budget in every session. **This is direct, hard evidence that the MPC is
compute-bound.**

S1 (40 % CTE) had VSCode's `code` process spiking to 259 % CPU; S2/S3
(17-22 %) did not. So contention *modulates* the CTE rate — but even the
quietest session sits at a 17 % floor. The cause is **both** intrinsic core
slowness **and** contention.

### Finding 4 — the fallback path is where the motion bugs live

`Maximum_CpuTime_Exceeded` → `cold_hold` / `fallback_extrap`. The fallback
path is exactly where this project's recent motion-quality investigations
landed: the hold-extrap positive-feedback chaos
(`2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md`), the
walk-forward singleton-emission jerk, the warm-start deadlock
(`2026-05-20-mpc-warmstart-deadlock-escape.md` — itself triggered by a
chronic run of CTE failures). So the documented "algorithmic bugs" are, to a
large extent, *symptoms entered because solves ran out of CPU time*. Remove
the budget violations and the fallback path stops executing.

### Finding 5 — the hand-timer experiment

Session 2 ran with `can_node`'s 500 Hz hand-telemetry timer disabled. That
cut `can_node` CPU by ~13 pp (95.7 % → 82.5 %) — the change worked
mechanically — but produced **no observable change in MPC motion**. On its
own this is weak evidence; combined with the rest, it indicates `can_node`
CPU load is *not* the primary cause of the MPC motion problem (though it is a
real, separate inefficiency).

### Finding 6 — `jetson_clocks` works; the DVFS warning was a harness bug

All three sessions ran after `sudo jetson_clocks`. tegrastats confirmed every
core pinned at min = max = 1510 MHz (vs. 729 MHz DVFS dips beforehand). The
harness's initial "CPU governor is DVFS-enabled" warning was a **false
positive** — it checked the governor *string* (`schedutil`), but
`jetson_clocks` pins frequency by clamping `scaling_min_freq =
scaling_max_freq` without changing the governor. Fixed: the analyzer now
decides DVFS status from observed frequency data.

## Discussion

**This investigation reversed its own central conclusion mid-stream, and the
reversal is the most important thing to record.**

The initial framing was sceptical of a hardware upgrade: the MPC problem was
described as a "regression", regressions are software events, and a faster
machine just runs buggy code faster. Initial stated confidence that a
compute upgrade (an OnLogic Helix 401 was the candidate) would fix the motion
problem: **~30-40 %**.

That framing was wrong, and the operator's pushback dismantled it point by
point:

1. **The "regression" anchor was false.** The earlier "dumber" controller
   was open-loop TRAP_TRAJ position targets at 100 Hz — computationally
   trivial, and janky by the operator's own account. It was *never* a smooth
   real-time baseline. So "the old controller worked on this Jetson,
   therefore the hardware is adequate" is a non-sequitur — the old controller
   never demonstrated closed-loop real-time capability at all.
2. **852 % demand / 600 % capacity is, definitionally, CPU-bound** at the
   system level. This was under-weighted in the initial framing.
3. **40 Hz was forced, not chosen.** The operator wanted a faster MPC rate;
   trial-and-error showed faster rates were even more compute-bound. Compute
   is therefore limiting the *control design*, not just incidental load.
4. **The "algorithmic bugs" are largely downstream of compute.** The fallback
   path is entered on `Maximum_CpuTime_Exceeded`. Finding 3's 17-40 % CTE
   rate is the upstream cause; the fallback amplifiers are the visible
   symptom.

The solve-time data (Finding 3) then settled it empirically. Revised
confidence that a compute upgrade substantially improves the MPC motion:
**~75-85 %**. The arithmetic is clean — a 3-4× faster core turns 17-20 ms
mean solves into ~5-6 ms, with p95/p99/max all comfortably under 25 ms; the
CTE rate goes to ~zero; the fallback path stops firing; and the MPC rate
could finally be raised.

The residual ~15-25 % uncertainty: (a) `can_node`'s ~290-430 µs/frame still
looks like a software inefficiency that a faster machine would *mask* rather
than fix; (b) some jerk could be genuinely algorithmic and survive clean
solves. Both are cheaply testable — which is why the recommendation is to run
the CPU-isolation experiment (see Outcome) *before* buying hardware, not to
buy on the strength of this entry alone.

**Why `sim` running smoothly is consistent with all of this:** `sim/` runs
the MPC without the contending ROS2 stack — the solver gets the whole
machine, makes budget, never enters fallback. On hardware the same solver
competes against ~850 % of demand, gets smeared over the 25 ms budget 17-40 %
of the time, and falls into the jerky fallback path. Sim-smooth-on-Jetson
indicates the controller core is sound on Jetson-class compute *when
uncontended* — pointing at the deployment environment, not the algorithm.

**Ruled out:** busy-poll in `can_node` (the `fetch_all` loop breaks on
`None`); rclpy executor head-of-line blocking propagating into the MPC (it is
a *separate process* with its own executor); DVFS throttling
(`jetson_clocks` confirmed active).

## Fix

No production code was changed. Concrete deliverables and actions:

- **New harness**: `tools/probes/profile_session.py`, with
  `tools/probes/README.md` updated to add the "live observer" probe category
  and document the new harness.
- **Corrective host config** (operator-applied, not in-repo): a
  `jetson-clocks.service` systemd unit to pin clocks at boot; a
  `/etc/sysctl.d/99-ptrace-permissive.conf` drop-in to allow `py-spy` attach
  (the `99-` prefix matters — a stock `10-ptrace.conf` sets `ptrace_scope=1`
  and a lower-sorting filename loses to it).
- **Unrelated working-tree change reverted**: an undescribed 9-line deletion
  in `can_node.py`'s `_watchdog_check` (removing the deferred-stow latch
  arming — see `2026-05-19-can-loss-fault-response-safety-inversion.md`) was
  found in the working tree, was not made deliberately, broke
  `test_watchdog_detection_arms_deferred_stow`, and was reverted with
  `git checkout`. `can_node.py` is at its committed state; net change zero.

## Outcome

Diagnosis complete: **the MPC is compute-bound on the Jetson Orin Nano.**
17-40 % of solves exceed the 40 Hz CPU-time budget, and the resulting
`Maximum_CpuTime_Exceeded` fallbacks are a primary driver of the observed
motion-quality problems. Confidence that a compute upgrade substantially
improves the MPC motion: ~75-85 %.

**Recommended next step — the CPU-isolation experiment.** Use `cpuset`
shielding to give `run_mpc.py` a dedicated, uncontended core on the *current*
hardware (`cset shield --cpu 5 --kthread on`, then launch `run_mpc.py` inside
the shield). This previews the "no contention" half of a hardware upgrade for
the price of an `apt install`. Re-running the Finding-3 solve-time analysis
on the shielded session tells us how much of the CTE rate is contention
(removed by isolation, and by extra cores on a new machine) vs. intrinsic
core speed (removed by a faster core). All outcomes point at the upgrade; the
experiment quantifies it and may yield a $0 interim fix (core-pinning).

Status `in-progress`: the diagnosis is done; the isolation experiment, the
compute-upgrade decision, and any `can_node` efficiency work are pending.

## Withdrawn claims

- **"The MPC problem is probably a software regression; a hardware upgrade
  likely won't fix it" (initial framing, stated confidence 30-40 %).**
  Withdrawn. The "regression" framing assumed the prior controller was a
  working smooth baseline; it was open-loop TRAP_TRAJ targets and janky by
  the operator's own account. The solve-time data (17-40 % of solves over
  budget) is direct evidence of compute-bound behaviour. Revised confidence:
  ~75-85 %. See Discussion.
- **The harness's "CPU governor is DVFS-enabled" warning.** False positive —
  it checked the governor string, not observed frequency. `jetson_clocks`
  pins frequency by clamping `scaling_min_freq = scaling_max_freq` without
  changing the governor. Harness fixed to use frequency data.
- **"`can_node` busy-polls `recv(timeout=0)`"** (an early hypothesis for the
  ~99 % core use). Ruled out — `CANBus.fetch_all` breaks the loop on `None`;
  it drains-and-exits rather than spinning. The real cost is per-frame
  dispatch + 1721 timer callbacks/s.

## Open Questions

- **CPU-isolation experiment result** — how much of the 17 % CTE floor is
  contention (removed by isolation / more cores) vs. intrinsic core speed
  (removed only by a faster core)?
- **py-spy flame graphs** — not captured this round: `ptrace_scope` was `1`
  (the sysctl drop-in lost a filename-ordering race to a stock
  `10-ptrace.conf`) *and* the harness attached py-spy at `start`, before the
  ROS2 stack existed. Both fixed (wait-and-attach + the `99-` prefix); next
  session should capture run_mpc.py and can_node flame graphs.
- **The 7-8 unidentified `python` processes** at 60-91 % CPU each — not the
  8 named launch nodes. The harness now snapshots `ps` every 5 s so the next
  session resolves them by command line.
- **`can_node` ~290-430 µs per CAN frame** — likely a software inefficiency
  worth its own investigation; a faster machine would mask it, not fix it.
- **What changed between the smooth 2026-05-08 session (commit `38d7415`)
  and now** — a candidate `/investigate`, including the option of checking
  out `38d7415` and re-running the 7-move battery as an A/B.
