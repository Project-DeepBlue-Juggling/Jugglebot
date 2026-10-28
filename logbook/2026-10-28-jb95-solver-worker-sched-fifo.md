---
title: "JB-95: the segment solver worker asks for SCHED_FIFO 10 on cores 1-5, fail-open to CFS; the box needs a one-time RT budget (limits.d rtprio + user.slice cpu.rt_runtime_us) and a reboot before it takes"
type: feature
date: 2026-10-28
status: in-progress
phase: "JB-95 - solver worker real-time priority"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/solver_worker.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - tests/motion/test_solver_worker.py
  - tests/ros/test_trajectory_node_rt_param.py
  - tools/probes/rt_policy_probe.py
  - tools/probes/profile_session.py
  - tools/probes/README.md
  - tools/systemd/99-jugglebot-rt.conf
  - tools/systemd/jugglebot-rt-budget.service
  - tools/systemd/README.md
subsystem:
  - motion
tags:
  - solver-worker
  - jetson-load
  - sched-fifo
---

# JB-95: the solver worker on SCHED_FIFO

## What

After JB-76 (`2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented.md`) the solver worker still waits on the fair-share run queue for about a fifth of its wall time: CATCH wall/CPU 1.27 p50 / 1.55 p90 at ~4.8 busy cores in the 21:02 Sitting. This change takes the worker process out of that queue. At start-up the worker (`python -m jugglebot.motion.skills.solver_worker --fd 3 --rt-prio 10`) calls `sched_setscheduler(SCHED_FIFO, 10)` on itself and, only if that succeeded, pins itself to CPUs 1–5; if the pin then fails it drops back to SCHED_OTHER rather than run FIFO on any core (the phase audit caught the first draft keeping FIFO there, which could have parked the worker on CPU0). Whatever happened is reported as a fifth element of the READY handshake and lands in `trajectory_node`'s READY INFO line as `sched=FIFO/10 cpus=1-5` or `sched=CFS (<reason>)`; when real-time was requested and CFS resulted, one WARNING names the reason. `trajectory_node` declares `solver_rt_priority` (default 10; 0 turns the request off). Nothing else changes class: the emitter thread, `teensy_bridge_node`, `mocap_node` and the `nice -n 10` prefixes stay as they are.

**Fail-open is the contract.** With no RT budget the worker stays on CFS exactly as today, no affinity change, READY still sent, the reason logged. That is also today's state on this box, and the owner-accepted recipe below is what changes it.

## Discussion

**Why FIFO on the worker and not core isolation.** The worker uses ~15 % of a core in 60–90 ms bursts. Dedicating a core wastes the other 85 % and pushes 4.8 cores of demand onto five; a real-time class costs nothing when idle and makes the burst preempt whatever CFS task holds a core, so wall ≈ CPU at any load. Priority 10 sits under every kernel RT task on this Tegra (IRQ threads and `watchdogd` at FIFO 50, `migration/*` at 99), so the worker can never delay an interrupt thread.

**Why the affinity mask excludes CPU0.** Both NICs' interrupts land on CPU0 and are not threaded: the can-bridge USB link (r8152, IRQ 219) and the mocap LAN (r8169, IRQ 295). Their receive work that overflows into `ksoftirqd/0` runs at CFS, so a FIFO worker parked on CPU0 for a 100 ms solve would stall UDP receive on both links for the whole solve. On {1..5} a FIFO wake-up lands on the lowest-priority CPU in the mask, normally an idle one, and preempts nobody. No single-core pin and no `isolcpus`: the worst case is one CFS task waiting one solve (~100 ms), which the emitter's 250 ms SETPOINT_STALE watchdog and mocap's 150-frame queue both absorb, but which would show as emitter gaps above today's 47 ms maximum, so `max_emit_gap_ms` is the abort gauge for the first Sitting.

**Why the rtprio limit alone does not work here (the Opus design pass's main finding).** This kernel (5.10.104-tegra) is built with `CONFIG_RT_GROUP_SCHED`. Every launch process sits in the cpu cgroup `/user.slice`, whose `cpu.rt_runtime_us` is 0, and the kernel then refuses SCHED_FIFO with EPERM to any caller in it, root included. So the recipe has two parts and a reboot: `jetson - rtprio 40` in `limits.d` (the limit the worker's 10 must fit under) and a root oneshot unit that writes 950000 to the slice's `cpu.rt_runtime_us` at every boot. The launch shell is a tmux pane whose server systemd started at boot via `tmux-sessions.service` (Linger), so its limits come from the user manager and only a reboot (or a `LimitRTPRIO=40` drop-in on `user@1000.service`, the documented fallback) propagates the new limit. `sched_rt_runtime_us` stays at 950000: it is the runaway net that caps a stuck RT task at 95 % of one core, on top of the parent's existing 2 s hang kill. Rejected: `sched_rt_runtime_us=-1` (removes the net), a sudoers `chrt` prefix (fails closed with no READY and lets anything run at FIFO 99), `setcap cap_sys_nice` on the interpreter (every Python program gains it, and glibc then ignores `LD_LIBRARY_PATH`, which rclpy needs). `mlockall` is not needed (zram swap, ms-scale target).

**Why the setter lives in `_main`, not `worker_main`.** `sched_setscheduler(0)` acts on the calling thread, and `tests/motion/test_solver_worker.py` runs `worker_main` in a pytest thread; the flag is also opt-in (`WorkerSegmentSolver(rt_priority=0)` default) so no test gets real-time by accident.

## Verification

- Code, 2026-10-28: `pytest -q tests/motion/test_solver_worker.py tests/motion/test_solver_worker_lateness.py tests/ros/test_install_segment.py tests/ros/test_trajectory_node_rt_param.py` 104 passed in 19.90 s (eight new worker tests: injected success + affinity, EPERM from the rlimit, EPERM from the cgroup, unreadable cgroup, prio 0, affinity failure after FIFO drops back to CFS, the default spawn has no `--rt-prio`, and an environment-agnostic process test that passes before and after the recipe; three node tests: default 10 reaches the factory, 0 reaches it, READY INFO carries `sched=`). The rlimit-EPERM branch is the one probed live on this box; the cgroup branch is injection-only until the recipe is installed.
- Probe, 2026-10-28, before the recipe: `python3 tools/probes/rt_policy_probe.py` prints `RLIMIT_RTPRIO (0, 0)`, `cpu cgroup /user.slice cpu.rt_runtime_us 0`, `RESULT: CFS EPERM: RLIMIT_RTPRIO=0 < 10`, exit 1. After the recipe and reboot it must print `RESULT: FIFO 10 ok`, exit 0.
- Recorder: `profile_session.py`'s ps snapshot now records `cls,rtprio`; the next Sitting's `report.md` shows a `sched` column (`FF/10` expected on the worker). Parser checked on a synthetic three-format block and on the 14:29 and 21:02 logs (old formats, 491 and 544 PIDs resolved).
- After the audit fixes (affinity fail-open, `Wants=user.slice` + `mkdir -p` in the unit, `os.sched_*` defaults resolved at call time so the module imports on the Windows sim box, README and docstring wording), 2026-10-28: the six scoped files `tests/motion/test_solver_worker.py test_solver_worker_lateness.py tests/ros/test_install_segment.py test_trajectory_node_rt_param.py test_launch_priority.py test_launch_nodes.py` 121 + 95 passed (one wall-clock flake while a gate ran concurrently, green alone).
- Gate, 2026-10-28, `./run_tests.sh -q` (`temp/logs/jb95-gate2.log`): **FAIL, 7548 passed / 40 skipped / 1 failed**, the one failure being `tests/sim/test_map_items_rated.py::test_every_item_is_rated[feature-solver-worker-on-sched-fifo-...]`: JB-95 had been moved past `inbox` without the four metric fields the same-day map-metrics change requires. Not a code failure, but the commit `cc80eeb4` went out anyway because the commit chain did not stop on the gate's result, and its message wrongly says "RESULT PASS" next to the real counts. Correction: the item was rated (`python -m mapkit item rate JB-95 --effort m --hardware robot --owner hands --radius cross-cutting`), `pytest -q tests/sim/test_map_items_rated.py tests/sim/test_map_items.py` 72 passed / 36 skipped, and the follow-up `map:` commit carries this record. Every code test in that gate passed.
- **Not yet verified live.** Pass marks for the first Sitting after the reboot: READY INFO `sched=FIFO/10 cpus=1-5`; `ps -o cls,rtprio,psr` on the worker `FF 10` and never CPU 0; CATCH wall/CPU p50 ≈ 1.0–1.1 and ivcsw per solve near zero in `solve_starvation_scan.py`; no `RT throttling` in `dmesg`. Abort on any SETPOINT_STALE, E-stop or NO_HEARTBEAT, `max_emit_gap_ms` above ~50 ms, mocap queue-full during goals, or heartbeat dropouts worse than the 21:02 Sitting. If the emitter gaps grow, the next step is FIFO 20 on the emitter thread itself, not a lower worker priority.

## After the recipe (owner, 2026-10-28 22:04)

The owner installed both files and rebooted. `rt_policy_probe.py` then read `RLIMIT_RTPRIO (40, 40)`, `cpu cgroup /user.slice cpu.rt_runtime_us 950000`, `RESULT: FIFO 10 ok`, and one warning: `could not restore SCHED_OTHER: [Errno 1] Operation not permitted`. Confirmed on the box: a task that set `SCHED_FIFO | SCHED_RESET_ON_FORK` is refused a plain `SCHED_OTHER` (an unprivileged task may not clear the reset-on-fork bit) and accepts `SCHED_OTHER | SCHED_RESET_ON_FORK`. The probe's restore and, more importantly, the worker's affinity-failure fall-back used the plain form, so that fall-back would have silently left the worker FIFO with an unrestricted mask, exactly what the audit fix was meant to prevent; both now keep the bit, and the injected test asserts the flag. Found before the first FIFO Sitting; no launch had run with the recipe.

## Outcome

Code landed fail-open; the box runs CFS until the owner installs `tools/systemd/99-jugglebot-rt.conf` and `tools/systemd/jugglebot-rt-budget.service` (recipe, checks and revert in `tools/systemd/README.md`) and reboots. JB-95 stays in progress until the scanned Sitting above.
