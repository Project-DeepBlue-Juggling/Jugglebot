---
id: FEAT-31
kind: feature
title: 'Solver worker on SCHED_FIFO: take the CATCH solve out of the fair-share queue (wall ~= CPU at any box load)'
status: backlog
created: '2026-10-28'
updated: '2026-11-27T04:11:50Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: cross-cutting
area: skill-stack
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
parent: two-ball-oval
filed_by: claude (BUG-21 21:02 Sitting read-out)
night_skip: waiting-trigger
night_skip_on: '2026-11-27'
night_skip_note: Off by owner decision; re-opens only if BUG-32's cpu_partition arm shows the Catch solve tail still biting.
---

Owner pick 2026-10-28 evening as the lever after BUG-21.

## Problem and idea

- Today CATCH wall/CPU is 1.27 p50 / 1.55 p90 at ~4.8 busy cores.
- A real-time policy for the worker process (python -m jugglebot.motion.skills.solver_worker, spawned by WorkerSegmentSolver) makes it preempt every CFS task, so wall ~= CPU (CATCH p50 ~65-80 ms) regardless of load, and the tail stops depending on what else runs.

## Risk

Bounded: a solve is <= ~100 ms CPU, so a runaway RT lock-up is not a realistic risk on 6 cores; still, fail-open: if RLIMIT_RTPRIO is 0 (today) the worker logs it and stays CFS.

## Needs

- A one-time sudo edit of the rtprio limit for the jetson user (security/limits.conf) and a re-login of the launch shell; alternative chrt via sudoers.
- Receipt: the worker reports its policy/priority at start and trajectory_node logs it with the blas line.
- Deliberately NOT the emitter thread (teensy_bridge) in the first step.

## Design (2026-10-28)

- Opus, 2026-10-28 evening: the kernel has CONFIG_RT_GROUP_SCHED and user.slice cpu.rt_runtime_us=0, so rtprio alone cannot give FIFO.
- Recipe = limits.d rtprio 40 + root oneshot unit writing 950000 to the user.slice RT budget + reboot (owner accepted).
- Code: worker --rt-prio from a trajectory_node ROS param (default 10, 0 = off), FIFO 10 then affinity cores 1-5 (NIC IRQs on CPU0), fail-open to CFS with the reason in the READY tuple; one WARN when RT requested but CFS resulted.
- Not touched: emitter thread, teensy_bridge, nice prefixes.

## Flown once (2026-10-28)

22:26 Sitting 2026-10-28:

- Wall/CPU 1.01 as predicted, but the FIFO worker stalled the Jetson setpoint stream 60-96 ms per CATCH solve (10 of 10 stalls inside solve windows; GIL-coupled preemption inferred), past the firmware 70 ms STOP cover -> lane refused -> hand MAX_DEVIATION E-stop on 3 of 5 goals.
- Hand, bridge, thermal ruled out on telemetry.
- Owner: default OFF (solver_rt_priority 0; the 21:02 CFS state is the accepted result), emitter-gap (45 ms) and sched_stops WARNs added so a stall fails loudly.
- Back to backlog: pick up only if a future Sitting shows the solve tail biting again; the design then is a CPU partition (worker on cores trajectory_node and teensy_bridge_node never run on), dress-rehearsed on the loaded box before the robot is powered.

## Progress

- 2026-10-28: Design done (Opus, 2026-10-28 evening): kernel has CONFIG_RT_GROUP_SCHED and user.slice cpu.rt_runtime_us=0, so rtprio alone cannot give FIFO (detail in the section above)
- 2026-10-28: Flown once (22:26 Sitting 2026-10-28): wall/CPU 1.01 as predicted, but the FIFO worker stalled the Jetson setpoint stream 60-96 ms per CATCH solve (detail in the section above)
- 2026-11-26: 2026-11-26 check: FIFO was flown once (2026-10-28 22:26) and stalled the stream; off by default by owner decision, to re-open only with cpu_partition. The launch file has no solver_rt_priority argument. tools/systemd/README.md said default 10; corrected to 0.
- 2026-11-27: Sitting 2026-11-27, BUG-32's cpu_partition arm: the wall tail still bites and was worse pinned. Wall p95 202.9 ms pinned against 143.9 unpinned; 14 % of pinned solves waited 80 ms or more with only 73 to 89 ms of CPU, not queued, no IMU upload running, many involuntary switches. One 30-solve sample per arm. The partition keeps nothing else off cpus 4 and 5 (unchecked).
