---
id: JB-95
kind: feature
title: 'Solver worker on SCHED_FIFO: take the CATCH solve out of the fair-share queue (wall ~= CPU at any box load)'
status: in_progress
created: '2026-10-28'
updated: '2026-10-28T10:34:05Z'
owners: []
priority: high
area: skill-stack
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
parent: two-ball-oval
filed_by: claude (JB-76 21:02 Sitting read-out)
---

Owner pick 2026-10-28 evening as the lever after JB-76. Today CATCH wall/CPU is 1.27 p50 / 1.55 p90 at ~4.8 busy cores; a real-time policy for the worker process (python -m jugglebot.motion.skills.solver_worker, spawned by WorkerSegmentSolver) makes it preempt every CFS task, so wall ~= CPU (CATCH p50 ~65-80 ms) regardless of load, and the tail stops depending on what else runs. Bounded: a solve is <= ~100 ms CPU, so a runaway RT lock-up is not a realistic risk on 6 cores; still, fail-open: if RLIMIT_RTPRIO is 0 (today) the worker logs it and stays CFS. Needs a one-time sudo edit of the rtprio limit for the jetson user (security/limits.conf) and a re-login of the launch shell; alternative chrt via sudoers. Receipt: the worker reports its policy/priority at start and trajectory_node logs it with the blas line. Deliberately NOT the emitter thread (teensy_bridge) in the first step.

## Progress

- 2026-10-28: Design done (Opus, 2026-10-28 evening): kernel has CONFIG_RT_GROUP_SCHED and user.slice cpu.rt_runtime_us=0, so rtprio alone cannot give FIFO; recipe = limits.d rtprio 40 + root oneshot unit writing 950000 to the user.slice RT budget + reboot (owner accepted). Code: worker --rt-prio from a trajectory_node ROS param (default 10, 0 = off), FIFO 10 then affinity cores 1-5 (NIC IRQs on CPU0), fail-open to CFS with the reason in the READY tuple; one WARN when RT requested but CFS resulted. Not touched: emitter thread, teensy_bridge, nice prefixes.
