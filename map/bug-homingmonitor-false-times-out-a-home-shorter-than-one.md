---
id: JB-72
kind: bug
title: HomingMonitor false-times-out a home shorter than one Diagnostic period
status: in_progress
created: '2026-10-27'
updated: '2026-10-28T00:43:58Z'
owners: []
priority: medium
blocked_by: []
links:
- teensy_link/homing.py
filed_by: claude
---

HomingMonitor (teensy_link/homing.py) only trusts a terminal HomingResult after it has seen HOMING_RUNNING in a Diagnostic, but the Diagnostic arrives at about 1 Hz per axis. A home shorter than one Diagnostic period (a leg already near its hardstop) goes NONE -> OK between two samples, so the observer reports "timed out (saw_running=False)" while the firmware reports HOMING_OK and the reference IS set.

Seen on the bench leg (axis 0, bridge FW 30) on 2026-10-27 during lead-clamp P2: a 1.25 rev move to the stop at 1.5 rev/s took about 0.8 s, and the result was succeeded=[] with failed={0: 'timed out after 35.0s (saw_running=False)'}, final pos -0.1005 rev and Diagnostic homing_result=2. Log: temp/logs/lead-clamp-p2-home-2.log. The bench driver tests/hardware/teensy_home_bench.py also looks frozen for 35 s when piped through tee, because stdout is block-buffered (run it with python -u).

teensy_bridge_node._run_home uses the same monitor (20 s backstop), so production /home would false-fail a leg that starts near its stop.

Possible fix: let the monitor accept an OK it sees after the HOME RPC was accepted, checked against an RPC-time epoch (for example, a per-home counter in the Diagnostic) rather than requiring RUNNING to have been observed.

## Progress

- 2026-10-28: 2026-10-28: FW 34 per-home epoch + monitor epoch gate built and tested; the 10-27 bench false-timeout was teensy_home_bench.py dropping homing_result (fixed); NOT flashed, bench homes outstanding
