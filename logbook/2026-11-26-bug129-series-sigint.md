---
title: "BUG-129 launch-series runner: its SIGINT never reached the launch (background jobs ignore SIGINT and the ignore survives exec), now one SIGINT to the launch's own process group with bounded TERM/KILL escalation and the ending signal recorded"
type: bugfix
date: 2026-11-26
status: resolved
phase: ""
related_plan: ""
related_issues: []
files_changed:
  - tools/probes/bug129_stop_group.sh
  - tools/probes/bug129_launch_series.sh
  - tests/sim/test_bug129_series_report.py
  - logbook/2026-11-26-bug129-series-sigint.md
subsystem: [tooling]
tags: [testing]
---

# BUG-129 launch-series runner: the shutdown SIGINT was a no-op

## Summary

Every one of the 24 launches of the 2026-11-25 series ended `bridge_exit=bridge SIGKILLed after 30 s; launch group SIGKILLed`
(`temp/logs/bug129_series_2026-11-25_*.meta`), and no launch log carries a SIGINT, shutdown or KeyboardInterrupt line. Cause: the runner
started the launch as `setsid bash -c ... &` in a non-interactive shell. There a background job has SIGINT set to ignored, and an
ignored signal is inherited across exec, so `ros2 launch` (Python leaves an inherited SIG_IGN alone) never installed its
KeyboardInterrupt handler. A first probe (`setsid bash -c 'exec python3 -c "print(signal.getsignal(SIGINT))"' &`) prints `SIG_IGN`.
The nodes therefore never shut down cleanly: the bridge was SIGKILLed with its shm segments left, which the parser's clean-shutdown
evidence could not see either.

Two defects, found in this order on real launches (the first fix alone was not enough). (1) `set -m` gives the launch its own process
group, but a runner that itself starts with SIGINT ignored (background job, nohup) cannot un-ignore it in bash, and the launch still showed
`SigIgn 0x1001006` (SIGINT ignored): after the first version only rosbag (C++) reacted, the Python nodes ran on and SIGTERM ended the launch
(`stop_signal=TERM`, A_01 and B_02 of the first run). (2) With SIGINT working, a SIGINT to the whole GROUP reached each node directly AND again
through the launch's own forwarding: KeyboardInterrupt tracebacks in trajectory_node, skill_node, ball_butler_node, spacemouse and the bridge
(its `on_shutdown` disarm-settle `time.sleep` was interrupted, so the profiled-stow step did not run). A terminal Ctrl-C is that group form, so
an operator Ctrl-C of a real launch likely does the same (unchecked here; see the report).

Fix: `tools/probes/bug129_stop_group.sh` (sourced by the runner and by the test). `group_start` runs the launch under `set -m` (own process
group, pgid = pid) through a python exec wrapper that resets SIGINT/SIGQUIT to default; `group_stop` sends ONE SIGINT to the LEADER (the
launch, which forwards one to each node), waits 30 s for the group to empty, then SIGTERM of the group, 10 s, then SIGKILL, and sets
`GROUP_STOP_SIGNAL`. Never `timeout -s INT` either (child and group). The runner writes `stop_signal=` to the launch's `.meta` and a `bug129_runner: launch ended by SIG...` line to its `.log`.
The abort paths (forbidden-line match, runner INT/TERM) use the same function. Series timing, guards and the parser are unchanged.

## Verification

- 2026-11-26, `pytest tests/sim/test_bug129_series_report.py -q`: 14 passed (readers of the script: grep -rln bug129_launch_series tools tests logbook
  map; only this file and the parser). The two new tests (`..._exactly_one_sigint_via_the_leader`, `..._escalates_when_sigint_is_ignored`) run a
  leader that forwards SIGINT like ros2 launch, a python child and a bash child, from a runner that starts with SIGINT ignored. Each fails without
  its piece: no SIGINT reset (hangs, tree never stopped), group-wide SIGINT (children record two).
- 2026-11-26, `tools/probes/bug129_launch_series.sh --stop-at 12:50 --launches-per-arm 1 --first-slot 1 --date 2026-11-26fin` (one real unpowered
  launch, arm B, auto_arm:=false, live_session clear before): `bridge_exit=clean`, `stop_signal=INT`, every node "exited cleanly" within 6 s
  (rosbag's exit code 2 is its normal SIGINT warning), zero Tracebacks after Ctrl-C, bridge ran its shutdown stow, no ros2/jugglebot process left,
  live_session clear. Logs: temp/logs/bug129_series_2026-11-26fin_B_02.*.
