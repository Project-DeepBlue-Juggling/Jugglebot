---
title: "Every Juggle goal ended COMPLETED (0/0) the moment it started, which also left the GUI Stop with no goal to cancel"
type: bugfix
date: 2026-10-02
status: resolved
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
subsystem:
  - ros
tags:
  - safety
---

# Juggle goal ended at start

## Problem

The owner saw it on 2026-10-02, through the new `skills/attempt` end event: every Juggle goal
reported `COMPLETED` the moment its pattern started, while the attempt went on running.

The code shows a second consequence. When that premature result arrives, orchestrator_node's
`_on_juggle_result` clears `_juggle_goal_handle`. The GUI Stop (`jugglebot/juggle_stop`, which
cancels the goal) then answers "no jugglebot/juggle goal is running" and stops nothing. A
Ctrl-C on a terminal `send_goal` has nothing left to cancel either. So the operator Stop
through the action was dead for any attempt in this state.

## Root Cause

`_juggle_execute` waits on `_goal_done_event`. `_on_tick` sets that event (through
`_maybe_signal_goal_done`) whenever it sees no executor and no reload context. `_start_pattern`
cleared the event at the top of a goal, then ran the frame check, the blocking `_prelevel`
round trip and the compile, and only then installed the executor. The 40 Hz tick runs on its
own thread throughout that window and sees exactly "no executor, no reload", so it fired the
event. The empty `_goal_end_code` then read as COMPLETED.

This is the same class as the reload-announcement window fixed on 2026-09-29
(`test_the_juggle_goal_is_not_done_while_the_announced_reload_compiles`). That fix closed one
window and left the goal-start window open.

## Fix

A `_goal_armed` flag, closing the whole class at the one place the event is set.
`_start_pattern` disarms it and clears the event when a goal begins. It re-arms the goal only
once a successful start has installed its executor or reload context. `_maybe_signal_goal_done`
sets the event only while the goal is armed. The flag, the clear and the set all happen under
`_reload_lock`. Simply moving the `clear()` after the install would leave a tick that had
already read "idle" free to set the event just after the clear.

Motion is unaffected: the tick dispatches exactly as before, and only the action's
bookkeeping changes. The GUI Stop now reaches `_stop_attempt` through the cancel again. That
is the designed rest-terminal stop, but it is probably its first live use since R4. **Check
that Stop ends a running pattern at the next powered sitting.**

## Verification

- 2026-10-02, `pytest tests/ros/test_skill_node.py -q -k start_compile` before the fix: 2
  failed (self_toss and columns), with "the goal was reported done before its attempt was
  installed". The test runs a tick inside the frame check, which both start paths call before
  installing.
- 2026-10-02, after the fix, `pytest tests/ros/test_skill_node.py
  tests/ros/test_skill_node_resend_param.py -q`: 187 passed.
- Gate: see the commit message.
