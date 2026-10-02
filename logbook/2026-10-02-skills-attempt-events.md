---
title: "skill_node announces every Juggle attempt on skills/attempt (start, refusal, end), so the GUI Event Log sees terminal goals too"
type: feature
date: 2026-10-02
status: resolved
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/gui/js/state-minimap.js
  - ros_ws/gui/js/main.js
  - ros_ws/docs/choreography.md
  - tests/ros/test_skill_node.py
subsystem:
  - ros
  - gui
---

# skills/attempt: Juggle attempt events for the GUI

## Summary

The GUI's Event Log used to show a bare `Juggle Start`, and only when the operator used the
GUI's own Start button. Now skill_node publishes a `diagnostic_msgs/DiagnosticStatus` on
`skills/attempt` for every Juggle goal, whatever sent it (the GUI relay, `ros2 action
send_goal`, or a script). There are three events: `juggle_start`, `juggle_refused` and
`juggle_end`. The GUI turns them into Event Log rows and chart markers, for example
`Juggle Start · Columns · 10 throws · reload`, `Juggle Refused · Hop · <reason>` and
`Juggle End · Columns · STOPPED · 7 throws, 5 caught`.

## Design

The first version labelled the event in the GUI at Start, reading skill_node's `n_throws`
parameter for the count. That approach was replaced for two reasons:
- **It never saw terminal goals.** A goal sent from a terminal never passes through the GUI.
- **It reported the wrong count for them.** A terminal goal with `num_cycles: 5` runs 5
  throws, not `n_throws`.

`_juggle_goal` is the one place every goal passes. So `_start_pattern` attaches the RESOLVED
request (the goal's fields, falling back to the node's parameters) to its own result, and
`_juggle_goal` publishes it. The values go on the result, not on `self`, because goal
callbacks run on a reentrant group. The end event is published from `_juggle_execute` and
names the pattern from `goal_handle.request`, so a new goal that arrives between attempts
cannot mislabel it. The GUI's click-time event was removed so a GUI start is not logged twice.

The event names are `ATTEMPT_*` constants in skill_node, and the GUI reads them as literals in
`minimapOnSkillAttempt`. `skills/attempt` joins `GUI_SUBSCRIBED_TOPICS`, as that set's contract
requires. The generated `choreography.md` lists the topic as having no subscribers, like the
other topics only the GUI reads.

## Verification

- 2026-10-02, `pytest tests/ros/test_skill_node.py -q -k TestJuggleAction`: 13 passed. That
  includes 4 new tests covering the start event (with the parameter fallback, and with a goal's
  own `num_cycles` taking precedence), a refusal with its reason, and the end event's
  outcome and counts.
- 2026-10-02, headless Chromium against a fake rosbridge publishing all three events: the GUI
  subscribed as `diagnostic_msgs/msg/DiagnosticStatus` and rendered all three Event Log rows.
- Gate: see the commit message.
- 2026-10-02, live stack (owner): start events appear for both GUI and terminal goals. The
  end events this produced exposed the goal-ends-at-start bug, fixed in
  `2026-10-02-juggle-goal-done-at-start`.
