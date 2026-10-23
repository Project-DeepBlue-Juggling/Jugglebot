---
title: Magazine throw goes hand-only (platform heave withdrawn), the target is nudgeable from the console, and the GUI gets a plain "Reload" button (BB feeds, JB catches and holds)
type: feature
date: 2026-10-13
status: open
phase: "two-ball-skill-stack — R5 (Ball Butler feed tuning)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/magazine_throw.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/magazine_target.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/gui/test_juggle_panel.html
  - tests/motion/test_magazine_throw.py
  - tests/motion/test_magazine_target.py
  - tests/motion/test_skills_schedule.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_magazine.py
  - tests/ros/test_skill_node_reload_only.py
  - tests/ros/test_gui_juggle_defaults.py
subsystem:
  - motion
  - ros
  - gui
tags:
  - feature
---

# Magazine throw: hand-only, console nudge, "Reload" button

Follow-up to `2026-10-13-magazine-throw-program.md` the same day, on three owner asks.

## Change

- **Heave withdrawn (owner: "too risky, asking too much of the legs, for little benefit").** `magazine_throw.py` keeps the platform STATIONARY through the stroke: the legs only tilt and offset it before the stroke (so the release lands on the site at the aim attitude) and bring it back to level rest after. The hand supplies the whole take-off speed along the tilted axis, up to its cap (5.22 m/s at 3225/3900 rev/s², 9.78 rev of stroke). An aim past 12° refuses `MAGAZINE_TILT` naming the flight time that brings it inside; a speed past the cap refuses `MAGAZINE_UNREACHABLE` naming the speed needed, the cap and the least-speed flight for that target (`least_speed_flight`). Gone with the heave: the platform speed budget, the lean-out past the tilt ceiling, and the `repo_minus_rigid_*` diagnostics (with the platform still, the repo cup map agrees with the rigid release to its own ~2 % lateral model gap at 12°, pinned by a test).
- **Reach, hand-only** (probe 2026-10-13, `scratchpad/proto/envelope.py`): max apex +1.39 m above the 860 mm release straight up; at the 12° ceiling a 45°-descending landing reaches +1.27 m up and 0.69 m out. A ladder toward the magazine passes the gate at 350/5000/200k up to 60 % of the way (plan (−754, −372, 1792), 5.16 m/s, 11.9°). **The real magazine (world ≈ (−1256, −620, 2413)) is out of reach without heave**: 14.2° at 0.9 s, and ≥ 6.67 m/s at any flight inside 12°. It has to come down and closer, or the loop runs at a nearer stand-in.
- **Console nudge.** skill_node parameters `magazine_target_base_mm` (elbow frame, mm) and `magazine_flight_s` override the yaml per request; all-zero / 0.0 (the defaults) mean "the file's value" (`magazine_target.with_overrides`, validated like the file). The accept line names the source of each. The yaml stays the persistent record.
- **"Reload" button.** New Juggle pattern `reload` (`_run_reload_only`): the self-throw `reload` choreography with `n_throws=0` — `compile_reload` now accepts 0 and ends at the DECAY REST (level at P1, hand home, the fed ball seated); `compile_one_ball` still refuses 0. No admissible-box/apex check (nothing is thrown). The GUI's hold-to-confirm "Reload" button sends the bare `reload` token with the magazine button's gate. The goal ends `COMPLETED` (0 throws) once the decay REST retires, `MISSED_CATCH` otherwise. "Home" here is the stack's level rest at P1, the state `magazine` plans from — not `trajectory/go_home`'s neutral pose.

## Verification

- Scoped (`python -m pytest -q -n 4 --dist loadfile tests/ros/test_skill_node.py tests/ros/test_skill_node_magazine.py tests/ros/test_skill_node_reload_only.py tests/motion/test_magazine_throw.py tests/motion/test_magazine_target.py tests/motion/test_skills_schedule.py tests/ros/test_gui_juggle_defaults.py tests/ros/test_install_segment.py tests/motion/test_skills_executor.py tests/ros/test_skills_plan_bench.py`, run 2026-10-13): **822 passed**.
- Full gate: see the commit message.
- Not verified on hardware: everything above. The `reload`-only path differs from the flown self-throw `reload` only in having no tail; the hand-only throw has not flown at any speed.

## How to use

```
ros2 service call /jugglebot/juggle_request jugglebot_interfaces/srv/SetString "{data: 'reload'}"
ros2 service call /jugglebot/juggle_request jugglebot_interfaces/srv/SetString "{data: 'magazine'}"
ros2 param set /skill_node magazine_target_base_mm "[560.0, 300.0, 750.0]"
ros2 param set /skill_node magazine_flight_s 0.8
```

Deploy: `colcon build --packages-select jugglebot` (no interface change this time); the GUI picks up the new button on a page refresh after `jugglebot-gui.service` is restarted.
