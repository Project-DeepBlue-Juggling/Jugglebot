---
title: "Launch files injected the MAIN checkout onto teensy_bridge_node's PYTHONPATH by default — a worktree launch ran protocol 6 against the freshly flashed FW 21 board and looked like a dead bridge; both launch files now resolve the repo that produced the install"
type: bugfix
date: 2026-09-11
status: resolved
phase: "two-ball-skill-stack — R1 sitting (first launch after the FW 21 / Platform 7 flash)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/repo_root.py (new)
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/launch/teensy_bridge_launch.py
  - tests/hardware/session_skill_stack_r1_flash.md
subsystem:
  - ros
  - teensy_link
tags:
  - bugfix
  - worktree
---

# Launch files hardcoded the main checkout as the bridge's live tree

**Symptom (owner, 2026-09-11 evening).** After flashing bridge FW 21 and Platform FW 7 and
passing ladder rows with the bench driver (launch down), the ROS launch from the
`skill-stack` worktree showed the bridge "not reporting anything", even after a clean
`colcon build` of the worktree's `ros_ws`.

**Cause.** `jugglebot_launch.py:444` and `teensy_bridge_launch.py:56` prepend a repo root to
`teensy_bridge_node`'s `PYTHONPATH` so the bridge runs the live `teensy_link` and
`config/generated/udp_protocol.py` (deliberately not installed into the ROS package). Both
defaulted to the literal `/home/jetson/Desktop/Jugglebot`. That checkout is on
`mvp-trajectory-bringup` at PROTOCOL_VERSION 6, so the worktree's own install ran the OTHER
tree's transport against a protocol-7 board: the designed link darkness, indistinguishable
from a dead board. Proven by emulating the injection: with the main root prepended,
`udp_protocol` resolves to the main tree at version 6; with the worktree, to version 7. The
bench driver was unaffected because it fixes its own `sys.path` (the same bug class, found
in `hand_stream_bench.py` the same morning). `jugglebot_launch.py` already carried a
worktree-aware `_repo_root()` walk-up, used only for the config-drift diagnostic.

**Fix.** New `jugglebot/repo_root.py::resolve_repo_root(anchor)`: `JUGGLEBOT_REPO` override
→ walk up from the installed launch script until `config/generate_config.py` → the canonical
path. Both launch files use it; `jugglebot_launch._repo_root()` delegates to it. A launch
built from any worktree now runs that worktree's transport.

**What "not reporting" was after the override.** With `JUGGLEBOT_REPO` set the link came UP
(`bridge_fw_version 21 (proto 7)`, `decode_errors 0`, `BRIDGE_FW_CHECK OK`, `PLATFORM_FW_CHECK
OK v7`, ODrive firmware checks passed), but every axis reported `INITIALIZING,
DC_BUS_UNDER_VOLTAGE` and the guard latched `ODRIVE_FATAL`; `/robot_state` is gated on real
telemetry (`if telem is None: return`), so it never published and the GUI stayed blank.
Uniform undervoltage on all seven axes is the unpowered-motor-bus signature
(`project_hardware_bench_facts`). Note for future readers: the bridge node's INFO lines do
NOT reach `~/.ros/log/*/launch.log` (one line there against twenty on screen), so an empty
launch log is not evidence of a silent node. Also: Foxy's `ros2 topic echo` has no `--once`;
the runbook now uses `timeout 5 ros2 topic echo /link_status | head`.

## Verification

- 2026-09-11 — `pytest tests/ros/test_launch_nodes.py tests/ros/test_teensy_bridge_node_install_skew.py tests/ros/test_choreography_map.py tests/ros/test_teensy_bridge_node_shutdown_stow.py -q` → **72 passed in 12.72 s**.
- 2026-09-11 — `colcon build --packages-select jugglebot` in the worktree, then
  `ros2 launch jugglebot teensy_bridge_launch.py` with NO override → `INSTALL_SKEW: OK` (running the worktree's install), `PLATFORM_FW_CHECK: OK v7`, `BRIDGE_FW_CHECK: OK — v21 (expected v21)`, `config identity` under `Jugglebot-skills` (`temp/logs/r1_bridge_probe_nooverride.log`). The same launch before the fix imported `udp_protocol` at PROTOCOL_VERSION 6 from `~/Desktop/Jugglebot` (emulated injection, same day).
