---
title: First nightly went RED (297 ModuleNotFoundError) — cron's bare env lacks the ROS PYTHONPATH every interactive run rides; runner now sources the overlays
type: bugfix
date: 2026-08-02
status: resolved
phase: "refactor-2026-07 Phase 2 follow-up — nightly runner"
files_changed:
  - tools/nightly_suite.sh
subsystem:
  - repo
---

# Nightly env parity

**Symptom**: the first live nightly (2026-08-02 04:03) wrote `RED
3822/4126 passed, 215 failed, 86 errored`. Triage from the junit: 211x
`ModuleNotFoundError: diagnostic_msgs` + 28x `ament_index_python`,
concentrated in the bridge/BB node test families; the same files pass
interactively (spot check 2026-08-02: 30/30 in 5.92 s).

**Root cause**: tests/ros mocks rclpy but imports the pure-Python ROS
packages for real — off the PYTHONPATH interactive shells inherit from
sourcing ROS + the workspace (`/opt/ros/foxy/...`, `ros_ws/install/...`).
Cron's bare env has no such PYTHONPATH, so every citation-worthy green
run to date silently depended on shell state the runner never had.
Verified both directions 2026-08-02: interactive venv imports
diagnostic_msgs from /opt/ros/foxy; `env -i` venv gets
ModuleNotFoundError; `env -i` + the fix's sourcing recipe: **30 passed
in 6.27 s** on the worst failing file.

**Fix**: `tools/nightly_suite.sh` sources `/opt/ros/foxy/setup.bash` +
`ros_ws/install/setup.bash` (nounset relaxed around them) BEFORE venv
activation, giving the nightly byte-for-byte the env every interactive
gate runs under. Manual re-run kicked after landing to flip `status`
with evidence.

**Latent follow-up (named, not done)**: the real hardening is
environment-independence — conftest fallback mocks for the pure-Python
ROS packages when absent. Deferred because `get_package_share_directory`
mocking changes resource-path semantics for the BB node tests and needs
care, not because it is wrong.
