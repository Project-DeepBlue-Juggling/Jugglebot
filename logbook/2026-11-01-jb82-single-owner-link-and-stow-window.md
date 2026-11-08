---
title: "BUG-26: the can-bridge UDP link is single-owner (no SO_REUSEADDR; a second process gets LinkBusyError), and the bridge node gets an 8 s SIGINT->SIGTERM window so its shutdown Stow can log its outcome"
type: bugfix
date: 2026-11-01
status: complete
phase: "night-shift follow-up"
subsystem: can-bridge
related_entries:
  - 2026-07-12-cone-sync-udp-link-contention.md
files_changed:
  - teensy_link/client.py
  - teensy_link/__init__.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/launch/teensy_bridge_launch.py
  - tests/teensy_link/test_single_owner.py
  - tests/ros/test_bridge_launch_sigterm.py
  - tests/teensy_link/test_client.py
  - tests/teensy_link/conftest.py
  - map/bug-teensy-bridge-node-s-shutdown-stow-can-be-cut-off-by-the.md
  - docs/project-map.md
  - logbook/INDEX.md
---

# BUG-26: single-owner link and the shutdown Stow window

**Symptom (from BUG-26, filed 2026-10-28 by the logbook backfill).** Two halves of one hazard:
the launch escalates SIGINT to SIGTERM after 5 s while the bridge node's shutdown Stow is budgeted
at about 8 s, and `TeensyLinkClient` lets a second process bind the same can-bridge UDP ports.
The Night Shift drafted options into question ticket TKT-19; the owner asked to discuss, then chose
the split recommended below.

## Discussion

**The premise was half wrong, which changed the choice.** TKT-19 said a cut-off Stow "leaves the
Platform wherever SIGTERM found it". Read against the code: the node sends the profiled DEACTIVATE
about 0.2 s after Ctrl+C, and from then on the ODrives run the TRAP_TRAJ descent and the bridge
firmware idles each leg on arrival (`leg_deactivate.cpp`: "The ODrive owns the trajectory"); no
step of it reads the Jetson link. So a SIGTERM at 5 s kills the node's watching and logging, not
the motion. Separately, the "firmware deferred-stow backstop" that the node docstring and the
2026-07-12 entry cite fires on CAN-bus silence (`fault_machine.cpp`, the CAN3 watchdog), NOT on a
lost Jetson link: on `LINK_LOST` the firmware only disables output and the ODrives hold the legs
where they stand. The launch logs: a healthy Stow takes about 1.5 s and most recent shutdowns are
clean (the July "killed every session" finding no longer holds); two on 2026-10-31 were killed,
13:06 because the Stow was REJECTED at once on a latched leg-0 fault and the node still did not exit
for 5 s (a teardown hang, which no timeout fixes; filed as its own bug), and 13:13 because the Stow
was still unconfirmed 4.8 s after it began (normal 1.5 s), so the kill hid whether it finished.

**Timeout: 8 s, not 12.** The window exists so the node can record the Stow's outcome, which is
bounded at `_SHUTDOWN_DEACTIVATE_TIMEOUT_S` (6 s) plus a 0.2 s settle; 8 s covers it with margin.
12 s (TKT-19 option 1) buys nothing more and makes Ctrl+C on a stuck node wait longer. Only the
bridge node gets the window; every other node keeps launch's 5 s.

**Interlock: drop `SO_REUSEADDR`, not a lock file.** TKT-19 option 3 proposed an `fcntl.flock`
keyed by host:port. A lock file only stops programs that use our client; the kernel's own bind rule
stops every program. Probed on the Jetson 2026-11-01 on spare ports (never 5005/5006): two UDP
sockets share a port only when BOTH set `SO_REUSEADDR`; with our socket lacking it, the second bind
is `EADDRINUSE` whether it comes first or second and whatever the other program set, including a
wildcard-versus-loopback pair; and a relaunch re-binds 0.08 ms after the old owner is SIGKILLed
(UDP has no TIME_WAIT, so the option never served a quick relaunch). The option came in with a June
probe tool (4d4010ca) with no recorded reason. No production path shares the ports by design (the
time-of-day server attaches to the node's own link in-process; every bench script already says to
stop the launch first).

**Control-system implications.** None on the 40 Hz path: the change is to socket bind flags at
start-up and to the launch's teardown escalation. A busy port now stops the bridge node at start-up
with a named error instead of starting deaf; the shutdown sequence and its firmware handshake are
unchanged, the node is simply not killed while it still has bounded work to do.

## Fix

- `teensy_link/client.py`: `_bind_exclusive` replaces `_reuse_fixed_port`; on `EADDRINUSE` for a
  fixed port it raises `LinkBusyError(OSError)` naming the role, host and port and the `ss` command
  that names the owner; a failed RPC bind closes the stream socket (never half a link).
  `LinkBusyError` is exported from `teensy_link`.
- Both launch files: `teensy_bridge_node` gets `sigterm_timeout='8'`.

## Verification

- Probe (scratchpad, uncommitted): the kernel table above, run 2026-11-01.
- New tests fail on the old code (checked: 6 of 7; the ephemeral-bind control passes either way):
  `tests/teensy_link/test_single_owner.py` (second client refused and named, relaunch, a foreign
  sharer blocked both ways, no leaked stream socket, ephemeral binds unaffected) and
  `tests/ros/test_bridge_launch_sigterm.py` (both launch files' window at least the node's Stow
  budget plus 1 s, at most 15 s).
- `tests/teensy_link/test_client.py::test_a_fixed_port_keeps_reuseaddr` pinned the old rule (fixed
  ports keep the option); it is inverted to `test_a_fixed_port_never_sets_reuseaddr`. The first gate
  run caught it (my importer grep had covered only the client file).
- Default gate: see the commit message (date, command, result).
- Not verified on the robot: the next Sitting's Ctrl+C should log `deactivate: platform stowed` or
  `shutdown stow complete` and `process has finished cleanly`, not `exit code -15`.
