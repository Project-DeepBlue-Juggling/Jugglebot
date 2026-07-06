---
title: Phase 13 — decommission the Jetson's legacy SocketCAN stack (can_node.py + bus.py deleted; can0 teardown handed to the operator)
type: refactor
date: 2026-07-06
status: fix-landed-pending-hardware-confirm
phase: "teensy-can-offload Phase 13"
related_plan: teensy-can-offload.md
related_entries:
  - 2026-07-06-can-node-parity-reconcile-decommission-precheck
  - 2026-06-27-can-node-teensy-parity-audit
files_changed:
  - ros_ws/src/jugglebot/jugglebot/can_node.py
  - ros_ws/src/jugglebot/jugglebot/can/bus.py
  - ros_ws/src/jugglebot/jugglebot/can/__init__.py
  - ros_ws/src/jugglebot/setup.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/launch/catching_cone_test.launch.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/requirements.txt
  - controller/teensy_link/fault_logic.py
  - tests/ros/test_can_node.py
  - tests/ros/test_bus.py
  - ros_ws/docs/can-node-teensy-parity.md
  - plans/active/teensy-can-offload.md
commits:
  - 7c7f61b
subsystem:
  - can
  - ros
tags:
  - decommission
  - safety
  - migration-parity
---

# Phase 13 — SocketCAN decommission

## Summary

The Jetson's legacy SocketCAN stack is deleted: `can_node.py` (1725 lines),
`can/bus.py` (206 lines), their test files (`test_can_node.py`, 76 tests;
`test_bus.py`, 22 tests), the `can_node` console-script entry point, and
`catching_cone_test.launch.py` (the last launch file that still started
can_node — the bridge owns the cone/BB conduit; operator chose delete over
rewire). Gated on the same-day decommission pre-check
([[2026-07-06-can-node-parity-reconcile-decommission-precheck]]: PARTIAL
safety scan clean, GAP dispositions confirmed) and the operator's explicit
four-part approval (2026-07-06). The `can0`/kernel-module teardown is
operator-run and pending: one command block (disable `init-can.service`,
down the link, unload the modules), with a one-command revert.

## What was deleted vs kept (grep-driven, verified zero live refs after)

**Deleted** — before: 7 executable/import references to `can_node`
(setup.py:46 entry point, catching_cone_test.launch.py:80 Node, the
jugglebot_launch.py "remains for bench use" comment, test_can_node.py:30
import, + docstring identity lines) and 2 `CANBus` references
(can/__init__.py:10 eager import, bus.py:16 itself). After: **0** executable
references; the sole remaining `CANBus` mention is the deliberate historical
note in `can/__init__.py`'s docstring.

**Kept** (the live protocol layer): `can/odrive.py`, `can/ball_butler.py`,
`can/catching_cone.py`, `can/motor_state.py`, `can/throw_ballistics.py` —
imported by `teensy_bridge_node.py:118-120`, `orchestrator_node.py:31`, and
`ball_butler_node.py:57`, and the ground
truth for the firmware byte-parity xref tests. `can/__init__.py` no longer
exports `CANBus` (its eager `from .bus import CANBus` would have broken every
`from jugglebot.can import …` after the delete — the one ripple a naive
delete-the-files pass would have missed).

**Kept in requirements.txt (deviation from the original plan line):**
`python-can==4.2.2` + `cantools==39.0.0`. The plan's "remove python-can"
assumed can_node was its only consumer; the pre-check disproved that —
`odrive.py:14` and `ball_butler.py:13` do `import can` at module scope and
every encoder returns a `can.Message` (in-memory container only; no SocketCAN
I/O remains anywhere on the Jetson). Removing that pin would have broken the
running bridge + orchestrator at import. **Correction (pre-commit audit
catch): cantools is NOT imported by the kept layer** — odrive.py's own
docstring says "no cantools/DBC dependency"; its only importer is
`archived/can_interface.py`. The gate presented "odrive.py imports both",
which was wrong; the operator's keep-both-pinned choice stands (python-can is
genuinely required; cantools is a harmless legacy pin), but cantools is
droppable at any time WITHOUT the dataclass refactor — only python-can needs
that refactor if zero-CAN-deps is ever wanted.

## Ripples handled in the same commit

- `setup.py`: `can_node` console-script removed (else `colcon build`/`ros2
  run` reference a deleted module).
- `jugglebot_launch.py`: the stale "executable remains available for legacy
  bench use" comment now records the deletion; `/platform_target_reached`
  removed from the rosbag record list (its only publisher was can_node —
  parity-matrix row 47 disposition).
- `teensy_bridge_node.py` + `controller/teensy_link/fault_logic.py` top
  docstrings: note that their many `can_node.py:NNN` parity citations refer to
  the last pre-deletion revision in git history (they are provenance, kept
  deliberately).
- Parity matrix: the "Phase 13 decommission is PARTIAL" cross-cutting caveat
  flipped to DONE with the kept/deleted split + the git-history note.
- Plan `teensy-can-offload.md`: Phase-13 status block rewritten (code DONE,
  can0 teardown approved + handed off); the "retained for legacy bench use"
  line at the cutover summary updated.

## The can0 teardown (operator-run, approved 2026-07-06)

Read-only verification first: `can0` is the Jetson's built-in Tegra
controller (`mttcan`), UP but idle — **RX 0 / TX 0 bytes**, `can_raw` module
use-count 0 (no open sockets). NVIDIA's stock L4T ships
`/etc/modprobe.d/denylist-mttcan.conf` (`blacklist mttcan`), so the modules
never autoload — the ONLY loader is the project's own `init-can.service` →
`/home/jetson/System_Scripts/init_can.sh` (modprobes can_raw/can/mttcan +
`ip link set can0 up ... bitrate 1000000`). Nothing else on the Jetson
references can0 (swept /etc + systemd units). Teardown therefore = disable
that one service + unload the modules for this boot; revert = re-enable the
service (one command). The plan's done-when (`lsmod | grep can` empty) is
achievable without any new blacklist file.

## Discussion

- **Why now / why safe.** Phase 13's original "keep as a fallback until
  operating time accumulates" rationale was moot before this session started:
  the legs live on CAN3 behind the can-bridge, so `can_node` + `can0` could
  not drive the robot at all (idle interface, RX 0). The deletion loses
  reference material only — and the reference is preserved in git history +
  the 117-row parity matrix, which was reconciled the same day precisely so
  the mapping outlives the source file.
- **The pre-check earned its keep twice.** It found the two facts that would
  have made a naive execution of the plan's own checklist break the running
  system: the eager `CANBus` import in `can/__init__.py`, and the
  python-can/cantools structural dependency of the KEPT encoder modules. Both
  were surfaced at the gate and the operator chose the safe forms
  (edit-the-init, keep-the-pins).
- **catching_cone_test.launch.py: delete over rewire (operator call).** The
  bridge serves the same `/bb/*` + cone surface, so the trimmed cone-test
  bring-up can be recreated against `teensy_bridge_node` from
  `jugglebot_launch.py` if a cone-only session is ever needed; keeping a
  broken launch (or rewiring it untested) had worse failure modes than
  regenerating it on demand.
- **One atomic commit.** The deletions, the `__init__`/setup/launch ripples,
  and the doc flips land together so a single revert restores the whole
  fallback surface consistently.

## Verification

- Kept-surface import check (venv python, post-delete): `from jugglebot.can
  import MotorStateTracker, odrive, ball_butler, catching_cone` all import
  cleanly; `CANBus` no longer exported.
- Full suite (`pytest tests/ -q`, run 2026-07-06, post-delete): **1956
  passed, 1 xfailed in 471.76 s** — 2054 → 1956 = exactly the 98 deleted
  tests (76 test_can_node + 22 test_bus); no other test was affected. Re-run
  after the audit's doc/docstring fixes (`pytest tests/ -q`, run 2026-07-06,
  pre-commit gate): **1956 passed, 1 xfailed in 475.16 s**. The 1 xfail is
  the pre-existing DOCUMENTED-PERMANENT marker.
- ROS package build (`cd ros_ws && colcon build --packages-select jugglebot`,
  run 2026-07-06): **SUCCESS** — "Summary: 1 package finished". NB an
  incremental build left a stale `can_node` wrapper script in
  `install/jugglebot/lib/jugglebot/` (colcon does not prune removed
  console-scripts), so `build/jugglebot` + `install/jugglebot` were wiped and
  clean-rebuilt (**SUCCESS, [3.19s]**) — the install tree now carries no
  `can_node` executable and only the kept `can/` protocol subpackage.
- can0 teardown: PENDING operator run (commands + PASS/ABORT in the session
  handoff + the plan's Phase-13 block). PASS = `lsmod | grep -E 'can|mttcan'`
  empty + `ip link show can0` reports no device + a normal bridge cold-start
  smoke. This entry stays `fix-landed-pending-hardware-confirm` until then.

## Related

- Gate artifact: [[2026-07-06-can-node-parity-reconcile-decommission-precheck]]
  (the PARTIAL safety scan + GAP dispositions this deletion was conditioned on).
- The living matrix: `ros_ws/docs/can-node-teensy-parity.md` — now the
  authoritative map from the deleted `can_node` surface to its bridge/firmware
  equivalents.
- Plan: `plans/active/teensy-can-offload.md` Phase 13. Remaining plan threads
  after this: Phase 12 (armed dynamic moves — handled separately) and the
  operator can0 run; the plan is then an `/archive-plan` candidate with
  dynamic-moves as the carried-forward thread.
