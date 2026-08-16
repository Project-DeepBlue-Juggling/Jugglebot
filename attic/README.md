# attic/

Retired code kept out of the live tree. Nothing under `attic/` is imported,
installed, tested, or collected — if a file here turns out to be referenced
from live code, that is a bug (move it back out deliberately).

Why not just delete? Git history retains everything, but these trees are
referenced by path from logbook entries and carry docstrings/comments that
investigations still consult. Parking them here keeps `git log --follow`
and those references cheap while getting ~16k dead lines out of every grep,
IDE index, and session context load of the live packages.

Contents:

- `ros-jugglebot-archived/` — formerly `ros_ws/src/jugglebot/jugglebot/archived/`
  (moved 2026-07-31; see `plans/parked/refactor-2026-07.md`). The pre-can-bridge
  era: `can_interface.py`, the state-manager/FSM stack, superseded BB volley
  nodes. Never part of the colcon install (`setup.py` packages list excluded it).

Policy: prefer deleting truly reference-free files outright (git retains
them); use the attic only when live documents reference the paths or the
code has archaeological value to active subsystems.
