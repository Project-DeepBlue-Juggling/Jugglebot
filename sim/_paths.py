"""Single sys.path bootstrap for runnable entry scripts (sim/ and its peers).

**Normative rule (Phase 6, refactor-2026-07):** there is exactly ONE import
root for sim code — the repo root — and exactly ONE canonical import style —
``sim.*``.  Before 2026-08-01 the tree mixed bare-style imports
(``from plant.mujoco_plant import ...``) with ``sim.*`` imports, and because
``sim/`` was also on ``sys.path`` the same file could be loaded under two
module identities in one interpreter.  Two identities means a ``patch()``
against one of them silently patches nothing (the "test patches nothing"
class, verified live at ``sim/reload_gate.py`` vs ``tests/sim/test_reload_gate.py``).

Division of responsibility:

* **Runnable entry scripts** (``sim/main.py``, ``sim/juggle_*.py``, the gates,
  ``sim/analysis/*`` CLIs, ``tools/probes/juggle_*.py``)
  call :func:`bootstrap_paths` exactly once, before importing project code.
* **Library modules** under ``sim/`` mutate ``sys.path`` never.  They import
  ``sim.*`` / ``controller.*`` / ``jugglebot.*`` and rely on their caller
  (an entry script, or ``tests/conftest.py``) having bootstrapped.

**Dual-role modules count as entry scripts.**  ``sim/analysis/*``,
``sim/viz/reference_plot.py`` and ``sim/tools/verify_motor_commands.py`` are
both importable libraries and ``python -m``-able CLIs, so they call
``bootstrap_paths()`` at module scope — a library import therefore mutates
``sys.path``.  That is deliberate and harmless (the call is idempotent and
installs exactly the roots ``tests/conftest.py`` already installed), but it
means "library modules never touch ``sys.path``" is a rule about *hand-rolled*
root derivation, not about the blessed call.  ``tests/sim/test_sim_import_style
.py`` encodes it that way: one mutation plus a ``bootstrap_paths()`` call is an
entry script; anything else is an offender.

The roots below are the same set ``tests/conftest.py`` installs, so a module
imports identically under pytest and under a direct script run.

**On the retired fourth root.**  The pre-2026-08-01 blocks injected ``sim/``
as well, purely so bare-style imports would resolve.  It is deliberately NOT
installed here: every capability it provided is now served by the repo root
(``sim.*``), it has zero in-repo consumers (grep-to-zero, pinned by
``tests/sim/test_sim_import_style.py``), and leaving it in would keep the
dual-identity mechanism alive in every bootstrapped process — including
pytest, where it is exactly the "test patches nothing" bug.  Direct script
runs still see ``sim/`` on ``sys.path`` because CPython always prepends the
*script's own directory*; that is unavoidable, which is why the durable
guarantee is the import-style contract, not path hygiene alone.
"""

from __future__ import annotations

import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: The import roots, in the order they are searched once installed.
PATH_ROOTS = (
    REPO_ROOT,                                              # sim.*, controller.*
    os.path.join(REPO_ROOT, 'ros_ws', 'src', 'jugglebot'),  # jugglebot.motion.* (pure Python)
    os.path.join(REPO_ROOT, 'config', 'generated'),         # hardware_config, protocol_config
)

#: The retired bare-style root.  Named so the omission reads as deliberate;
#: never add it to PATH_ROOTS (see the module docstring).
LEGACY_SIM_ROOT = os.path.join(REPO_ROOT, 'sim')


def bootstrap_paths() -> str:
    """Install the project import roots on ``sys.path``.  Idempotent.

    Returns the repo root, so callers that need it for file paths can write
    ``_REPO_ROOT = bootstrap_paths()`` instead of recomputing it.
    """
    for _root in reversed(PATH_ROOTS):
        if _root not in sys.path:
            sys.path.insert(0, _root)
    return REPO_ROOT
