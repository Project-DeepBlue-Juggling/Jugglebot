"""conftest.py — Path setup for MPC / MuJoCo simulation tests.

Sim code is imported as ``sim.*`` off the repo root, which the parent
tests/conftest.py already installs (together with the ros_ws and
config/generated roots).  ``sim/`` itself is deliberately NOT added here:
with both roots live, a bare-style import (``from plant.x import ...``)
would load the same file a second time under a second module identity, and
a ``patch()`` against one identity then silently patches nothing.  Keeping
it off the path makes any such import fail loudly at collection instead —
a second enforcement point behind tests/sim/test_sim_import_style.py.
"""

import os
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))

# Allow `from helpers import ...` in test files within this directory
if _TESTS_DIR not in sys.path:
    sys.path.insert(0, _TESTS_DIR)
