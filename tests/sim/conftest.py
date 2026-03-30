"""conftest.py — Path setup for MPC / MuJoCo simulation tests.

Adds sim/ to sys.path so tests can import from plant.*, hand.*, ball.*,
input.*, viz.*, etc.  The parent tests/conftest.py already adds the repo
root (for controller.*) and ros_ws paths (for jugglebot.*).
"""

import os
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_TESTS_DIR))
_SIM_DIR = os.path.join(_REPO_ROOT, 'sim')

if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)

# Allow `from helpers import ...` in test files within this directory
if _TESTS_DIR not in sys.path:
    sys.path.insert(0, _TESTS_DIR)
