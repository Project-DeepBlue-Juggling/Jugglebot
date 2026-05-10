"""conftest.py — Shared path setup for all test subdirectories.

Adds project paths to sys.path so that test modules can import from:
  - ``jugglebot.*``  (ROS2 package: motion, can, tracking, nodes)
  - ``protocol_config`` / ``hardware_config``  (generated config modules)
  - ``controller.*``  (MPC solver package)

Also registers hypothesis profiles (ci-fast / ci-deep / dev) by
importing ``conftest_hypothesis`` for its module-level side effects.

Subdirectory-specific setup (ROS2 mocking, sim paths) lives in each
subdirectory's own conftest.py.
"""

import os
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_TESTS_DIR)

# jugglebot package (ros_ws/src/jugglebot is the package root)
_ROS_PKG_DIR = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot')
if _ROS_PKG_DIR not in sys.path:
    sys.path.insert(0, _ROS_PKG_DIR)

# Generated config modules (protocol_config, hardware_config)
_CONFIG_DIR = os.path.join(_REPO_ROOT, 'config', 'generated')
if _CONFIG_DIR not in sys.path:
    sys.path.insert(0, _CONFIG_DIR)

# Repo root (for controller.* imports)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

# tests/ on sys.path so sibling helper modules (conftest_hypothesis) import.
if _TESTS_DIR not in sys.path:
    sys.path.insert(0, _TESTS_DIR)

# Register hypothesis profiles (ci-fast default; ci-deep for nightly).
import conftest_hypothesis  # noqa: F401, E402
