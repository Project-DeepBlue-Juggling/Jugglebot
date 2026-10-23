"""mapkit is pure Python: no module imports rclpy or any ROS package."""
from __future__ import annotations

import os
import re
import subprocess
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
MAPKIT = os.path.join(REPO_ROOT, "mapkit")
FORBIDDEN = re.compile(r"\b(rclpy|std_msgs|jugglebot_interfaces|ros2)\b")


def test_no_ros_names_in_source():
    """No mapkit/*.py source mentions rclpy, std_msgs, jugglebot_interfaces or ros2."""
    bad = []
    for name in sorted(os.listdir(MAPKIT)):
        if name.endswith(".py"):
            with open(os.path.join(MAPKIT, name), encoding="utf-8") as fh:
                for i, line in enumerate(fh, 1):
                    if FORBIDDEN.search(line):
                        bad.append("%s:%d: %s" % (name, i, line.strip()))
    assert not bad, "\n".join(bad)


def test_import_graph_has_no_ros():
    """Importing mapkit in a fresh interpreter loads no rclpy/ROS module."""
    code = ("import sys, mapkit; bad = [k for k in sys.modules if k.split('.')[0] in "
            "('rclpy', 'std_msgs', 'jugglebot_interfaces', 'rosidl_runtime_py')]; "
            "print(repr(bad)); sys.exit(1 if bad else 0)")
    res = subprocess.run([sys.executable, "-c", code], cwd=REPO_ROOT,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         universal_newlines=True)
    assert res.returncode == 0, res.stdout + res.stderr
