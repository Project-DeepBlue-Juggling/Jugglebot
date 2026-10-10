"""The replay backend must stay STDLIB ONLY.

The systemd unit (tools/systemd/jugglebot-gui.service) runs gui_server.py under
/usr/bin/python3 without the project venv, so a stray numpy/msgpack/mcap/yaml
import in the server side would crash the GUI at boot. Only the overview
worker (replay.overview, replay.decode) may use third-party packages.
"""
from __future__ import annotations

import ast
import os
import subprocess
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
GUI = os.path.join(REPO, "ros_ws", "gui")
FILES = [os.path.join(GUI, "gui_server.py")] + [
    os.path.join(GUI, "replay", n) for n in ("schema.py", "recordings.py", "api.py")]

CODE = """
import sys
sys.path.insert(0, {gui!r})
import gui_server
import replay.schema, replay.recordings, replay.api
bad = [m for m in ("numpy", "msgpack", "mcap", "rosbags", "yaml") if m in sys.modules]
if bad:
    sys.stderr.write("third-party modules loaded: %s\\n" % bad)
    sys.exit(1)
"""

ALLOW = {"argparse", "functools", "gzip", "http", "json", "os", "re", "shutil",
         "subprocess", "sys", "threading", "time", "typing", "urllib", "datetime",
         "glob", "struct", "errno", "io", "pathlib", "collections", "__future__",
         "replay", "gui_server", "socketserver", "signal", "logging", "tempfile"}


def test_import_pulls_no_third_party_modules():
    r = subprocess.run([sys.executable, "-I", "-c", CODE.format(gui=GUI)],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                       universal_newlines=True)
    assert r.returncode == 0, r.stderr


def test_top_level_imports_are_stdlib():
    std = getattr(sys, "stdlib_module_names", None)
    for path in FILES:
        with open(path) as f:
            tree = ast.parse(f.read())
        for node in tree.body:
            mods = []
            if isinstance(node, ast.Import):
                mods = [a.name.split(".")[0] for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                mods = [node.module.split(".")[0]]
            for m in mods:
                ok = (m in std or m in ("replay", "gui_server")) if std is not None else m in ALLOW
                assert ok, "{}: non-stdlib import {}".format(os.path.basename(path), m)
