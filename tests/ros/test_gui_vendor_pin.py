"""Pin the vendored MCAP bundle: sha256 matches lib/VENDORED.md, package.json versions
are exact, and a node import exposes the named exports. Offline: never runs npm."""
from __future__ import annotations

import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
BUNDLE = ROOT / "ros_ws" / "gui" / "lib" / "mcap-bundle.min.js"
VENDORED = ROOT / "ros_ws" / "gui" / "lib" / "VENDORED.md"
PKG = ROOT / "tools" / "gui_vendor" / "package.json"

PINS = {
    "@mcap/core": "2.3.0",
    "@foxglove/rosmsg": "5.0.5",
    "@foxglove/rosmsg2-serialization": "3.1.2",
    "esbuild": "0.28.2",
}


def _row():
    for line in VENDORED.read_text().splitlines():
        if line.startswith("| `mcap-bundle.min.js`"):
            cells = [c.strip() for c in line.strip("|").split("|")]
            return int(cells[3]), cells[4].strip("`")
    raise AssertionError("no mcap-bundle.min.js row in VENDORED.md")


def test_bundle_sha256_and_size_match_vendored_md():
    size, sha = _row()
    data = BUNDLE.read_bytes()
    assert len(data) == size
    assert hashlib.sha256(data).hexdigest() == sha


def test_package_json_versions_are_exact_pins():
    pkg = json.loads(PKG.read_text())
    deps = {**pkg["dependencies"], **pkg["devDependencies"]}
    assert deps == PINS
    assert all(re.fullmatch(r"\d+\.\d+\.\d+", v) for v in deps.values())


def test_lockfile_resolves_the_pins():
    lock = json.loads((PKG.parent / "package-lock.json").read_text())["packages"]
    for name, ver in PINS.items():
        assert lock["node_modules/" + name]["version"] == ver


@pytest.mark.skipif(shutil.which("node") is None, reason="node not installed")
def test_node_import_exposes_named_exports(tmp_path):
    (tmp_path / "package.json").write_text('{"type":"module"}')
    shutil.copy(BUNDLE, tmp_path / "b.js")
    js = ("import * as m from './b.js';"
          "console.log(JSON.stringify(Object.keys(m).sort()),"
          "typeof m.McapIndexedReader, typeof m.parse, typeof m.MessageReader)")
    out = subprocess.run(["node", "--input-type=module", "-e", js], cwd=tmp_path,
                         capture_output=True, text=True, check=True).stdout.split()
    assert out == ['["McapIndexedReader","MessageReader","parse"]', "function", "function", "function"]
