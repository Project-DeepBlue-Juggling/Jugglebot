# -*- coding: utf-8 -*-
"""Typed columnar slot records (memory block M2a): layout round trip, worker transfer, adoption.

The decode core builds per-leaf typed columns (``js/replay/mcap-decode.js`` ``buildColumns``), the worker
TRANSFERS every typed buffer (``mcap-worker.js``), and ``chunk.js`` adopts them as is and rebuilds the exact
row objects on demand. Value equality with the Python oracle is ``test_replay_mcap_oracle.py``; this file pins
the layout rules, hydration of scalar/bool/string/non-finite/array/object-array columns, that nothing is
memoised per row, and that the worker's transfer list covers every typed buffer (a buffer that is not listed
is silently structured-cloned, which is the 600 ms main-thread stall this layout removes).
"""
from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from tests.ros._replay_fixture import write_bag

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
JS = REPO / "tests" / "ros" / "js"


def _find_node():
    found = shutil.which("node") or shutil.which("nodejs")
    if found:
        return found
    for pat in (os.path.expanduser("~/.nvm/versions/node/*/bin/node"), "/usr/local/bin/node", "/usr/bin/node"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[-1]
    return None


NODE = _find_node()
pytestmark = pytest.mark.skipif(NODE is None, reason="node not installed")


@pytest.fixture(scope="module")
def sandbox(tmp_path_factory):
    sb = tmp_path_factory.mktemp("typed")
    (sb / "js" / "replay").mkdir(parents=True)
    (sb / "lib").mkdir()
    for name in ("mcap-decode.js", "mcap-worker.js", "allowlist.js", "chunk.js"):
        shutil.copy(GUI / "js" / "replay" / name, sb / "js" / "replay" / name)
    shutil.copy(GUI / "lib" / "mcap-bundle.min.js", sb / "lib" / "mcap-bundle.min.js")
    shutil.copy(JS / "replay_typed_columns_harness.js", sb / "layout.js")
    shutil.copy(JS / "mcap_worker_transfer_harness.js", sb / "transfer.js")
    (sb / "package.json").write_text('{"type": "module"}')
    return sb


def _node(args, cwd):
    p = subprocess.run([NODE, *map(str, args)], capture_output=True, text=True, timeout=120, cwd=cwd)
    assert p.returncode == 0, p.stderr
    return json.loads(p.stdout.strip().splitlines()[-1])


@pytest.fixture(scope="module")
def layout(sandbox):
    return _node([sandbox / "layout.js"], sandbox)


def test_kinds_classification(layout):
    k = layout["kinds"]
    assert k["a"] == "f64" and k["nan"] == "f64" and k["b"] == "u8" and k["s"] == "str"
    assert k["arr"] == {"csr": "f64"} and k["var"] == {"csr": "f64"}
    assert k["objs"] == {"csr": {"leaves": {"id": "f64", "p.x": "f64", "p.y": "f64", "tag": "str", "inner": "any"}}}
    t = layout["types"]
    assert t["a"] == "Float64Array" and t["b"] == "Uint8Array" and t["s"] == "Array"
    assert t["arr"] == ["offsets", "flat"] and t["objs"] == ["offsets", "leaves"]


def test_hydration_round_trips_by_value(layout):
    h = layout["hydrated"]
    # non-finite -> null (rosbridge rule), also inside arrays and nested leaves
    assert h[0] == {"a": 1.5, "b": True, "s": "x", "nan": None, "arr": [1, 2, 3], "var": [],
                    "objs": [{"id": 1, "p": {"x": 1, "y": 2}, "tag": "u", "inner": [1]},
                             {"id": 2, "p": {"x": None, "y": 4}, "tag": "v", "inner": []}]}
    assert h[1] == {"a": -2, "b": False, "s": "y", "nan": 7, "arr": [4, 5, 6], "var": [9], "objs": []}
    assert h[2]["a"] is None and h[2]["nan"] is None and h[2]["arr"] == [7, None, 9]
    assert h[2]["objs"] == [{"id": 3, "p": {"x": 5, "y": 6}, "tag": "w", "inner": [2, 3]}]


def test_hydrate_allocates_per_call_and_memoises_nothing(layout):
    assert layout["hydrate_twice_distinct"] is True
    assert layout["after_mutation"]["a"] == 1.5 and layout["after_mutation"]["objs"][0]["id"] == 1


def test_irregular_columns_fall_back_to_any(layout):
    assert layout["irregular"] == {"m": "any", "o": "any"}
    assert layout["irregular_rows"][0] == {"m": 1, "o": [{"x": 1}]}
    assert layout["irregular_rows"][1] == {"m": "a", "o": [{"y": 2}]}


def test_nested_shape_mismatch_and_edge_leaves(layout):
    n = layout["shape_nested"]
    assert n["kind"] == "any"
    assert n["rows"] == [[{"a": {"x": 1}}], [{"a": {"x": 2, "y": 3}}]]
    assert layout["shape_empty"]["rows"] == [[{"a": {}, "b": 1}]]
    assert layout["shape_null"]["rows"] == [[{"x": 1}, None]]
    assert layout["shape_bool"]["rows"] == [[{"f": True, "x": 1}, {"f": False, "x": 2}]]


def test_undefined_values_are_omitted_from_hydrated_rows(layout):
    assert layout["undefined_omitted"] == {"has_k0": False, "has_k1": True}


def test_transfer_and_adoption_in_layout_harness(layout):
    assert layout["buffers"]["count"] == layout["buffers"]["expect"]
    assert layout["detached"] == {"t": True, "a": True, "offs": True, "leaf": True}
    assert layout["adopted"] is True and layout["strings_cloned"] is True
    assert layout["after_transfer"] == layout["hydrated"]


def test_worker_transfers_every_typed_buffer(sandbox, tmp_path):
    bag = tmp_path / "bag_0.mcap"
    write_bag(bag, 25.0, seed=3)
    r = _node([sandbox / "transfer.js", bag, 1], sandbox)
    assert r["typedBufferCount"] > 20 and r["transferCount"] == r["typedBufferCount"]
    assert r["everyTypedBufferListed"] is True
    assert r["senderViewsDetached"] == r["senderViewsTotal"] > 0        # a copied (unlisted) buffer would stay attached
    assert r["hydratedRows"] > 0 and r["adopted"] is True
