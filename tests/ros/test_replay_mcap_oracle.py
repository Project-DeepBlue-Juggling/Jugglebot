# -*- coding: utf-8 -*-
"""Phase 4 contract (design § 5a, 5d): the browser's direct MCAP decode
(``js/replay/mcap-decode.js``) equals the Python converter's chunk records.

A synthetic rosbag2-shaped bag (uncompressed, with equal-timestamp pairs on slot
edges) is converted by ``replay.convert`` (the ORACLE) and decoded by the JS core
read directly from the file (node IReadable, and again through ``httpReadable``
over a fake Range fetch). Every slot's topic set, ``n``, ``t`` array, column
names and values must agree (floats within 1e-9; non-finite floats are tagged by
the harness, not nulled). ``slotOf`` is pinned against ``schema.chunk_index`` on
a vector that includes CPython's float-floordiv rounding case. The JS allow-list
is pinned against ``schema.ALLOWLIST``.

The sandbox keeps the ``js/replay/`` + ``lib/`` layout so the module's relative
bundle import resolves, and is ``"type": "module"``.
"""
from __future__ import annotations

import glob
import gzip
import json
import math
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import msgpack
import pytest

from tests.ros._replay_fixture import write_bag

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))

from replay import schema  # noqa: E402
from replay.convert import convert  # noqa: E402

HARNESS = REPO / "tests" / "ros" / "js" / "mcap_oracle_harness.js"
DURATION = 35.0


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


def _sandbox(tmp_path):
    sb = tmp_path / "sb"
    (sb / "js" / "replay").mkdir(parents=True)
    (sb / "lib").mkdir()
    for name in ("mcap-decode.js", "allowlist.js"):
        shutil.copy(GUI / "js" / "replay" / name, sb / "js" / "replay" / name)
    shutil.copy(GUI / "lib" / "mcap-bundle.min.js", sb / "lib" / "mcap-bundle.min.js")
    shutil.copy(HARNESS, sb / "harness.js")
    (sb / "package.json").write_text('{"type": "module"}')
    return sb


def _run(sb, bag, out, mode="file", extra=()):
    p = subprocess.run([NODE, str(sb / "harness.js"), str(bag), str(out), mode, *extra],
                       capture_output=True, text=True, timeout=180)
    return p


def _decode(sb, bag, out, mode="file", extra=()):
    p = _run(sb, bag, out, mode, extra)
    assert p.returncode == 0, p.stderr
    return json.loads(Path(out).read_text())


@pytest.fixture(scope="module")
def world(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("mcap_oracle")
    bag_dir = tmp / "bag"
    bag_dir.mkdir()
    bag = bag_dir / "bag_0.mcap"
    info = write_bag(bag, DURATION, seed=7, edge_messages=True)
    cache = tmp / "cache"
    res = convert(str(bag), str(cache), recording_id="oracle")
    assert res["status"] == schema.STATUS_COMPLETE
    manifest = json.loads((cache / schema.MANIFEST).read_text())
    sb = _sandbox(tmp)
    js = _decode(sb, bag, tmp / "js_file.json", "file", ("--vector",))
    return {"tmp": tmp, "bag": bag, "info": info, "cache": cache, "manifest": manifest,
            "sb": sb, "js": js}


def _oracle_chunk(cache, i):
    raw = gzip.decompress((Path(cache) / schema.chunk_name(i)).read_bytes())
    return msgpack.unpackb(raw, raw=False)


def _untag(v):
    if isinstance(v, dict):
        if set(v) == {"$f"}:
            return {"nan": math.nan, "inf": math.inf, "-inf": -math.inf}[v["$f"]]
        return {k: _untag(x) for k, x in v.items()}
    if isinstance(v, list):
        return [_untag(x) for x in v]
    return v


def _same(a, b, where):
    """Strict structure, floats within 1e-9, NaN == NaN (both sides non-finite)."""
    if isinstance(a, float) or isinstance(b, float):
        if isinstance(a, bool) or isinstance(b, bool):
            assert a == b, where
            return 1
        assert isinstance(a, (int, float)) and isinstance(b, (int, float)), (where, a, b)
        if math.isnan(a) or math.isnan(b):
            assert math.isnan(a) and math.isnan(b), (where, a, b)
        elif math.isinf(a) or math.isinf(b):
            assert a == b, (where, a, b)
        else:
            assert abs(a - b) <= 1e-9 + 1e-12 * abs(a), (where, a, b)
        return 1
    if isinstance(a, list):
        assert isinstance(b, list) and len(a) == len(b), (where, len(a), b if not isinstance(b, list) else len(b))
        return sum(_same(x, y, where + "[%d]" % k) for k, (x, y) in enumerate(zip(a, b)))
    if isinstance(a, dict):
        assert isinstance(b, dict) and set(a) == set(b), (where, sorted(a), sorted(b) if isinstance(b, dict) else b)
        return sum(_same(a[k], b[k], where + "." + k) for k in a)
    assert a == b and type(a) == type(b) or (isinstance(a, int) and isinstance(b, int) and a == b), (where, a, b)
    return 1


def _compare(world, js):
    m = world["manifest"]
    assert js["opened"]["slots"] == m["chunks_total"]
    assert js["opened"]["t0"] == m["t0"]
    assert js["opened"]["t1"] == m["t1"]
    assert js["opened"]["topics"] == m["topics"]
    assert js["opened"]["skipped"] == {}
    assert len(js["slots"]) == m["chunks_total"]
    n_values = 0
    for s in js["slots"]:
        ref = _oracle_chunk(world["cache"], s["i"])
        assert s["t0"] == ref["t0"] and s["t1"] == ref["t1"], s["i"]
        assert set(s["topics"]) == set(ref["topics"]), ("slot %d topics" % s["i"])
        assert s["dropped"] == {}
        for topic, r in ref["topics"].items():
            j = s["topics"][topic]
            where = "slot %d %s" % (s["i"], topic)
            assert j["type"] == r["type"] and j["n"] == r["n"], where
            assert j["t"] == r["t"], where + " t"          # exact: same int/1e9 rounding
            assert set(j["cols"]) == set(r["cols"]), where + " columns"
            for col, vals in r["cols"].items():
                assert len(j["cols"][col]) == r["n"], where + "." + col
                n_values += _same(_untag(j["cols"][col]), vals, where + "." + col)
    return n_values


def test_js_decode_equals_python_oracle(world):
    js = world["js"]
    n_values = _compare(world, js)
    n_slots = len(js["slots"])
    assert n_slots == 4
    # Edge messages really exercised: ties kept in order on the slot-edge, none lost.
    orch = [v for s in js["slots"] for v in s["topics"].get("/orchestrator_state", {}).get("cols", {}).get("data", [])]
    assert orch.count("TIE_A") == 1 and orch.index("TIE_A") + 1 == orch.index("TIE_B")
    assert orch.index("TIE_C") + 1 == orch.index("TIE_D") and "EDGE_LO" in orch
    print("\nORACLE slots=%d values=%d mismatches=0 open_ms=%.1f decode_ms=%.1f"
          % (n_slots, n_values, js["openMs"], js["decodeMs"]))


def test_http_readable_equals_file_readable(world):
    js = _decode(world["sb"], world["bag"], world["tmp"] / "js_http.json", "http")
    assert _compare(world, js) > 0
    assert js["http"]["heads"] == 1
    # open (~5 exact reads) + at most one prefetch GET per slot; nothing per-message.
    assert js["http"]["requests"] <= 12 + len(js["slots"]), js["http"]


def test_slot_of_matches_python_chunk_index(world):
    vec = world["js"]["vector"]
    assert len(vec) >= 6
    for t, t0, got in vec:
        assert got == schema.chunk_index(t, t0), (t, t0, got)


def test_latest_row_reads_back_past_sixty_seconds(world):
    lat = world["js"]["latest"]
    info = world["info"]
    t0 = world["manifest"]["t0"]
    skill = lat["/skills/attempt@34"]
    want = max(t for t, _ in info["topics"]["/skills/attempt"] if t / 1e9 <= t0 + 34)
    assert skill["t"] == want / 1e9
    assert lat["/cone/catch_event@30"]["t"] > 0
    assert lat["/skills/attempt@1"] is None          # nothing at or before t0 + 1


def test_refusals(world, tmp_path):
    sb = world["sb"]
    z = tmp_path / "z"; z.mkdir()
    write_bag(z / "bag_0.mcap", 12.0, seed=1, compression="zstd")
    p = _run(sb, z / "bag_0.mcap", tmp_path / "o.json")
    assert p.returncode != 0 and re.search(r"reason.*compressed|compressed", p.stderr)
    u = tmp_path / "u"; u.mkdir()
    write_bag(u / "bag_0.mcap", 20.0, seed=1, unindexed=True, chunk_size=16 * 1024)
    p = _run(sb, u / "bag_0.mcap", tmp_path / "o.json")
    assert p.returncode != 0 and "no_index" in p.stderr


def test_allowlist_js_pin():
    text = (GUI / "js" / "replay" / "allowlist.js").read_text()
    js = re.findall(r"'(/[^']+)'", text)
    assert js == list(schema.ALLOWLIST), "js/replay/allowlist.js drifted from schema.ALLOWLIST"
    assert len(set(js)) == len(js)


def test_head_409_reason_comes_from_the_header(tmp_path):
    """Found by the Phase 4 smoke: the size probe is a HEAD (no body), so httpReadable must take the refusal
    reason from X-Replay-Reason, not only the JSON body (a killed recording read as "in_progress")."""
    sb = _sandbox(tmp_path)
    (sb / "refusal.js").write_text(
        "import { httpReadable } from './js/replay/mcap-decode.js';\n"
        "const mk = (h, body) => async () => ({ status: 409, ok: false, headers: { get: (k) => h[k] || null },\n"
        "  json: async () => { if (body === null) throw new Error('no body'); return body; } });\n"
        "const out = {};\n"
        "for (const [name, f] of Object.entries({\n"
        "  head_no_index: mk({ 'X-Replay-Reason': 'no_index' }, null),\n"
        "  head_in_progress: mk({ 'X-Replay-Reason': 'recording_in_progress' }, null),\n"
        "  body_only_no_index: mk({}, { reason: 'no_index' }),\n"
        "  fetch_rejects: async () => { throw new TypeError('network down'); },\n"
        "})) { try { await httpReadable('http://x/f', { fetch: f }).size(); out[name] = 'resolved'; } catch (e) { out[name] = e.reason; } }\n"
        "console.log(JSON.stringify(out));\n")
    p = subprocess.run([NODE, str(sb / "refusal.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert p.returncode == 0, p.stderr.decode()
    assert json.loads(p.stdout.decode().strip().splitlines()[-1]) == {
        "head_no_index": "no_index", "head_in_progress": "in_progress", "body_only_no_index": "no_index",
        "fetch_rejects": "http"}
