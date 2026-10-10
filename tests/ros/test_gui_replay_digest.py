# -*- coding: utf-8 -*-
"""Far-tier digest data path (post-Phase-4 "10-minute view"): digest.js + chart-store two-tier view + worker lanes.

Node loads VERBATIM copies of the shipped modules over the Python oracle's chunks
(``tests/ros/_replay_chunks.py``): ``replay_digest_harness.js`` drives the real chart store and digester through
fake source/engine/cache objects, ``replay_digest_lane_harness.js`` drives the real ``mcap-worker.js`` handler
over a stub ``mcap-decode.js`` whose slot decodes the harness gates by hand.  The lite-load / no-memo contract of
the source itself lives in ``test_gui_replay_mcap_source.py``.
"""
from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from tests.ros._replay_chunks import export_chunks
from tests.ros._replay_fixture import write_bag

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
JS = REPO / "tests" / "ros" / "js"
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))

from replay.convert import convert  # noqa: E402


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

STUB_DECODE = """
// Stub mcap-decode.js: slot decodes are gated by the harness (state.gated) and recorded in start order.
export const state = {
  started: [], gated: false, waiters: [],
  releaseAll() { const w = state.waiters.splice(0); for (const r of w) r(); },
};
export async function openRecording() { return { t0: 0, t1: 1, slots: 100, topics: {}, skipped: {}, chunkCount: 1 }; }
export async function decodeSlot(rec, i) {
  state.started.push(i);
  if (state.gated) await new Promise((r) => state.waiters.push(r));
  return { i, t0: i * 10, t1: i * 10 + 10, topics: {}, dropped: {}, bytes: 0 };
}
export async function latestRow() { state.started.push('latest'); return null; }
export function httpReadable() { return {}; }
export function topicBuffers(topic, out = new Set()) { out.add(topic.t.buffer); return out; }
"""


def _run(sb, harness, *args):
    proc = subprocess.run([NODE, str(sb / harness)] + [str(a) for a in args],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=180)
    if proc.returncode != 0:
        pytest.fail("node harness failed in %s\n%s" % (sb, proc.stderr.decode("utf-8", "replace")))
    return json.loads(proc.stdout.decode("utf-8"))


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    d = tmp_path_factory.mktemp("digest")
    write_bag(d / "bag_0.mcap", 100.0, seed=4)
    cache = d / "cache"
    m = convert(str(d / "bag_0.mcap"), str(cache))
    assert m["status"] == "complete" and len(m["chunks"]) == 10
    sb = d / "sandbox"
    (sb / "replay").mkdir(parents=True)
    for name in ("chunk.js", "chart-store.js", "digest.js", "slot.js", "allowlist.js", "mcap-worker.js"):
        shutil.copy(GUI / "js" / "replay" / name, sb / "replay" / name)
    shutil.copy(GUI / "js" / "replay" / "mcap-worker.js", sb / "mcap-worker.js")
    shutil.copy(GUI / "js" / "replay" / "allowlist.js", sb / "allowlist.js")
    (sb / "mcap-decode.js").write_text(STUB_DECODE)
    chunks = d / "chunks"
    assert export_chunks(cache, chunks) == 10
    shutil.copy(JS / "replay_test_support.js", sb / "replay_test_support.js")
    for h in ("replay_digest_harness.js", "replay_digest_lane_harness.js"):
        shutil.copy(JS / h, sb / h)
    (sb / "package.json").write_text('{"type": "module"}\n')
    return {"main": _run(sb, "replay_digest_harness.js", chunks), "lane": _run(sb, "replay_digest_lane_harness.js")}


def test_node_check_modules():
    for rel in ("js/replay/digest.js", "js/replay/chart-store.js", "js/replay/mcap-worker.js", "js/replay/sources.js",
                "js/replay/cache.js"):
        r = subprocess.run([NODE, "--check", str(GUI / rel)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        assert r.returncode == 0, (rel, r.stderr.decode())


def test_bins_equal_a_brute_force_reference(out):
    b = out["main"]["bins"]
    assert b["bad"] == 0 and b["cmp"] > 500
    assert b["centresOk"] is True
    assert b["n_sum_is_samples"] is True            # every sample lands in exactly one bin (checked for both paths)
    assert b["resident_equals_far"] is True            # the far (non-resident) derivation bins identically
    assert b["open_chunk_null"] is True and b["axes_per_digest"] == 9


def test_fill_is_nearest_first_around_the_playhead_through_the_lite_lane(out):
    f = out["main"]["fill"]
    assert f["order"] == [5, 6, 4, 7, 3, 8]            # ahead wins the tie
    assert f["all_lite"] is True
    assert f["slots"] == [3, 4, 5, 6, 7, 8] and f["store_count"] == 6 and f["status_wanted"] == 6
    assert f["digestAt5"] is True and f["digestAt0"] is True


def test_retarget_only_after_a_30_s_move_and_evicts_beyond_the_span(out):
    r = out["main"]["retarget"]
    assert r["small"]["loads"] == 0                     # a 10 s move does not re-target
    assert r["centre"] != r["small"]["centre"]
    assert r["slots"] == [0, 1, 2, 3, 4] and r["store_count"] == 5      # 5..8 evicted, 3 and 4 kept
    assert r["new_order"] == [1, 2, 0]                                    # 3 and 4 were already digested; nearest first


def test_unit_toggle_clears_and_refills(out):
    i = out["main"]["invalidate"]
    assert i["cleared"] == 0 and i["refilled"] == 5 and i["reloaded"] == 5
    assert out["main"]["dispose"]["ring"] == 0


def test_paused_while_buffering_or_the_cache_is_loading(out):
    p = out["main"]["paused"]
    assert p["buffering"] == 0                          # a seek / stall: no digest load is issued
    assert p["resumed_loads"] == 6                      # and it resumes on buffering -> false
    assert p["busy"] == 0 and p["after_busy"] == 6      # cache has a load in flight; its own event resumes
    assert p["by_product_while_buffering"] is True and p["z_loads"] == 0   # CPU-only by-product digests are not paused
    assert p["inflight"] == {"first": 1, "after": 1, "ring": 1}            # the running lite finishes; nothing new starts


def test_resident_chunks_are_digested_as_a_by_product_without_a_load(out):
    b = out["main"]["byproduct"]
    assert b["not_loaded"] is True and b["digested"] is True and b["rest_loaded"] is True
    f = out["main"]["failed"]
    assert f["tries6"] == 1 and 6 not in f["others"] and 5 in f["others"]
    u = out["main"]["unsealed"]
    assert u["tries6"] == 1 and u["has6"] is False   # unsealed: failed once, not re-requested


def test_ring_never_exceeds_the_memory_bound(out):
    b = out["main"]["bound"]
    assert b["limit"] == 68
    assert b["full_ring"] >= 60 and b["max_ring"] <= b["limit"] and b["max_update"] <= b["limit"]
    assert b["lo_hi"][1] - b["lo_hi"][0] <= 66


def test_store_composed_data_is_monotonic_full_where_resident_means_elsewhere(out):
    s = out["main"]["store"]
    assert s["version_bumped"] and s["old_snapshot_untouched"]
    assert s["no_digest_same"] is True                                   # no digests: exactly the zero-copy full tier
    assert s["no_env"] == {"t": 0, "min": 0, "split": 0}
    assert s["monotonic"] is True and s["cols_len"] is True
    L = s["lengths"]
    assert L["total"] == L["want"] and L["nb"] > 0 and L["na"] > 0
    assert s["full_exact"] is True and s["means_ok"] is True
    assert s["after_shift"]["total"] == s["after_shift"]["want"]         # a partly-overlapped slot is dropped, not doubled
    assert s["no_resident"]["total"] == s["no_resident"]["want"]
    assert s["cleared"] is True


def test_store_envelope_exists_only_in_digest_regions(out):
    e = out["main"]["store"]["env"]
    assert e["n"] == e["want"] and e["split"] == e["want_split"]
    assert e["outside_resident"] is True and e["min_le_max"] is True and e["min_len"] is True
    s = out["main"]["store"]
    assert s["store_env_delegate"] is True and s["listener_fired"] is True
    assert s["invalidate"] == {"inv": 1, "digests_dropped": True}


def test_worker_serves_every_normal_load_before_a_lite_one(out):
    ln = out["lane"]
    assert ln["same_tick"] == [2, 3, "latest", 7, 8]    # full lane FIFO first, then the lite lane FIFO
    assert ln["replies"] == [2, 3, 7, 8]


def test_worker_does_not_preempt_a_running_lite_load_but_orders_the_next(out):
    ln = out["lane"]
    assert ln["running_lite_first"] == [5]
    assert ln["then_full_before_next_lite"] == [5, 1, 6]
