"""Route half of design 06 § 5(b): Range / ETag / refusals on recordings/<id>/file."""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from _replay_srv import Srv, make_bag  # noqa: E402

A = "2026-10-01_10-00-00"


@pytest.fixture
def srv(tmp_path):
    s = Srv(tmp_path)
    yield s
    s.close()


def url(rid=A):
    return "/api/replay/recordings/{}/file".format(rid)


@pytest.fixture
def bag(srv):
    p = make_bag(srv.root, A)
    with open(p, "rb") as f:
        return p, f.read()


def test_plain_get_and_head(srv, bag):
    _p, data = bag
    code, h, body = srv.req(url())
    assert code == 200 and body == data
    assert h["Accept-Ranges"] == "bytes" and int(h["Content-Length"]) == len(data)
    assert h["Content-Type"] == "application/octet-stream"
    assert h["ETag"].startswith('"')
    assert "Content-Range" in h["Access-Control-Expose-Headers"]
    code, h2, body = srv.req(url(), "HEAD")
    assert code == 200 and body == b"" and int(h2["Content-Length"]) == len(data)
    assert h2["ETag"] == h["ETag"]


def test_no_store_on_every_file_response(srv, bag):
    assert srv.req(url())[1]["Cache-Control"] == "no-store"
    assert srv.req(url(), headers={"Range": "bytes=0-3"})[1]["Cache-Control"] == "no-store"
    assert srv.req(url(), headers={"Range": "bytes=99999999-"})[1]["Cache-Control"] == "no-store"


@pytest.mark.parametrize("spec", ["bytes=0-15", "bytes=100-", "bytes=-37", "bytes=5-5",
                                  "bytes=10-99999999", "bytes=-999999999"])
def test_single_range_206_bytes_equal_slice(srv, bag, spec):
    _p, data = bag
    size = len(data)
    a, b = spec[len("bytes="):].split("-")
    if a == "":
        first, last = max(0, size - int(b)), size - 1
    else:
        first, last = int(a), (size - 1 if b == "" else min(int(b), size - 1))
    code, h, body = srv.req(url(), headers={"Range": spec})
    assert code == 206
    assert body == data[first:last + 1]
    assert h["Content-Range"] == "bytes {}-{}/{}".format(first, last, size)
    assert int(h["Content-Length"]) == len(body)


def test_head_with_range(srv, bag):
    code, h, body = srv.req(url(), "HEAD", {"Range": "bytes=0-9"})
    assert code == 206 and body == b"" and h["Content-Length"] == "10"


def test_range_beyond_one_block_streams_intact(tmp_path):
    s = Srv(tmp_path)
    try:
        big = os.urandom(3 * (1 << 20) + 123)
        make_bag(s.root, A, body=big)
        data = open(os.path.join(s.root, A, A + "_0.mcap"), "rb").read()
        code, _h, body = s.req(url(), headers={"Range": "bytes=7-"})
        assert code == 206 and body == data[7:]
    finally:
        s.close()


@pytest.mark.parametrize("spec", ["bytes=0-1,5-6", "bytes=99999999-", "bytes=9-3",
                                  "bytes=-0", "bytes=-", "items=0-1", "bytes=a-b"])
def test_unsatisfiable_or_multi_range_is_416_never_200(srv, bag, spec):
    _p, data = bag
    code, h, body = srv.req(url(), headers={"Range": spec})
    assert code == 416
    assert h["Content-Range"] == "bytes */{}".format(len(data))
    assert len(body) < 200  # a JSON error, never the file


def test_if_match_ok_then_412_after_file_changes(srv, bag):
    p, data = bag
    etag = srv.req(url())[1]["ETag"]
    code, _h, body = srv.req(url(), headers={"If-Match": etag, "Range": "bytes=0-3"})
    assert code == 206 and body == data[:4]
    make_bag(srv.root, A, body=data[8:-37] + b"more")  # rewritten mid-session, still indexed
    code, h, _b = srv.req(url(), headers={"If-Match": etag, "Range": "bytes=0-3"})
    assert code == 412 and h["Cache-Control"] == "no-store"
    assert srv.req(url(), "HEAD", {"If-Match": etag})[0] == 412
    new = srv.req(url())[1]["ETag"]
    assert new != etag
    assert srv.req(url(), headers={"If-Match": new})[0] == 200


def test_file_changed_between_listing_and_open_is_412_before_headers(srv, bag, monkeypatch):
    """The ETag/size come from find_recording; a rewrite before open() must not stream under a 200/206."""
    mod = sys.modules[type(srv.backend).__module__]
    real = mod.recordings.find_recording
    def stale(root, rid):
        info = real(root, rid)
        info["size_bytes"] += 1          # the listing predates a rewrite
        return info
    monkeypatch.setattr(mod.recordings, "find_recording", stale)
    code, h, _b = srv.req(url())
    assert code == 412 and h["Cache-Control"] == "no-store"
    assert srv.req(url(), "HEAD")[0] == 200      # HEAD never opens the file


def test_409_in_progress_for_growing_file(srv):
    make_bag(srv.root, A, kind="killed", age_s=1.0)
    code, j = srv.jreq(url())
    assert code == 409 and j == {"status": "refused", "reason": "recording_in_progress"}
    assert srv.req(url(), "HEAD")[0] == 409


@pytest.mark.parametrize("kind", ["killed", "nosummary"])
def test_409_no_index_for_footerless_or_summaryless(srv, kind):
    make_bag(srv.root, A, kind=kind, age_s=3600.0)
    code, j = srv.jreq(url(), headers={"Range": "bytes=0-3"})
    assert code == 409 and j == {"status": "refused", "reason": "no_index"}


def test_bad_and_unknown_ids(srv, bag):
    assert srv.req("/api/replay/recordings/..%2Fx/file")[0] == 400
    assert srv.req("/api/replay/recordings/x/file")[0] == 400
    assert srv.req("/api/replay/recordings/{}%0A/file".format(A))[0] == 400   # trailing newline: fullmatch
    assert srv.req(url("2030-01-01_00-00-00"))[0] == 404
    assert srv.req(url() + "/extra")[0] == 404
    assert srv.req("/api/replay/recordings/{}/file".format(A), "POST")[0] in (404, 501)


def test_deleted_routes_are_gone(srv, bag):
    base = "/api/replay/recordings/{}/".format(A)
    for tail, method in (("open", "POST"), ("status", "GET"), ("manifest", "GET"),
                         ("chunks/0", "GET")):
        assert srv.req(base + tail, method)[0] == 404, tail


def test_static_files_still_served_with_head(srv):
    code, h, body = srv.req("/index.html", "HEAD")
    assert code == 200 and body == b"" and h["Access-Control-Allow-Origin"] == "*"


# ---- design 06 § 5(b) file-vs-server half (node-free) -----------------------

class _RangeFile(object):
    """A seekable read-only file over the server's Range route (one GET per read)."""

    def __init__(self, srv, rid):
        code, h, _ = srv.req(url(rid), "HEAD")
        assert code == 200
        self.srv, self.rid, self.size, self.etag = srv, rid, int(h["Content-Length"]), h["ETag"]
        self.pos, self.gets = 0, 0

    def seek(self, off, whence=0):
        self.pos = (off, self.pos + off, self.size + off)[whence]
        return self.pos

    def tell(self):
        return self.pos

    def read(self, n=-1):
        if n is None or n < 0:
            n = self.size - self.pos
        n = min(n, self.size - self.pos)
        if n <= 0:
            return b""
        code, _h, body = self.srv.req(url(self.rid), headers={
            "Range": "bytes={}-{}".format(self.pos, self.pos + n - 1), "If-Match": self.etag})
        assert code == 206 and len(body) == n
        self.gets += 1
        self.pos += n
        return body


def test_mcap_read_through_range_equals_read_from_file(srv, tmp_path):
    """The same MCAP reader over the Range client and over the file yields the
    same summary and the same (topic, log_time, bytes) message sequence."""
    from _replay_fixture import write_bag
    from mcap.reader import SeekingReader
    d = os.path.join(srv.root, A)
    os.makedirs(d)
    path = os.path.join(d, A + "_0.mcap")
    write_bag(path, 35.0, seed=1)

    def read(f):
        r = SeekingReader(f)
        s = r.get_summary()
        meta = (s.statistics.message_count, s.statistics.message_start_time,
                sorted(c.topic for c in s.channels.values()), len(s.chunk_indexes))
        return meta, [(ch.topic, m.log_time, bytes(m.data)) for _s, ch, m in r.iter_messages()]

    with open(path, "rb") as f:
        want = read(f)
    rf = _RangeFile(srv, A)
    got = read(rf)
    assert rf.size == os.path.getsize(path) and rf.gets > 2
    assert got == want


@pytest.mark.parametrize("kind,age,reason", [("killed", 1.0, "recording_in_progress"), ("killed", 3600.0, "no_index")])
def test_409_reason_rides_a_header_so_head_carries_it(srv, kind, age, reason):
    """Found by the Phase 4 smoke: the browser's first request is a HEAD (size probe), which has no body, so a
    body-only reason made a killed recording read as "in progress". The header survives HEAD."""
    make_bag(srv.root, A, kind=kind, age_s=age)
    code, h, body = srv.req(url(), "HEAD")
    assert code == 409 and body == b""
    assert {k.lower(): v for k, v in h.items()}["x-replay-reason"] == reason
