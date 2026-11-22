"""IMU capture routes (ros_ws/gui/imu_api.py via gui_server), real server on an ephemeral port.

The server code is stdlib only; this test process has numpy and writes the fixtures
(.npz with the platform_imu_capture keys + a sidecar .json) with it.
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import threading
import urllib.error
import urllib.request

import numpy as np
import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
GUI = os.path.join(REPO, "ros_ws", "gui")
if GUI not in sys.path:
    sys.path.insert(0, GUI)

import gui_server  # noqa: E402

STEM_A = "20261120T100000Z_AAAA"
STEM_B = "20261120T110000Z_BBBB"
FREEZE_A = 1795170000.0
FREEZE_B = 1795173600.0
RATE = 1000.0


def write_capture(d, stem, freeze_s, n=64, complete=True, wall_ok=True, seed=0):
    rng = np.random.default_rng(seed)
    t_us = freeze_s * 1e6 - (n - 1 - np.arange(n)) * 1000.0
    accel = rng.normal(size=(n, 3)).astype(np.float32)
    gyro = rng.normal(size=(n, 3)).astype(np.float32) * 100
    valid = np.ones(n, dtype=bool)
    valid[3] = False
    np.savez_compressed(os.path.join(d, stem + ".npz"), raw=np.zeros((n, 6), np.int16),
                        valid=valid, t_wall_us=t_us, accel_g=accel, gyro_dps=gyro,
                        anchors=np.zeros((1, 3)), k0=np.array(0))
    side = {"label": "operator", "attempt_id": None, "span_wall_s": [None, None],
            "n_samples_written": n,
            "flags": {"complete": complete, "crc_ok": True},
            "fit": {"rate_hz": RATE, "wall_ok": wall_ok},
            "header": {"capture_id": 1234, "freeze_wall_us": int(freeze_s * 1e6),
                       "temp_c_freeze": 24.5}}
    with open(os.path.join(d, stem + ".json"), "w") as f:
        json.dump(side, f)
    return t_us, accel, gyro, valid


@pytest.fixture
def srv(tmp_path):
    d = tmp_path / "imu"
    d.mkdir()
    args = argparse.Namespace(
        port=0, host="127.0.0.1", rosbags_dir=str(tmp_path / "bags"),
        overview_dir=str(tmp_path / "ov"), worker_python="/nonexistent/python",
        imu_capture_dir=str(d), imu_mount=str(tmp_path / "no_such_mount.json"))
    server, _backend = gui_server.make_server(args)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()

    class S:
        dir = str(d)
        mount = str(tmp_path / "no_such_mount.json")   # absent: the Platform frame is not available

        @staticmethod
        def req(path):
            try:
                with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path), timeout=10) as r:
                    return r.status, r.headers, r.read()
            except urllib.error.HTTPError as e:
                return e.code, e.headers, e.read()

    yield S
    server.shutdown()
    server.server_close()


def test_list_sorted_with_fields(srv):
    write_capture(srv.dir, STEM_B, FREEZE_B)
    write_capture(srv.dir, STEM_A, FREEZE_A, complete=False)
    code, h, body = srv.req("/api/imu/captures")
    assert code == 200 and h["Cache-Control"] == "no-cache"
    caps = json.loads(body)["captures"]
    assert [c["stem"] for c in caps] == [STEM_A, STEM_B]
    a = caps[0]
    assert a["capture_id"] == 1234 and a["label"] == "operator" and a["attempt_id"] is None
    assert a["t_end"] == pytest.approx(FREEZE_A)
    assert a["t_start"] == pytest.approx(FREEZE_A - 64 / RATE)
    assert a["n_samples"] == 64 and a["complete"] is False and a["crc_ok"] is True
    assert a["wall_ok"] is True and a["temp_c_freeze"] == 24.5


def test_filter_by_overlap(srv):
    write_capture(srv.dir, STEM_A, FREEZE_A)
    write_capture(srv.dir, STEM_B, FREEZE_B)
    stems = lambda q: [c["stem"] for c in json.loads(srv.req("/api/imu/captures" + q)[2])["captures"]]  # noqa: E731
    assert stems("?from=%f" % (FREEZE_A + 10)) == [STEM_B]
    assert stems("?to=%f" % (FREEZE_A + 10)) == [STEM_A]
    assert stems("?from=%f&to=%f" % (FREEZE_A - 1, FREEZE_B + 1)) == [STEM_A, STEM_B]
    assert stems("?from=%f&to=%f" % (FREEZE_A + 10, FREEZE_B - 10)) == []
    assert stems("?from=%f" % (FREEZE_A - 0.01)) == [STEM_A, STEM_B]  # overlaps the tail


def test_corrupt_and_odd_sidecars_skipped(srv):
    write_capture(srv.dir, STEM_A, FREEZE_A)
    with open(os.path.join(srv.dir, "20261120T120000Z_CCCC.json"), "w") as f:
        f.write("{not json")
    with open(os.path.join(srv.dir, "20261120T130000Z_DDDD.json"), "w") as f:
        f.write("[1, 2]")
    with open(os.path.join(srv.dir, "20261120T140000Z_EEEE.json"), "w") as f:
        f.write('{"n_samples_written": 5}')
    with open(os.path.join(srv.dir, "notes.json"), "w") as f:
        f.write("{}")
    code, _h, body = srv.req("/api/imu/captures")
    assert code == 200
    assert [c["stem"] for c in json.loads(body)["captures"]] == [STEM_A]


def test_missing_dir_is_empty_list(srv):
    os.rmdir(srv.dir)
    code, _h, body = srv.req("/api/imu/captures")
    assert code == 200 and json.loads(body) == {"captures": []}


def test_data_round_trip(srv):
    t_us, accel, gyro, valid = write_capture(srv.dir, STEM_A, FREEZE_A, n=100, seed=3)
    code, h, body = srv.req("/api/imu/captures/%s/data" % STEM_A)
    assert code == 200 and h["Content-Type"] == "application/octet-stream"
    assert h["Cache-Control"] == "no-cache"
    n = 100
    assert body[:8] == b"JBIMU1\0\0"
    assert struct.unpack("<II", body[8:16]) == (n, 0)
    assert len(body) == 16 + n * 8 + n * 12 + n * 12 + n
    o = 16
    t = np.frombuffer(body, "<f8", n, o); o += n * 8
    a = np.frombuffer(body, "<f4", n * 3, o).reshape(n, 3); o += n * 12
    g = np.frombuffer(body, "<f4", n * 3, o).reshape(n, 3); o += n * 12
    v = np.frombuffer(body, "u1", n, o)
    np.testing.assert_allclose(t, t_us / 1e6, rtol=0, atol=1e-6)
    np.testing.assert_array_equal(a, accel)
    np.testing.assert_array_equal(g, gyro)
    np.testing.assert_array_equal(v, valid.astype("u1"))
    assert v[3] == 0 and v[0] == 1


@pytest.mark.parametrize("bad", [
    "..%2F..%2Fetc%2Fpasswd", "..", "20261120T100000Z_AAAA%2F..", "20261120T100000Z_ZZZZ",
    "20261120T100000Z_AAAA.npz", "x"])
def test_bad_stems_refused(srv, bad):
    write_capture(srv.dir, STEM_A, FREEZE_A)
    code, h, body = srv.req("/api/imu/captures/%s/data" % bad)
    assert code == 404
    assert "error" in json.loads(body)


def test_unknown_stem_404_json(srv):
    code, h, body = srv.req("/api/imu/captures/20261120T100000Z_AAAA/data")
    assert code == 404 and "json" in h["Content-Type"]
    assert "error" in json.loads(body)


def test_suffix_stem_allowed(srv):
    stem = STEM_A + "_2"
    write_capture(srv.dir, stem, FREEZE_A)
    assert srv.req("/api/imu/captures/%s/data" % stem)[0] == 200


def test_unreadable_npz_is_500_json(srv):
    with open(os.path.join(srv.dir, STEM_A + ".npz"), "wb") as f:
        f.write(b"this is not a zip")
    code, h, body = srv.req("/api/imu/captures/%s/data" % STEM_A)
    assert code == 500 and "json" in h["Content-Type"]
    assert "cannot read capture" in json.loads(body)["error"]


def test_default_dir_is_repo_temp(tmp_path):
    p = gui_server.build_parser().parse_args([])
    assert os.path.abspath(p.imu_capture_dir) == os.path.join(REPO, "temp", "imu_capture")


REAL = os.path.expanduser("~/Desktop/Jugglebot/temp/imu_capture")


@pytest.mark.skipif(not os.path.isdir(REAL) or not [f for f in os.listdir(REAL) if f.endswith(".npz")],
                    reason="no real captures on this box")
def test_real_capture_matches_numpy():
    sys.path.insert(0, GUI)
    import imu_api
    f = sorted(x for x in os.listdir(REAL) if x.endswith(".npz"))[0]
    body = imu_api.encode_capture(os.path.join(REAL, f))
    z = np.load(os.path.join(REAL, f))
    n = len(z["t_wall_us"])
    assert struct.unpack("<I", body[8:12])[0] == n
    np.testing.assert_allclose(np.frombuffer(body, "<f8", n, 16), z["t_wall_us"] / 1e6, atol=1e-6)
    np.testing.assert_array_equal(np.frombuffer(body, "<f4", n * 3, 16 + n * 8).reshape(n, 3),
                                  z["accel_g"])


# ---- the mount calibration: Platform frame by default, ?frame=imu for raw ----

REAL_MOUNT = os.path.join(REPO, "config", "platform_imu_mount.json")
# Mean accelerometer in the IMU frame at the 2026-11-22 Sitting's Level window (accel reads +1 g up at rest).
LEVEL_IMU = (0.99857, -0.00193, 0.05278)


@pytest.fixture
def srv_cal(tmp_path):
    """Same server, but pointed at a copy of the committed calibration."""
    import shutil
    d = tmp_path / "imu"
    d.mkdir()
    mount = tmp_path / "platform_imu_mount.json"
    shutil.copy(REAL_MOUNT, mount)
    args = argparse.Namespace(
        port=0, host="127.0.0.1", rosbags_dir=str(tmp_path / "bags"),
        overview_dir=str(tmp_path / "ov"), worker_python="/nonexistent/python",
        imu_capture_dir=str(d), imu_mount=str(mount))
    server, _backend = gui_server.make_server(args)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()

    class S:
        dir = str(d)
        mount_path = str(mount)

        @staticmethod
        def req(path):
            try:
                with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path), timeout=10) as r:
                    return r.status, r.headers, r.read()
            except urllib.error.HTTPError as e:
                return e.code, e.headers, e.read()

    yield S
    server.shutdown()
    server.server_close()


def _level_capture(d, stem, n=50):
    t_us = FREEZE_A * 1e6 - (n - 1 - np.arange(n)) * 1000.0
    accel = np.tile(np.array(LEVEL_IMU, np.float32), (n, 1))
    gyro = np.tile(np.array([0.351, 0.262, -0.057], np.float32), (n, 1))
    np.savez_compressed(os.path.join(d, stem + ".npz"), raw=np.zeros((n, 6), np.int16),
                        valid=np.ones(n, bool), t_wall_us=t_us, accel_g=accel, gyro_dps=gyro,
                        anchors=np.zeros((1, 3)), k0=np.array(0))
    with open(os.path.join(d, stem + ".json"), "w") as f:
        json.dump({"label": "operator", "n_samples_written": n, "flags": {"complete": True, "crc_ok": True},
                   "fit": {"rate_hz": RATE, "wall_ok": True},
                   "header": {"capture_id": 1, "freeze_wall_us": int(FREEZE_A * 1e6)}}, f)


def _arrays(body, n):
    o = 16 + n * 8
    acc = np.frombuffer(body, "<f4", n * 3, o).reshape(n, 3)
    gyr = np.frombuffer(body, "<f4", n * 3, o + n * 12).reshape(n, 3)
    return acc, gyr


def test_list_says_imu_frame_without_calibration(srv):
    write_capture(srv.dir, STEM_A, FREEZE_A)
    row = json.loads(srv.req("/api/imu/captures")[2])["captures"][0]
    assert row["frame"] == "imu" and row["cal_date"] is None


def test_data_without_calibration_is_raw_and_platform_is_refused(srv):
    t_us, accel, gyro, valid = write_capture(srv.dir, STEM_A, FREEZE_A, n=20, seed=2)
    code, _h, body = srv.req("/api/imu/captures/%s/data" % STEM_A)
    assert code == 200
    acc, _gyr = _arrays(body, 20)
    assert np.array_equal(acc, accel)
    code, _h, body = srv.req("/api/imu/captures/%s/data?frame=platform" % STEM_A)
    assert code == 409 and "no mount calibration" in json.loads(body)["error"]


def test_list_says_platform_frame_and_date_with_calibration(srv_cal):
    _level_capture(srv_cal.dir, STEM_A)
    row = json.loads(srv_cal.req("/api/imu/captures")[2])["captures"][0]
    assert row["frame"] == "platform" and row["cal_date"] == "2026-11-22"


def test_data_defaults_to_platform_frame_level_reads_plus_one_g_up(srv_cal):
    """The accelerometer reports specific force: at the Level window the rotated reading is about
    (0, 0, +1 g) in the Platform frame (+z up). The gyro is rotated, not de-biased."""
    _level_capture(srv_cal.dir, STEM_A)
    code, _h, body = srv_cal.req("/api/imu/captures/%s/data" % STEM_A)
    assert code == 200
    acc, gyr = _arrays(body, 50)
    assert np.allclose(acc.mean(axis=0), [0, 0, 1.0], atol=5e-4)
    # the gyro bias (0.351, 0.262, -0.057) dps in the IMU frame, rotated by the same matrix and not subtracted
    assert np.allclose(gyr.mean(axis=0), [-0.2635, 0.0725, 0.3470], atol=2e-3)
    code, _h, same = srv_cal.req("/api/imu/captures/%s/data?frame=platform" % STEM_A)
    assert code == 200 and same == body


def test_frame_imu_returns_raw_with_calibration_present(srv_cal):
    _level_capture(srv_cal.dir, STEM_A)
    code, _h, body = srv_cal.req("/api/imu/captures/%s/data?frame=imu" % STEM_A)
    assert code == 200
    acc, gyr = _arrays(body, 50)
    assert np.allclose(acc.mean(axis=0), LEVEL_IMU, atol=1e-6)
    assert np.allclose(gyr.mean(axis=0), [0.351, 0.262, -0.057], atol=1e-6)


def test_bad_frame_value_is_400(srv_cal):
    _level_capture(srv_cal.dir, STEM_A)
    assert srv_cal.req("/api/imu/captures/%s/data?frame=world" % STEM_A)[0] == 400


def test_malformed_calibration_is_loud_not_silently_raw(srv_cal):
    _level_capture(srv_cal.dir, STEM_A)
    with open(srv_cal.mount_path, "w") as f:
        f.write("{broken")
    code, _h, body = srv_cal.req("/api/imu/captures/%s/data" % STEM_A)
    assert code == 500 and "mount calibration unusable" in json.loads(body)["error"]
    listing = json.loads(srv_cal.req("/api/imu/captures")[2])
    assert listing["captures"][0]["frame"] == "imu" and "cal_error" in listing
    assert srv_cal.req("/api/imu/captures/%s/data?frame=imu" % STEM_A)[0] == 200


def test_calibration_edit_invalidates_the_cached_blob(srv_cal):
    _level_capture(srv_cal.dir, STEM_A)
    first = srv_cal.req("/api/imu/captures/%s/data" % STEM_A)[2]
    doc = json.load(open(srv_cal.mount_path))
    doc["rotation"]["matrix"] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    with open(srv_cal.mount_path, "w") as f:
        json.dump(doc, f)
    second = srv_cal.req("/api/imu/captures/%s/data" % STEM_A)[2]
    acc, _ = _arrays(second, 50)
    assert second != first and np.allclose(acc.mean(axis=0), LEVEL_IMU, atol=1e-6)
