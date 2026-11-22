"""Platform IMU mount calibration (IDEA-20 P5): file validation, absent-file behaviour and the direction of
the rotation, checked against facts measured at the 2026-11-22 Sitting. Synthetic arrays only."""

from __future__ import annotations

import copy
import json
import os

import numpy as np
import pytest

from teensy_link import platform_imu_capture as cap
from teensy_link import platform_imu_mount as pm

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
REAL = os.path.join(REPO, "config", "platform_imu_mount.json")


@pytest.fixture
def doc():
    with open(REAL) as f:
        return json.load(f)


def _write(tmp_path, d):
    p = tmp_path / "mount.json"
    p.write_text(json.dumps(d))
    return str(p)


def test_committed_file_loads_and_is_a_rotation():
    m = pm.load_mount(REAL)
    r = np.array(m.matrix)
    assert m.date == "2026-11-22"
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-12) and np.linalg.det(r) == pytest.approx(1.0)
    assert m.gyro_bias_dps == [0.351, 0.262, -0.057]
    assert m.doc["lever_arm_mm"]["value"] == [5.0, -115.0, 18.5]
    assert "not measured" in m.doc["lever_arm_mm"]["status"]


def test_committed_file_is_three_degrees_from_the_cad_nominal(doc):
    r = np.array(doc["rotation"]["matrix"])
    nominal = np.array(doc["rotation"]["cad_nominal"])
    # CAD: IMU x = Platform +z, IMU y = Platform -x, IMU z = Platform -y
    assert np.allclose(nominal @ [1, 0, 0], [0, 0, 1]) and np.allclose(nominal @ [0, 1, 0], [-1, 0, 0])
    assert np.allclose(nominal @ [0, 0, 1], [0, -1, 0])
    angle = np.degrees(np.arccos((np.trace(r @ nominal.T) - 1) / 2))
    assert angle == pytest.approx(doc["rotation"]["deviation_from_cad_deg"]["total_angle"], abs=0.01)


def test_absent_file_returns_none(tmp_path):
    assert pm.load_mount(str(tmp_path / "nope.json")) is None


@pytest.mark.parametrize("mutate,msg", [
    (lambda d: d.update(schema_version=2), "schema_version"),
    (lambda d: d["rotation"].update(matrix=[[1, 0, 0], [0, 1, 0]]), "3 rows"),
    (lambda d: d["rotation"].update(matrix=[[1, 0, 0], [0, 1, 0], [0, 0, "a"]]), "non-numeric"),
    (lambda d: d["rotation"].update(matrix=[[1, 0, 0], [0, 1, 0], [0, 0, float("nan")]]), "non-finite"),
    (lambda d: d["rotation"].update(matrix=[[1, 0, 0], [0, 1, 0], [0, 0, 1.01]]), "not orthonormal"),
    (lambda d: d["rotation"].update(matrix=[[1, 0, 0], [0, 1, 0], [0, 0, -1]]), "determinant"),
    (lambda d: d["rotation"].update(matrix=[[-1, 0, 0], [0, 1, 0], [0, 0, 1]]), "determinant"),
    (lambda d: d["provenance"].pop("date"), "provenance.date"),
    (lambda d: d["gyro_bias"].update(values_dps_imu_frame=[0, 0]), "gyro_bias"),
    (lambda d: d.pop("rotation"), "rotation"),
])
def test_malformed_file_fails_loudly(tmp_path, doc, mutate, msg):
    d = copy.deepcopy(doc)
    mutate(d)
    with pytest.raises(pm.MountCalError, match=msg):
        pm.load_mount(_write(tmp_path, d))


def test_invalid_json_fails_loudly(tmp_path):
    p = tmp_path / "m.json"
    p.write_text("{oops")
    with pytest.raises(pm.MountCalError, match="not valid JSON"):
        pm.load_mount(str(p))


# Mean accelerometer, IMU frame (g), from the 2026-11-22 Sitting's still windows (capture 572E).
LEVEL = (0.99857, -0.00193, 0.05278)
RX5 = (0.99899, -0.00313, -0.03438)
RY5 = (0.99460, 0.08529, 0.05186)


def test_level_reads_plus_one_g_up_in_the_platform_frame():
    """Sign convention of the capture: the accelerometer reports specific force, +1 g up at rest."""
    m = pm.load_mount(REAL)
    out = cap.to_platform_frame(np.array([LEVEL]), m)[0]
    assert np.allclose(out, [0, 0, 1], atol=5e-4)


def test_rx_plus_5_gives_plus_y_and_ry_plus_5_gives_minus_x():
    """Rx +5 (right-handed about Platform x) tilts the specific force toward +y: +0.0872 g. Ry +5 toward -x."""
    m = pm.load_mount(REAL)
    rx = cap.to_platform_frame(np.array([RX5]), m)[0]
    ry = cap.to_platform_frame(np.array([RY5]), m)[0]
    assert rx[1] == pytest.approx(np.sin(np.radians(5)), abs=5e-4) and abs(rx[0]) < 5e-4
    assert ry[0] == pytest.approx(-np.sin(np.radians(5)), abs=5e-4) and abs(ry[1]) < 5e-4


def test_direction_is_imu_to_platform_not_the_inverse():
    """The transpose would put gravity on the wrong axis: guards a swapped direction."""
    m = pm.load_mount(REAL)
    wrong = (np.array(m.matrix).T @ np.array(LEVEL))
    assert abs(wrong[2] - 1) > 0.01


def test_apply_is_rotation_only_and_leaves_input_untouched():
    m = pm.load_mount(REAL)
    v = np.array([LEVEL, RX5, RY5])
    keep = v.copy()
    out = cap.to_platform_frame(v, m)
    assert np.array_equal(v, keep)                                     # raw stays raw
    assert np.allclose(np.linalg.norm(out, axis=1), np.linalg.norm(v, axis=1), atol=1e-12)   # no scale, no gravity removed
    assert np.allclose(out, v @ np.array(m.matrix).T)
    with pytest.raises(ValueError):
        cap.to_platform_frame(np.zeros((4, 2)), m)


def test_stdlib_rotation_matches_numpy():
    import array
    m = pm.load_mount(REAL)
    rng = np.random.default_rng(1)
    v = rng.normal(size=(200, 3)).astype(np.float32)
    out = np.frombuffer(pm.rotate_xyz(array.array("f", v.ravel().tolist()), m.matrix), np.float32).reshape(-1, 3)
    assert np.allclose(out, v.astype(np.float64) @ np.array(m.matrix).T, atol=1e-5)
