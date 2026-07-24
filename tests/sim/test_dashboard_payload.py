"""Unit tests for ``sim.viz.dashboard.server._record_to_payload``.

Locks the SSE payload shape consumed by the browser dashboard.  Phase 0
of the dashboard-3d-mesh-and-sim-port plan adds ball positions/velocities,
per-leg accelerations, and throw-director state to the broadcast; the
front-end fork in Phase 2+ depends on those keys being present.
"""
from __future__ import annotations

import json

from controller.telemetry import StepRecord
from sim.viz.dashboard.server import _record_to_payload


def _make_populated_record() -> StepRecord:
    """A StepRecord with every new Phase-0 field set to a distinctive value
    so a wrong field-to-key mapping shows up as a wrong numeric reading."""
    return StepRecord(
        time=1.25,
        leg_acc_0=10.0, leg_acc_1=11.0, leg_acc_2=12.0,
        leg_acc_3=13.0, leg_acc_4=14.0, leg_acc_5=15.0,
        ball0_x=100.0, ball0_y=200.0, ball0_z=300.0,
        ball0_vx=1.0, ball0_vy=2.0, ball0_vz=3.0,
        ball0_held=1,
        ball1_x=400.0, ball1_y=500.0, ball1_z=600.0,
        ball1_vx=4.0, ball1_vy=5.0, ball1_vz=6.0,
        ball1_held=0,
        throw_phase="hand_catch",
        catches_total=7,
    )


def test_payload_includes_leg_accelerations():
    payload = _record_to_payload(_make_populated_record())
    assert payload["leg_acc"] == [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]


def test_payload_includes_two_ball_block():
    payload = _record_to_payload(_make_populated_record())
    balls = payload["balls"]
    assert len(balls) == 2
    assert balls[0] == {
        "pos": [100.0, 200.0, 300.0],
        "vel": [1.0, 2.0, 3.0],
        "held": 1,
    }
    assert balls[1] == {
        "pos": [400.0, 500.0, 600.0],
        "vel": [4.0, 5.0, 6.0],
        "held": 0,
    }


def test_payload_includes_throw_block():
    payload = _record_to_payload(_make_populated_record())
    assert payload["throw"] == {"phase": "hand_catch", "catches": 7}


def test_payload_is_json_serialisable():
    """The SSE handler serialises payloads with json.dumps; any non-JSON
    value (numpy scalar, set, NaN-shaped object) would break the stream."""
    text = json.dumps(_record_to_payload(_make_populated_record()))
    parsed = json.loads(text)
    assert parsed["t"] == 1.25
    assert parsed["throw"]["catches"] == 7


def test_default_record_has_zeroed_new_fields():
    """A default StepRecord (the hot-loop pool slot's initial state)
    serialises with zero/empty new fields — guarantees prod / MPC paths
    that never populate balls or throw state don't leak stale data."""
    payload = _record_to_payload(StepRecord())
    assert payload["leg_acc"] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert payload["balls"] == [
        {"pos": [0.0, 0.0, 0.0], "vel": [0.0, 0.0, 0.0], "held": 0},
        {"pos": [0.0, 0.0, 0.0], "vel": [0.0, 0.0, 0.0], "held": 0},
    ]
    assert payload["throw"] == {"phase": "", "catches": 0}
