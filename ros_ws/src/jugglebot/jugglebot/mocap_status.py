"""The ``mocap/status`` contract — one definition of "QTM is ready" (F4/Q1-Q3).

``mocap_node`` publishes a DiagnosticStatus on ``mocap/status`` at 5 Hz; two
consumers gate BB calibration on it:

* ``teensy_bridge_node._svc_bb_calibrate`` — refuses the service call, so a
  manual Calibrate press with QTM dark never dispatches an RPC;
* ``orchestrator_node._dispatch_request('bb_calibrate')`` — SKIPS the HOMING
  step instead of calling the service, so homing completes with a WARN.

WHY THIS MODULE EXISTS RATHER THAN A ``_qtm_ready`` IN EACH NODE. The two
consumers must answer the same question the same way. If they drift — one
node's marker floor raised, the other's not — the failure is silent and
directional: the orchestrator would skip calibration during HOMING while the
GUI button still dispatched it (or the reverse, homing calling a service that
always refuses, which lands on ``operation_result False`` and FAULTs the
machine). Both nodes keep their own ``_qtm_ready()`` method — the cached
snapshot and its age are per-node state — but the PREDICATE and the two
thresholds live here, once.

Pure Python: no ROS imports, so ``motion/``-style test access is free and the
module can be imported from anywhere in the tree.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional, Tuple

#: Topic the status is published on — documentation and test reference ONLY.
#: The three call sites (mocap_node's create_publisher, the bridge's and the
#: orchestrator's create_subscription) spell the string out as a LITERAL on
#: purpose: tools/gen_choreography_map.py resolves endpoint names by AST and
#: refuses, by its honesty rule, to follow an attribute expression, so a
#: constant at the call site would erase the wire from
#: ros_ws/docs/choreography.md. That generated map — pinned by
#: tests/ros/test_choreography_map.py — is what actually keeps the publisher
#: and its two subscribers spelling the same topic.
MOCAP_STATUS_TOPIC = 'mocap/status'

#: KeyValue keys. Named here so a typo in a consumer is a NameError at import
#: rather than a silently-missing key that reads as "not ready" forever.
KEY_QTM_RECEIVING = 'qtm_receiving'
KEY_BB_MARKERS_VISIBLE = 'bb_markers_visible'
KEY_MARKER3_VISIBLE = 'marker3_visible'
KEY_ALIGNED = 'aligned'
KEY_QTM_SYNCED = 'qtm_synced'

#: A ``mocap/status`` sample older than this counts as no signal at all. The
#: publisher runs at 5 Hz, so 1.0 s is five consecutive missed cycles — far
#: beyond executor jitter (a loaded box cannot manufacture a refusal) yet fast
#: enough that a dead mocap_node is caught before the sweep is commanded.
MOCAP_STATUS_MAX_AGE_S = 1.0

#: BB fiducials that must be visible. ``bb_calibration.find_rotation_axis``
#: needs >= 2 markers to average an axis; 3 is one marker of margin against a
#: single occlusion mid-sweep turning a valid calibration into a wasted one.
MIN_BB_MARKERS_VISIBLE = 3

#: Refusal codes. TWO of them, because they send the operator to different
#: subsystems (the toss-ladder doctrine, ``toss_sequencer._step_checking``):
#: QTM_STALE is "nothing in ROS knows what QTM is doing" — the QTM machine or
#: mocap_node. BB_MARKERS_NOT_VISIBLE is "QTM streams fine but cannot resolve
#: the BB constellation" — occlusion or marker placement at the robot.
#: Collapsing them into one code would be the cheaper implementation and the
#: more expensive bench session.
CODE_QTM_STALE = 'QTM_STALE'
CODE_BB_MARKERS_NOT_VISIBLE = 'BB_MARKERS_NOT_VISIBLE'


def decode_status(values: Iterable[Any]) -> dict:
    """DiagnosticStatus ``values`` (KeyValue pairs) → a plain ``{key: value}``."""
    return {kv.key: kv.value for kv in values}


def evaluate(kv: Optional[dict], age_s: float) -> Tuple[bool, str, str]:
    """Is QTM in a state where a BB calibration sweep can produce data?

    Args:
        kv:     the decoded KeyValues of the most recent ``mocap/status``, or
                ``None`` if none has ever arrived.
        age_s:  seconds since that message arrived (ignored when ``kv`` is None).

    Returns:
        ``(ready, code, detail)``. ``code``/``detail`` are empty on success.

    Fails CLOSED: no status, an unparseable count, or a status that says QTM is
    dark all refuse. The alternative — assume ready when unsure — commands a
    physical sweep that at best produces nothing and at worst produces a
    fittable fragment of garbage that ``ball_butler_node`` then aims every
    throw with.
    """
    if kv is None:
        return (False, CODE_QTM_STALE,
                'no mocap/status has ever been received — is mocap_node running?')

    if age_s > MOCAP_STATUS_MAX_AGE_S:
        return (False, CODE_QTM_STALE,
                f'no mocap status for {age_s:.1f} s '
                f'(limit {MOCAP_STATUS_MAX_AGE_S:.1f} s)')

    if kv.get(KEY_QTM_RECEIVING, '0') != '1':
        return (False, CODE_QTM_STALE,
                'mocap_node reports no QTM packets arriving')

    try:
        visible = int(kv.get(KEY_BB_MARKERS_VISIBLE, '0'))
    except (TypeError, ValueError):
        visible = 0
    marker3 = kv.get(KEY_MARKER3_VISIBLE, '0') == '1'
    if visible < MIN_BB_MARKERS_VISIBLE or not marker3:
        return (False, CODE_BB_MARKERS_NOT_VISIBLE,
                f'{visible}/5 visible (need >= {MIN_BB_MARKERS_VISIBLE} '
                f'incl. Marker 3, seen={"yes" if marker3 else "no"})')

    return (True, '', '')
