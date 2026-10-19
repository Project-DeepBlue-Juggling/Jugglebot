"""The committed robot-frame resource (resources/jugglebot_robot_frame.json), built by
tools/robot_frame_build.py from the kincal + 2026-10-16_22-01-50 bags."""
from __future__ import annotations

import json
import os

import numpy as np

from jugglebot import robot_frame as rf

RESOURCE = os.path.join(os.path.dirname(rf.__file__), '..', 'resources', 'jugglebot_robot_frame.json')


def _tpl():
    return rf.load_robot_frame_template(RESOURCE)


def test_resource_loads_with_seven_labels():
    t = _tpl()
    assert t.labels == tuple('Base - %d' % i for i in range(1, 8))
    assert t.points_mm.shape == (7, 3) and np.all(np.isfinite(t.points_mm))


def test_provenance_names_kincal_bag_and_registration():
    with open(RESOURCE) as f:
        d = json.load(f)
    prov = d['provenance']
    assert '2026-09-27_14-31-13' in prov['reference_bag']
    assert 'kincal' in prov['frame'] and 'Base = QTM global' in prov['frame']
    assert prov['registration']['rms_mm'] < 1.0


def test_resource_has_only_the_schema_1_keys():
    # The tilt-from-build reference was retired 2026-10-19 (yaw-only fit).
    with open(RESOURCE) as f:
        assert set(json.load(f)) == {'schema', 'frame', 'markers', 'provenance'}


def test_template_fits_itself_as_identity():
    t = _tpl()
    rows = [(lb, *map(float, p)) for lb, p in zip(t.labels, t.points_mm)]
    fit = rf.fit_robot_frame(rows, t)
    assert fit.ok and fit.frame.is_identity()
    assert fit.rms_mm == 0.0
