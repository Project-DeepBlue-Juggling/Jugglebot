"""``motion/skills/sites`` — cup-opening sites (plan § 2.2).

Unmarked and parallel-safe: pure arithmetic, nothing touches the filesystem or
a shared clock.

Plan: ``plans/active/two-ball-skill-stack.md`` § 2.2.
"""

from __future__ import annotations

import numpy as np
import pytest

from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.skills import sites as st


def test_site_rejects_a_bad_shape():
    with pytest.raises(ValueError, match='cup_mm'):
        st.Site('P1', np.array([0.0, 0.0]))


def test_site_rejects_non_finite():
    with pytest.raises(ValueError, match='finite'):
        st.Site('P1', np.array([0.0, 0.0, float('nan')]))


def test_site_rejects_an_empty_name():
    with pytest.raises(ValueError, match='name'):
        st.Site('', np.array([0.0, 0.0, 830.0]))


def test_throw_catch_rest_site_mm_share_the_sites_xy():
    """The three helpers move only z; xy is the site's own, untouched."""
    site = st.Site('P1', np.array([12.5, -7.0, 830.0]))
    throw = site.throw_site_mm()
    catch = site.catch_site_mm()
    rest = site.rest_site_mm()
    for arr in (throw, catch, rest):
        assert arr[0] == pytest.approx(12.5)
        assert arr[1] == pytest.approx(-7.0)
    assert throw[2] == pytest.approx(st.RELEASE_CUP_Z_MM)
    assert catch[2] == pytest.approx(st.CATCH_CUP_Z_MM)
    assert rest[2] == pytest.approx(st.REST_CUP_Z_MM)


def test_the_helpers_accept_an_explicit_z_override():
    site = st.Site('P1', np.array([0.0, 0.0, 830.0]))
    assert site.throw_site_mm(900.0)[2] == pytest.approx(900.0)
    assert site.catch_site_mm(800.0)[2] == pytest.approx(800.0)
    assert site.rest_site_mm(700.0)[2] == pytest.approx(700.0)


def test_rest_cup_z_mm_is_unified_cycles_own_settle_clamp():
    """No second copy of the settle height: the two constants are the SAME
    float, not merely numerically close."""
    assert st.REST_CUP_Z_MM == uc.SETTLE_CUP_Z_MM


def test_columns_sites_are_symmetric_about_the_origin_and_catch_z():
    p1, p2 = st.columns_sites(100.0)
    assert p1.name == 'P1' and p2.name == 'P2'
    np.testing.assert_allclose(p1.cup_mm, [-50.0, 0.0, st.CATCH_CUP_Z_MM])
    np.testing.assert_allclose(p2.cup_mm, [50.0, 0.0, st.CATCH_CUP_Z_MM])
    assert (p2.cup_mm[0] - p1.cup_mm[0]) == pytest.approx(100.0)


def test_columns_sites_rejects_a_non_positive_separation():
    with pytest.raises(ValueError, match='separation_mm'):
        st.columns_sites(0.0)
    with pytest.raises(ValueError, match='separation_mm'):
        st.columns_sites(-10.0)


def test_release_and_catch_z_match_the_reload_coordinators_sittings_geometry():
    """PROVENANCE pin: these are the same numbers
    ``reload_coordinator_node._UNIFIED_THROW_CUP_Z_MM`` /
    ``_UNIFIED_CATCH_CUP_Z_MM`` carry (checked by literal value, since
    ``motion/`` may not import that ROS module to check it by reference)."""
    assert st.RELEASE_CUP_Z_MM == 860.0
    assert st.CATCH_CUP_Z_MM == 830.0
