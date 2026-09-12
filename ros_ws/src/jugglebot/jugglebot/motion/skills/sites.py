"""Cup-opening sites the skill stack throws to and catches at (plan § 2.2).

A :class:`Site` names one xy location on the platform and knows the three cup
heights a skill visits it at: a THROW releases at :data:`RELEASE_CUP_Z_MM`, a
CATCH is aimed at :data:`CATCH_CUP_Z_MM`, and a REST settles at
:data:`REST_CUP_Z_MM`. Pure Python + numpy, no ROS imports (``motion/`` rule).
"""

from __future__ import annotations

import dataclasses
from typing import Tuple

import numpy as np

from jugglebot.motion import unified_cycle as uc

#: Cup-opening world z (mm) a THROW releases at, and a CATCH is aimed at — the
#: sittings' geometry (``reload_coordinator_node._UNIFIED_THROW_CUP_Z_MM`` /
#: ``_UNIFIED_CATCH_CUP_Z_MM``) and the R2 sizing sweep confirming both stay
#: inside the cup box at the owner's R2 operating point (apex 0.9 m, dwell
#: 0.30 s, leg 300/5000/200000, hand acc 3500) — see ``tools/probes/
#: skills_sizing_sweep.py`` runs 2026-09-11/12, ``temp/probes/
#: skills_sizing_frontier2.md``. Deliberately NOT imported from
#: ``reload_coordinator_node`` — that module is ROS (``jugglebot`` node code)
#: and ``motion/`` may never import it; this is the ONE definition, and R4
#: re-points the coordinator's copies at it (tracked in the plan, not here).
RELEASE_CUP_Z_MM = 860.0
CATCH_CUP_Z_MM = 830.0

#: The cup-opening height a cycle settles at between skills. ``unified_cycle``
#: already derives this (the settle clamp every LANDING/SETTLE window aims
#: at) — imported rather than restated so the two can never drift apart.
REST_CUP_Z_MM = uc.SETTLE_CUP_Z_MM


def _vec3_mm(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (3,):
        raise ValueError('%s must be a 3-vector, got shape %s'
                          % (name, np.shape(value)))
    if not np.all(np.isfinite(arr)):
        raise ValueError('%s must be finite, got %r' % (name, arr.tolist()))
    return arr


@dataclasses.dataclass(frozen=True)
class Site:
    """A cup-opening position: xy PLATFORM frame, z GLOBAL — the ``CycleGoals``
    convention (see ``unified_cycle``'s module docstring, "FRAMES AND UNITS").

    ``cup_mm`` carries the site's CATCH z by convention (what
    :func:`columns_sites` builds); the three ``*_site_mm`` helpers below
    re-target z for a THROW, a CATCH or a REST at this site's xy without a
    second copy of any of the module's three z constants.
    """

    name: str
    cup_mm: np.ndarray

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name:
            raise ValueError('name must be a non-empty string, got %r'
                              % (self.name,))
        object.__setattr__(self, 'cup_mm', _vec3_mm(self.cup_mm, 'cup_mm'))

    def throw_site_mm(self, release_z_mm: float = RELEASE_CUP_Z_MM
                       ) -> np.ndarray:
        """This site's xy at a THROW's release z."""
        return np.array([self.cup_mm[0], self.cup_mm[1], float(release_z_mm)])

    def catch_site_mm(self, catch_z_mm: float = CATCH_CUP_Z_MM) -> np.ndarray:
        """This site's xy at a CATCH's aim z."""
        return np.array([self.cup_mm[0], self.cup_mm[1], float(catch_z_mm)])

    def rest_site_mm(self, rest_z_mm: float = REST_CUP_Z_MM) -> np.ndarray:
        """This site's xy at the settle z."""
        return np.array([self.cup_mm[0], self.cup_mm[1], float(rest_z_mm)])


def columns_sites(separation_mm: float) -> Tuple[Site, Site]:
    """The two columns-pattern sites, ``separation_mm`` apart, straddling x=0.

    Plan § 1.2 / § 2.4: ball A lives at ``P1``, ball B at ``P2`` — a columns
    pattern is two SELF-tosses run out of phase, so ``separation_mm`` is the xy
    gap between the two hands' cups, not a throw's lateral travel (a columns
    throw's landing target is its own site — see ``schedule.compile_columns``).
    Symmetric about the platform origin so a session is centred rather than
    biased to one side.
    """
    if not float(separation_mm) > 0.0:
        raise ValueError('separation_mm must be > 0, got %r' % (separation_mm,))
    half = float(separation_mm) / 2.0
    z = CATCH_CUP_Z_MM
    return (Site('P1', np.array([-half, 0.0, z])),
            Site('P2', np.array([half, 0.0, z])))
