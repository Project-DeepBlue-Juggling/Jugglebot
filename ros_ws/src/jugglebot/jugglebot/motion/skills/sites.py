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

#: Cup-opening world z (mm) a THROW releases at — the sittings' geometry
#: (``reload_coordinator_node._UNIFIED_THROW_CUP_Z_MM``, that module since
#: deleted at R4 under tag ``fsm-final``) and the R2 sizing sweep confirming
#: it stays inside the cup box at the owner's R2 operating point (apex 0.9 m,
#: dwell 0.30 s, leg 300/5000/200000, hand acc 3500) — see ``tools/probes/
#: skills_sizing_sweep.py`` runs 2026-09-11/12, ``temp/probes/
#: skills_sizing_frontier2.md``. (``CATCH_CUP_Z_MM`` below shares this history
#: up to 2026-10-05, when CATCH HIGH moved it off this pair's original 830 mm
#: — see its own paragraph.) This is the ONE definition; ``motion/`` may never
#: import ROS node code to check against it.
RELEASE_CUP_Z_MM = 860.0

#: CATCH HIGH (owner decision, 2026-10-05): 830 -> 930. ``scratchpad/s6/
#: catch_height_probe.py`` (real chain, LAUNCH+STEADYx3, apex 0.95, sep 125,
#: dwell 0.27, leg 350/5000/200000, hand 3900) measured, by catch z:
#:
#: | catch z | empty drop before contact | cup v at contact | with-ball stroke | leg a / j      | hand a |
#: | 830     | 169 mm                     | -1.61 m/s         | 137 mm            | 4133 / 159920  | 3528   |
#: | 930     | 68 mm                      | -0.96 m/s         | 237 mm            | 4073 / 157709  | 3509   |
#: | 950     | 48 mm                       | -0.80 m/s         | 251 mm (bottom lifts to 0.49 rev) | 4074 / 154400 | 3513 |
#:
#: All feasible 830-970; 930 sits just below the knee where the DWELL, not the
#: stroke, starts limiting depth -- i.e. the hand waits near the top and
#: carries the ball down through the main stroke rather than diving to meet
#: it. The C-CUP-2 contact floor (<= 0.7 g downward in the 0.125 s before
#: touch-down) is UNCHANGED; it is why the empty drop is ~146-169 mm at 830
#: (reaching 1.6 m/s under 0.7 g takes that much descent) and shrinks at 930.
#:
#: ``RELEASE_CUP_Z_MM`` stays 860 -- raising it was tried and refused on the
#: same probe: release 880 needs 4416 rev/s^2 of post-release hand braking
#: against the 3900 cap; 900 needs 4907; 920 is QP-infeasible. The climb
#: after release is the hand braking from ~4.3 m/s, forced by physics, not a
#: tuning knob.
CATCH_CUP_Z_MM = 930.0

#: The cup-opening height a cycle settles at between skills. ``unified_cycle``
#: already derives this (the settle clamp every LANDING/SETTLE window aims
#: at) — imported rather than restated so the two can never drift apart.
REST_CUP_Z_MM = uc.SETTLE_CUP_Z_MM

#: THE SCHEDULE'S ONE HAND HOME (rev): the slider position whose LEVEL
#: realisation puts the cup opening at :data:`REST_CUP_Z_MM` — 0.3071 rev
#: through ``unified_cycle.hand_rev_for_cup_z``, the module's own export of
#: that map (never a fourth spelling of it; see that function's docstring).
#:
#: This is where every REST in a schedule leaves the hand, so it is also where
#: the NEXT schedule's opening REST must start from — the one reference the
#: opening REST is sized against (``schedule.floor_lift_s``). It is NOT the
#: bridge's ACTIVATE park (``JB_OP_HAND_ACTIVATE_POSITION_REV`` = 0.0 rev,
#: what ``/recover`` parks to): the two are 0.307 rev apart, and on 2026-09-18
#: measuring a schedule's start against the PARK refused every attempt after
#: the first (``hand park REFUSED — the hand is at +0.3063 rev``, which is
#: exactly where the previous attempt's REST correctly left it). A hand at the
#: park is simply 0.307 rev from home and gets homed like any other
#: displacement — no special case anywhere.
REST_HAND_REV = uc.hand_rev_for_cup_z(REST_CUP_Z_MM)


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

    Plan § 1.2 / § 2.4: a columns pattern is two SELF-tosses run out of
    phase, so ``separation_mm`` is the xy gap between the two hands' cups,
    not a throw's lateral travel (a columns throw's landing target is its
    own site — see ``schedule.compile_columns``). The pattern itself is
    symmetric — a plain vertical run names ball A at ``P1``, ball B at
    ``P2`` — but a Ball-Butler-FED start (``SkillNode._run_columns``) holds
    A at ``P2`` and feeds ``P1`` instead: ``P1`` is the site NEAREST Ball
    Butler along its feed bearing, so the incoming ball's descent never
    crosses the held ball's own column (2026-10-02 sitting 2 — the flown
    P1-holds/P2-feeds layout let ball B's +x,+y transit pass through ball
    A's column ~0.09 s before landing, merging in all 4 of 4 attempts; see
    ``logbook/2026-10-02-skill-stack-r5-sitting-2.md``).
    Symmetric about the platform origin so a session is centred rather than
    biased to one side.
    """
    if not float(separation_mm) > 0.0:
        raise ValueError('separation_mm must be > 0, got %r' % (separation_mm,))
    half = float(separation_mm) / 2.0
    z = CATCH_CUP_Z_MM
    return (Site('P1', np.array([-half, 0.0, z])),
            Site('P2', np.array([half, 0.0, z])))
