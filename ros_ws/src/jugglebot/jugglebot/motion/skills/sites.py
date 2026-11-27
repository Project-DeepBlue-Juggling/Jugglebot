"""Cup-opening sites the skill stack throws to and catches at (plan § 2.2).

A :class:`Site` names one xy location on the platform and knows the three cup
heights a skill visits it at: a THROW releases at :data:`RELEASE_CUP_Z_MM`, a
CATCH is aimed at :data:`CATCH_CUP_Z_MM`, and a REST settles at
:data:`REST_CUP_Z_MM`. Pure Python + numpy, no ROS imports (``motion/`` rule).
"""

from __future__ import annotations

import dataclasses
import math
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

#: CATCH HIGH history. 830 mm until 2026-10-05; CATCH HIGH (owner decision,
#: 2026-10-05) raised it to 930; owner decision 2026-10-06 returned it to
#: **830**, restoring the original contact speed -- see the two paragraphs
#: below in order.
#:
#: 2026-10-05 (830 -> 930). ``scratchpad/s6/catch_height_probe.py`` (real
#: chain, LAUNCH+STEADYx3, apex 0.95, sep 125, dwell 0.27, leg 350/5000/200000,
#: hand 3900) measured, by catch z:
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
#:
#: 2026-10-06 (930 -> 830, owner decision). The hand brakes to the top of its
#: stroke (~998 mm cup z) after every throw at EITHER plane; the plane only
#: sets the empty drop before contact and, through the C-CUP-2 contact floor
#: above, the cup speed at contact: 830 = 169 mm drop, cup -1.61 m/s,
#: ball-cup relative impact ~2.7 m/s, hold at the top 0.08-0.11 s (measured
#: 2026-10-04 bag); 930 = 68 mm drop, cup -0.96 m/s, relative impact
#: ~3.2 m/s, hold 0.16-0.19 s (2026-10-06 bag). Seen at 930 on 2026-10-06: a
#: slow with-ball carry-down the owner read as a lazy hand, a cup-sensor seat
#: at +0.23..+0.29 s after the landing (around the re-release), and the
#: columns apex ceiling at 0.950 m (was 1.047 at 830; sitting 5 entry §
#: "What catch high costs"). Returning to 830 restores the original contact
#: speed; the hold at the top is kept by the stroke itself, not by the plane.
#: No learner memory migration is needed -- rows are release-relative since
#: ``schedule.apex_from_crossing``; the 10-05 migration was a definition
#: change (``tools/migrate_memory_catch_plane.py`` docstring).
CATCH_CUP_Z_MM = 830.0

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


def polygon_sites(n: int, edge_mm: float) -> Tuple[Site, ...]:
    """The ``polygon_walk`` Sites (JP-5, IDEA-9): a regular ``n``-gon of side
    ``edge_mm`` centred on the Platform centre, ``n >= 3``, in walk order
    (``P1`` .. ``Pn``, counter-clockwise).  ``P1`` is at (-R, 0), where
    :func:`columns_sites` puts its ``P1`` (nearest Ball Butler, the Feed Site),
    with R = edge / (2 sin(pi / n)); a square of edge 100 mm has R = 70.7 mm.
    The ball is thrown ``P(k)`` -> ``P(k+1)``, so ``edge_mm`` is the Throw's
    lateral travel (the Hop's ``separation_mm`` is the same quantity)."""
    n = int(n)
    if n < 3:
        raise ValueError('polygon_n must be >= 3, got %r' % (n,))
    if not float(edge_mm) > 0.0:
        raise ValueError('edge_mm (separation_mm) must be > 0, got %r'
                         % (edge_mm,))
    radius = float(edge_mm) / (2.0 * math.sin(math.pi / n))
    out = []
    for k in range(n):
        a = math.pi + 2.0 * math.pi * k / n
        out.append(Site('P%d' % (k + 1), np.array(
            [radius * math.cos(a), radius * math.sin(a), CATCH_CUP_Z_MM])))
    return tuple(out)


#: ``centre_orbit`` (JP-14, IDEA-14): the radius (mm) and the Sites-per-lap
#: count the compile accepts, both inclusive.  ORIGIN: the 2026-10-30 offline
#: probe (radius 95-105 mm, 6-12 steps) re-measured in the sim skills gate
#: and the nomination check 2026-11-25 (logbook
#: ``2026-11-25-jp14-centre-and-orbit-compile.md``).  Outside them the compile
#: REFUSES by name (:data:`ORBIT_RADIUS_OUT_OF_RANGE`,
#: :data:`ORBIT_N_OUT_OF_RANGE`); nothing is clamped.
ORBIT_RADIUS_RANGE_MM = (95.0, 105.0)
ORBIT_N_RANGE = (6, 12)
ORBIT_RADIUS_OUT_OF_RANGE = 'ORBIT_RADIUS_OUT_OF_RANGE'
ORBIT_N_OUT_OF_RANGE = 'ORBIT_N_OUT_OF_RANGE'
#: An ODD Throw count ends on Ball 1's Stop onto Ball 2 at an orbit Site, the
#: closest ball-to-ball approach of the Pattern (71.8 mm centre distance at
#: r 95 N 6, below the 74 mm ball diameter; owner decision 2026-11-25: refuse
#: it, never round it up). :class:`schedule.Pattern` refuses it by this name.
ORBIT_ODD_THROW_COUNT = 'ORBIT_ODD_THROW_COUNT'


def centre_orbit_sites(radius_mm: float, n: int) -> Tuple[Site, ...]:
    """The ``centre_orbit`` Sites (JP-14, IDEA-14): ``(C, O1, .., On)``.

    ``C`` is the Platform centre (0, 0), Ball 1's column.  ``O1`` .. ``On``
    are Ball 2's Sites, a regular ``n``-gon inscribed in a circle of
    ``radius_mm`` about ``C``, in lap order (counter-clockwise), ``O1`` at
    (-r, 0): where :func:`columns_sites` puts ``P1``, the Site nearest Ball
    Butler, so it is the Feed Site.  Every cup transit is C <-> Ok, exactly
    ``radius_mm`` (Columns at separation r); Ball 2's Throw Ok -> Ok+1 travels
    the chord ``2 r sin(pi / n)`` (76.5 mm at r 100, n 8).

    Raises ``ValueError`` naming :data:`ORBIT_RADIUS_OUT_OF_RANGE` or
    :data:`ORBIT_N_OUT_OF_RANGE` outside :data:`ORBIT_RADIUS_RANGE_MM` /
    :data:`ORBIT_N_RANGE` -- a refusal, never a clamp."""
    r = float(radius_mm)
    lo, hi = ORBIT_RADIUS_RANGE_MM
    if not (math.isfinite(r) and lo <= r <= hi):
        raise ValueError('%s: centre_orbit radius (separation_mm) %r mm is '
                         'outside the compiled range %g..%g mm'
                         % (ORBIT_RADIUS_OUT_OF_RANGE, radius_mm, lo, hi))
    if isinstance(n, bool) or int(n) != n:
        raise ValueError('%s: orbit_n must be an integer, got %r'
                         % (ORBIT_N_OUT_OF_RANGE, n))
    n = int(n)
    nlo, nhi = ORBIT_N_RANGE
    if not nlo <= n <= nhi:
        raise ValueError('%s: orbit_n %d Sites per lap is outside the '
                         'compiled range %d..%d'
                         % (ORBIT_N_OUT_OF_RANGE, n, nlo, nhi))
    z = CATCH_CUP_Z_MM
    out = [Site('C', np.array([0.0, 0.0, z]))]
    for k in range(n):
        a = math.pi + 2.0 * math.pi * k / n
        out.append(Site('O%d' % (k + 1), np.array(
            [r * math.cos(a), r * math.sin(a), z])))
    return tuple(out)


#: JP-11 rotating Columns (map IDEA-12): the Site pair turns about the Platform
#: origin by ``step_deg`` every Beat.  The step range is the ladder's top
#: (5 / 10 / 20 deg per Beat); outside ``(0, 20]`` is REFUSED by name, never
#: clamped.  ``0`` / ``None`` is plain Columns (the Pattern field's default).
ROTATING_STEP_RANGE_DEG = (0.0, 20.0)
ROTATING_STEP_OUT_OF_RANGE = 'ROTATING_STEP_OUT_OF_RANGE'
ROTATING_APEX_PAIR_UNEQUAL = 'ROTATING_APEX_PAIR_UNEQUAL'
ROTATING_PHANTOM_REFUSED = 'ROTATING_PHANTOM_REFUSED'
ROTATING_NEEDS_HOP_ENTRY = 'ROTATING_NEEDS_HOP_ENTRY'
ROTATING_NOT_A_PAIR = 'ROTATING_NOT_A_PAIR'
ROTATING_WITH_ORBIT = 'ROTATING_WITH_ORBIT'


def check_rotating_step(step_deg) -> float:
    """``step_deg`` as a float in ``(0, 20]``, else ``ValueError`` naming
    :data:`ROTATING_STEP_OUT_OF_RANGE` (a refusal, never a clamp)."""
    lo, hi = ROTATING_STEP_RANGE_DEG
    try:
        d = float(step_deg)
    except (TypeError, ValueError):
        d = float('nan')
    if isinstance(step_deg, bool) or not (math.isfinite(d) and lo < d <= hi):
        raise ValueError('%s: rotating Columns step %r deg per Beat is '
                         'outside (%g, %g]' % (ROTATING_STEP_OUT_OF_RANGE,
                                               step_deg, lo, hi))
    return d


def rotating_sites(pair: Tuple[Site, Site], step_deg: float,
                   n_throws: int) -> Tuple[Site, ...]:
    """The Sites ``S(0) .. S(n_throws + 1)`` of a rotating-Columns stream
    (JP-11).  ``pair = (sites[0], sites[1])`` is a Columns pair about the
    Platform origin (``sites[1]`` the Feed Site, ``columns_sites``' ``P1``,
    angle ``phi1``; ``sites[0]`` its antipode): Throw ``k`` leaves ``S(k)`` and
    is caught at ``S(k + 2)``, where it is thrown again.  ``S(k)`` sits at
    polar angle ``phi1 + (k - 1) * (step + 180) deg`` and radius ``r`` (half
    the pair separation): the pair turns ``step`` per Beat and the two balls
    stay on opposite sides.  ``S(1)`` is exactly ``sites[1]`` (the Feed Site at
    pair angle 0, Ball Butler's lane and the hop entry are Columns' own) and
    ``S(0)`` is exactly ``sites[0]`` (the unrotated partner Site; under the
    hop entry Throw 0 leaves ``sites[1]``, so ``S(0)`` is never a Catch Site).
    Chord of a Throw ``S(k) -> S(k + 2)``: ``2 r sin(step)``; transit
    ``S(k) -> S(k + 1)``: ``2 r cos(step / 2)``.  Names are unique per Site
    (``R<k>@<angle>``): the executor keys its outward unit and the stand-in on
    names."""
    d = check_rotating_step(step_deg)
    a, b = pair
    ra, rb = np.asarray(a.cup_mm, dtype=float), np.asarray(b.cup_mm, dtype=float)
    r = math.hypot(float(rb[0]), float(rb[1]))
    if (r <= 0.0 or float(ra[2]) != float(rb[2])
            or math.hypot(float(ra[0] + rb[0]), float(ra[1] + rb[1])) > 1e-6):
        raise ValueError('%s: the pair must be a Columns pair about the '
                         'Platform origin, got %s %s and %s %s'
                         % (ROTATING_NOT_A_PAIR, a.name, ra.tolist(),
                            b.name, rb.tolist()))
    phi1 = math.atan2(float(rb[1]), float(rb[0]))
    z = float(rb[2])
    out = [a, b]
    for k in range(2, int(n_throws) + 2):
        ang = phi1 + math.radians((k - 1) * (d + 180.0))
        out.append(Site('R%d@%g' % (k, round(math.degrees(ang) % 360.0, 3)),
                        np.array([r * math.cos(ang), r * math.sin(ang), z])))
    return tuple(out)


def columns_sites(separation_mm: float,
                  angle_deg: float = 0.0) -> Tuple[Site, Site]:
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

    **Roles are a property of the PATTERN, not of the site pair**
    (``schedule.OneBallPattern.roles``, two-ball-oval P1, 2026-10-18). The pair
    is returned as ``(sites[0], sites[1])`` and each pattern reads them its own
    way: the alternating patterns (hop, columns) use both as throw and catch
    sites in turn; the fixed-role OVAL (``roles='fixed'``, schedule label
    ``oval_1ball``) reads ``sites[0]`` = ``P1`` as the THROW site T (nearest
    Ball Butler, which feeds it) and ``sites[1]`` = ``P2`` as the CATCH site C:
    every throw leaves T for C and every catch is at C, the loaded cup carrying
    the ball C -> T inside the dwell.

    ``angle_deg`` (default 0.0, JP-10): rotate the pair about the origin; the
    Sites are then named ``P1@<angle>`` / ``P2@<angle>`` (see the body).
    """
    if not float(separation_mm) > 0.0:
        raise ValueError('separation_mm must be > 0, got %r' % (separation_mm,))
    half = float(separation_mm) / 2.0
    z = CATCH_CUP_Z_MM
    angle = float(angle_deg)
    if not math.isfinite(angle):
        raise ValueError('angle_deg must be finite, got %r' % (angle_deg,))
    if angle == 0.0:
        # Bit-identical to the pre-angle pair (no -0.0 from -half * sin 0).
        return (Site('P1', np.array([-half, 0.0, z])),
                Site('P2', np.array([half, 0.0, z])))
    # TURNED pair (JP-10, ``columns_turned``): the same pair rotated by
    # ``angle_deg`` about the Platform origin (+z), P1 at (-half cos, -half
    # sin), P2 at (+half cos, +half sin) -- 90 deg puts P1 at -y, P2 at +y.
    # The Sites carry DISTINCT names ('P1@45'): until 2026-11-06 the
    # Admissible box was keyed on the site-pair NAMES, so a turned cell
    # sharing 'P1' would collide with the straight cell in one YAML; the
    # distinct names still keep a turned pair from being mistaken for the
    # straight one by name-keyed lookups.
    rad = math.radians(angle)
    c, s = math.cos(rad), math.sin(rad)
    tag = '@%g' % (angle,)
    return (Site('P1' + tag, np.array([-half * c, -half * s, z])),
            Site('P2' + tag, np.array([half * c, half * s, z])))
