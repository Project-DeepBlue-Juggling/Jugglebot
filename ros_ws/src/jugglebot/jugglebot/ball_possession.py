"""Ball-possession verdicts — the single place a "did we catch it?" claim is minted.

Pure Python (no ROS2, no config imports) so the whole verdict surface is testable
without a node and without a bag. The ROS wrapper is
``reload_coordinator_node._possession_confirmed``.

Normative contract: ``ros_ws/docs/ball_possession_contract.md`` (**C-POSSESS-1**).
Read it before changing anything here — the bound below is not a tunable.

WHY THIS MODULE EXISTS
----------------------
Until 2026-07-28 the coordinator judged a tracker ``CAUGHT`` with a single
``AND`` of two spatial bounds — ``xy <= 200 mm AND |z - catch_z| <= 150 mm`` —
and the z half was **structurally impossible to satisfy**. Measured over the
whole 2026-07-27 sitting (bag ``2026-07-27_15-39-38``, 17 self-tosses, all of
them real catches the operator watched): ``xy`` error **0.30 - 3.88 mm** against
``z`` error **305 - 1007 mm**. Every one failed. `success` was therefore False by
construction on every ball op the machine has ever run.

The z half could never have worked, and the reason is in the tracker's own
definition of CAUGHT (``tracking/matcher.py::_check_lifecycle``):

    a ball is declared CAUGHT *because its mocap marker disappeared* around the
    predicted landing time.

So at the CAUGHT instant ``BallState.position`` is not an observation — it is the
Kalman filter's **dead-reckoned free-fall extrapolation** from the last real
sighting, and it is then frozen for the ~2 s the terminal track is retained
(measured: positional variance **exactly 0.000 mm** after the first CAUGHT sample,
across all 60+ CAUGHT tracks in that bag). Under free fall the extrapolation error
splits by axis:

    z  error  ~  |v_z|·dt + g·dt²/2       -> 305-1007 mm at the measured dt
    xy error  ~  |v_xy|·dt                -> 0.30-3.88 mm for a vertical toss

which is exactly the measured split. **z carries the artefact; xy carries the
information.** A bound on z is a bound on how long the tracker had already lost
sight of the ball — i.e. on the very thing CAUGHT means.

THE CLASS OF FAILURE, not the one bound
---------------------------------------
The defect was not "150 was too small". It was *a plausibility bound applied to an
observable whose error model was never written down*. Had the model above been
stated, no z bound would have been written. C-POSSESS-1 therefore requires a
source to declare, per observable, what it measures and what error it carries —
and to report ``UNKNOWN`` for any claim it cannot observe, never ``True``.

TWO INDEPENDENT CLAIMS
----------------------
A possession verdict answers two questions that are NOT the same question:

  ARRIVAL    the ball reached the cup;
  RETENTION  it was still there afterwards (it did not bounce out).

``TrackerArrivalSource`` can answer ARRIVAL and **cannot** answer RETENTION, and
that is a measured fact rather than a modelling convenience: the track freezes at
CAUGHT (above), it is pruned ~2 s later, and a bounce-out's descent to the floor
raises no successor track — checked for all three of the 2026-07-27 bounce-outs
(balls 6, 9, 11): **zero** new ``/balls`` tracks appear between each one's CAUGHT
verdict and its mocap-measured floor arrival ~0.4-0.6 s later. So the source
reports ``RETENTION_UNKNOWN`` and says so in the verdict, in the log line, and here.

The ball-in-cup hand sensor (installed 2026-07-28) is the source that CAN answer
RETENTION. It slots in as the PRIMARY source through :class:`PossessionSource`
without touching any caller — see the contract's "Adding a source" section. No
sensor code exists yet; do not add any here speculatively.
"""

from __future__ import annotations

import math
from typing import NamedTuple, Sequence, Tuple

# ── Retention states (C-POSSESS-1 §2) ─────────────────────────────────────────
# UNKNOWN is NOT a soft "probably fine": it is the mandatory answer for a source
# that cannot observe retention, and the contract forbids collapsing it to True.
RETENTION_CONFIRMED = 'CONFIRMED'
RETENTION_REJECTED = 'REJECTED'
RETENTION_UNKNOWN = 'UNKNOWN'

# ── Source identifiers (carried in every verdict, so a log line names its author)
SOURCE_TRACKER_ARRIVAL = 'tracker_arrival'
# Reserved for the ball-in-cup hand sensor. Declared here (and only here) so the
# name cannot be invented twice when that phase lands; no implementation exists.
SOURCE_HAND_BALL_SENSOR = 'hand_ball_sensor'


class PossessionVerdict(NamedTuple):
    """One source's answer about one ball.

    ``arrival_err_mm`` and ``plane_drop_mm`` are always populated so a rejected
    verdict is as diagnosable as an accepted one.

    ``plane_drop_mm`` (catch-plane z minus the estimate's z, so positive = the
    estimate has coasted *below* the plane) is **REPORT-ONLY**. It is the
    dead-reckoning depth described in the module docstring, and gating on it is
    precisely the defect C-POSSESS-1 exists to prevent. It is kept because it is
    a genuinely useful diagnostic — it says how much of this verdict is
    extrapolation — but no code may branch on it.
    """
    source: str
    arrival_ok: bool
    retention: str
    arrival_err_mm: float
    plane_drop_mm: float
    reason: str

    @property
    def confirmed(self) -> bool:
        """The single boolean callers act on: the ball arrived, and nothing
        observed contradicts it still being there.

        ``RETENTION_UNKNOWN`` does not block confirmation — a source that cannot
        see retention must not be able to veto a catch it did observe arriving.
        ``RETENTION_REJECTED`` does: that is a source that positively saw the
        ball leave.
        """
        return bool(self.arrival_ok) and self.retention != RETENTION_REJECTED


def lateral_miss_mm(point_mm: Sequence[float], ref_point_mm: Sequence[float]) -> float:
    """Horizontal (xy) distance between an estimate and a reference point, mm.

    The single formula for "how far off was it" — the possession bound and the
    reported ``catch_error_mm`` must never be two different computations.
    """
    dx = float(point_mm[0]) - float(ref_point_mm[0])
    dy = float(point_mm[1]) - float(ref_point_mm[1])
    return float(math.hypot(dx, dy))


class PossessionSource:
    """Protocol for a possession source (C-POSSESS-1 §3).

    A source is anything that can look at the world and answer the two claims.
    Implementations must:

      - set ``name`` to one of the ``SOURCE_*`` constants;
      - return a fully-populated :class:`PossessionVerdict` from :meth:`judge`;
      - report ``RETENTION_UNKNOWN`` for retention unless they positively
        observe it. Returning ``RETENTION_CONFIRMED`` is a claim that the ball
        was seen to still be held.

    Kept as a documentation base class rather than a ``typing.Protocol``: this is
    the one place the two-claim obligation is written next to the signature, and
    the ``NotImplementedError`` turns a half-written source into an immediate,
    named failure instead of an ``AttributeError`` three frames away. Inheriting
    is NOT required — the coordinator's seam is duck-typed, so a source only needs
    ``name`` and ``judge`` (which is what
    ``test_possession_source_is_pluggable_at_one_seam`` substitutes).
    """

    name = ''

    def judge(self, ball_xyz_mm: Sequence[float],
              ref_point_mm: Sequence[float]) -> PossessionVerdict:
        raise NotImplementedError


class TrackerArrivalSource(PossessionSource):
    """Possession from the ball tracker's ``CAUGHT`` estimate — ARRIVAL only.

    ``arrival_tol_mm`` is the horizontal radius within which the CAUGHT estimate
    must sit for the ball to have plausibly entered the catching structure. It is
    supplied by the caller (the node sources it from the machine geometry) so this
    module stays free of config imports.

    WHAT IT REJECTS, measured on ``~/Desktop/rosbags/2026-07-27_15-39-38``:
    every ``destination='jugglebot'`` reload track in that session is a **split
    track** — its Kalman filter is fed by the WRONG marker, so its CAUGHT estimate
    lands **204.9 - 752.9 mm** from the catch point while a *separate*, untagged
    track carries the real ball. Thirteen of those eighteen were real catches and
    three were the eye-confirmed bounce-outs, and the tracker's estimate is
    **indistinguishable** between them (drops 702.6/721.1/726.4 mm; catches
    204.9-752.9 mm). No bound on that data can be right, and the honest verdict
    for all of them is "no evidence" — which is what this source returns.

    (The mis-association mechanism — NOT "the filter got no measurements", which
    is measurably false: all 18 tagged tracks reach ``tracking=CONFIRMED``, which
    ``matcher.py:344-347`` sets only inside ``kf.update()`` — is written up in the
    contract's § 4. Corrected 2026-07-28 during finalize; the original wording
    would have sent the tracker investigation after the wrong defect.)

    WHAT IT ACCEPTS: the 17 self-tosses of the same session, whose tracks *are*
    measurement-driven and whose CAUGHT estimates sit **0.30 - 3.88 mm** from the
    nominated landing point.

    So the separation the bound has to survive is 3.88 mm on one side and 204.9 mm
    on the other — a 53x gap.

    THE BOUND'S DIRECTIONAL RISK — **measured, and larger than first written.**
    The dead-reckoning lateral drift is ``|v_xy|·dt``. A self-toss arrives with
    ``|v_xy| ~ 0`` (measured 0.8-7.0 mm/s at CAUGHT, 17/17), so its drift is
    negligible and 70 mm clears the worst real self-toss (3.88 mm) by **18x**.
    A **reload** ball does not: the Ball Butler sits ~0.95 m from the catch point
    over a ~0.9 s flight, so it arrives at roughly 1 m/s laterally.

    The original wording called that risk "derived, not measured — no reload track
    in the capture carries real velocity to measure it from". **That was wrong, and
    the capture does measure it.** The reload era's *untagged* tracks — the ones
    carrying the real marker, i.e. exactly how a tagged track will behave once the
    mis-association is fixed — reach CAUGHT at

        ids 57 / 33 / 69 / 15  ->  34.4 / 34.9 / 37.6 / **68.4** mm

    so a genuine reload catch already sits **1.6 mm inside** this bound: a
    **1.02x** margin, the identical shape of the defect C-POSSESS-1 was written to
    close (the old 200 mm bound's 1.02x margin against the 204.9 mm corrupt floor).
    A second, independent term pushes the same way: ``JB_TRAJ_CATCH_REACH_ENVELOPE_MM``
    is **80 mm**, and the reference point here does NOT move with the reach, so a
    catch the platform reached for can read up to the full envelope even with a
    perfect estimate.

    **The bound is therefore knowingly under-sized for the reload path, and is not
    re-tuned here.** Re-tuning it needs data from a *fixed* tracker, which does not
    exist yet; sizing it against the 204.9 mm corrupt floor — the only number
    available today — would be sizing a bound against a bug, which is the same
    error one level down. The obligation is recorded in the contract's § 4 and
    pinned by ``test_the_measured_reload_band_sits_against_the_bound``. It is inert
    today: every tagged reload track is refused at 204.9-752.9 mm regardless of
    whether this bound is 70 or 200, so nothing observable changes until the
    tracker phase lands — which is exactly the phase that must re-derive it.

    A false MISSED is the conservative direction and cannot cause motion. Note the
    alternative observable was measured and **rejected**: judging on
    ``landing_position.xy`` instead of ``position.xy`` is *worse* for these tracks
    (ids 33 and 69 read 140.0 and 120.3 mm there, against 34.9 and 37.6 mm here) —
    the ballistic projection amplifies the velocity error it was meant to dodge.
    """

    name = SOURCE_TRACKER_ARRIVAL

    def __init__(self, arrival_tol_mm: float):
        self.arrival_tol_mm = float(arrival_tol_mm)

    def judge(self, ball_xyz_mm: Sequence[float],
              ref_point_mm: Sequence[float]) -> PossessionVerdict:
        err = lateral_miss_mm(ball_xyz_mm, ref_point_mm)
        drop = float(ref_point_mm[2]) - float(ball_xyz_mm[2])
        ok = err <= self.arrival_tol_mm
        if ok:
            reason = 'ARRIVAL_OK'
        else:
            reason = 'ARRIVAL_FAR'
        return PossessionVerdict(
            source=self.name,
            arrival_ok=bool(ok),
            # NEVER CONFIRMED. The tracker freezes the track at CAUGHT and prunes
            # it ~2 s later, so it has no post-arrival observation to offer. A
            # future edit that returns CONFIRMED here is claiming an observation
            # this source does not make — pinned by
            # tests/ros/test_ball_possession.py::
            # test_tracker_source_never_claims_retention.
            retention=RETENTION_UNKNOWN,
            arrival_err_mm=err,
            plane_drop_mm=drop,
            reason=reason)


def describe(verdict: PossessionVerdict, tol_mm: float) -> Tuple[str, str]:
    """-> (severity, one-line human summary) for the coordinator's log.

    Lives here, next to the semantics, so the operator-facing wording and the
    verdict cannot drift. Severity is 'info' for both outcomes: a refused CAUGHT
    is the *expected* reading of a corrupt split track and fires on essentially
    every reload, so logging it as a warning would make a healthy sequence look
    broken (the reason the pre-2026-07-28 line was INFO too).
    """
    if verdict.arrival_ok:
        return 'info', (
            f'possession CONFIRMED by {verdict.source} — arrival '
            f'{verdict.arrival_err_mm:.1f} mm <= {tol_mm:.0f} mm; retention '
            f'{verdict.retention} (this source cannot observe it — '
            f'ros_ws/docs/ball_possession_contract.md)')
    return 'info', (
        f'possession REFUSED by {verdict.source} — arrival '
        f'{verdict.arrival_err_mm:.0f} mm > {tol_mm:.0f} mm from the catch point '
        f'(estimate sits {verdict.plane_drop_mm:.0f} mm below the catch plane: a '
        f'CAUGHT estimate is a dead-reckoned free-fall extrapolation, REPORT-only '
        f'— see ros_ws/docs/ball_possession_contract.md). Not counted')
