"""Ball-possession verdicts — the single place a "did we catch it?" claim is minted.

Pure Python (no ROS2, no config imports) so the whole verdict surface is testable
without a node and without a bag. The ROS wrapper is
``reload_coordinator_node._possession_observed`` (TICK-driven since 2026-08-26).

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

SENSOR-ONLY SINCE 2026-08-26 (owner decision D1)
------------------------------------------------
:func:`merge_possession` no longer falls back to the tracker for ARRIVAL. The
tracker is consulted for **nothing** a possession verdict rests on; it supplies
``arrival_err_mm`` / ``plane_drop_mm`` (report-only) and keeps every one of its
other roles (landing announcements, aim/ILC channels, ball tracking).

The evidence is bag ``2026-08-26_14-25-16``, 27 self-toss cycles that produced a
cup verdict. The cup sensor called **31/31** outcomes correctly (27 here plus the
four no-release/no-ball rows — 2x ``ABORTED_CANT_MAKE_RELEASE``, 2x
``REJECTED_NO_BALL``). The tracker-primary path — the FSMs only *asked*
the possession question on a tracker ``CAUGHT``, so the tracker could veto by
silence and mint by speaking — scored **11 CAUGHT / 16 MISSED** against the cup's
**23 / 4**:

  * **15 false MISSED.** All genuine catches; cup edges +0.143 … +0.303 s past the
    scheduled landing, every one inside the 0.56 s confirm window. Twelve had no
    confirmed tracker track at ALL (a mocap coverage hole — the tracker cannot
    veto by silence if it is never asked), three had a tracker ``CAUGHT`` that
    arrived 0.62 / 0.615 / 0.83 s past the landing, i.e. AFTER the deadline.
  * **3 false CAUGHT.** The tracker minted, the cup was empty. One of them drove a
    phantom reload.

Each false MISSED also charged the next cycle
``toss_session.DEFAULT_SESSION_MISS_CLEANUP_S``, which is what turned an even
cadence into the operator's observed irregular spacing.

``TrackerArrivalSource`` is therefore now a REPORTING source only. It is kept
(not deleted) because ``arrival_err_mm`` is the catch-accuracy number the hardware
runbooks score and the cup cannot supply it, and because a future tracker phase
that fixes the split-track mis-association may earn a corroboration role back —
which is a decision to re-take with data, not a fallback to leave armed.

``TrackerArrivalSource`` can answer ARRIVAL and **cannot** answer RETENTION, and
that is a measured fact rather than a modelling convenience: the track freezes at
CAUGHT (above), it is pruned ~2 s later, and a bounce-out's descent to the floor
raises no successor track — checked for all three of the 2026-07-27 bounce-outs
(balls 6, 9, 11): **zero** new ``/balls`` tracks appear between each one's CAUGHT
verdict and its mocap-measured floor arrival ~0.4-0.6 s later. So the source
reports ``RETENTION_UNKNOWN`` and says so in the verdict, in the log line, and here.

The ball-in-cup hand sensor (installed 2026-07-28) is the source that CAN answer
RETENTION, and as of 2026-08-10 it is implemented here as
:class:`HandBallSensorSource` — the **PRIMARY** source. It is a *tick-driven*
source (C-POSSESS-1 § 3.2): it answers from the cup itself and needs no ball, so
it cannot use the ball-shaped ``judge`` signature. ``TrackerArrivalSource`` is
demoted to the arrival CORROBORATOR and is still the only supplier of
``arrival_err_mm`` — the catch-accuracy number the hardware runbooks score, which
the sensor cannot supply. :func:`merge_possession` is the one place the two are
combined.

WHY ARRIVAL IS TRI-STATE AS OF 2026-08-10
-----------------------------------------
The verdict originally carried ``arrival_ok: bool``, which quietly violated the
contract it enforces: C-POSSESS-1 § 2 requires a source to report ``UNKNOWN`` for
any part it does not positively observe, and a bare bool forces "I cannot see"
and "I looked and it did not arrive" onto the same value. That is exactly the
"bound without an error model" defect one level down, and the sensor is the first
source that can genuinely be blind (a stale reply, a dead poller, a boot before
the first TxSdo). ``arrival`` is now the tri-state truth and ``arrival_ok`` is a
derived PROPERTY, so no source can set the two inconsistently.
"""

from __future__ import annotations

import collections
import math
import threading
from typing import (Any, Deque, Dict, Iterable, List, NamedTuple, Optional,
                    Sequence, Tuple)

# ── Retention states (C-POSSESS-1 §2) ─────────────────────────────────────────
# UNKNOWN is NOT a soft "probably fine": it is the mandatory answer for a source
# that cannot observe retention, and the contract forbids collapsing it to True.
RETENTION_CONFIRMED = 'CONFIRMED'
RETENTION_REJECTED = 'REJECTED'
RETENTION_UNKNOWN = 'UNKNOWN'

# ── Arrival states (C-POSSESS-1 §2, made tri-state 2026-08-10) ────────────────
# CONFIRMED: positively observed to arrive. REJECTED: positively observed NOT to
# (an estimate outside the bound; a sensor window that closed with no rise).
# UNKNOWN: the source could not look — never collapses to either of the others.
ARRIVAL_CONFIRMED = 'CONFIRMED'
ARRIVAL_REJECTED = 'REJECTED'
ARRIVAL_UNKNOWN = 'UNKNOWN'

# ── The two UNKNOWNs, which are NOT the same thing (added 2026-08-26, D1) ─────
# `ARRIVAL_UNKNOWN` covers two states a caller must be able to tell apart once the
# sensor is the SOLE source:
#
#   "I could not look"     — SENSOR_BLIND / SENSOR_NO_LANDING /
#                            SENSOR_BAND_CLAMPED. Nothing watched the window
#                            through. A consumer must REFUSE and say so.
#   "I am still looking"   — SENSOR_WINDOW_OPEN. The window has simply not closed.
#                            A consumer that has its OWN deadline (both FSMs
#                            terminalise at landing + CATCH_CONFIRM_WINDOW_S,
#                            which IS ARRIVAL_BAND_MAX_S) has watched the whole
#                            measured band by then, so "no rise yet" is a real
#                            MISS there and not blindness.
#
# SENSOR_BAND_CLAMPED is a could-not-look, not a still-looking (audit fix,
# 2026-08-26): the schedule CLOSED the window inside the ball's measured arrival
# band, so the source stopped watching before the evidence ran out. That it was a
# CADENCE number rather than a dead poller that closed the eye changes who to
# blame, not whether anything saw the ball — and `_arrival_state`'s own normative
# comment already says it out loud ("this is the difference between 'the ball
# missed' and 'we looked away'. Say the second one out loud").
#
# Keyed on the REASON rather than on a new tri-state member on purpose: the states
# are already distinguished, and a fourth ARRIVAL_* would force every existing
# `arrival == ARRIVAL_UNKNOWN` comparison in the tree to be re-audited for which
# of the two it meant.
BLIND_REASONS = frozenset({'SENSOR_BLIND', 'SENSOR_NO_LANDING',
                           'SENSOR_BAND_CLAMPED'})

# ── Live cup evidence (the ball-evidence PRECONDITION, distinct from ARRIVAL) ──
# ARRIVAL answers "did a ball come in around the predicted landing"; EVIDENCE
# answers "is a ball in the cup right now". The toss's CHECKING gate needs the
# second one and nothing else: it runs before any ball flies.
EVIDENCE_SEATED = 'SEATED'
EVIDENCE_EMPTY = 'EMPTY'
EVIDENCE_UNKNOWN = 'UNKNOWN'

# ── The measured sensor-band constants (C-POSSESS-1 § 3.4) ────────────────────
# These live HERE, in the contract's enforcement module, because three separate
# consumers size windows off them and a second copy is how two windows that must
# ABUT start to overlap: `toss_record` (the offline label), `toss_sequencer` (the
# FSM's MISSED deadline) and this module (the live verdict).
#
#: How far BEFORE an announced release a held->empty edge is still OUR throw.
#: Measured on 2026-08-10_16-30-44: across 32 self-tosses the RAW held->empty
#: edge lands at +148..+212 ms AFTER the announced throw_time (median +172), and
#: the mocap backcast of the same tosses puts the physical release 4.6 ms EARLY.
#: So 0.30 s is headroom for the sensor channel's own lag plus the one fault
#: direction the edges cannot resolve (an early release).
#:
#: It is used at BOTH ends of the same instant, which is the point of naming it
#: once: `release_t - RELEASE_GUARD_S` OPENS the next toss's departure search
#: (`SensorWindows.departure_lead_s`, this same constant — `DEPARTURE_LEAD_S`
#: was an alias of it and was dropped at the 2026-09-13 toss_record merge, see
#: below) and simultaneously CLOSES this toss's retention horizon
#: (:meth:`HandBallSensorSource.observe`).
#: The two windows therefore abut exactly and can neither overlap nor leave a
#: gap, at ANY dwell — which is what stops a legitimate throw reading as a
#: bounce-out once the dwell drops under the retention window (census D1).
RELEASE_GUARD_S = 0.30
#: The measured post-landing arrival band CEILING — the LATEST empty->held edge
#: observed on a real catch. **+554.7 ms** over 33 announced catches across four
#: post-FW-14 bags (2026-08-18_18-42-19, 2026-08-20_21-51-39,
#: 2026-08-21_10-11-42, 2026-08-23_19-14-54; median +183.9, earliest +87.6,
#: nothing before landing); ceiled to the next 10 ms so the constant is a bound
#: rather than a datum — the same sizing the retired 0.80 got from its +798 ms.
#:
#: RE-MEASURED AND APPLIED 2026-08-24, closing the post-FW-14 re-measure this
#: comment carried as PENDING for two weeks
#: (`tests/hardware/session_cadence_ladder.md` § 3.1;
#: `logbook/2026-08-24-arrival-band-remeasure.md`). The retired band was
#: +137..+798 ms, median +399, n=35 over three 2026-08-10 bags, captured while
#: the can-bridge uptime-dependent dispatch shift was +54..+133 ms. FW 14 cut
#: that to 10-20 ms (`logbook/2026-08-15-fw14-validated-arc-closed.md`) and the
#: band collapsed with it: the CEILING by 243 ms, the MEDIAN by 215 ms.
#:
#: The 2026-08-23 report-only reading this supersedes (+46.5..+267.5 ms, n=15,
#: ONE dwell) was right to refuse to ship — its own text said 15 self-tosses at
#: one dwell is thinner than the 35 the 0.80 was cut from. The corpus that DID
#: ship is four bags, two bridge FW versions, and eight distinct flight times
#: from 0.549 s to 1.069 s, n=33 CAUGHT (34 rows carry the pair; the 34th is a
#: rimshot — see ARRIVAL_BAND_MIN_S below, where it matters).
#:
#: ⚠ The offset is FLIGHT-DEPENDENT — r = 0.49 over the corpus, ~320 ms of
#: offset per second of flight — and BOTH rows above +300 ms sit at flights
#: (0.903 s, 1.069 s) LONGER than any published ladder rung. Inside the ladder's
#: own envelope (flight <= 0.798 s) the observed ceiling is +271 ms, so 0.56
#: leaves >2x headroom everywhere the cadence census actually operates. It is
#: sized on the WHOLE corpus regardless, because a bound that only holds inside
#: the envelope is not a bound. What remains uncovered is extrapolation ABOVE
#: the corpus: the C-HAND-3 flight ceiling is 1.1485 s, the longest flight
#: measured here is 1.069 s (n=1), and the fitted slope would put ~+580 ms
#: there. **More samples at long flights is the one thing that can move this
#: number again** — not another sitting at the ladder's own flights.
ARRIVAL_BAND_MAX_S = 0.56
#: The same band's FLOOR — the EARLIEST empty->held edge observed on a real
#: catch: **+87.6 ms** past the announced landing, over the same 33 catches
#: (nothing before landing at all).  It is the twin of the ceiling above and it
#: lives beside it for the same reason: the ceiling sizes how long a verdict may
#: take to ARRIVE, this sizes how soon one can EXIST, and one re-measure moves
#: both from one place.  Deliberately NOT rounded up: 0.087 is the datum floored
#: to the millisecond so it stays a true lower bound, and any tick allowance
#: belongs in the consumer.  Retired value 0.137 (n=35, 2026-08-10).
#:
#: Its consumer is the session's dwell margin — the landing -> next-cycle-start
#: handoff (``toss_session.DEFAULT_SESSION_DWELL_MARGIN_S``), which was sized on
#: the mocap tracker's CAUGHT verdict until 2026-08-22 and is now sized on this,
#: because possession became sensor-PRIMARY on 2026-08-10 and the tracker is the
#: FALLBACK.  It is the ONLY production consumer: nothing in this module reads
#: it, the arrival window opens at ``landing - arrival_lead_s`` and admits an
#: early edge whatever this says, and the labeller never sees it.
#:
#: ⚠ **And that consumer does not actually read THIS constant at runtime.**
#: ``DEFAULT_SESSION_DWELL_MARGIN_S`` is the no-config FALLBACK; the node passes
#: in ``hw.JB_OP_TOSS_SESSION_DWELL_MARGIN_S``, generated from
#: ``config/hardware_config.yaml: toss_session_dwell_margin_s`` — a re-typed
#: literal of this value.  Moving this name alone changes the fallback and
#: leaves the robot where it was.  The two are pinned equal by
#: ``test_local_constants_match_generated_config``, which is what makes the
#: duplication safe; a band re-measure is a YAML edit + ``generate_config.py`` +
#: ``colcon build --packages-select jugglebot``, not a one-line Python change.
#:
#: **Why the CAUGHT-only minimum and not the corpus minimum.** One row in the
#: 2026-08-24 corpus carries an edge at **+45.4 ms**, 42 ms below this floor: a
#: rimshot (sensor_edge_count 4, held for 122 ms, labelled BOUNCED). A rim graze
#: does produce a verdict, so it is a fair datum for "a verdict existed" — but
#: the quantity the dwell margin needs is the earliest instant a verdict about a
#: CAUGHT ball can exist, and no catch in 33 has ever seated before +87.6 ms.
#: Sizing the handoff on the graze would put the margin 42 ms below that, which
#: is the fail-OPEN direction for the one thing this models. Recorded here so a
#: future reader finds the 45.4 ms already accounted for rather than
#: re-discovering it as a contradiction.
#:
#: **It is inert at every published rung, and that is worth knowing before
#: anyone spends a sitting on it.** ``toss_session.handoff_margin_s`` WAS
#: ``max(dwell_margin_s, hand_stroke.catch_park_reentry_s(...))`` pre-R1
#: (``hand_stroke`` and the FSM callers of it are deleted at R1); the park
#: term is >= 0.1416 s at every rung R0-R5 once layer 3 can trim the speed
#: (0.1204 s at R0-R3 with the aim disarmed). So the 50 ms this floor just gave
#: back buys **0 ms of dwell on the binding column** and 16.6 ms at R0-R3 on the
#: disarmed one. The re-measure's value is almost entirely in the CEILING.
ARRIVAL_BAND_MIN_S = 0.087


def arrival_boundary_t(earlier_landing_t: float, later_landing_t: float,
                       arrival_lead_s: float) -> float:
    """THE instant that separates two adjacent ARRIVAL windows (C-POSSESS-1
    § 3.4, clause C.1). One home — used directly by :func:`label_from_sensor`
    below rather than re-derived, for the same reason its departure lead uses
    ``RELEASE_GUARD_S`` rather than a second name: two computations of a
    boundary is how an abutment stops abutting.

        ``max(b - lead, min(a + ARRIVAL_BAND_MAX_S, b))``

    The boundary belongs to the EARLIER ball for as long as its measured band
    runs — never past the later scheduled landing, and never earlier than the
    lead-based instant, so no window that works today gets narrower.

    WHY NOT SIMPLY ``b - lead``, which is what C-POSSESS-1.C shipped: that
    charges the LATER window's guard (a property of the SCHEDULE — headroom for a
    landing prediction that runs late) to the EARLIER window's evidence (a
    property of the BALL — the measured +87.6..+554.7 ms band). Once the cycle
    period drops under ``ARRIVAL_BAND_MAX_S + lead`` = 0.760 s that subtraction
    closes a window INSIDE its own ball's band, which drops ``catch_event_dt_s``
    for the tail and — far worse — mints ``ARRIVAL_REJECTED``, a positive claim
    of non-arrival that VETOES a tracker CAUGHT (§ 3.2). A schedule number must
    never manufacture a refusal about a ball.

    The lead is not deleted, it is RELOCATED: the later window gives up its
    pre-landing lead before the earlier one gives up any band. Measured cost:
    zero. Across the 33 catches the band was cut from, nothing arrived before
    its announced landing at ALL (earliest +87.6 ms; +45.4 ms counting the one
    rimshot), so the pre-landing lead has never been the term that caught an
    edge — while the band's tail demonstrably has.

    Both neighbours evaluate THIS function on the SAME pair, so the abutment of
    C-POSSESS-1.C is preserved exactly: window(L) closes at
    ``arrival_boundary_t(L, N)`` and window(N) opens at the identical call.
    Non-finite inputs are the caller's problem — pass only values that
    :meth:`HandBallSensorSource._finite` has already accepted."""
    return max(later_landing_t - arrival_lead_s,
               min(earlier_landing_t + ARRIVAL_BAND_MAX_S, later_landing_t))


# ── Source identifiers (carried in every verdict, so a log line names its author)
SOURCE_TRACKER_ARRIVAL = 'tracker_arrival'
SOURCE_HAND_BALL_SENSOR = 'hand_ball_sensor'
# RETIRED AS AN AUTHOR 2026-08-26 (D1) and deliberately not deleted. It was the
# composite author `merge_possession` stamped while the tracker could still supply
# the ARRIVAL half; since D1 the cup is the sole author, so nothing MINTS this any
# more. The name survives because it appears in bagged log lines from every
# session between 2026-08-10 and 2026-08-26, and `describe` must keep rendering
# those correctly when an old record is replayed.
SOURCE_MERGED = 'hand_ball_sensor+tracker_arrival'


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

    ``arrival`` is the tri-state truth (``ARRIVAL_*``); ``arrival_ok`` is its
    boolean projection and is a property on purpose — a NamedTuple field would
    let a source set the two inconsistently, which is the whole failure the
    tri-state exists to prevent.
    """
    source: str
    arrival: str
    retention: str
    arrival_err_mm: float
    plane_drop_mm: float
    reason: str

    @property
    def arrival_ok(self) -> bool:
        """Only CONFIRMED is True. ``ARRIVAL_UNKNOWN`` reads False — the
        conservative direction, and the one that cannot mint a catch out of
        blindness."""
        return self.arrival == ARRIVAL_CONFIRMED

    @property
    def confirmed(self) -> bool:
        """The single boolean callers act on: the ball arrived, and nothing
        observed contradicts it still being there.

        ``RETENTION_UNKNOWN`` does not block confirmation — a source that cannot
        see retention must not be able to veto a catch it did observe arriving.
        ``RETENTION_REJECTED`` does: that is a source that positively saw the
        ball leave.
        """
        return self.arrival_ok and self.retention != RETENTION_REJECTED


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
            # NEVER ARRIVAL_UNKNOWN: this source always has an estimate in hand
            # (it is only ever called from a tracker CAUGHT), so both of its
            # answers are positive observations.
            arrival=ARRIVAL_CONFIRMED if ok else ARRIVAL_REJECTED,
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


class HandBallSensorSource:
    """Possession from the ball-in-cup hand sensor — the PRIMARY source, and the
    only one that can answer RETENTION (C-POSSESS-1 § 3.2).

    TICK-DRIVEN, not ball-driven. It reads the cup, so it needs no tracker
    estimate and no reference point, and it therefore does NOT implement
    :class:`PossessionSource`'s ``judge(ball_xyz_mm, ref_point_mm)`` — the
    contract's § 3 already records that a ball-shaped signature "cannot originate
    a possession claim" and names the tick-driven query as the sensor phase's
    design work. This is that query.

    Feed it with :meth:`note_sample` from every ``/hand_telemetry`` message; then
    ask :meth:`observe` (the two-part verdict, for a given predicted landing),
    :meth:`evidence` (the live "is a ball in the cup right now" precondition — a
    DIFFERENT question, and the only one CHECKING needs) or
    :meth:`evidence_settled` (the same question asked conservatively, for the one
    caller whose wrong answer COMMANDS something).

    The predicted landing is passed **per query**, never latched. A latched window
    would outlive its goal, and a window that outlives its goal is a window that
    can veto the NEXT ball's tracker CAUGHT with a stale "nothing arrived" — the
    arm/disarm lifecycle bug class, deleted rather than guarded.

    RAW FOR THE LIVE QUERY, DEBOUNCED FOR THE VERDICT (census D3)
    ------------------------------------------------------------
    ``/hand_telemetry`` carries both bits. The DEBOUNCED one (five missed polls
    to drop, one hit to restore) is the VERDICT bit — it is what the arrival and
    retention EDGES are taken from, because an edge is a claim about the ball and
    a single dropped poll is not one. The RAW one is what :meth:`evidence`
    answers from, and that split is a measured safety requirement rather than a
    preference:

      the debounce is **asymmetric and much slower than its nominal 100 ms** —
      measured ``held->empty`` **232 / 241 / 295 ms**, ``empty->held``
      **0 / 0 / 0 ms** (`plans/archived/toss-selftuning.md` § Open findings).

    Those debounce numbers are NOT re-measured and still stand. What HAS moved is
    the poll cadence they were once explained by: this docstring used to add
    "*the poll cadence itself measures ~71 ms, not the configured 20 ms, and that
    gap has no diagnosis yet*", and both halves of that are now wrong. The 71 ms
    was a measurement of ONE sitting on a degraded plant, not a property of this
    robot — read ``sensor_poll_dt_ms_median`` per record — and the mechanism was
    diagnosed on 2026-08-24 as the pre-FW-14 FlexCAN_T4 RX-ring leak, already
    fixed on 2026-08-15; the post-FW-14 corpus medians at the configured 20 ms.
    That **removes** the cadence as a candidate explanation for the 232/241/295 ms
    fall lag, which leaves that asymmetry *without* a mechanism rather than with
    one — so the raw/debounced split below is more load-bearing, not less.
    ``logbook/2026-08-24-hand-sensor-poll-cadence.md``.

    So for ~241 ms after a ball leaves the cup the debounced bit still reads
    HELD. Once the dwell approaches that number — which is the whole point of the
    cadence ladder — a CHECKING gate reading the debounced bit would pass an
    EMPTY cup on the previous ball's stale HELD. That is a **fail-OPEN possession
    gate**, the exact inversion of C-POSSESS-1's posture ("a dead sensor refuses,
    it does not pass"), and it gets worse as the machine gets faster. The raw bit
    falls with the ball, so :meth:`evidence` fails CLOSED at every dwell.

    The cost of raw is chatter: a carry-flicker can read EMPTY over a seated
    ball. That direction is a REFUSAL (``REJECTED_NO_BALL``) and therefore
    harmless — except for the ONE caller where a refusal is not the end of it,
    the auto-reload interlude, which answers an empty cup by asking BallButler to
    throw a ball at it. That caller uses :meth:`evidence_settled`, which requires
    the two bits to AGREE and answers ``UNKNOWN`` when they do not.

    THE THREE STATES, AND WHY UNKNOWN IS LOAD-BEARING
    -------------------------------------------------
    ``ball_held`` on ``/hand_telemetry`` is meaningless unless ``ball_held_valid``
    (plans/archived/hand-ball-sensor.md § Architecture, normative). Boot before the
    first TxSdo reply, a stale reply, an un-anchored bridge clock and a
    ``Get_Version`` gate failure are ALL ``UNKNOWN``. BallButler's equivalent
    boots ``ball_in_hand_ = true`` — a fail-OPEN default that mis-reports
    possession — and that is a recorded live bug this class deliberately does not
    copy (same plan, § "Three BallButler properties deliberately not copied").
    Blindness is tracked as spans, not as a flag: an invalidity gap that straddles
    the arrival window makes ARRIVAL ``UNKNOWN`` even though a HELD sample arrives
    afterwards, because an edge that was not seen cannot be timed, and inventing
    one from "was empty, is now held" is how a blind window mints a catch.

    WINDOW SIZING — MEASURED, not chosen
    ------------------------------------
    All three windows are config keys (``JB_BD_ARRIVAL_LEAD_S`` /
    ``JB_BD_ARRIVAL_WINDOW_S`` / ``JB_BD_RETENTION_WINDOW_S``) and their shipped
    defaults come from the three 2026-08-10 retest bags — ``2026-08-10_16-04-26``,
    ``_16-13-48``, ``_16-30-44``; 203,922 ``/hand_telemetry`` samples,
    **100 % ``ball_held_valid``** — replayed by
    ``tools/probes/hand_sensor_verdict_replay.py``:

      * **ARRIVAL.** Over 35 announcements (both throwers) whose next empty→held
        edge opened a segment lasting ≥ 1.5 s, that edge landed **+137 … +798 ms**
        after the announced ``landing_time`` (median +399 ms), and **never before
        it**. The next non-catch edge in the same population is **+3194 ms**. So
        the catch band and the re-seat band are separated ~4x, and a window of
        ``[-0.20, +1.50] s`` sits 1.9x above the catch band's top and 2.1x below
        the re-seat floor.  (Those are the 2026-08-10 numbers this window was
        SIZED from and they are left as the sizing record; the band itself was
        re-measured 2026-08-24 to **+87.6 … +554.7 ms**, median +183.9, n=33 —
        see ``ARRIVAL_BAND_MAX_S``. The window only got MORE margin.) The lead is a guard, not a fit: nothing was observed
        early, and a ball that never left the cup raises no EDGE at all, so the
        lead cannot manufacture an arrival.

        The window has to be generous for a measured reason: the physical release
        lags its announcement by a can-bridge-**uptime**-dependent amount
        (+54–63 ms fresh 2026-07-27, **+118–133 ms at ~16 h** on these bags), so a
        window fitted tightly to a fresh-boot session would start refusing real
        catches as a sitting wore on.

      * **RETENTION.** The longest seat-then-leave the sensor resolved in the same
        bags is **0.999 s** (three of them, 0.571 / 0.989 / 0.999 s, all in
        ``_16-30-44`` at ``pos_meas ≈ 0`` in a 54 s window with no announcement —
        i.e. the operator hand-loading, NOT post-catch bounce-outs; they are
        ground truth for the sensor's ability to RESOLVE a sub-second
        seat-then-leave, which is the property the window rests on, and nothing
        more). 1.50 s is 1.5x that.

        > **⚠ THE UPPER JUSTIFICATION IS DEAD — replaced 2026-08-21 (census D1).**
        > It read: *"the upper constraint is the machine's own cadence:
        > ``MIN_TOSS_THROW_DELAY_S`` is 3.5 s, so the window closes 2.3x before
        > the earliest legitimate departure a throw can produce."* That premise
        > was retired with the delay floor itself. At the tuning-phase operating
        > point (dwell **0.49 s**) every legitimate throw departs 0.49 s after
        > the LANDING — the dwell is defined landing->release — which is within
        > 0.353 s of seating at the arrival band's floor and BEFORE seating at
        > its top. Either way it is INSIDE a 1.50 s retention window, and faster
        > than the fastest seat-then-leave the sensor has ever been shown to
        > resolve, so a fixed
        > window would return ``RETENTION_REJECTED`` and label **every successful
        > cycle a bounce-out**. The window is therefore no longer fixed: it is
        > clamped to the machine's own next scheduled release (§ 3.4 below), and
        > 1.50 s survives only as the ceiling that applies when nothing is
        > scheduled after this ball.

    WINDOW HORIZONS — C-POSSESS-1 § 3.4, the cadence clamp
    -----------------------------------------------------
    Both windows are clamped by the machine's OWN schedule, passed per query
    alongside the landing (and, like the landing, never latched):

      * ``next_release_t`` closes RETENTION at ``next_release_t -
        RELEASE_GUARD_S`` — the same instant that opens the next toss's departure
        search, so a departure inside the announced throw window is OUR throw and
        is excluded from the bounce test by construction rather than by a
        tolerance;
      * ``next_landing_t`` closes ARRIVAL at
        ``arrival_boundary_t(landing_t, next_landing_t)`` — the same instant that
        OPENS the next cycle's arrival window, so two adjacent cycles' arrival
        windows can never overlap and claim the same edge;
      * ``prev_landing_t`` OPENS ARRIVAL at
        ``arrival_boundary_t(prev_landing_t, landing_t)`` — the other end of the
        same identity, added 2026-08-23 with clause C.1. Until then the closing
        was ``next_landing_t - arrival_lead_s``, which bought the NEXT window's
        pre-landing guard out of THIS ball's measured band and, under a
        ``BAND_MAX + lead`` cycle period (1.000 s then, 0.760 s since the
        2026-08-24 re-measure), refused real catches with ``ARRIVAL_REJECTED``. The
        boundary now surrenders the guard instead of the band, and the opening
        has to move with it or the two stop abutting.

    All three default to ``None`` (= nothing is scheduled around this ball), which is
    the honest answer for a single ``Toss`` and for a session's last cycle, and
    which reproduces the pre-2026-08-21 behaviour exactly.

    **The residual, sized rather than argued away.** On a session's LAST cycle
    the clamp still applies (the node cannot know at cycle N whether N+1 will
    run), so a bounce-out after ``next_release_t - RELEASE_GUARD_S`` — i.e. later
    than ``dwell - 0.30 s`` measured from the LANDING, which is less than that
    measured from the seat edge — reads ``RETENTION_UNKNOWN`` instead of
    ``REJECTED``. UNKNOWN does not veto
    (§ 2 consequence 3), so that is a REPORTING residual on one cycle per
    session, in the same class as — and strictly smaller than — the one
    C-POSSESS-1 § 7 already accepts. It is the price of never mislabelling a
    good cycle, and the trade is the right way round: the mislabel would fire on
    EVERY cycle and, with ``on_empty_cup: RELOAD``, would route good cycles into
    an interlude that throws a second ball at a full cup.

    WHAT RETENTION CANNOT DO, AND WHY THAT IS DELIBERATE
    ---------------------------------------------------
    ``toss_sequencer._step_in_flight`` finishes the goal on the first confirmed
    tick (C-POSSESS-1 § 3, "a source cannot answer late"). Requiring
    ``RETENTION_CONFIRMED`` before minting the catch would therefore hold the
    toss's terminal open for the whole retention window — a 1.5 s actuation-timing
    change on the one path (``RECENTER``/``STAY``) whose safety argument was
    written against today's timing. So retention does NOT gate the catch verdict:
    a fresh arrival mints ``RETENTION_UNKNOWN``, which § 2 consequence 3 forbids
    from vetoing. Retention binds where it can still act — the NEXT cycle's
    ball-evidence precondition, which is a live :meth:`evidence` read, and the
    possession latch, which a valid EMPTY now clears without release evidence.
    Those are exactly the two edits C-POSSESS-1 § 7.1 named as outstanding.

    THREAD SAFETY — required, not defensive
    ---------------------------------------
    In the node this object is FED from the ``/hand_telemetry`` subscription
    callback and QUERIED from the action thread, concurrently and by design: the
    subscription is deliberately kept out of the action's reentrant callback
    group under a multi-threaded executor so its stamps keep advancing while a
    hand ladder blocks in ``time.sleep`` (see ``_hand_telemetry_mono``'s
    CONSTRAINT comment in ``reload_coordinator_node``). Appending to a deque while
    another thread iterates it raises ``RuntimeError: deque mutated during
    iteration``, so every public method takes ``_mu``. The lock lives HERE rather
    than being borrowed from the node because the invariant is this object's:
    holding the node lock across a verdict query would also mean holding it across
    the log emission at the call site.
    """

    name = SOURCE_HAND_BALL_SENSOR

    def __init__(self, arrival_lead_s: float, arrival_window_s: float,
                 retention_window_s: float, stale_s: float):
        self._mu = threading.Lock()
        self.arrival_lead_s = float(arrival_lead_s)
        self.arrival_window_s = float(arrival_window_s)
        self.retention_window_s = float(retention_window_s)
        self.stale_s = float(stale_s)
        # Edge log: (t, held_after). Bounded — only edges inside the two live
        # windows are ever consulted, and a session's worth would leak.
        # EVICTION, since both bounds fail in a direction and neither is asserted:
        # a deque drops from the LEFT (oldest first), so overflow can only discard
        # evidence older than the live windows — unless the machine produced >64
        # edges or >32 invalidity gaps INSIDE one 1.5 s window, which is a sensor
        # chattering at ~40 Hz, not a catch. If it ever happened, an evicted blind
        # span fails OPEN (an honest UNKNOWN becomes CONFIRMED/REJECTED), so size
        # these up rather than down if the sample rate ever rises.
        self._edges: Deque[Tuple[float, bool]] = collections.deque(maxlen=64)
        # Closed blind spans (start, end); the open one is tracked separately so
        # "blind right now" needs no sentinel end time.
        self._blind_spans: Deque[Tuple[float, float]] = collections.deque(maxlen=32)
        self._blind_from: Optional[float] = None
        self._held: Optional[bool] = None      # None = never seen a good sample
        # The RAW per-sample bit, kept alongside the debounced one. It carries NO
        # edge log on purpose: edges are verdict-grade claims and the raw bit
        # chatters, so it answers only the LIVE question (see the class docstring,
        # "RAW FOR THE LIVE QUERY").
        self._held_raw: Optional[bool] = None
        self._first_good: Optional[float] = None   # everything before it is blind
        self._last_t: Optional[float] = None
        self._sample_ok = False

    # ── feed ──────────────────────────────────────────────────────────────────

    def note_sample(self, t: float, held: bool, valid: bool,
                    raw: Optional[bool] = None) -> None:
        """One ``/hand_telemetry`` sample. ``t`` is the node's monotonic clock
        (``time.perf_counter``), NOT the bridge stamp: staleness here is about
        whether the Jetson is still hearing the sensor, and the bridge stamp is
        wall-epoch only once the bridge's clock anchor lands.

        ``held`` is the DEBOUNCED bit (``ball_held``) and ``raw`` the undebounced
        one (``ball_held_raw``). ``raw=None`` means the caller has no raw bit and
        it degrades to the debounced bit rather than to ``False``: a missing field
        must never read as "no ball".

        **What that path does and does not cover** (audit fix, 2026-08-22 — this
        docstring previously implied more than it delivers). It serves callers
        that hand over a *dict-shaped* sample: a decoded bag, an analysis script,
        a test harness. It does NOT cover the live node, because
        ``ball_held_raw`` is a declared ``bool`` field on
        ``HandTelemetryMessage`` — ``getattr(msg, 'ball_held_raw', None)`` on any
        real message returns ``False``, never ``None``, so a publisher that fills
        ``ball_held``/``ball_held_valid`` and forgets the raw bit yields
        ``raw=False`` and reads exactly as the empty cup this rule forbids. A
        bare ``bool`` cannot carry "unset", and widening the message to a
        tri-state for it would be a wire change for a case no shipped publisher
        produces.

        The defence that DOES hold there is :meth:`evidence_settled`, which the
        one consumer that COMMANDS anything (the auto-reload interlude) uses: it
        requires the raw and debounced bits to AGREE, so a stuck-low raw bit over
        a seated ball answers ``EVIDENCE_UNKNOWN`` and the interlude declines to
        act. The designed defence is real; this documented one is narrower than
        it read.

        A sample that is invalid, or that arrives after a gap longer than
        ``stale_s``, opens a blind span and updates NOTHING else — in particular
        it never moves ``_held``, so it can never synthesise an edge.

        WARM-UP, a consequence of that rule worth stating because it is invisible
        at the call site: the FIRST sample after construction has no predecessor,
        so its gap is infinite and it opens a blind span like any stale one. The
        source therefore needs TWO samples before :meth:`evidence` can answer
        anything but ``UNKNOWN`` — ~10-20 ms at the 100 Hz ``/hand_telemetry``
        cadence, and in the conservative direction (a goal issued in that window
        refuses rather than guesses)."""
        t = float(t)
        with self._mu:
            gap = float('inf') if self._last_t is None else t - self._last_t
            ok = bool(valid) and gap <= self.stale_s
            if not ok:
                if self._blind_from is None:
                    # Blind from the last GOOD sample (a stale gap started there),
                    # or from now if we have never had one.
                    self._blind_from = (self._last_t if self._last_t is not None
                                        else t)
            else:
                if self._blind_from is not None:
                    self._blind_spans.append((self._blind_from, t))
                    self._blind_from = None
                    self._held = bool(held)    # re-seed, deliberately NO edge
                elif self._held is None:
                    self._held = bool(held)    # first good sample: seed, no edge
                elif bool(held) != self._held:
                    self._held = bool(held)
                    self._edges.append((t, self._held))
                self._held_raw = bool(held) if raw is None else bool(raw)
                if self._first_good is None:
                    self._first_good = t
            self._last_t = t
            self._sample_ok = ok

    # ── queries ───────────────────────────────────────────────────────────────

    def _live_ok(self, now: float) -> bool:
        """Are we hearing the sensor RIGHT NOW? (``_mu`` already held.)"""
        return bool(self._sample_ok
                    and self._last_t is not None
                    and (float(now) - self._last_t) <= self.stale_s)

    def evidence(self, now: float) -> str:
        """Live cup state -> ``EVIDENCE_SEATED`` / ``EVIDENCE_EMPTY`` /
        ``EVIDENCE_UNKNOWN``. The toss's ball-evidence precondition, and nothing
        to do with ARRIVAL: it asks what is in the cup right now.

        Answers from the RAW bit (census D3). The debounced bit's measured
        ``held->empty`` lag is ~241 ms, so once the dwell approaches that number
        this query would return SEATED over a cup the previous ball has already
        left — a fail-OPEN possession gate. See the class docstring."""
        with self._mu:
            if self._held_raw is None or not self._live_ok(now):
                return EVIDENCE_UNKNOWN
            return EVIDENCE_SEATED if self._held_raw else EVIDENCE_EMPTY

    def evidence_settled(self, now: float) -> str:
        """The same live question, asked CONSERVATIVELY: the raw and debounced
        bits must AGREE, and a disagreement answers ``EVIDENCE_UNKNOWN``.

        For the one consumer whose wrong answer COMMANDS something. The
        auto-reload interlude answers an EMPTY cup by asking BallButler to throw
        a ball at it, so a raw carry-flicker over a seated ball would put a
        second ball into a full cup. Disagreement between two readings of one
        observable is exactly "I could not look with confidence", which
        C-POSSESS-1 § 2 spells ``UNKNOWN`` — and the interlude gate already
        refuses on UNKNOWN without moving anything.

        Note this is NOT simply "the slower bit": during the ~241 ms fall lag the
        two disagree, so a departure reads UNKNOWN here where the debounced bit
        alone would have said SEATED. Both of its answers are conservative."""
        with self._mu:
            if (self._held is None or self._held_raw is None
                    or not self._live_ok(now)):
                return EVIDENCE_UNKNOWN
            if self._held != self._held_raw:
                return EVIDENCE_UNKNOWN
            return EVIDENCE_SEATED if self._held else EVIDENCE_EMPTY

    def arrival_time(self, landing_t: Optional[float],
                     next_landing_t: Optional[float] = None,
                     prev_landing_t: Optional[float] = None) -> float:
        """The observed arrival edge's instant (monotonic), or NaN when no edge
        fell inside the window around ``landing_t``. This is the **catch-event
        time** — the first quantity this machine has ever had that says WHEN the
        ball entered the cup, rather than when a tracker stopped seeing it.

        ``next_landing_t`` / ``prev_landing_t`` clamp the search window exactly
        as they do in :meth:`observe`; passing them here as well is what keeps the
        reported catch-event time and the ARRIVAL verdict reading the same edge.
        That identity is also what made C-POSSESS-1.C's boundary bug EXACT rather
        than approximate: a window closed inside the band dropped this measurand
        for the tail at the same instant it started refusing the catch."""
        with self._mu:
            rise = self._arrival_edge(landing_t, next_landing_t, prev_landing_t)
        return rise if rise is not None else float('nan')

    def observe(self, now: float, landing_t: Optional[float] = None, *,
                next_release_t: Optional[float] = None,
                next_landing_t: Optional[float] = None,
                prev_landing_t: Optional[float] = None) -> PossessionVerdict:
        """The two-part verdict for a ball predicted to land at ``landing_t``
        (monotonic; ``None``/NaN = no goal is expecting one, so ARRIVAL is
        honestly ``UNKNOWN`` — and since D1, 2026-08-26, that REFUSES; there is
        no tracker fallback. The reason is ``SENSOR_NO_LANDING``, which is in
        :data:`BLIND_REASONS`, so the consumer mints ``MISSED_SENSOR_BLIND``).

        ``next_release_t`` / ``next_landing_t`` are the machine's OWN next
        scheduled instants in the same clock — the cadence clamp of C-POSSESS-1
        § 3.4, documented in the class docstring under "WINDOW HORIZONS". Both
        default to ``None`` (nothing scheduled after this ball), which reproduces
        the pre-2026-08-21 fixed windows exactly.

        ``prev_landing_t`` is the PREVIOUS cycle's landing, and it is the other
        half of clause C.1's boundary (added 2026-08-23): the caller passes the
        very number it passed as ``landing_t`` one cycle ago, so this window's
        opening and the previous one's closing are the same ``arrival_boundary_t``
        call on the same pair and abut exactly. ``None`` on a first cycle or a
        single ``Toss`` — it can only ever move the opening LATER, so its absence
        is the shipped behaviour and never a widened window.

        ``arrival_err_mm`` / ``plane_drop_mm`` are NaN: this source measures the
        cup, not a position, and reporting a zero would claim an accuracy it never
        measured."""
        now = float(now)
        with self._mu:
            arrival, reason = self._arrival_state(now, landing_t, next_landing_t,
                                                  prev_landing_t)
            retention = self._retention_state(now, arrival, landing_t,
                                              next_landing_t, next_release_t,
                                              prev_landing_t)
        return PossessionVerdict(
            source=self.name,
            arrival=arrival,
            retention=retention,
            arrival_err_mm=float('nan'),
            plane_drop_mm=float('nan'),
            reason=reason)

    # ── internals (ALL of these run with ``_mu`` already held) ────────────────

    @staticmethod
    def _finite(value: Optional[float]) -> Optional[float]:
        """``float(value)`` when it is a real number, else None. One helper, so
        every schedule argument treats None and NaN identically — a NaN that
        slipped through as "unknown" must never become a horizon of NaN, which
        compares False against everything and would silently disable the clamp."""
        if value is None:
            return None
        try:
            out = float(value)
        except (TypeError, ValueError):
            return None
        return out if math.isfinite(out) else None

    def _window(self, landing_t: Optional[float],
                next_landing_t: Optional[float] = None,
                prev_landing_t: Optional[float] = None
                ) -> Optional[Tuple[float, float]]:
        land = self._finite(landing_t)
        if land is None:
            return None
        w0 = land - self.arrival_lead_s
        w1 = land + self.arrival_window_s
        nxt = self._finite(next_landing_t)
        if nxt is not None:
            # C-POSSESS-1 § 3.4 clause C.1: close at the boundary this pair of
            # landings shares, so two adjacent arrival windows abut and can never
            # claim one edge — and so the boundary is never bought out of THIS
            # ball's measured band while the NEXT window still holds a
            # pre-landing lead it has never once needed.
            w1 = min(w1, arrival_boundary_t(land, nxt, self.arrival_lead_s))
        prv = self._finite(prev_landing_t)
        if prv is not None:
            # The SAME call the PREVIOUS cycle made to close its own window. It
            # can only move this opening LATER (the boundary is never below
            # `land - lead`), so a first cycle, a single Toss, or a caller that
            # does not know the previous landing keeps exactly the shipped
            # opening — this can never widen a window.
            w0 = max(w0, arrival_boundary_t(prv, land, self.arrival_lead_s))
        # A schedule tighter than the lead itself degenerates the window to a
        # single instant rather than inverting it. An inverted window would make
        # `w0 <= t <= w1` vacuously false AND `_blind_between` vacuously false,
        # i.e. it would answer REJECTED — inventing a refusal out of arithmetic.
        return (w0, max(w0, w1))

    def _band_watched_out(self, landing_t: float, w1: float) -> bool:
        """Did the window outlast the ball's measured arrival band?

        C-POSSESS-1 § 3.4 clause C.2. A window closing before
        ``landing + ARRIVAL_BAND_MAX_S`` has not watched the whole band, so
        "no rise" is not evidence of no arrival and REJECTED is not available —
        see :meth:`_arrival_state`. Reachable below an ``ARRIVAL_BAND_MAX_S``
        cycle period — 0.800 s until the 2026-08-24 re-measure, **0.560 s**
        since, which is what takes the deferred R6 fork's 0.7529 s period out of
        this clause entirely — where
        the next ball lands before this one's band has closed and NO boundary
        rule can give both balls their whole band (the deferred R6 fork). Also
        true of an ``arrival_window_s`` configured shorter than the band, which
        is why this is written as an invariant and not as a cadence special
        case."""
        return w1 >= landing_t + ARRIVAL_BAND_MAX_S

    def _blind_between(self, a: float, b: float) -> bool:
        """True if the sensor was unreadable at any point in ``[a, b]``."""
        if b < a:
            return False
        if self._first_good is None or self._first_good > a:
            return True                        # never heard from it, or not yet then
        for s, e in self._blind_spans:
            if s <= b and e >= a:
                return True
        if self._blind_from is not None and self._blind_from <= b:
            return True
        # An OPEN gap: no sample has arrived since `_last_t`, so everything past
        # `_last_t + stale_s` is unwatched. A blind span is only RECORDED when the
        # next sample lands, so a feed that simply STOPS would otherwise read as
        # "still looking" for ever — the commonest dead-sensor mode, laundered
        # into a MISS. (`evidence` already answers UNKNOWN here, via `_live_ok`.)
        if self._last_t is not None and b > self._last_t + self.stale_s:
            return True
        return False

    def _arrival_edge(self, landing_t: Optional[float],
                      next_landing_t: Optional[float] = None,
                      prev_landing_t: Optional[float] = None
                      ) -> Optional[float]:
        win = self._window(landing_t, next_landing_t, prev_landing_t)
        if win is None:
            return None
        w0, w1 = win
        for t, held in self._edges:
            if held and w0 <= t <= w1:
                return t
        return None

    def _arrival_state(self, now: float, landing_t: Optional[float],
                       next_landing_t: Optional[float] = None,
                       prev_landing_t: Optional[float] = None
                       ) -> Tuple[str, str]:
        win = self._window(landing_t, next_landing_t, prev_landing_t)
        land = self._finite(landing_t)
        if win is None or land is None:
            return ARRIVAL_UNKNOWN, 'SENSOR_NO_LANDING'
        w0, w1 = win
        rise = self._arrival_edge(landing_t, next_landing_t, prev_landing_t)
        if rise is not None:
            return ARRIVAL_CONFIRMED, 'SENSOR_ARRIVED'
        # No edge YET. Blindness beats both remaining answers: a window we could
        # not watch is UNKNOWN even after it closes.
        if self._blind_between(w0, min(now, w1)):
            return ARRIVAL_UNKNOWN, 'SENSOR_BLIND'
        if now < w1:
            return ARRIVAL_UNKNOWN, 'SENSOR_WINDOW_OPEN'
        # C-POSSESS-1 § 3.4 clause C.2. The window is CLOSED and empty — but if
        # the schedule truncated it inside the ball's measured band, we stopped
        # looking before the evidence ran out, and REJECTED would be a positive
        # claim of non-arrival drawn from a cadence number. It also VETOES a
        # tracker CAUGHT (§ 3.2), so this is the difference between "the ball
        # missed" and "we looked away". Say the second one out loud.
        if not self._band_watched_out(land, w1):
            return ARRIVAL_UNKNOWN, 'SENSOR_BAND_CLAMPED'
        return ARRIVAL_REJECTED, 'SENSOR_NO_ARRIVAL'

    def _retention_horizon(self, rise: float,
                           next_release_t: Optional[float]) -> float:
        """The instant retention stops looking for a departure (C-POSSESS-1
        § 3.4). ``<= rise`` means there is no observable retention interval at
        all, which the caller must read as UNKNOWN and never as CONFIRMED."""
        r1 = rise + self.retention_window_s
        rel = self._finite(next_release_t)
        if rel is not None:
            # The SAME instant that opens the next toss's departure search. A
            # fall at or after it is OUR throw, not a bounce-out — excluded by
            # construction rather than by a tolerance.
            r1 = min(r1, rel - RELEASE_GUARD_S)
        return r1

    def _retention_state(self, now: float, arrival: str,
                         landing_t: Optional[float],
                         next_landing_t: Optional[float] = None,
                         next_release_t: Optional[float] = None,
                         prev_landing_t: Optional[float] = None) -> str:
        if arrival != ARRIVAL_CONFIRMED:
            return RETENTION_UNKNOWN
        rise = self._arrival_edge(landing_t, next_landing_t, prev_landing_t)
        if rise is None:                       # unreachable; belt for a future edit
            return RETENTION_UNKNOWN
        r1 = self._retention_horizon(rise, next_release_t)
        if r1 <= rise:
            # The dwell leaves no interval between the seat edge and the next
            # release. Nothing about retention is OBSERVABLE, so the honest
            # answer is UNKNOWN — which § 2 consequence 3 forbids from vetoing.
            # Answering CONFIRMED here would claim an observation never made;
            # answering REJECTED is the census-D1 inversion this clamp exists to
            # kill (every good cycle read as a bounce-out).
            return RETENTION_UNKNOWN
        for t, held in self._edges:
            if (not held) and rise < t <= r1:
                return RETENTION_REJECTED      # it arrived and then left: bounce-out
        if self._blind_between(rise, min(now, r1)):
            return RETENTION_UNKNOWN
        if now >= r1:
            return RETENTION_CONFIRMED
        return RETENTION_UNKNOWN               # window still open


def merge_possession(sensor: PossessionVerdict,
                     tracker: Optional[PossessionVerdict] = None
                     ) -> PossessionVerdict:
    """THE possession verdict — the contract's ONE enforcement point
    (C-POSSESS-1 § 3.2 — read it before changing this).

    Three rules, each stated by the failure it prevents:

    1. **RETENTION is the sensor's, always.** The tracker is contract-forbidden
       from claiming retention (§ 2 consequence 2) — its track freezes at CAUGHT
       and is pruned — so there is nothing to merge. Taking anything else from the
       tracker here would re-open § 7's accepted bounce-out trap.
    2. **ARRIVAL is the sensor's, always — the tracker is not consulted at all.**
       Owner decision D1, 2026-08-26. Until then the tracker was the fallback for
       a sensor ``ARRIVAL_UNKNOWN``, and the *asking* was gated on a tracker
       ``CAUGHT`` at both FSM call sites, which made the tracker primary in
       practice however this function was written. Measured cost on bag
       ``2026-08-26_14-25-16``: 15 false MISSED (12 of them tracks the tracker
       never confirmed — it vetoed by silence) and 3 false CAUGHT, against a cup
       sensor that called 31/31. See the module docstring for the full census.

       The fallback is not merely unhelpful, it is unsound: it re-admits an
       observable whose error model runs through a dead-reckoned free-fall
       extrapolation *precisely* on the ticks where the trustworthy observable
       said nothing, i.e. it is a fallback to the less reliable source, keyed on
       the more reliable one being quiet.

       Blindness therefore refuses, and it must be told apart from "still
       looking" — :func:`arrival_blind`.
    3. **``arrival_err_mm`` / ``plane_drop_mm`` stay the TRACKER's, always.** They
       are the catch-accuracy numbers the hardware runbooks score and the sensor
       cannot supply them (§ 3, "the one thing a new source must not forget").
       They are REPORT-ONLY and nothing here or downstream branches on them.

    ``tracker`` is OPTIONAL since D1, and that is the shape change that lets the
    question be asked on a TICK rather than on a tracker event: with no CAUGHT
    estimate in hand there is no ball to judge, and the verdict is the cup's alone
    with NaN report fields. Passing one adds the cross-check numbers and nothing
    else — it can no longer move the verdict in either direction.
    """
    err = tracker.arrival_err_mm if tracker is not None else float('nan')
    drop = tracker.plane_drop_mm if tracker is not None else float('nan')
    reason = (sensor.reason if tracker is None
              else '{}/{}'.format(sensor.reason, tracker.reason))
    return PossessionVerdict(
        # The AUTHOR is the cup, whether or not a tracker estimate rode along:
        # naming a composite author would claim the tracker contributed to the
        # decision, and since D1 it contributes only to the report fields.
        source=SOURCE_HAND_BALL_SENSOR,
        arrival=sensor.arrival,
        retention=sensor.retention,
        arrival_err_mm=err,
        plane_drop_mm=drop,
        reason=reason)


def arrival_blind(verdict: PossessionVerdict) -> bool:
    """True iff this verdict is ``ARRIVAL_UNKNOWN`` because the sensor **could not
    look**, as opposed to because its window has not closed yet.

    The distinction only became load-bearing with D1 (2026-08-26): while the
    tracker was the fallback, every UNKNOWN degraded to a second opinion, so a
    caller never had to ask *which* UNKNOWN it had. With the cup as the sole
    source the two demand opposite handling — a consumer with its own deadline
    reads "still looking" at that deadline as a genuine MISS (the deadline IS
    ``ARRIVAL_BAND_MAX_S``, so the whole measured band has been watched), while
    "could not look" is a machine fault that must be named, never a miss.

    Reads the reason string, which the merge joins with ``/``, so it works on both
    a bare sensor verdict and a merged one.
    """
    if verdict.arrival != ARRIVAL_UNKNOWN:
        return False
    return any(part in BLIND_REASONS for part in str(verdict.reason).split('/'))


def _tracker_cross_check(verdict: PossessionVerdict, tol_mm: float) -> str:
    """The TRACKER's number rendered as CORROBORATION of a decision already made,
    never as the decision itself.

    `tol_mm` is the TRACKER's bound, so it may only ever be printed against the
    tracker's own `arrival_err_mm`, and "agrees"/"DISAGREES" is decided by
    comparing what the tracker would have said (`err <= tol_mm`) with what the
    verdict actually says. Both directions of disagreement are now normal readings
    rather than contradictions — the sensor confirming a split track the tracker
    refuses, and the sensor vetoing a tracker CAUGHT (C-POSSESS-1 § 3.2 rule 2) —
    so one helper serves both branches and neither can drift from the other.
    """
    err = verdict.arrival_err_mm
    if not (err == err):                           # NaN — no tracker in the merge
        return 'tracker cross-check unavailable'
    tracker_arrived = err <= tol_mm
    rel = (f'{err:.1f} mm <= {tol_mm:.0f} mm' if tracker_arrived
           else f'{err:.0f} mm > {tol_mm:.0f} mm')
    if tracker_arrived == verdict.arrival_ok:
        return f'tracker cross-check {rel} — agrees'
    return (f'tracker cross-check {rel} — DISAGREES (the tracker estimate is a '
            f'dead-reckoned free-fall extrapolation and splits on reload tracks; '
            f'the cup is the primary observation)')


def describe(verdict: PossessionVerdict, tol_mm: float) -> Tuple[str, str]:
    """-> (severity, one-line human summary) for the coordinator's log.

    Lives here, next to the semantics, so the operator-facing wording and the
    verdict cannot drift. Severity is 'info' for both outcomes: a refused CAUGHT
    is the *expected* reading of a corrupt split track and fires on essentially
    every reload, so logging it as a warning would make a healthy sequence look
    broken (the reason the pre-2026-07-28 line was INFO too).

    Three shapes since 2026-08-10, because ARRIVAL is tri-state: CONFIRMED,
    REFUSED (positively not observed) and UNKNOWN — which is itself two states
    the line has to tell apart, "nothing could look" (`arrival_blind`) and "the
    window has not closed yet". The third shape must never read like the second —
    an operator who sees "REFUSED" goes hunting for a miss, where a BLIND UNKNOWN
    sends them to the sensor, which is the actual fault. Both non-confirmed
    shapes end "Not counted" so the bench tally is unambiguous either way.

    REACHABILITY. Until 2026-08-26 the UNKNOWN shape was DEFENSIVE: the only
    caller ran on a tracker CAUGHT and the merge fell back to it, so the merged
    arrival was never UNKNOWN. D1 deleted that fallback and made the question
    tick-driven, so **UNKNOWN is now the common shape** — it is what every tick
    before the arrival edge produces. It stays INFO and stays out of the tally,
    and `arrival_blind` is what separates the machine-fault UNKNOWN from the
    ordinary still-watching one — so the UNKNOWN line PRINTS that split rather
    than asserting the machine fault on every healthy pre-arrival tick. It used
    to say "nothing could look" and render ``nan mm`` unconditionally, which
    under D1 is a 100 % false-positive sensor alarm at ~25 lines per cycle; the
    tracker number now goes through `_tracker_cross_check`, which prints
    "unavailable" instead of ``nan`` when no tracker rode along. The REFUSED
    shape has TWO authors and prints
    differently for each (below) — that split is not cosmetic: since 2026-08-10
    the commonest REFUSED on a self-toss is the sensor vetoing a tracker CAUGHT,
    where the tracker's own number is *small*.
    """
    if verdict.arrival_ok:
        return 'info', (
            f'possession CONFIRMED by {verdict.source} — '
            f'{_tracker_cross_check(verdict, tol_mm)}; retention '
            f'{verdict.retention} [{verdict.reason}] — '
            f'ros_ws/docs/ball_possession_contract.md')
    if verdict.arrival == ARRIVAL_UNKNOWN:
        blind = arrival_blind(verdict)
        return 'info', (
            f'possession UNKNOWN from {verdict.source} — '
            + ('the sensor COULD NOT LOOK across the arrival window'
               if blind else 'the arrival window has not closed yet')
            + f' [{verdict.reason}]; {_tracker_cross_check(verdict, tol_mm)} — '
            f'ros_ws/docs/ball_possession_contract.md. Not counted')
    if verdict.source in (SOURCE_MERGED, SOURCE_HAND_BALL_SENSOR):
        # The SENSOR refused. `merge_possession` names itself the author only when
        # the sensor positively observed the arrival state, so in this branch the
        # cup — not the tracker — is what said no, and the tracker's `tol_mm` must
        # NOT be printed as the reason. Printing it was the exact mirror of the
        # CONFIRMED-branch wart fixed on 2026-08-10: a sensor veto of a tracker
        # CAUGHT rendered as "arrival 3 mm > 70 mm from the catch point", which
        # sends an operator hunting a tracker fault that is not there.
        return 'info', (
            f'possession REFUSED by {verdict.source} — the cup sensor did not '
            f'observe the ball arrive in its window; '
            f'{_tracker_cross_check(verdict, tol_mm)} [{verdict.reason}] — see '
            f'ros_ws/docs/ball_possession_contract.md. Not counted')
    return 'info', (
        f'possession REFUSED by {verdict.source} — arrival '
        f'{verdict.arrival_err_mm:.0f} mm > {tol_mm:.0f} mm from the catch point '
        f'[{verdict.reason}] (estimate sits {verdict.plane_drop_mm:.0f} mm below '
        f'the catch plane: a CAUGHT estimate is a dead-reckoned free-fall '
        f'extrapolation, REPORT-only — see '
        f'ros_ws/docs/ball_possession_contract.md). Not counted')


# ── Toss-record schema, the announced-ball latch, sensor edges/label and the
#    corpus join (moved verbatim from toss_record.py, 2026-09-13 R3-f1 — the
#    learning-stack deletion; plans/active/two-ball-skill-stack.md § 4 R3) ──
#
# latch_announced_ball, FlightLatch/advance_flight_latches/flight_in_progress,
# SensorSample, edges, poll_dt_steps_ms/poll_dt_ms_median
# and label_from_sensor are LIVE: the FSM (reload_coordinator_node) calls
# latch_announced_ball every cycle and skill_node drives it through the
# flight-latch layer (one latch per announced release), and label_from_sensor
# is the one definition of "caught" for the offline corpus (module docstring
# below, formerly toss_record.py's). The schema (Field/FIELDS) and join() are
# kept only so tools/probes/possession_replay.py's corpus join stays callable;
# its only reachable caller (possession_replay.main()) is presently blocked
# earlier by the toss_record_miner.py deletion (the same rung), so this is
# currently dead code kept for interface fidelity rather than a live path —
# worth revisiting if that probe's bag-reading side is ever rebuilt.

"""THE definition of "caught" for the offline corpus (label_from_sensor)
and the wire schema a joined record is written in (Field/FIELDS/join).
Formerly toss_record.py's docstring; see git history for the full RAW
EDGES / CLOCK DOMAINS background this module's functions below assume.
"""

# ── Schema version ────────────────────────────────────────────────────────────
#: Bumps on field REMOVAL or a semantic change; purely additive fields do not
#: bump (plan § 3.7). ``tests/ros/test_ball_possession.py`` pins both the version
#: and the whole field list, so an accidental removal is a red test, not a corpus
#: that silently stops joining.
SCHEMA = 'toss_record/1'

#: Join tolerance on ``announce_throw_time_ros`` (plan § 3.4). Over-determined by
#: ~1000x: the 3.5 s throw-delay floor and the 5.60 s dwell floor put consecutive
#: tosses >= 5 s apart, and the node writes the SAME float into
#: ``ThrowAnnouncement.throw_time`` and into the declaration, so a real match is
#: exact and 5 ms only absorbs float round-tripping through JSON.
JOIN_TOL_S = 0.005

# ── Labels ────────────────────────────────────────────────────────────────────
LABEL_CAUGHT = 'CAUGHT'
LABEL_BOUNCED = 'BOUNCED'
LABEL_MISSED = 'MISSED'
LABEL_NO_RELEASE = 'NO_RELEASE'
LABEL_UNKNOWN = 'UNKNOWN'

LABELS = (LABEL_CAUGHT, LABEL_BOUNCED, LABEL_MISSED, LABEL_NO_RELEASE,
          LABEL_UNKNOWN)

#: Provenance of a joined row (plan § 3.3, ``record_provenance``).
PROV_BOTH = 'declared+mined'
PROV_MINED = 'mined-only'
PROV_DECLARED = 'declared-only'

# ── The schema ────────────────────────────────────────────────────────────────

class Field(NamedTuple):
    """One record field.

    ``origin``  'D' declared by the node | 'M' mined offline | 'DM' both
                (independently recoverable — which is what makes the join
                self-checking) | 'X' derived at join.
    ``kind``    's' str | 'f' float | 'i' int | 'b' bool | 'f2'/'f3' fixed-length
                float lists | 'l' list | 'o' object.
    Every field is nullable unless ``required``; ``None`` is the only null, and
    NaN is never encoded (JSON has no NaN, and a bare ``NaN`` token is not
    portable JSON — :func:`encode` maps non-finite floats to ``None``).
    """
    name: str
    block: str
    origin: str
    kind: str
    doc: str
    required: bool = False


FIELDS: Tuple[Field, ...] = (
    # ── Identity ──────────────────────────────────────────────────────────────
    Field('schema', 'identity', 'D', 's', 'wire schema version', True),
    Field('toss_uid', 'identity', 'D', 's',
          '<session_id>-<goal_id8>-<cycle_index>', True),
    Field('session_id', 'identity', 'D', 's', 'one node launch'),
    Field('goal_id', 'identity', 'D', 's', 'action goal uuid, hex'),
    Field('action', 'identity', 'D', 's', 'toss | toss_continuous', True),
    Field('cycle_index', 'identity', 'DM', 'i',
          '1-based within a session; 1 for a single Toss'),
    Field('announce_throw_time_ros', 'identity', 'DM', 'f',
          'THE JOIN KEY — ThrowAnnouncement.throw_time, ros'),
    Field('announce_landing_time_ros', 'identity', 'DM', 'f',
          'ThrowAnnouncement.landing_time, ros'),
    Field('t_record_ros', 'identity', 'D', 'f', 'when the declaration was minted'),
    Field('perf_minus_ros_s', 'identity', 'D', 'f',
          'FILTERED clock_offset (10 samples, 20-deep history)'),
    Field('perf_minus_ros_inst_s', 'identity', 'D', 'f',
          'single-read offset — the _announcement_landing_perf variant, carried '
          'so the standing reconciliation gap is a measurement not an argument'),

    # ── Provenance ────────────────────────────────────────────────────────────
    Field('git_sha', 'provenance', 'D', 's', 'HEAD at node launch'),
    Field('git_dirty', 'provenance', 'D', 'b', 'working tree dirty at launch'),
    Field('toss_cal_version', 'provenance', 'D', 's', 'phase 2b: motion/toss_cal.map_version'),
    Field('toss_cal_loaded', 'provenance', 'D', 'b', 'a map is present + valid'),
    Field('toss_cal_applied', 'provenance', 'D', 'b',
          'loaded AND the D3 provenance gate agrees — loaded is not applied'),
    Field('tilt_map_version', 'provenance', 'DM', 's', 'TrajectoryStatus'),
    Field('tilt_map_applied', 'provenance', 'DM', 'b',
          'tilt_map_loaded AND gravity_correction_loaded, read together'),
    Field('gravity_correction_loaded', 'provenance', 'DM', 'b', 'TrajectoryStatus'),
    Field('level_offset_rad', 'provenance', 'DM', 'f2', 'the session level'),
    Field('toss_tier', 'provenance', 'D', 's', '8a | 8b'),
    Field('bridge_fw_version', 'provenance', 'M', 's', 'link_status'),
    Field('platform_fw_version', 'provenance', 'M', 's', 'link_status'),
    Field('uptime_ms_at_release', 'provenance', 'M', 'i',
          'can-bridge uptime at the release instant — the R3 partition key'),
    Field('hand_odrive_config_sha', 'provenance', 'D', 's',
          'nullable: a KNOWN GAP in the partition key, never an assumed match'),
    Field('catch_knobs', 'provenance', 'D', 'o', 'the catch-side tunables'),

    # ── Goal as requested ─────────────────────────────────────────────────────
    Field('goal_catch_xyz_stow_mm', 'goal', 'D', 'f3', 'nominated catch pose B'),
    Field('goal_throw_height_m', 'goal', 'D', 'f', 'RESOLVED'),
    Field('goal_throw_height_m_raw', 'goal', 'D', 'f',
          'AS REQUESTED, 0-sentinel preserved'),
    Field('goal_throw_delay_s', 'goal', 'D', 'f', 'RESOLVED'),
    Field('goal_throw_delay_s_raw', 'goal', 'D', 'f', 'AS REQUESTED'),
    Field('goal_catch_vel_scale', 'goal', 'D', 'f', 'RESOLVED'),
    Field('goal_catch_vel_scale_raw', 'goal', 'D', 'f', 'AS REQUESTED'),
    Field('goal_num_throws', 'goal', 'D', 'i', 'session only'),
    Field('goal_dwell_time_s', 'goal', 'D', 'f', 'session only, RESOLVED'),
    Field('goal_stop_on_miss', 'goal', 'D', 'b', 'session only'),
    Field('goal_on_empty_cup', 'goal', 'D', 's', 'session only, null until 2d'),
    Field('goal_max_reloads', 'goal', 'D', 'i', 'session only, null until 2d'),

    # ── Resolved release ──────────────────────────────────────────────────────
    Field('flight_time_s', 'release', 'D', 'f', 'internal flight time'),
    Field('apex_height_m', 'release', 'D', 'f', 'apex above the release plane'),
    Field('event_vel_mps', 'release', 'D', 'f', 'as sent to the hand'),
    Field('event_delay_s', 'release', 'D', 'f', 'as sent, already latency-shifted'),
    Field('release_latency_ms_applied', 'release', 'D', 'f', 'the config shift'),
    Field('release_pos_global_mm', 'release', 'D', 'f3', ''),
    Field('launch_vel_mms', 'release', 'D', 'f3', ''),
    Field('catch_point_global_mm', 'release', 'D', 'f3',
          'via the SINGLE conversion point toss_release.stow_to_global_mm; the '
          'miner recomputes it independently and fails LOUD on a mismatch '
          '(plan § 7 R1)'),
    Field('aim_tilt_rx_rad', 'release', 'D', 'f', 'commanded throw tilt, 0 in 8a'),
    Field('aim_tilt_ry_rad', 'release', 'D', 'f', ''),
    Field('throw_site_xy_mm', 'release', 'D', 'f2', 'tier 8b throw site A'),

    # ── Applied calibration (all null/zero through phase 2a) ──────────────────
    Field('map_aim_rad', 'calibration', 'D', 'f2', 'phase 2b'),
    Field('trim_aim_rad', 'calibration', 'D', 'f2',
          'layer 2 CONTRIBUTED to the commanded aim. A STRUCTURAL zero since '
          '2026-08-21 — the layer-2 aim estimator is monitor-only (C4)'),
    # ADDITIVE, so no schema bump (§ 3.7 item 1). Deliberately a SEPARATE key
    # from trim_aim_rad rather than a repurposing of it: everything that
    # reconstructs the applied aim from a record — toss_trim.applied_aim_rad's
    # map+trim fallback, the miner, ilc_fit_lib — reads trim_aim_rad as "what was
    # applied", and folding a monitor value into that key would re-apply it on
    # paper, which is exactly the C4 double-count wearing a different hat.
    Field('trim_monitor_aim_rad', 'calibration', 'D', 'f2',
          'layer 2 ESTIMATED but did not command, rad. Its divergence from '
          'ilc_aim_rad is the standing validation of the C3 by-decision '
          'resolution (the two reduce different residual channels)'),
    Field('trim_authority', 'calibration', 'D', 's',
          "MONITOR since 2026-08-21 — toss_trim.AIM_AUTHORITY, recorded so a "
          "corpus can prove which build's records carry a commanded trim"),
    Field('total_aim_rad', 'calibration', 'D', 'f2', 'what was actually commanded'),
    Field('map_aim_mm_at_h', 'calibration', 'D', 'f2',
          'REPORT field: mm at THIS toss apex, never the stored unit'),
    Field('trim_aim_mm_at_h', 'calibration', 'D', 'f2', 'REPORT field'),
    # Layer 3 — the critical-point ILC correction (critical-point-ilc.md
    # Phase 2). ADDITIVE fields, so no schema bump (§ 3.7 item 1).
    #
    # POST-GATE, both of them, and that is the whole reason they exist: the plan's
    # risk 5 is that a correction partially truncated by the D7 total-aim clamp
    # (or refused by validate_event_vel) desynchronises applied-u from recorded-u,
    # and the learner then fits against a command the machine never flew. The
    # apply seam refuses rather than truncates and writes the REFUSED value —
    # exactly (0, 0) / 0.0 — here. `total_aim_rad` already carries the sum, so
    # these two say how much of it layer 3 asked for AND GOT.
    Field('ilc_aim_rad', 'calibration', 'D', 'f2',
          'layer 3: the TOTAL ILC aim contribution APPLIED, rad — spatial + '
          'session. Explicit zeros when the feature is off, the artifact is '
          'absent/dormant, the goal cell missed with no session common mode, or '
          'the total-aim clamp REFUSED it'),
    # The C1 split of the field above. ADDITIVE, so no schema bump (s 3.7 item
    # 1), and deliberately a SPLIT rather than a replacement: everything that
    # subtracts what layer 3 applied -- toss_trim.ilc_aim_rad's C4 subtraction
    # first among them -- needs the SUM, and a consumer that had to add two
    # fields to get it would eventually add only one.
    Field('ilc_spatial_aim_rad', 'calibration', 'D', 'f2',
          'layer 3a: the per-cell SPATIAL RESIDUAL applied, rad. Exactly zero '
          'on a cell miss -- toss_ilc.lookup interpolates nothing'),
    Field('ilc_session_aim_rad', 'calibration', 'D', 'f2',
          'layer 3b: the SESSION-LOCAL common mode applied, rad (C1). RAM only, '
          'seeded from the artifact anchor prior, discarded at goal end, and '
          'NOT keyed on a cell hit -- a common mode is not a function of the '
          'cell. This is the quantity a re-level() moves, kept out of every '
          'persisted cell on purpose'),
    Field('ilc_session_applied', 'calibration', 'D', 'b',
          'layer 3b cleared its evidence gate and was commanded'),
    Field('ilc_session_reason', 'calibration', 'D', 's',
          'why layer 3b commanded nothing: no_artifact | no_anchor | '
          'insufficient_evidence | below_se_gate | inside_deadband | '
          'refused_total_aim. Empty when applied -- "commanded nothing" and '
          '"had nothing to command" are different facts about a session. '
          'refused_total_aim is the only one the session component does not '
          'mint itself: the node D7 clamp drops BOTH layer-3 components when '
          'map+ilc exceeds the authority, and before 2026-08-22 this field kept '
          'reading APPLIED on those goals'),
    Field('ilc_session_n', 'calibration', 'D', 'i',
          'independent evidence units (sessions, i.e. level() draws) behind the '
          'anchor prior; 0 when there is none'),
    Field('ilc_vel_trim', 'calibration', 'D', 'f',
          'layer 3: the ILC event_vel trim APPLIED, k_v - 1. Explicit zero on '
          'every path above plus a validate_event_vel refusal'),
    Field('speed_bias_applied', 'calibration', 'D', 'f', 'k_v, phase 2e'),
    Field('timing_bias_applied_ms', 'calibration', 'D', 'f', 'tau, phase 2e'),
    Field('clamp_hits', 'calibration', 'D', 'l', 'per-channel clamp names'),
    Field('trim_source_n', 'calibration', 'D', 'i', ''),
    Field('trim_state', 'calibration', 'D', 's',
          'WARMUP | ACTIVE | CONVERGED | FROZEN_<reason>'),
    Field('trim_reset_reason', 'calibration', 'D', 's', ''),

    # ── Dwell tilt (Layer 1.5) — COVARIATE ONLY, zero control authority ───────
    # Schema lands in 2a (this phase); the dwell read schedule is wired in 2d.
    # Until then every field here is null, which is a LEGAL record: plan § 3.10's
    # degrade-never-delay rule already makes ``dwell_tilt_n = 0`` legal, so the
    # analysers must tolerate absence from day one rather than from 2d.
    Field('dwell_tilt_rad', 'dwell_tilt', 'D', 'f2', 'mean of the N reads'),
    Field('dwell_tilt_sd_rad', 'dwell_tilt', 'D', 'f2', ''),
    Field('dwell_tilt_n', 'dwell_tilt', 'D', 'i', '0 is legal'),
    Field('dwell_tilt_span_s', 'dwell_tilt', 'D', 'f', 'first read -> last read'),
    Field('dwell_tilt_last_read_to_release_s', 'dwell_tilt', 'D', 'f',
          'proves the reads never overlapped PREPARE->THROW'),
    Field('dwell_tilt_degraded', 'dwell_tilt', 'D', 'b',
          'read count reduced to protect the throw'),

    # ── FSM / dispatch ────────────────────────────────────────────────────────
    Field('outcome', 'fsm', 'D', 's', 'verbatim TossResult.outcome', True),
    Field('success', 'fsm', 'D', 'b', 'verbatim TossResult.success'),
    Field('phase_at_terminal', 'fsm', 'D', 's', ''),
    Field('throw_dispatch_class', 'fsm', 'D', 's', 'ok | ambiguous | rejected'),
    Field('throw_dispatch_message', 'fsm', 'D', 's', ''),
    Field('prepare_ok', 'fsm', 'D', 'b', ''),
    Field('position_accepted', 'fsm', 'D', 'b', ''),
    Field('position_planned_s', 'fsm', 'D', 'f', ''),
    Field('position_code', 'fsm', 'D', 's', ''),
    # ADDITIVE, so no SCHEMA bump (§ 3.7 item 1 — see the pipeline block below).
    # Recorded even when it is 0.0: a zero is a MEASUREMENT ("this cycle was never
    # refused BUSY"), and a null is "no FSM ran at all" (the REJECTED_BAD_GOAL
    # row). Without it a REJECTED_POSITION(BUSY) row cannot be told apart from a
    # cycle that waited out the previous catch's settle hold and threw.
    Field('position_busy_wait_s', 'fsm', 'D', 'f',
          'seconds POSITIONING spent absorbing go_to_pose BUSY re-polls before '
          'the move was accepted (or the patience ran out). Nonzero on a '
          'SUCCESSFUL cycle is the chained-cadence seam working; nonzero on a '
          'REJECTED_POSITION(BUSY) row is a wedge that outlived the settle hold'),
    Field('position_busy_polls', 'fsm', 'D', 'i',
          'how many times POSITIONING RE-EMITTED go_to_pose after a BUSY. 0 on '
          'every cycle that was never refused; the HOW MANY beside '
          'position_busy_wait_s\'s HOW LONG, as commit_slips is to '
          'commit_slip_s'),
    Field('catch_target_accepted', 'fsm', 'D', 'b', ''),
    Field('announce_lead_short', 'fsm', 'D', 'b', ''),
    Field('throw_stroke_seen', 'fsm', 'D', 'b', 'G1 release evidence'),
    Field('ball_track_confirmed', 'fsm', 'D', 'b', 'G1 release evidence'),
    Field('t_accept_perf', 'fsm', 'D', 'f', ''),
    Field('t_release_perf', 'fsm', 'D', 'f', ''),
    Field('t_landing_sched_perf', 'fsm', 'D', 'f', ''),
    Field('reload_settle', 'fsm', 'D', 'b',
          'the cycle after a reload interlude — guard G10. null until 2d'),
    Field('retry_of', 'fsm', 'D', 's',
          'the toss_uid this cycle retried — guard G11. null until 2d'),
    # Diagnostics, NOT truth. catch_error_mm is the tracker Kalman filter's
    # dead-reckoned free-fall extrapolation and is FORBIDDEN as an estimator
    # input (plan F5 / D5); it is recorded so that forbidding it is checkable.
    Field('achieved_flight_s_fsm', 'fsm', 'D', 'f', 'diagnostic, not truth'),
    Field('catch_error_mm_fsm', 'fsm', 'D', 'f',
          'diagnostic ONLY — never an estimator input (D5)'),
    Field('catch_event_dt_s_fsm', 'fsm', 'D', 'f',
          'the live hand-sensor catch_dt the node already logs'),

    # ── Sensor (mined from /hand_telemetry) ───────────────────────────────────
    Field('sensor_valid_frac', 'sensor', 'M', 'f',
          'ball_held_valid fraction over the DECISIVE window'),
    Field('sensor_n_samples', 'sensor', 'M', 'i', 'samples in the decisive window'),
    Field('sensor_held_at_dispatch', 'sensor', 'M', 'b', ''),
    Field('t_departure_raw_ros', 'sensor', 'M', 'f', 'RAW held->empty — a TIME'),
    Field('t_departure_deb_ros', 'sensor', 'M', 'f',
          'DEBOUNCED — carried only to MEASURE the ~241 ms asymmetric lag'),
    Field('t_catch_raw_ros', 'sensor', 'M', 'f', 'RAW empty->held — a TIME'),
    Field('t_catch_deb_ros', 'sensor', 'M', 'f', 'DEBOUNCED — the VERDICT edge'),
    Field('t_dropout_ros', 'sensor', 'M', 'f', 'first DEBOUNCED fall after a catch'),
    Field('held_at_catch_plus_retention', 'sensor', 'M', 'b', ''),
    Field('sensor_edge_count', 'sensor', 'M', 'i',
          'DEBOUNCED edges in the decisive window; > 2 is a rimshot candidate'),
    Field('sensor_poll_dt_ms_median', 'sensor', 'M', 'f',
          'MEASURED from ball_held_stamp advances — never assumed'),
    Field('ball_held_stamp_wall_anchored', 'sensor', 'M', 'b',
          'false => every edge would be mis-timed by the whole bridge uptime'),

    # ── Mocap (mined; estimator IMPORTED from tools/probes/ball_arrival_offset) ─
    Field('land_xy_global_mm', 'mocap', 'M', 'f2', ''),
    Field('land_err_mm', 'mocap', 'M', 'f2',
          'land_xy - catch_point_global_mm[:2] — THE aim observable'),
    Field('land_err_norm_mm', 'mocap', 'M', 'f', ''),
    Field('n_fit', 'mocap', 'M', 'i', ''),
    Field('fit_rms_mm', 'mocap', 'M', 'f', 'G2 track quality'),
    Field('fit_sparse', 'mocap', 'M', 'b', ''),
    Field('apex_z_mm', 'mocap', 'M', 'f', ''),
    Field('achieved_flight_s_mocap', 'mocap', 'M', 'f',
          'MEASURED release -> catch-plane crossing, both instants read off the '
          'SAME mocap arc in the bag clock (so no clock crossing enters it). '
          'Defined by tools/probes/toss_record_miner.mine_arc; null in every '
          'corpus written before 2026-08-12'),
    Field('t_land_bag', 'mocap', 'M', 'f', 'bag clock — /mocap_data has no header'),
    Field('qtm_offset_s', 'mocap', 'M', 'f', ''),
    Field('mocap_gap_ms_max', 'mocap', 'M', 'f', ''),
    Field('land_plane_mm', 'mocap', 'M', 'f',
          'the fit plane USED — so a historical plane mismatch is detectable'),
    Field('floor_arrival', 'mocap', 'M', 'i', 'REPORT-only floor census'),

    # ── Command reference for the mined errors (ILC Phase 0a/0c) ──────────────
    # The commanded release state the arrival/backcast errors are differenced
    # against, RECORDED rather than assumed: on a mined-only row every 'D' field
    # (launch_vel_mms, flight_time_s) is null by construction, so without these
    # the errors below would not be auditable from the corpus alone.
    Field('cmd_launch_vel_mms', 'command_ref', 'M', 'f3',
          'commanded release velocity the mined errors are taken against; '
          'equals the declared launch_vel_mms when a declaration joined'),
    Field('cmd_flight_time_s', 'command_ref', 'M', 'f',
          'commanded release->cup flight time, same source as '
          'cmd_launch_vel_mms'),
    Field('cmd_release_source', 'command_ref', 'M', 's',
          "where cmd_* came from. 'announcement' is the only value the miner "
          'emits: the announcement is filled from the same ReleaseState the '
          'declaration reports, and it is the ONLY source on a bag predating '
          '/toss/record. The slot is named rather than assumed so a corpus can '
          'still say so if that ever changes'),

    # ── The WHOLE-ARC fit (ILC entry condition E-1, resolved 2026-08-13) ──────
    # Definition point: tools/probes/toss_record_miner.mine_arc. A THIRD ballistic
    # fit over the union of the two branches, and it owns every LATERAL velocity
    # component below. WHY, in one paragraph: the tracked point is the centroid of
    # the visible retroreflective cap, so it carries a height-locked position bias
    # b(z) of ~20 mm; height is EVEN about a ballistic apex, so a per-branch line
    # fit reads v_true +- <db/dz.|vz|> and the two branches disagree by 104 mm/s
    # on exactly the aim channels, repeatably. A whole-arc slope cancels that by
    # parity, leaving only the sample coverage's asymmetry about the apex — which
    # is what `coverage_asym_s` measures and what gates `usable_for_lateral_fit`.
    # Evidence and the refuted alternatives: plans/archived/critical-point-ilc.md
    # § Phase 1 E-1; probe tools/probes/mocap_parity_bias.py.
    Field('arc_fit_n', 'arc', 'M', 'i',
          'rows in the WHOLE-ARC fit (both branches, apex row de-duplicated)'),
    Field('arc_fit_rms_mm', 'arc', 'M', 'f',
          '3-D RMS residual of the whole-arc ballistic fit'),
    Field('arc_lateral_vel_se_mms', 'arc', 'M', 'f',
          'worst of the two 1-sigma standard errors on the fitted LATERAL '
          'velocities — the noise floor of every direction channel below'),
    Field('coverage_asym_s', 'arc', 'M', 'f',
          'mean sample time MINUS the fitted apex instant, seconds. The one '
          'term the whole-arc estimator does not cancel: cov(tau, b) is exactly '
          'zero for coverage symmetric about the apex, so this is the bias-leak '
          'driver. A GROSS-TRUNCATION guard, not a leak predictor (one 0.0003 s '
          'arc still leaks 6 mm/s). Measured on the 2026-08-13 re-mine, by '
          'population: over the 19 usable_for_release_fit rows, 0.0001-0.0732 s '
          '(median 0.0148), 0 refused; over all 32 mined arcs, median 0.052 s, '
          'worst 0.600 s, 12 refused — every one a half-seen arc the '
          'release-fit gate already refuses. Refused past '
          'COVERAGE_ASYM_MAX_S = 0.1 s'),

    # ── Arrival kinematics (ILC Phase 0a; VERTICAL from the DESCENDING branch) ─
    # Definition point: tools/probes/toss_record_miner.mine_arc. The nominal is
    # ballistics_bc.arrival_velocity(cmd_launch_vel_mms, cmd_flight_time_s) —
    # the PRODUCTION function, never a second copy (plan constraint 1), and it
    # is deliberately NOT stored: one production call reproduces it, a stored
    # copy could drift from it.
    Field('arrival_vel_mms', 'arrival', 'M', 'f3',
          'MEASURED ball velocity at the catch-plane crossing (global mm/s). '
          'xy from the WHOLE-ARC fit (E-1), z from the descending branch'),
    Field('arrival_dir_err_rad', 'arrival', 'M', 'f2',
          'measured - nominal per-axis lean of the arrival velocity off the '
          'vertical it is travelling along: [atan2(vx,|vz|), atan2(vy,|vz|)]. '
          'Bias-immune since E-1: the numerator is whole-arc'),
    Field('arrival_dir_err_norm_rad', 'arrival', 'M', 'f',
          'total angle between the measured and nominal arrival directions'),
    Field('arrival_speed_err_mms', 'arrival', 'M', 'f',
          '|v_measured| - |v_nominal| at the catch plane'),
    Field('t_arrival_fit_bag', 'arrival', 'M', 'f',
          'MEASURED catch-plane crossing instant, bag clock (t_land_bag is the '
          'ANNOUNCED landing crossed into the bag clock — not the same number)'),
    Field('arrival_fit_n', 'arrival', 'M', 'i', 'rows in the descending fit'),
    Field('arrival_fit_rms_mm', 'arrival', 'M', 'f',
          '3-D RMS residual of the descending ballistic fit'),
    Field('arrival_vel_se_mms', 'arrival', 'M', 'f',
          '1-sigma standard error on the fitted VERTICAL arrival velocity. THE '
          'noise floor this phase owes Phase 1 — a short branch fits a clean '
          'RMS and a badly wrong velocity, and only this number says so. The '
          'lateral components are ~10x better determined (their residual is '
          '2-8 mm against 25-34 mm in z, because the bag clock error shows up '
          'multiplied by the axis speed)'),
    Field('flight_time_err_s', 'arrival', 'M', 'f',
          'achieved_flight_s_mocap - cmd_flight_time_s'),

    # ── Release-state backcast (ILC Phase 0c; ASCENDING branch) ──────────────
    # The throw critical point's INPUT-side truth. Everything here is
    # attributable to the RELEASE (hand stroke, dispatch timing, aim);
    # everything in `land_err_flight_mm` below is what the release does NOT
    # explain. That split is the discriminator the whole ILC leans on.
    Field('release_pos_track_mm', 'backcast', 'M', 'f3',
          'MEASURED release point: the ascending fit evaluated where it crosses '
          'the release plane'),
    Field('release_vel_track_mms', 'backcast', 'M', 'f3',
          'MEASURED release velocity at that crossing (global mm/s). xy from '
          'the WHOLE-ARC fit (E-1), z from the ascending branch'),
    Field('t_release_fit_bag', 'backcast', 'M', 'f',
          'MEASURED release instant, bag clock'),
    Field('release_time_err_ms', 'backcast', 'M', 'f',
          't_release_fit crossed to ros MINUS announce_throw_time_ros — an '
          'INDEPENDENT dispatch-shift measurement (the sensor departure edge is '
          'the other one, and they disagree; see the 0a/0c logbook)'),
    Field('release_vel_err_mms', 'backcast', 'M', 'f3',
          'release_vel_track_mms - cmd_launch_vel_mms, per axis'),
    Field('release_speed_err_mms', 'backcast', 'M', 'f',
          '|measured| - |commanded| release speed'),
    Field('release_dir_err_rad', 'backcast', 'M', 'f2',
          'per-axis lean error of the release velocity, same convention as '
          'arrival_dir_err_rad, and whole-arc in the same sense. NOTE the '
          'consequence: release and arrival now share ONE lateral velocity, so '
          'these two channels differ only through their |vz| denominators and '
          'their nominals — they are not independent lateral measurements'),
    Field('backcast_fit_n', 'backcast', 'M', 'i', 'rows in the ascending fit'),
    Field('backcast_fit_rms_mm', 'backcast', 'M', 'f',
          '3-D RMS residual of the ascending ballistic fit'),
    Field('release_vel_se_mms', 'backcast', 'M', 'f',
          '1-sigma standard error on the fitted VERTICAL release velocity — the '
          'arrival_vel_se_mms twin, and the same warning applies'),

    # ── Release-vs-flight split of the landing error (ILC Phase 0c) ──────────
    # By construction land_err_release_mm + land_err_flight_mm == land_err_mm,
    # exactly. The first half is what the MEASURED release state already
    # predicts (propagated to the cup plane by the production ballistics).
    #
    # RE-MARKED 2026-08-13 (E-1's resolution). Both halves are LATERAL (xy)
    # quantities, and under the whole-arc estimator the second half is NOT
    # measurable flight-phase physics:
    #   * the parity decomposition puts an upper bound on any lateral
    #     aerodynamic force well under the artefact it was competing with —
    #     Magnus would need 3.4-8.8 rev/s against an observed ~0.5, and would
    #     land in the ODD channel about the apex, which is empty;
    #   * so what remains in land_err_flight_mm is the disagreement between two
    #     ESTIMATORS of the same crossing — the descending-band position fit
    #     behind land_xy_global_mm, against the release state propagated
    #     forward — plus the unmeasured ABSOLUTE centroid bias, which both
    #     halves carry and neither resolves.
    # Read it as an estimator-agreement diagnostic. **Lateral landing error is
    # RELEASE-side**, and nothing may fit land_err_flight_mm as a flight
    # channel. The VERTICAL split is untouched and stays meaningful:
    # release_speed_err_mms against flight_time_err_s are two disjoint branches
    # measuring one throw, which is exactly what the ILC's V2b cross-check
    # leans on.
    Field('land_err_release_mm', 'split', 'M', 'f2',
          'RELEASE-attributable part of land_err_mm — where the MEASURED '
          'release state lands under production ballistics, minus the cup. It '
          'equals "measured release vs COMMANDED release" only because the '
          'announced landing IS the commanded release own ballistic landing '
          '(build_announcement_fields); an announcement inconsistent with its '
          'own release state would land its inconsistency here. Since E-1 this '
          'is the whole of the lateral landing error, to estimator agreement'),
    Field('land_err_flight_mm', 'split', 'M', 'f2',
          'land_err_mm minus the release-attributable part. NOT a flight-physics '
          'channel since E-1 (2026-08-13) — an estimator-agreement diagnostic; '
          'see the block comment above'),

    # ── Plant (mined; row builder imported from the bench-mining probe) ──────
    Field('stroke_peak_rev', 'plant', 'M', 'f', ''),
    Field('dip_below_x3_rev', 'plant', 'M', 'f', 'the Phase-0 gate row'),
    Field('pullback_rps', 'plant', 'M', 'f', ''),
    Field('trunc', 'plant', 'M', 'b', ''),
    Field('seeds', 'plant', 'M', 'i', ''),
    Field('iq_brake_min_a', 'plant', 'M', 'f', 'the braking-clamp diagnostic'),
    Field('dispatch_shift_ms', 'plant', 'M', 'f', 'rel_fit - rel_ann'),
    Field('can_errors', 'plant', 'M', 'i', 'per-cycle delta'),
    Field('bridge_tx_diag', 'plant', 'M', 's', 'per-cycle delta, verbatim'),
    Field('plant_block_source', 'plant', 'M', 's', 'trace | bag'),

    # ── Loop timing (declared; INSTRUMENT ONLY, no control authority) ────────
    # The node's own tick loop, censused per cycle by
    # `toss_sequencer.LoopPeriodCensus`. Scoped to the PRE-DISPATCH ticks
    # (CHECKING/POSITIONING/PREPARING) because those are exactly what
    # `pre_dispatch_budget_s` charges: the post-dispatch majority is idle
    # flight-waiting and would dilute any whole-cycle statistic.
    #
    # These exist to answer whether NODE_LOOP_PERIOD_S is still a bound, and if
    # not, WHICH cost moved. They must never be read back by a gate — a budget
    # that re-derives itself from the last cycle tracks a degradation instead of
    # exposing it.
    Field('loop_n_pre', 'timing', 'D', 'i',
          'complete pre-dispatch iterations censused (the terminal tick has no '
          'trailing sleep and is deliberately excluded)'),
    Field('loop_period_max_pre_s', 'timing', 'D', 'f',
          'THE number NODE_LOOP_PERIOD_S must bound: worst pre-dispatch '
          'iteration, top-of-loop to top-of-loop'),
    Field('loop_period_mean_pre_s', 'timing', 'D', 'f',
          'mean pre-dispatch iteration — the trend line; the MAX is what the '
          'budgets are sized on'),
    Field('loop_work_max_pre_s', 'timing', 'D', 'f',
          'worst (obs + body), i.e. the iteration minus its sleep. Max of the '
          'per-iteration SUM, not the sum of the maxima. This is the headroom '
          'number if the loop ever becomes deadline-compensated'),
    Field('loop_obs_max_pre_s', 'timing', 'D', 'f',
          'worst _build_toss_observations — the per-tick rebuild'),
    Field('loop_body_max_pre_s', 'timing', 'D', 'f',
          'worst step + action dispatch + publishes, obs excluded — where a '
          'blocking service call in the PREPARE bundle shows up'),
    Field('loop_sleep_max_pre_s', 'timing', 'D', 'f',
          'worst MEASURED sleep, not assumed to be NODE_TICK_S: scheduler '
          'overshoot lands here rather than being misattributed to body'),
    Field('loop_n_over_pre', 'timing', 'D', 'i',
          'pre-dispatch iterations exceeding NODE_LOOP_PERIOD_S. Nonzero on a '
          'SUCCESSFUL cycle is the early warning that was invisible before'),
    Field('loop_n_post', 'timing', 'D', 'i', 'post-dispatch iterations'),
    Field('loop_period_max_post_s', 'timing', 'D', 'f',
          'worst post-dispatch iteration, for contrast'),

    # ── The two-slot pipeline (declared; null on every serial cycle) ──────────
    # ADDITIVE, so no SCHEMA bump — the schema's own rule (§ 3.7 item 1) is that
    # purely additive fields do not bump, because a reader that does not know
    # them reads nulls and a reader that does gets the whole story. All five are
    # null for every cycle of a `toss_pipeline_enabled: false` sitting, which is
    # itself the partition key a corpus needs: "did this cycle stage?" is
    # answerable from the record rather than from the build.
    #
    # `commit_slip_s` is the row the operator scores the plan's own § 1.4
    # prediction from (runbook row PIPE-1: ~0 ms at h = 1.3, ~45-65 ms at
    # h = 1.0, and a slip RISING across a session is a loop-cost regression).
    Field('staged_at_s', 'pipeline', 'D', 'f',
          'perf instant this cycle entered PHASE_STAGED, i.e. finished its '
          'preamble inside the PREVIOUS cycle\'s flight. Null = it never '
          'staged (serial cycle, or the first cycle of a pipelined sitting, '
          'which has nothing to pipeline behind)'),
    Field('commit_at_s', 'pipeline', 'D', 'f',
          'the COMMIT instant as SCHEDULED — t_release - commit_budget_s at '
          'start(), before any slip. Paired with commit_slip_s it gives the '
          'instant the arm point actually fired'),
    Field('commit_slip_s', 'pipeline', 'D', 'f',
          'commit-time minus SCHEDULED commit. The measured lateness of the '
          'arm point, never negative (the gate is polled). Phase C\'s '
          'bounded-slip policy is a CONSUMER of this, which is why it is '
          'recorded a phase before any policy reads it'),
    Field('commit_slips', 'pipeline', 'D', 'i',
          'how many times the COMMIT gate re-armed before it resolved. Paired '
          'with commit_slip_s: that says HOW LATE, this says HOW MANY '
          'ITERATIONS — one late tick on a healthy loop vs a loop chronically '
          'over period'),
    Field('staged_discarded_reason', 'pipeline', 'D', 's',
          'why a STAGED slot was dropped without ever committing — the § 2.4.3 '
          'unwind. Non-null ⇒ this cycle never released, and its row exists so '
          'a discard is countable rather than being an absence in the census'),

    # ── Label / quality (derived at join) ─────────────────────────────────────
    Field('label', 'quality', 'X', 's', ' | '.join(LABELS)),
    Field('label_source', 'quality', 'X', 's', ''),
    Field('label_confidence', 'quality', 'X', 'f', ''),
    Field('label_reason', 'quality', 'X', 's', 'which gate decided'),
    Field('rimshot', 'quality', 'X', 'b', 'PROVISIONAL, REPORT-only'),
    Field('disagreement', 'quality', 'X', 'l',
          'D-vs-M conflicts, NEVER silently resolved'),
    Field('record_provenance', 'quality', 'X', 's',
          ' | '.join((PROV_BOTH, PROV_MINED, PROV_DECLARED))),
    Field('join_residual_ms', 'quality', 'X', 'f', ''),
    Field('usable_for_aim_fit', 'quality', 'X', 'b', ''),
    Field('usable_for_timing_fit', 'quality', 'X', 'b', ''),
    Field('usable_for_speed_fit', 'quality', 'X', 'b', ''),
    Field('usable_for_release_fit', 'quality', 'X', 'b',
          'the ARC fits are trustworthy: both branches long enough and the '
          'fitted release velocity SE small. NOT implied by _speed_fit, which '
          'gates on the landing fit'),
    Field('usable_for_lateral_fit', 'quality', 'X', 'b',
          'the WHOLE-ARC lateral estimate is admissible (E-1, 2026-08-13): '
          'coverage_asym_s present and within COVERAGE_ASYM_MAX_S. A separate '
          'flag for the same reason _release_fit is one — _aim_fit gates the '
          'aim MAP, which consumes only the landing POSITION and never carried '
          'the branch artefact. ABSENT coverage_asym_s refuses: that is a '
          'pre-E-1 mine, whose lateral velocities ARE the artefact'),
    Field('excluded_reason', 'quality', 'X', 's',
          'every refusal this row accumulated, comma-joined and sorted — across '
          'ALL the usable_* flags, not one of them. A null coverage_asym_s is '
          'deliberately NOT a reason: there is no lateral estimate to exclude'),
)

FIELD_NAMES: Tuple[str, ...] = tuple(f.name for f in FIELDS)
_BY_NAME: Dict[str, Field] = {f.name: f for f in FIELDS}

# ── Blank / encode / decode / validate ────────────────────────────────────────

def blank_record() -> Dict[str, Any]:
    """A record with every field present and null.

    Present-and-null, never absent: a reader that has to distinguish "the field
    does not exist in this schema" from "nothing was measured" is a reader that
    will eventually guess.
    """
    rec = {name: None for name in FIELD_NAMES}
    rec['schema'] = SCHEMA
    return rec

# ── The announced-ball latch (extracted from reload_coordinator_node,
#    2026-08-10; re-homed from toss_record.py 2026-09-13) ───────────────────

def latch_announced_ball(balls: Iterable[Any], *, robot_name: str,
                         announced_id: Optional[int],
                         preexisting_ids: Iterable[int],
                         untagged_latch: bool,
                         in_flight_status: int) -> Tuple[Optional[int], bool]:
    """Correlate the tracker-assigned id of OUR announced ball. Pure.

    -> ``(announced_id, untagged_latch)``. The caller owns the state; this
    function owns the RULE, so the node and the offline miner apply the same one.
    Extracted verbatim from ``reload_coordinator_node._update_announced_ball_latch``
    (2026-08-10, plan D11) — the node now calls this and keeps only the locking.

    The tracker puts our ball IN_FLIGHT (``destination == robot_name``) after
    correlating the announcement; latch that id, then confirm ONLY that id's
    CAUGHT. This rejects a stray caught ball — a different id, e.g. a leftover
    from a prior throw — that would otherwise falsely confirm the catch.

    Two hardening passes carried over from the 2026-07-23 re-test:

    1. ids already IN_FLIGHT when the throw was accepted are excluded (a phantom
       untagged track that predated the throw was latched as "our" ball);
    2. a destination match is preferred over the empty-destination fallback, and
       an UNTAGGED latch stays PROVISIONAL — the moment a destination-tagged
       candidate appears the latch moves to it, because the tagged track is the
       tracker's own claim about OUR ball.
    """
    preexisting = set(int(i) for i in preexisting_ids)
    candidates = [b for b in balls
                  if int(b.status) == int(in_flight_status)
                  and int(b.id) not in preexisting]
    dest_match = next((int(b.id) for b in candidates
                       if b.destination == robot_name), None)
    if announced_id is None:
        if dest_match is not None:
            return dest_match, False
        for b in candidates:
            if not b.destination:
                return int(b.id), True
        return None, untagged_latch
    if untagged_latch and dest_match is not None:
        return dest_match, False
    return announced_id, untagged_latch


# ── The flight in progress (2026-09-16) ───────────────────────────────────────
#
# `latch_announced_ball` above answers "which tracker id is OUR announced
# ball?" for ONE announcement. A chained self-toss asks a harder question: the
# schedule re-throws the SAME schedule ball every beat, the tracker mints a NEW
# id per announcement, and the next throw is announced at its DISPATCH — about
# 1.25 s before its release, i.e. while the PREVIOUS flight of that same
# schedule ball is still in the air (measured, bag `2026-09-16_16-22-22`:
# announcement at 1789540418.439 for a release at 1789540419.661, with id 48
# airborne and id 49 announced-but-unreleased).
#
# Keying one latch per SCHEDULE ball id therefore loses the flight in
# progress twice over: the new announcement resets the latch, and its
# `preexisting` set — "ids already IN_FLIGHT are phantoms" — excludes the very
# ball that is flying. The latch then resolves to the NEXT flight's id the
# instant that id goes IN_FLIGHT, which is BEFORE the current flight's outcome
# row has closed. That is how the 2026-09-16 sitting's rows came to hold
# `t_land ≈ release + flight + beat`
# (`logbook/2026-09-16-tracker-correlation-follows-the-flight-in-progress.md`).
#
# The rule that closes the class: **correlate per RELEASE, not per ball, and
# never read a release that has not happened yet.** One `FlightLatch` per
# announced release; a latch whose release instant is still in the future is
# left unlatched (the tracker keeps that ball TO_BE_THROWN until exactly the
# same instant — `tracking/matcher.py::_check_throw_times`), and the flight in
# progress is the latch with the LATEST release that has passed.

#: How far before its announced release a latch may already resolve. The
#: tracker flips TO_BE_THROWN -> IN_FLIGHT on its own clock read of the same
#: announced `throw_time` (measured skew on 2026-09-16: 3 ms), so a latch must
#: not refuse the id purely because the two clock reads straddle the instant.
#: Small against the shortest beat this stack schedules (~1.15 s), so it can
#: never admit the next flight early.
RELEASE_LATCH_EPS_S = 0.050

#: How many announced releases to keep per schedule ball. Only the latest
#: released latch is ever read, plus the still-flying ones it must not steal an
#: id from; two flights can overlap at most (a catch and its carried throw), so
#: four is slack. Bounded because `_correlation` lives for the node's life.
MAX_FLIGHT_LATCHES = 4


class FlightLatch(NamedTuple):
    """One announced release, and the tracker id it has been correlated to.

    ``t_release_s`` is the announcement's own ``throw_time`` (ROS epoch
    seconds) — the SAME instant the tracker uses to put the ball IN_FLIGHT, so
    "has this release happened?" is one comparison and not an estimate.
    ``announced_id`` is None until the latch resolves; ``untagged`` marks a
    PROVISIONAL latch onto an untagged track (see `latch_announced_ball`);
    ``preexisting`` are the ids already IN_FLIGHT when this release was
    announced.
    """
    t_release_s: float
    announced_id: Optional[int] = None
    untagged: bool = False
    preexisting: Tuple[int, ...] = ()


def advance_flight_latches(balls: Iterable[Any], *, robot_name: str,
                           latches: Sequence[FlightLatch], now_s: float,
                           in_flight_status: int,
                           max_latches: int = MAX_FLIGHT_LATCHES,
                           ) -> Tuple[FlightLatch, ...]:
    """Refine every RELEASED latch against one ``/balls`` snapshot. Pure.

    Latches are resolved in release order, and each one excludes the ids its
    siblings have already claimed: two flights of one schedule ball overlap in
    ``/balls`` (the caught ball's track outlives the next release), and
    `latch_announced_ball` would otherwise hand the newer release the older
    flight's id — it prefers the first destination-tagged candidate it sees.

    A latch whose ``t_release_s`` is still in the future (beyond
    :data:`RELEASE_LATCH_EPS_S`) is returned untouched: whatever is IN_FLIGHT
    now belongs to some other release.

    -> the latches, newest ``max_latches`` kept, in release order.
    """
    balls = list(balls)
    ordered = sorted(latches, key=lambda l: float(l.t_release_s))
    claimed = {int(l.announced_id) for l in ordered
               if l.announced_id is not None}
    out: List[FlightLatch] = []
    for latch in ordered:
        if (latch.announced_id is None
                and float(latch.t_release_s) > float(now_s) + RELEASE_LATCH_EPS_S):
            out.append(latch)
            continue
        mine = None if latch.announced_id is None else int(latch.announced_id)
        exclude = set(int(i) for i in latch.preexisting) | (claimed - {mine})
        announced_id, untagged = latch_announced_ball(
            balls, robot_name=robot_name, announced_id=mine,
            preexisting_ids=exclude, untagged_latch=bool(latch.untagged),
            in_flight_status=in_flight_status)
        if announced_id is not None:
            if mine is not None and int(announced_id) != mine:
                claimed.discard(mine)
            claimed.add(int(announced_id))
            latch = latch._replace(announced_id=int(announced_id),
                                   untagged=bool(untagged))
        out.append(latch)
    return tuple(out[-int(max_latches):]) if max_latches > 0 else tuple(out)


def flight_in_progress(balls: Iterable[Any], *,
                       latches: Sequence[FlightLatch], now_s: float,
                       in_flight_status: int, confirmed_tracking: int,
                       ) -> Optional[int]:
    """The tracker id whose flight is actually in progress, or None. Pure.

    THE canonical read of a correlation: the latch with the latest release that
    has passed, and only if its id is both IN_FLIGHT and CONFIRMED. Never a
    latch whose release is still in the future (that ball has not been thrown),
    and never an EARLIER latch as a fallback — a release only happens once the
    previous flight of that schedule ball is over, so an earlier latch's
    landing is a finished flight's, which is the class of contamination this
    function exists to refuse. "No landing yet" is the honest answer.
    """
    released = [l for l in latches
                if float(l.t_release_s) <= float(now_s) + RELEASE_LATCH_EPS_S]
    if not released:
        return None
    latch = max(released, key=lambda l: float(l.t_release_s))
    if latch.announced_id is None:
        return None
    tracker_id = int(latch.announced_id)
    ball = next((b for b in balls if int(b.id) == tracker_id), None)
    if ball is None:
        return None
    if int(ball.status) != int(in_flight_status):
        return None
    if int(ball.tracking) != int(confirmed_tracking):
        return None
    return tracker_id


# ── Sensor edges and the label ────────────────────────────────────────────────

class SensorSample(NamedTuple):
    """One ``/hand_telemetry`` sample, in the caller's chosen ``ros``/``bag``
    clock. ``held`` is the DEBOUNCED verdict, ``raw`` the undebounced bit."""
    t: float
    held: bool
    raw: bool
    valid: bool
    stamp: float = 0.0     # ball_held_stamp, for the MEASURED poll cadence


def edges(samples: Sequence[SensorSample], *, debounced: bool = True
          ) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    """-> ``(rises, falls)`` transition instants over VALID samples only.

    Invalid samples are SKIPPED, not treated as a level: ``ball_held_valid ==
    false`` is UNKNOWN, and letting UNKNOWN act as EMPTY would mint a departure
    edge out of a telemetry hiccup (C-POSSESS-1 § 2, and D13 here).

    The same rule ``ball_possession.HandBallSensorSource`` applies, so this
    ledger and a live verdict are two views of one stream rather than two
    conventions.
    """
    rises: List[float] = []
    falls: List[float] = []
    prev: Optional[bool] = None
    for s in samples:
        if not s.valid:
            continue
        cur = bool(s.held if debounced else s.raw)
        if prev is None:
            prev = cur
            continue
        if cur != prev:
            prev = cur
            (rises if cur else falls).append(float(s.t))
    return tuple(rises), tuple(falls)


# ── Record-specific search windows (MEASURED, not inherited) ──────────────────
# The arrival and retention windows are the SHIPPED ``JB_BD_*`` constants and
# are INJECTED (see SensorWindows) so this module's two verdict paths cannot
# disagree. The window below has no shipped equivalent — it exists only to
# bound the DEPARTURE search — so it is pinned here, from measurement. The
# departure LEAD is RELEASE_GUARD_S directly (no second name — see its own
# docstring above); ``DEPARTURE_LEAD_S`` was an alias of it in toss_record.py
# and was dropped at the 2026-09-13 merge (R3-f1, the learning-stack deletion).
#
# Measured on 2026-08-10_16-30-44 (2026-08-10): across all 32 ``jugglebot``
# self-tosses the RAW held->empty edge lands at **+148 .. +212 ms** after the
# announced ``throw_time`` (median +172 ms); the debounced edge at +385 .. +455.
# Every ``ball_butler`` announcement in the same bag has NO departure edge,
# which is the correct answer — a Butler ball never leaves our cup.
#
#: How far after the announced throw_time a departure edge is still ours. 4.7x
#: the measured worst case (+212 ms), and the window is additionally clamped to
#: end at ``landing - arrival_lead`` so it can never overlap the arrival search
#: and mistake a bounce-out for a departure.
DEPARTURE_WINDOW_S = 1.00


class SensorWindows(NamedTuple):
    """The hand-sensor windows, INJECTED — never imported.

    ``arrival_lead_s`` / ``arrival_window_s`` / ``retention_window_s`` are the
    shipped ``JB_BD_*`` values; both callers build this from the generated config
    (the node from ``hw``, the miner from ``hw``) and a drift-guard test pins the
    miner's construction equal to ``hw``. Same discipline as
    ``ball_possession.HandBallSensorSource``, and for the same reason: a bench
    instrument that scores a capture against windows the robot never ran agrees
    with the robot only by coincidence.

    **Do NOT substitute ``toss_sequencer.CATCH_CONFIRM_WINDOW_S`` for
    ``arrival_window_s``.** That constant is the FSM's terminal deadline, not a
    sensor search window; the plan's § 3.3 draft used it. Measured on the
    reference bag (2026-08-10, 25 catches): at the 0.70 s it then carried,
    exactly ONE arrival relabels MISSED — the +798 ms row, which is the
    population MAXIMUM. The runner-up (+675 ms) sits 25 ms inside that boundary,
    so the margin was one session's variation wide.

    > **Updated 2026-08-21 (census D7).** ``CATCH_CONFIRM_WINDOW_S`` is now
    > derived from ``ball_possession.ARRIVAL_BAND_MAX_S`` (0.80 s then, **0.56 s**
    > since the 2026-08-24 post-FW-14 re-measure — the measured band ceiling,
    > rounded up), so it no longer sits BELOW the band it has to outlast. The two
    > constants remain different quantities and must not be substituted for one
    > another: ``ARRIVAL_BAND_MAX_S`` is the band *ceiling* a deadline must
    > clear, 1.50 s is the search *window* sized with margin above it — a margin
    > the re-measure widened from 1.9x to 2.7x, which is why
    > ``JB_BD_ARRIVAL_WINDOW_S`` did NOT follow the band down.
    """
    arrival_lead_s: float
    arrival_window_s: float
    retention_window_s: float
    departure_lead_s: float = RELEASE_GUARD_S
    departure_window_s: float = DEPARTURE_WINDOW_S



#: Tolerance on "this ``ball_held_stamp`` is wall-epoch rather than
#: bridge-boot-relative". Blunt on purpose: a wall stamp sits within a minute
#: of the sample's own ROS stamp, a boot-relative one is a small number of
#: seconds since 1970. One constant, so a caller's own wall-epoch check and
#: the guard inside :func:`poll_dt_steps_ms` cannot drift onto two different
#: notions of which epoch a stamp is in.
STAMP_WALL_TOL_S = 60.0

def _stamp_is_wall(sample: SensorSample) -> bool:
    """Is THIS sample's ``ball_held_stamp`` in the wall epoch? See
    :data:`STAMP_WALL_TOL_S`."""
    return abs(float(sample.stamp) - float(sample.t)) < STAMP_WALL_TOL_S


class PollSteps(NamedTuple):
    """Cleaned ``ball_held_stamp`` advances in ms, plus what was refused.

    The refusals are COUNTED rather than swallowed: a stream that steps
    backwards, or that changes stamp epoch mid-capture, is a finding about the
    bridge clock and not a cadence.
    """
    steps_ms: Tuple[float, ...]
    n_backwards: int
    n_domain_breaks: int


def poll_dt_steps_ms(samples: Sequence[SensorSample]) -> PollSteps:
    """Every MEASURED sensor poll interval in ``samples``, in ms.

    A step is the advance of ``ball_held_stamp`` between two **consecutive
    valid** samples. Three things are deliberately NOT steps, because each one
    turns a clock artefact into a fake cadence:

    **Repeats are not steps.** ``/hand_telemetry`` republishes the cached bit at
    100 Hz, so most consecutive samples carry the SAME stamp. Counting messages
    would report the publish rate and be wrong by the whole factor this
    measurement exists to expose.

    **A backwards step is counted and RESETS the tracker.** The stamp is
    bridge-sourced; a re-anchor makes it go down. A negative folded into the
    statistics drags the median toward zero, and any consumer that DIVIDES by it
    (a "how many polls fit in this window" line) gets an absurd or negative
    count. The sibling census in ``tools/probes/hand_sensor_settle.poll_cadence``
    has always dropped-and-counted these; this is that rule, at the shared
    definition. Dropping the negative is only half of it, though: carrying
    ``prev`` forward onto the LOW stamp makes the very next sample a step of the
    whole re-anchor distance in the POSITIVE direction — a 0.5 s re-anchor mints
    a 500 ms "poll". The clock moved under the measurement, so the next
    difference has no valid reference either; ``prev`` resets exactly as it does
    on an invalid sample, and the one interval that spans the re-anchor is lost
    rather than invented.

    **A step across an invalid span, or across a change of stamp epoch, is not
    a step at all.** ``prev`` resets on any invalid sample: the bridge may have
    polled several times while ``ball_held_valid`` was false, so the stamp
    difference either side of the gap spans an unknown number of polls and is
    not one interval. And ``ball_held_stamp`` is wall-epoch only *after* the
    bridge's wall anchor lands (module docstring, CLOCK DOMAINS) — before it,
    the stamp is boot-relative. A capture that spans the anchor therefore
    contains one step of the whole wall epoch: **+1.79e12 ms**, measured on
    ``2026-08-12_14-55-18`` (17.251 s -> 1786510522.066 s). A cadence is a
    difference, so a wholly boot-relative stream still measures correctly — it
    is only the CHANGE of epoch that has to be refused.

    Pure, and the one definition of a poll interval in this repo. Consumers that
    want a p95 or a max take them from ``steps_ms`` rather than re-deriving the
    steps, so a guard added here reaches all of them.
    """
    steps: List[float] = []
    n_backwards = 0
    n_domain_breaks = 0
    prev: Optional[Tuple[float, bool]] = None   # (stamp, is-wall-epoch)
    for s in samples:
        if not s.valid:
            prev = None
            continue
        stamp = float(s.stamp)
        wall = _stamp_is_wall(s)
        if prev is not None:
            prev_stamp, prev_wall = prev
            if wall != prev_wall:
                n_domain_breaks += 1        # the anchor landed mid-capture
            elif stamp < prev_stamp:
                n_backwards += 1
                prev = None                 # the clock re-anchored under us
                continue
            elif stamp > prev_stamp:
                steps.append((stamp - prev_stamp) * 1e3)
        prev = (stamp, wall)
    return PollSteps(tuple(steps), n_backwards, n_domain_breaks)


def poll_dt_ms_median(samples: Sequence[SensorSample]) -> Optional[float]:
    """MEASURED sensor poll cadence: the median advance of ``ball_held_stamp``
    over valid samples, in ms. ``None`` when no stamp ever advances.

    Not the ``/hand_telemetry`` publish rate — that is 100 Hz and says nothing
    about how often the bridge actually got an SDO reply. On
    2026-08-10_16-30-44 the two differed by 7x (10 ms publish, 71 ms median
    poll, against a configured ``JB_BD_CHECK_INTERVAL_MS`` of 20) — a
    measurement of THAT sitting, not a property of the plant; see the module
    docstring. :func:`poll_dt_steps_ms` defines what counts as a step.
    """
    steps = sorted(poll_dt_steps_ms(samples).steps_ms)
    if not steps:
        return None
    return steps[len(steps) // 2]


def _first_in(instants: Sequence[float], lo: float, hi: float) -> Optional[float]:
    for t in instants:
        if lo <= t <= hi:
            return float(t)
    return None


def _finite_or_none(value: Any) -> Optional[float]:
    """``float(value)`` when it is a real number, else None.

    One helper so None and NaN are treated identically everywhere a SCHEDULE
    instant is optional. A NaN that reached a horizon would compare False against
    everything and silently disable the clamp it was meant to apply — the sibling
    of ``ball_possession.HandBallSensorSource._finite``, and for the same reason."""
    if value is None:
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


class SensorBlock(NamedTuple):
    """The mined sensor block plus the label it implies."""
    fields: Dict[str, Any]
    label: str
    reason: str
    confidence: float


def label_from_sensor(samples: Sequence[SensorSample], *,
                      throw_time: float, landing_time: float,
                      windows: SensorWindows,
                      next_release_time: Optional[float] = None,
                      next_landing_time: Optional[float] = None,
                      prev_landing_time: Optional[float] = None,
                      stamp_wall_anchored: Optional[bool] = None) -> SensorBlock:
    """THE definition of "caught" for the corpus. Pure; one implementation.

    ``samples`` must cover at least the decisive window; extra samples either
    side are ignored. ``throw_time`` / ``landing_time`` are the announcement's
    own stamps in the SAME clock as ``samples[i].t``.

    ``next_release_time`` / ``next_landing_time`` are the NEXT cycle's announced
    instants in that same clock, or ``None`` when this is the last toss of a
    chain. They are the cadence clamp of C-POSSESS-1 § 3.4 and they are what
    keeps this label honest as the dwell shrinks — see "THE CADENCE CLAMP" below.
    The live verdict takes the identical pair through
    ``ball_possession.HandBallSensorSource.observe``; the two implementations of
    "caught" must agree, so they clamp the same way from the same constants.

    **The decisive window** is ``[throw - departure_lead, landing +
    arrival_window + retention_window]``, both ends subject to the clamp —
    release evidence at one end, the latest instant at which CAUGHT can still
    turn into BOUNCED at the other. ``sensor_valid_frac`` is computed over
    exactly that span, because a sensor that was healthy an hour earlier says
    nothing about this toss.

    **The label, in gate order** (plan § 3.3):

    1. ``sensor_valid_frac < 1.0`` in the decisive window ⇒ **UNKNOWN**, and
       UNKNOWN never collapses to a verdict (D13, C-POSSESS-1 § 2). Treating
       "no valid sample" as "no ball" mints a false MISSED on every telemetry
       hiccup — and those false MISSEDs are exactly the records the aim fit wants
       most, because a MISS with a clean mocap fit is the most informative aim
       datum there is.
    2. no departure edge ⇒ **NO_RELEASE**;
    3. departure, no catch edge ⇒ **MISSED**;
    4. departure, catch edge, dropout inside the retention horizon ⇒ **BOUNCED**;
    5. otherwise ⇒ **CAUGHT**.

    Edge TIMES come from the RAW bit and the VERDICT from the DEBOUNCED one
    (D12) — see the module docstring for the measured ~241 ms asymmetric fall lag
    that makes the distinction worth a field rather than a footnote.

    **THE CADENCE CLAMP — why gate 4 is not a fixed 1.5 s (census D1).**
    ``retention_window_s`` was sized 1.5x the longest seat-then-leave the sensor
    has resolved (0.999 s) and justified at the top by a delay floor of 3.5 s
    that no longer exists. At the tuning-phase dwell of **0.49 s** a legitimate
    throw departs the cup 0.49 s after seating — comfortably inside a fixed
    1.5 s window — so gate 4 would fire on EVERY successful cycle and the corpus
    would read ``BOUNCED`` for a perfect session. Worse, that same route is what
    ``on_empty_cup: RELOAD`` uses to decide a ball is on the floor.

    So the horizon closes at ``next_release_time - departure_lead_s`` — the very
    instant the NEXT toss's departure search opens. A fall at or after it is that
    throw's departure by definition and cannot also be this toss's bounce-out.
    Likewise the arrival search is bounded by ``ball_possession.arrival_boundary_t``
    at BOTH ends — ``arr_hi`` where the next row's search opens, ``arr_lo`` where
    the previous row's closed — so no edge is claimed twice (census D2). When the
    horizon leaves no interval at all the bounce test is SKIPPED rather than
    resolved either way: there is nothing to observe, and inventing a BOUNCED
    from arithmetic is the failure this clamp exists to prevent.

    **C-POSSESS-1.C.1 / C.2, added 2026-08-23.** ``arr_hi`` used to be
    ``next_landing_time - arrival_lead_s`` outright, which pays the NEXT row's
    pre-landing guard out of THIS row's measured arrival band and, once the cycle
    period drops under ``ARRIVAL_BAND_MAX_S + arrival_lead_s`` (1.000 s when this
    clause landed, 0.760 s since the 2026-08-24 re-measure), mints a
    **false MISSED** on a catch whose seat edge merely landed in the band's tail.
    ``arrival_boundary_t`` surrenders the guard rather than the band; and where
    the schedule truncates the band anyway (a period under ``ARRIVAL_BAND_MAX_S``
    — 0.800 s then, 0.560 s now, which is BELOW the deferred R6 fork's own
    0.7529 s), gate 3 answers **UNKNOWN, not MISSED**, because a window that
    stopped short of the evidence is not a positive observation of non-arrival.
    ``prev_landing_time`` is the other end of the same boundary: pass the
    previous row's ``landing_time`` and adjacent rows abut exactly.
    """
    dep_lo = throw_time - windows.departure_lead_s
    # Clamped so the departure search can NEVER reach into the arrival search and
    # mistake a bounce-out for a departure. On the shipped 0.8 s flight that
    # leaves +0.60 s of departure window against a measured worst case of
    # +0.212 s (2.8x).
    dep_hi = min(throw_time + windows.departure_window_s,
                 landing_time - windows.arrival_lead_s)
    arr_lo = landing_time - windows.arrival_lead_s
    arr_hi = landing_time + windows.arrival_window_s
    next_land = _finite_or_none(next_landing_time)
    if next_land is not None:
        # Census D2 / clause C.1 — close at the boundary this pair of landings
        # shares, which is where the NEXT row's arrival search opens.
        arr_hi = max(arr_lo, min(arr_hi, arrival_boundary_t(
            landing_time, next_land, windows.arrival_lead_s)))
    prev_land = _finite_or_none(prev_landing_time)
    if prev_land is not None:
        # The identical call the PREVIOUS row made to close its own search. Only
        # ever moves this opening LATER, so its absence is the shipped window.
        arr_lo = max(arr_lo, arrival_boundary_t(
            prev_land, landing_time, windows.arrival_lead_s))
        # Degenerate to a single INSTANT rather than inverting, and degenerate to
        # the same instant `HandBallSensorSource._window` does — the low end wins
        # in both. An inverted search would make every `lo <= t <= hi` vacuously
        # false and mint a MISSED out of arithmetic. Only reachable on a schedule
        # whose next landing precedes this one, i.e. never from a real
        # announcement stream; it is written down because the two implementations
        # are required to clamp the SAME way, including where they fail.
        arr_hi = max(arr_lo, arr_hi)
    # Clause C.2: did the search outlast the ball's measured band? A window the
    # SCHEDULE truncated inside the band has not seen the evidence out, so "no
    # rise" is not a positive observation of non-arrival — gate 3 below.
    band_watched_out = arr_hi >= landing_time + ARRIVAL_BAND_MAX_S
    next_rel = _finite_or_none(next_release_time)
    win_lo = dep_lo
    win_hi = arr_hi + windows.retention_window_s
    if next_rel is not None:
        # Census D1 — and never SHORTER than the arrival search itself, or the
        # decisive-sample filter would starve the very gates it feeds.
        win_hi = max(arr_hi, min(win_hi, next_rel - windows.departure_lead_s))

    decisive = [s for s in samples if win_lo <= s.t <= win_hi]
    n = len(decisive)
    n_valid = sum(1 for s in decisive if s.valid)
    valid_frac = (float(n_valid) / n) if n else 0.0

    rises_deb, falls_deb = edges(decisive, debounced=True)
    rises_raw, falls_raw = edges(decisive, debounced=False)

    t_dep_raw = _first_in(falls_raw, dep_lo, dep_hi)
    t_dep_deb = _first_in(falls_deb, dep_lo, dep_hi)
    t_catch_raw = _first_in(rises_raw, arr_lo, arr_hi)
    t_catch_deb = _first_in(rises_deb, arr_lo, arr_hi)

    # The retention HORIZON (C-POSSESS-1 § 3.4), mirroring
    # HandBallSensorSource._retention_horizon exactly.
    ret_hi = None
    if t_catch_deb is not None:
        ret_hi = t_catch_deb + windows.retention_window_s
        if next_rel is not None:
            ret_hi = min(ret_hi, next_rel - windows.departure_lead_s)

    t_dropout = None
    if t_catch_deb is not None and ret_hi > t_catch_deb:
        t_dropout = _first_in(falls_deb, t_catch_deb, ret_hi)

    held_at_dispatch = None
    for s in decisive:
        if s.valid and s.t <= throw_time:
            held_at_dispatch = bool(s.held)
    held_at_retention = None
    if t_catch_deb is not None and ret_hi > t_catch_deb:
        for s in decisive:
            if s.valid and s.t <= ret_hi:
                held_at_retention = bool(s.held)

    edge_count = len(rises_deb) + len(falls_deb)

    fields: Dict[str, Any] = {
        'sensor_valid_frac': valid_frac,
        'sensor_n_samples': n,
        'sensor_held_at_dispatch': held_at_dispatch,
        't_departure_raw_ros': t_dep_raw,
        't_departure_deb_ros': t_dep_deb,
        't_catch_raw_ros': t_catch_raw,
        't_catch_deb_ros': t_catch_deb,
        't_dropout_ros': t_dropout,
        'held_at_catch_plus_retention': held_at_retention,
        'sensor_edge_count': edge_count,
        'sensor_poll_dt_ms_median': poll_dt_ms_median(decisive),
        'ball_held_stamp_wall_anchored': stamp_wall_anchored,
    }

    # Gate 1 — UNKNOWN, and it never collapses.
    if n == 0 or valid_frac < 1.0:
        return SensorBlock(fields, LABEL_UNKNOWN,
                           'sensor_valid_frac={:.3f} over {} samples'.format(
                               valid_frac, n),
                           0.0)
    # Gate 2 — the ball never left the cup.
    if t_dep_deb is None and t_dep_raw is None:
        return SensorBlock(fields, LABEL_NO_RELEASE,
                           'no held->empty edge in [{:+.3f}, {:+.3f}] s of the '
                           'announced throw'.format(dep_lo - throw_time,
                                                    dep_hi - throw_time),
                           1.0)
    # Gate 3 — it left and never came back.
    if t_catch_deb is None:
        dep_at = (t_dep_raw if t_dep_raw is not None else t_dep_deb) - throw_time
        if not band_watched_out:
            # C-POSSESS-1.C.2. The search closed inside the measured band — the
            # next landing truncated it, or `arrival_window_s` is configured
            # under the band — so the corpus does not know whether the ball
            # missed or whether we looked away. MISSED is a POSITIVE claim and is
            # not available; UNKNOWN never collapses to one (gate 1's rule, same
            # reason).
            return SensorBlock(
                fields, LABEL_UNKNOWN,
                'departure at {:+.3f} s, no empty->held edge in [{:+.3f}, '
                '{:+.3f}] s of landing — but the search was band clamped, '
                'closing at {:+.3f} s short of the measured band ceiling '
                '{:+.3f} s, so non-arrival was never observed'.format(
                    dep_at, arr_lo - landing_time, arr_hi - landing_time,
                    (landing_time + ARRIVAL_BAND_MAX_S) - arr_hi,
                    ARRIVAL_BAND_MAX_S),
                0.0)
        return SensorBlock(fields, LABEL_MISSED,
                           'departure at {:+.3f} s, no empty->held edge in '
                           '[{:+.3f}, {:+.3f}] s of landing'.format(
                               dep_at,
                               arr_lo - landing_time, arr_hi - landing_time),
                           1.0)
    # Gate 4 — it arrived and then left again inside the retention HORIZON. A
    # departure at or after `next_release - departure_lead` is the next toss's
    # release and was excluded above, by construction rather than by tolerance.
    if t_dropout is not None:
        return SensorBlock(fields, LABEL_BOUNCED,
                           'arrival at {:+.3f} s, dropout {:+.3f} s later '
                           '(retention horizon {:.2f} s)'.format(
                               t_catch_deb - landing_time,
                               t_dropout - t_catch_deb,
                               ret_hi - t_catch_deb),
                           1.0)
    # Gate 5.
    if ret_hi <= t_catch_deb:
        # The cadence leaves NO interval between the seat edge and the next
        # release, so retention was never observable. CAUGHT is still the right
        # label — the ball demonstrably arrived — but the reason must say the
        # bounce test never ran, and the confidence must not claim it did.
        return SensorBlock(fields, LABEL_CAUGHT,
                           'arrival at {:+.3f} s; retention NOT OBSERVABLE (the '
                           'next release at {:+.3f} s leaves no horizon)'.format(
                               t_catch_deb - landing_time,
                               (next_rel or float('nan')) - landing_time),
                           0.5)
    return SensorBlock(fields, LABEL_CAUGHT,
                       'arrival at {:+.3f} s, held through retention '
                       '({:.2f} s horizon)'.format(
                           t_catch_deb - landing_time, ret_hi - t_catch_deb),
                       1.0)


# ── The join ──────────────────────────────────────────────────────────────────

def names_by_origin(origin: str) -> Tuple[str, ...]:
    """Field names with the given origin — ``'D'`` / ``'M'`` / ``'DM'`` / ``'X'``.

    Public because it is how a corpus reader answers *"which half of the record
    did this number come from?"* without a per-field provenance marker in every
    row. The design (§ 3.3) asks the declaration to upgrade fields from mined to
    declared; the row-level ``record_provenance`` plus this origin table says the
    same thing deterministically and at a fraction of the schema cost — on a
    ``mined-only`` row every ``D`` field is null by construction, and on a
    ``declared+mined`` row the declaration won every ``D`` and ``DM`` field with
    any conflict already listed in ``disagreement``.
    """
    return tuple(f.name for f in FIELDS if f.origin == origin)


def _both_origin_names() -> Tuple[str, ...]:
    return tuple(f.name for f in FIELDS if f.origin == 'DM')


def _declared_names() -> Tuple[str, ...]:
    return tuple(f.name for f in FIELDS if f.origin == 'D')


def _mined_names() -> Tuple[str, ...]:
    return tuple(f.name for f in FIELDS if f.origin == 'M')


def _derived_names() -> Tuple[str, ...]:
    return tuple(f.name for f in FIELDS if f.origin == 'X')


def join(declarations: Sequence[Dict[str, Any]],
         mined: Sequence[Dict[str, Any]],
         *, tol_s: float = JOIN_TOL_S) -> List[Dict[str, Any]]:
    """Join declarations to mined rows on ``announce_throw_time_ros``.

    -> one merged record per mined row, plus one per unmatched declaration.
    Nothing is dropped: a silently missing row is how a replay overstates its own
    agreement, and an unmatched declaration is itself a finding (the bag lost the
    announcement, or the toss never announced).

    Merge rule, by field origin:

    ``D``   the declaration wins outright — the node is the only witness of what
            it commanded;
    ``M``   the mined value wins outright;
    ``DM``  the declaration wins, and a difference is appended to
            ``disagreement`` rather than averaged away. A ``DM`` field exists
            precisely so the two halves can be cross-checked; resolving a
            conflict silently throws away the only signal the redundancy buys.
    """
    out: List[Dict[str, Any]] = []
    remaining = list(range(len(declarations)))
    d_names = _declared_names()
    m_names = _mined_names()
    dm_names = _both_origin_names()
    x_names = _derived_names()

    for m in mined:
        key = m.get('announce_throw_time_ros')
        best = None
        best_dt = None
        if key is not None:
            for idx in remaining:
                k = declarations[idx].get('announce_throw_time_ros')
                if k is None:
                    continue
                dt = abs(float(k) - float(key))
                if dt <= tol_s and (best_dt is None or dt < best_dt):
                    best, best_dt = idx, dt
        rec = blank_record()
        for name in m_names:
            rec[name] = m.get(name)
        # Every X (derived) field the miner already computed carries over.
        # Iterating the ORIGIN rather than a hand-written name list is what stops
        # a field added to FIELDS from being silently dropped at the join — the
        # three the join owns itself are overwritten below.
        for name in x_names:
            if m.get(name) is not None:
                rec[name] = m.get(name)
        disagreement: List[str] = []
        if best is None:
            for name in dm_names:
                rec[name] = m.get(name)
            rec['record_provenance'] = PROV_MINED
            rec['join_residual_ms'] = None
        else:
            d = declarations[best]
            remaining.remove(best)
            for name in d_names:
                rec[name] = d.get(name)
            for name in dm_names:
                dv, mv = d.get(name), m.get(name)
                rec[name] = dv if dv is not None else mv
                # The JOIN KEY is excluded from the disagreement scan: the two
                # halves are matched ON it, within tol_s, so any difference is
                # by definition the join residual — which has its own field.
                # Reporting it here would duplicate join_residual_ms on every
                # single row and bury the disagreements that mean something.
                if name == 'announce_throw_time_ros':
                    continue
                if dv is not None and mv is not None and not _agree(dv, mv):
                    disagreement.append('{}: declared={!r} mined={!r}'.format(
                        name, dv, mv))
            rec['record_provenance'] = PROV_BOTH
            rec['join_residual_ms'] = float(best_dt) * 1e3
        rec['schema'] = SCHEMA
        rec['disagreement'] = disagreement
        if rec.get('toss_uid') is None:
            rec['toss_uid'] = m.get('toss_uid') or _mined_uid(m)
        if rec.get('action') is None:
            rec['action'] = m.get('action') or 'unknown'
        if rec.get('outcome') is None:
            rec['outcome'] = m.get('outcome') or 'UNDECLARED'
        out.append(rec)

    for idx in remaining:
        d = declarations[idx]
        rec = blank_record()
        for name in d_names + dm_names:
            rec[name] = d.get(name)
        rec['schema'] = SCHEMA
        rec['record_provenance'] = PROV_DECLARED
        rec['disagreement'] = []
        out.append(rec)
    return out


def _agree(a: Any, b: Any) -> bool:
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) \
            and not isinstance(a, bool) and not isinstance(b, bool):
        return abs(float(a) - float(b)) <= 1e-6 * max(1.0, abs(float(a)))
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_agree(x, y) for x, y in zip(a, b))
    return a == b


def _mined_uid(m: Dict[str, Any]) -> str:
    """A stable id for a row nobody declared: the join key itself.

    Millisecond resolution — the join tolerance is 5 ms, so two rows that would
    collide here would also have joined to the same declaration.
    """
    key = m.get('announce_throw_time_ros')
    if key is None:
        return 'mined-unkeyed'
    return 'mined-{:.3f}'.format(float(key))

