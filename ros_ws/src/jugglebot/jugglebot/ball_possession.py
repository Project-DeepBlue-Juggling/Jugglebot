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
from typing import Deque, NamedTuple, Optional, Sequence, Tuple

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

# ── Live cup evidence (the ball-evidence PRECONDITION, distinct from ARRIVAL) ──
# ARRIVAL answers "did a ball come in around the predicted landing"; EVIDENCE
# answers "is a ball in the cup right now". The toss's CHECKING gate needs the
# second one and nothing else: it runs before any ball flies.
EVIDENCE_SEATED = 'SEATED'
EVIDENCE_EMPTY = 'EMPTY'
EVIDENCE_UNKNOWN = 'UNKNOWN'

# ── Source identifiers (carried in every verdict, so a log line names its author)
SOURCE_TRACKER_ARRIVAL = 'tracker_arrival'
SOURCE_HAND_BALL_SENSOR = 'hand_ball_sensor'
# The merged verdict's author (C-POSSESS-1 § 3.2) — a composite so an operator
# reading one log line can tell it was not minted by either source alone.
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
    ask :meth:`observe` (the two-part verdict, for a given predicted landing) or
    :meth:`evidence` (the live "is a ball in the cup right now" precondition — a
    DIFFERENT question, and the only one CHECKING needs).

    The predicted landing is passed **per query**, never latched. A latched window
    would outlive its goal, and a window that outlives its goal is a window that
    can veto the NEXT ball's tracker CAUGHT with a stale "nothing arrived" — the
    arm/disarm lifecycle bug class, deleted rather than guarded.

    THE THREE STATES, AND WHY UNKNOWN IS LOAD-BEARING
    -------------------------------------------------
    ``ball_held`` on ``/hand_telemetry`` is meaningless unless ``ball_held_valid``
    (plans/archived/2026-08-15 hand-ball-sensor.md § Architecture, normative). Boot before the
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
        the re-seat floor. The lead is a guard, not a fit: nothing was observed
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
        more). 1.50 s is 1.5x that. The upper constraint is the machine's own
        cadence: ``toss_sequencer.MIN_TOSS_THROW_DELAY_S`` is **3.5 s**, so the
        window closes 2.3x before the earliest legitimate departure a throw can
        produce and a real throw can never be mis-read as a bounce-out.

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
        self._first_good: Optional[float] = None   # everything before it is blind
        self._last_t: Optional[float] = None
        self._sample_ok = False

    # ── feed ──────────────────────────────────────────────────────────────────

    def note_sample(self, t: float, held: bool, valid: bool) -> None:
        """One ``/hand_telemetry`` sample. ``t`` is the node's monotonic clock
        (``time.perf_counter``), NOT the bridge stamp: staleness here is about
        whether the Jetson is still hearing the sensor, and the bridge stamp is
        wall-epoch only once the bridge's clock anchor lands.

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
                if self._first_good is None:
                    self._first_good = t
            self._last_t = t
            self._sample_ok = ok

    # ── queries ───────────────────────────────────────────────────────────────

    def evidence(self, now: float) -> str:
        """Live cup state -> ``EVIDENCE_SEATED`` / ``EVIDENCE_EMPTY`` /
        ``EVIDENCE_UNKNOWN``. The toss's ball-evidence precondition, and nothing
        to do with ARRIVAL: it asks what is in the cup right now."""
        with self._mu:
            if self._held is None or not self._sample_ok:
                return EVIDENCE_UNKNOWN
            if self._last_t is None or (float(now) - self._last_t) > self.stale_s:
                return EVIDENCE_UNKNOWN
            return EVIDENCE_SEATED if self._held else EVIDENCE_EMPTY

    def arrival_time(self, landing_t: Optional[float]) -> float:
        """The observed arrival edge's instant (monotonic), or NaN when no edge
        fell inside the window around ``landing_t``. This is the **catch-event
        time** — the first quantity this machine has ever had that says WHEN the
        ball entered the cup, rather than when a tracker stopped seeing it."""
        with self._mu:
            rise = self._arrival_edge(landing_t)
        return rise if rise is not None else float('nan')

    def observe(self, now: float,
                landing_t: Optional[float] = None) -> PossessionVerdict:
        """The two-part verdict for a ball predicted to land at ``landing_t``
        (monotonic; ``None``/NaN = no goal is expecting one, so ARRIVAL is
        honestly ``UNKNOWN`` and the merge falls back to the tracker).

        ``arrival_err_mm`` / ``plane_drop_mm`` are NaN: this source measures the
        cup, not a position, and reporting a zero would claim an accuracy it never
        measured."""
        now = float(now)
        with self._mu:
            arrival, reason = self._arrival_state(now, landing_t)
            retention = self._retention_state(now, arrival, landing_t)
        return PossessionVerdict(
            source=self.name,
            arrival=arrival,
            retention=retention,
            arrival_err_mm=float('nan'),
            plane_drop_mm=float('nan'),
            reason=reason)

    # ── internals (ALL of these run with ``_mu`` already held) ────────────────

    def _window(self, landing_t: Optional[float]) -> Optional[Tuple[float, float]]:
        if landing_t is None or not math.isfinite(float(landing_t)):
            return None
        land = float(landing_t)
        return (land - self.arrival_lead_s, land + self.arrival_window_s)

    def _blind_between(self, a: float, b: float) -> bool:
        """True if the sensor was unreadable at any point in ``[a, b]``."""
        if b < a:
            return False
        if self._first_good is None or self._first_good > a:
            return True                        # never heard from it, or not yet then
        for s, e in self._blind_spans:
            if s <= b and e >= a:
                return True
        return self._blind_from is not None and self._blind_from <= b

    def _arrival_edge(self, landing_t: Optional[float]) -> Optional[float]:
        win = self._window(landing_t)
        if win is None:
            return None
        w0, w1 = win
        for t, held in self._edges:
            if held and w0 <= t <= w1:
                return t
        return None

    def _arrival_state(self, now: float,
                       landing_t: Optional[float]) -> Tuple[str, str]:
        win = self._window(landing_t)
        if win is None:
            return ARRIVAL_UNKNOWN, 'SENSOR_NO_LANDING'
        w0, w1 = win
        rise = self._arrival_edge(landing_t)
        if rise is not None:
            return ARRIVAL_CONFIRMED, 'SENSOR_ARRIVED'
        # No edge YET. Blindness beats both remaining answers: a window we could
        # not watch is UNKNOWN even after it closes.
        if self._blind_between(w0, min(now, w1)):
            return ARRIVAL_UNKNOWN, 'SENSOR_BLIND'
        if now < w1:
            return ARRIVAL_UNKNOWN, 'SENSOR_WINDOW_OPEN'
        return ARRIVAL_REJECTED, 'SENSOR_NO_ARRIVAL'

    def _retention_state(self, now: float, arrival: str,
                         landing_t: Optional[float]) -> str:
        if arrival != ARRIVAL_CONFIRMED:
            return RETENTION_UNKNOWN
        rise = self._arrival_edge(landing_t)
        if rise is None:                       # unreachable; belt for a future edit
            return RETENTION_UNKNOWN
        r1 = rise + self.retention_window_s
        for t, held in self._edges:
            if (not held) and rise < t <= r1:
                return RETENTION_REJECTED      # it arrived and then left: bounce-out
        if self._blind_between(rise, min(now, r1)):
            return RETENTION_UNKNOWN
        if now >= r1:
            return RETENTION_CONFIRMED
        return RETENTION_UNKNOWN               # window still open


def merge_possession(sensor: PossessionVerdict,
                     tracker: PossessionVerdict) -> PossessionVerdict:
    """Combine the PRIMARY sensor verdict with the tracker CORROBORATOR
    (C-POSSESS-1 § 3.2 — read it before changing this).

    Three rules, each stated by the failure it prevents:

    1. **RETENTION is the sensor's, always.** The tracker is contract-forbidden
       from claiming retention (§ 2 consequence 2) — its track freezes at CAUGHT
       and is pruned — so there is nothing to merge. Taking anything else from the
       tracker here would re-open § 7's accepted bounce-out trap.
    2. **ARRIVAL is the sensor's whenever the sensor observed it; the tracker's
       only when the sensor is UNKNOWN.** The sensor reads the cup, so it is the
       one source whose error model does not run through a dead-reckoned free-fall
       extrapolation — that is why it is PRIMARY, and why a valid sensor
       ``ARRIVAL_REJECTED`` vetoes a tracker CAUGHT (the false-CAUGHT class § 7
       sized and accepted). The fallback is not politeness: a blind sensor that
       refused everything would leave the machine strictly LESS capable than
       before the sensor landed, so the degradation path is exactly today's
       tracker-only behaviour.
    3. **``arrival_err_mm`` / ``plane_drop_mm`` stay the TRACKER's, always.** They
       are the catch-accuracy numbers the hardware runbooks score and the sensor
       cannot supply them (§ 3, "the one thing a new source must not forget").
    """
    arrival = (sensor.arrival if sensor.arrival != ARRIVAL_UNKNOWN
               else tracker.arrival)
    author = (SOURCE_MERGED if sensor.arrival != ARRIVAL_UNKNOWN
              else tracker.source)
    return PossessionVerdict(
        source=author,
        arrival=arrival,
        retention=sensor.retention,
        arrival_err_mm=tracker.arrival_err_mm,
        plane_drop_mm=tracker.plane_drop_mm,
        reason='{}/{}'.format(sensor.reason, tracker.reason))


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
    REFUSED (positively not observed) and UNKNOWN (nothing could look). The third
    must never read like the second — an operator who sees "REFUSED" goes hunting
    for a miss, where "UNKNOWN" sends them to the sensor, which is the actual
    fault. Both non-confirmed shapes end "Not counted" so the bench tally is
    unambiguous either way.

    REACHABILITY, so nobody hunts for a line that cannot be emitted: through
    today's only caller (`_possession_confirmed`) the UNKNOWN shape is DEFENSIVE —
    it runs on a tracker CAUGHT, the tracker always has an estimate in hand, and
    the merge falls back to it, so the merged arrival is never UNKNOWN. It exists
    for the first tick-driven consumer that calls `observe` with no tracker
    verdict; rendering that as "REFUSED … 0 mm from the catch point" would be
    actively misleading. The REFUSED shape has TWO authors and prints
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
        return 'info', (
            f'possession UNKNOWN from {verdict.source} — no source positively '
            f'observed an arrival [{verdict.reason}]; tracker cross-check '
            f'{verdict.arrival_err_mm:.0f} mm. UNKNOWN is NOT "no ball": it is '
            f'"nothing could look" (C-POSSESS-1 § 2) — see '
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
