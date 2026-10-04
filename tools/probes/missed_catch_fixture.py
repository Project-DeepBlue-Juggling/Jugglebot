"""Generate ``tests/motion/fixtures/missed_catch_20261004.csv`` — one row per
REAL ``/hand_telemetry`` sample, for every fed-columns attempt of the
2026-10-04 evening sitting (R5 sitting 4), for the ``MISSED_CATCH`` evidence
rule's replay test (``tests/motion/test_skills_executor.py::
test_missed_catch_fixture_replay``).

THIS IS A BAG REPLAY, NOT A SYNTHESIS (2026-10-04, R5 sitting 4, unit B1b).
The previous generator of this fixture synthesised a representative 100 Hz
``(valid, raw, held)`` sequence per attempt from ``report_a1.md`` Q1's table
instead of decoding the bags. CLAUDE.md's rule is a production-faithful
offline replay, so this version re-opens the two sitting bags and reads the
REAL raw bits.

Bags + logs (``~/.ros/log/<dir>/launch.log``, ``~/Desktop/rosbags/<bag>``):

* L3 — ``2026-10-04-19-52-08-155768-jetson-3952806`` /
  ``2026-10-04_19-52-08`` (419 MB): 20 fed-columns attempts (owner # 1-20).
* L4 — ``2026-10-04-20-10-26-379875-jetson-3962551`` /
  ``2026-10-04_20-10-26`` (54 MB): 2 more (owner # R1, R2), before a ball
  pinched under the hand and the owner E-STOPped.

Per-attempt landing L: the real scheduled feed-catch landing, read from each
log's ``CATCH-AIM skill 1: ... t_land=<abs s>`` line that immediately
follows its ``columns feed accepted`` line (one pair per owner-numbered
attempt, in log order — same regex family as ``a2_catch_timing.py`` in
``temp/probes/`` scratch, R5 sitting-4 analysis). This is the ball-B feed
catch's own committed landing instant, NOT the ``columns feed accepted``
line's timestamp (that is when the skill was installed, well before the
ball lands).

Window per attempt: real ``/hand_telemetry`` samples (``ball_held``,
``ball_held_raw``, ``ball_held_valid``) with LOG-arrival time (the live ring
buffer's clock, ``skill_node._on_hand_telemetry``) in ``[L - 0.5, L + 1.0]``
s — comfortably wider than the ``[-0.30, +0.55]`` s span the live constants
ever open a fed-columns catch's evidence window to, so every real sample the
rule could query is present. ``t_rel_s`` in the CSV is this log-arrival time
minus L (NOT the header stamp — the live rule reads the ring buffer's
arrival clock, so the fixture must replay on the same clock; see
``tools/probes/missed_catch_fixture.py`` module docstring in the test file's
own comment for why the two differ).

Label: 'caught' iff any VALID sample with ``ball_held_raw`` True lands in
``[L, L + 0.5]`` s (Q3's own finding, ``report_a1.md`` Q1/Q2: the cup sensor
alone separates the groups exactly — zero raw SEATED samples on every missed
feed, one between +0.163 and +0.272 s on every real catch). 'miss' otherwise.

PER-ATTEMPT WINDOW COLUMNS (2026-10-04, R5 sitting 4, unit B1c): every row of
an attempt also carries ``t_open_rel_s``/``t_close_rel_s`` — the REAL
``[t_open, t_close]`` the live rule would arm for that attempt's own catch,
relative to L, computed with the SAME arithmetic
:meth:`SkillExecutor._register_catch` uses (its ``MISSED_CATCH_*`` constants
are imported from ``executor.py`` rather than restated, so the two can never
drift; the two PURE helpers it calls, ``_latest_release_before`` /
``_next_landing_other_ball``, take a ``Schedule`` of ``Skill`` objects, which
this log-text replay does not have, so this module MIRRORS their arithmetic
over the launch log's own dispatch lines instead of calling them):

* ``t_open_rel_s`` = (the latest scheduled release of ANY ball before L) -
  L + :data:`MISSED_CATCH_OPEN_AFTER_RELEASE_S`. The release instant is read
  from the log's own ``skill announced ball %d: release in %.3f s`` line
  (the SCHEDULED release — ``now_ros + delta_s`` at the moment the skill is
  dispatched, :meth:`SkillNode.announce_throw` — not the raw bit's later
  physical fall): the last such line between this attempt's ``columns feed
  accepted`` and its own ``CATCH-AIM skill 1`` line, which schedule order
  guarantees is ball A's throw-1 release immediately before this feed catch.
* ``t_close_rel_s`` = ``min(DWELL_S + MISSED_CATCH_CLOSE_AFTER_RELEASE_S,
  (other ball's next scheduled landing - L) -
  MISSED_CATCH_OTHER_LANDING_GUARD_S)`` — ``DWELL_S`` (0.27 s,
  ``skill_node._DEFAULT_DWELL_S``) is this ball's own carried release
  relative to L and is a fixed session constant, not read per attempt; the
  other ball's next landing is read from the next ``CATCH-AIM skill N:
  ... t_land=`` line after this one whose preceding ``install_segment CATCH
  ball`` tag names ball 0 (A) — ball A always holds P2 and ball B is always
  the one fed, so the other ball is always id 0.

Run (project venv active, from the repo root)::

    python tools/probes/missed_catch_fixture.py

Regenerates the committed CSV in place from the two bags above (a few
minutes: it reads the 419 MB L3 bag end to end for ``/hand_telemetry``).
"""
from __future__ import annotations

import csv
import glob
import os
import re
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# jugglebot.* is a ROS2 package dir, but executor.py is pure Python (no ROS
# imports) by its own module contract -- safe to import here for the three
# MISSED_CATCH_* constants, so this replay can never drift from the live
# rule's own numbers. insert(0, ...) -- not append -- because a stale
# colcon install of jugglebot (from the MAIN checkout, PYTHONPATH) is
# already on sys.path and lacks motion.skills entirely; same pattern as
# every other bag probe here (e.g. hand_jam_replay.py).
sys.path.insert(0, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'))
from jugglebot.motion.skills.executor import (  # noqa: E402
    MISSED_CATCH_OPEN_AFTER_RELEASE_S, MISSED_CATCH_CLOSE_AFTER_RELEASE_S,
    MISSED_CATCH_OTHER_LANDING_GUARD_S)

#: Ball B's carried release relative to its own landing L
#: (``skill_node._DEFAULT_DWELL_S``) — a fixed session constant for every
#: attempt here, not re-measured per attempt.
_DWELL_S = 0.27

#: The other ball (A always holds P2; B is always the one fed to P1 here).
_OTHER_BALL_ID = 0

_RE_FEED = re.compile(r'columns feed accepted')
_RE_AIM = re.compile(r'CATCH-AIM skill 1: .*\bt_land=([\d.]+)')
_RE_AIM_ANY = re.compile(r'CATCH-AIM skill \d+: .*\bt_land=([\d.]+)')
#: Both spellings: pre-2026-10-05 logs say ``ball <id>`` (schedule id 0/1);
#: later logs say ``Ball <n>`` (the owner's operator name, n = id + 1 --
#: ``schedule.ball_label``). :func:`_ball_id_of` maps either to the id.
_RE_CATCH_INSTALL = re.compile(r'install_segment CATCH (ball|Ball) (\d+):')
_RE_ANNOUNCE = re.compile(
    r'^[\d.]+ \S+ \S+ \[([\d.]+)\] \[skill_node\]: '
    r'skill announced (?:ball|Ball) \d+: release in ([\d.]+) s')


def _ball_id_of(word: str, number: str) -> int:
    """Schedule ball id from a log line's ``ball <id>`` / ``Ball <n>`` text."""
    return int(number) - 1 if word == 'Ball' else int(number)

#: (tag, launch.log, bag dir, ordered owner attempt ids for that log's
#: 'columns feed accepted' lines, in log order).
_RUNS = (
    ('L3',
     os.path.expanduser('~/.ros/log/2026-10-04-19-52-08-155768-jetson-3952806/launch.log'),
     os.path.expanduser('~/Desktop/rosbags/2026-10-04_19-52-08'),
     list(range(1, 21))),
    ('L4',
     os.path.expanduser('~/.ros/log/2026-10-04-20-10-26-379875-jetson-3962551/launch.log'),
     os.path.expanduser('~/Desktop/rosbags/2026-10-04_20-10-26'),
     ['R1', 'R2']),
)

_WINDOW_BEFORE_S = 0.5
_WINDOW_AFTER_S = 1.0
#: sub-window used only to decide the miss/caught LABEL (Q3: a real catch's
#: first raw SEATED sample falls between +0.163 and +0.272 s of L).
_SEAT_LO_S, _SEAT_HI_S = 0.0, 0.5

_OUT = os.path.join(_REPO, 'tests', 'motion', 'fixtures', 'missed_catch_20261004.csv')


def _feed_landings(lines):
    """Ordered list of ``(L, feed_idx, aim_idx)``: one 'CATCH-AIM skill 1'
    ``t_land`` per 'columns feed accepted' line, in log order, plus both
    lines' indices into ``lines`` so the window helpers below can scan
    around them."""
    out = []
    feed_idx = None
    for i, line in enumerate(lines):
        if _RE_FEED.search(line):
            feed_idx = i
            continue
        if feed_idx is not None:
            m = _RE_AIM.search(line)
            if m:
                out.append((float(m.group(1)), feed_idx, i))
                feed_idx = None
    return out


def _latest_release_before(lines, feed_idx, aim_idx, L):
    """The latest scheduled release abs time STRICTLY BEFORE ``L`` from a
    'skill announced ... release in D s' line between ``feed_idx`` and
    ``aim_idx`` (inclusive) -- mirrors :func:`executor._latest_release_before`
    exactly (max of every release < ``t_before``), not "the last matching
    line by log position": LOG order is not release-time order here --
    the feed catch itself (``CATCH-AIM skill 1``) carries its OWN future
    release (``then_throw``), announced a few lines before the CATCH-AIM
    line but for an instant well AFTER ``L`` (that is
    :data:`MISSED_CATCH_CLOSE_AFTER_RELEASE_S`'s own anchor, computed
    separately below, via ``_DWELL_S`` -- not this one). ``None`` if no
    announce line in range describes a release before ``L``."""
    candidates = []
    for line in lines[feed_idx:aim_idx + 1]:
        m = _RE_ANNOUNCE.search(line)
        if m:
            release_abs = float(m.group(1)) + float(m.group(2))
            if release_abs < float(L):
                candidates.append(release_abs)
    return max(candidates) if candidates else None


def _other_ball_next_landing(lines, aim_idx, stop_idx):
    """The first ``CATCH-AIM skill N`` landing after ``aim_idx`` (and before
    ``stop_idx``) whose preceding ``install_segment CATCH ball`` tag names
    :data:`_OTHER_BALL_ID`. ``None`` if absent in range."""
    last_catch_ball = None
    for line in lines[aim_idx + 1:stop_idx]:
        m = _RE_CATCH_INSTALL.search(line)
        if m:
            last_catch_ball = _ball_id_of(m.group(1), m.group(2))
            continue
        m = _RE_AIM_ANY.search(line)
        if m and last_catch_ball == _OTHER_BALL_ID:
            return float(m.group(1))
    return None


def _hand_samples(bag_dir, lo, hi):
    """(log_arrival_s, header_stamp_s, valid, raw, held) for every
    ``/hand_telemetry`` sample with log-arrival time in [lo, hi]."""
    from mcap_ros2.reader import read_ros2_messages
    rows = []
    for mcap in sorted(glob.glob(os.path.join(bag_dir, '*.mcap'))):
        for m in read_ros2_messages(mcap, topics=['/hand_telemetry']):
            tl = m.log_time_ns * 1e-9
            if tl < lo or tl > hi:
                continue
            r = m.ros_msg
            t = r.timestamp.sec + 1e-9 * r.timestamp.nanosec
            rows.append((tl, t, bool(r.ball_held_valid), bool(r.ball_held_raw),
                        bool(r.ball_held)))
    rows.sort(key=lambda row: row[0])
    return rows


def main() -> None:
    rows_out = []
    summary = []
    for tag, log_path, bag_dir, attempt_ids in _RUNS:
        with open(log_path, errors='replace') as f:
            lines = f.readlines()
        hits = _feed_landings(lines)
        if len(hits) != len(attempt_ids):
            raise SystemExit(
                '%s: found %d feed landings in %s, expected %d (attempt ids %r)'
                % (tag, len(hits), log_path, len(attempt_ids), attempt_ids))
        landings = [h[0] for h in hits]
        lo = min(landings) - _WINDOW_BEFORE_S - 1.0
        hi = max(landings) + _WINDOW_AFTER_S + 1.0
        samples = _hand_samples(bag_dir, lo, hi)
        for i, (attempt, (L, feed_idx, aim_idx)) in enumerate(
                zip(attempt_ids, hits)):
            stop_idx = hits[i + 1][1] if i + 1 < len(hits) else len(lines)
            release_abs = _latest_release_before(lines, feed_idx, aim_idx, L)
            if release_abs is None:
                raise SystemExit(
                    '%s attempt %s: no "skill announced" release line found '
                    'between the feed-accepted and CATCH-AIM lines (lines '
                    '%d-%d) -- cannot anchor t_open' % (tag, attempt,
                                                        feed_idx, aim_idx))
            other_landing_abs = _other_ball_next_landing(lines, aim_idx,
                                                         stop_idx)
            t_open_rel_s = (release_abs - L) + MISSED_CATCH_OPEN_AFTER_RELEASE_S
            close_opt1 = _DWELL_S + MISSED_CATCH_CLOSE_AFTER_RELEASE_S
            if other_landing_abs is not None:
                close_opt2 = ((other_landing_abs - L)
                             - MISSED_CATCH_OTHER_LANDING_GUARD_S)
                t_close_rel_s = min(close_opt1, close_opt2)
            else:
                t_close_rel_s = close_opt1
            t_open_rel_s = round(t_open_rel_s, 4)
            t_close_rel_s = round(t_close_rel_s, 4)

            win = [row for row in samples
                   if L - _WINDOW_BEFORE_S <= row[0] <= L + _WINDOW_AFTER_S]
            seated = [row for row in win
                      if row[2] and row[3] and L + _SEAT_LO_S <= row[0] <= L + _SEAT_HI_S]
            label = 'caught' if seated else 'miss'
            first_seat = (seated[0][0] - L) if seated else None
            summary.append((tag, attempt, L, label, first_seat, len(win),
                           t_open_rel_s, t_close_rel_s))
            for (tl, _t, valid, raw, held) in win:
                rows_out.append((attempt, label, round(tl - L, 4), valid, raw,
                                held, t_open_rel_s, t_close_rel_s))

    out_path = os.path.normpath(_OUT)
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['attempt', 'label', 't_rel_s', 'valid', 'raw', 'held',
                   't_open_rel_s', 't_close_rel_s'])
        for row in rows_out:
            w.writerow(row)

    n_miss = sum(1 for s in summary if s[3] == 'miss')
    n_caught = sum(1 for s in summary if s[3] == 'caught')
    print('wrote %d rows (%d attempts: %d miss, %d caught) to %s'
         % (len(rows_out), len(summary), n_miss, n_caught, out_path))
    for (tag, attempt, L, label, first_seat, n, t_open_rel_s,
        t_close_rel_s) in summary:
        print('  %-2s attempt %-3s L=%.3f label=%-6s first_seat=%s '
             'n_samples=%d window=[%+.3f, %+.3f]'
             % (tag, attempt, L, label,
                'none' if first_seat is None else '%+.3f' % first_seat, n,
                t_open_rel_s, t_close_rel_s))


if __name__ == '__main__':
    main()
