"""Throw identity on the ``bb/throw_outcome`` String (JB-21).

Pure Python, no ROS import: ``ball_butler_node`` (producer) and ``skill_node``
(consumer) both import it, and the tests drive it directly.

The wire is a ``std_msgs/String`` whose LEADING TOKEN is the
``BallButlerCommandOutcome`` member name (``NAME (axis=..., detail1=...)``);
every consumer compares that token by name, so the identity is appended as a
SUFFIX at the very end and the prefix is never touched::

    THROW_REJECTED_BAD_STATE (axis=n/a, detail1=0) release_wall_s=1761957123.456789

``release_wall_s`` is the absolute release instant (ROS wall clock,
CLOCK_REALTIME) that ``ball_butler_node`` put on the action goal -- the one
number that is unique per Throw and that the consumer can bound from what it
asked for (its own ``now + throw_delay_s``). A message WITHOUT the suffix (an
older producer) parses to ``None`` and is never rejected by `verdict_matches`.
"""
from __future__ import annotations

import re
from typing import Optional, Tuple

#: Suffix key; also the token the regex anchors on.
RELEASE_KEY = 'release_wall_s'

_SUFFIX_RE = re.compile(r' ' + RELEASE_KEY + r'=(-?\d+(?:\.\d+)?)\s*$')

#: Window around the requested release instant: the lower side covers BB's
#: command-to-release latency compensation (44 ms) with margin, the upper side
#: the `bb/throw_at_target` round trip (the producer stamps its OWN `now`,
#: somewhere inside the consumer's blocking call) plus the same margin.
WINDOW_BEFORE_S = 0.25
WINDOW_AFTER_S = 0.75


def tag_message(message: str, release_wall_s: float) -> str:
    """``message`` with the identity suffix appended; unchanged when the throw
    has no release instant (``release_wall_s <= 0``, i.e. an aim)."""
    message = str(message)
    if not release_wall_s or release_wall_s <= 0.0:
        return message
    return '%s %s=%.6f' % (message, RELEASE_KEY, float(release_wall_s))


def parse_release_wall_s(text: object) -> Optional[float]:
    """The release instant a ``bb/throw_outcome`` String carries, or ``None``
    when it has no (well-formed) suffix."""
    m = _SUFFIX_RE.search(str(text))
    return float(m.group(1)) if m else None


def release_window(now_wall_s: float, delay_s: float) -> Tuple[float, float]:
    """Wall-clock window the release of the throw requested NOW with
    ``delay_s`` must fall in."""
    t = float(now_wall_s) + float(delay_s)
    return (t - WINDOW_BEFORE_S, t + WINDOW_AFTER_S)


def verdict_matches(text: object,
                    window: Optional[Tuple[float, float]]) -> bool:
    """True when the verdict may be attributed to the throw ``window`` was
    built for. No window (nothing recorded) or no suffix (older producer):
    True, i.e. today's behaviour. Only a verdict that CARRIES an identity
    outside the window is rejected."""
    if window is None:
        return True
    rel = parse_release_wall_s(text)
    if rel is None:
        return True
    return window[0] <= rel <= window[1]
