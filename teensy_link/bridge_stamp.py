"""Bridge-tick time for the Jetson-stamped streams (BUG-134).

One pure function, the single enforcement point for "may a Telemetry
``t_teensy_us`` be published as a bridge stamp?". No ROS imports.
"""
from __future__ import annotations

from typing import Optional

# A bridge clock more than this far from the Jetson clock is not on the shared
# epoch (uptime-epoch before the first sync anchor, or far off after a step).
MAX_SKEW_US = 1_000_000


def bridge_stamp_or_none(t_teensy_us: Optional[int], time_synced: bool,
                         now_jetson_us: int,
                         max_skew_us: int = MAX_SKEW_US) -> Optional[int]:
    """Return the bridge wall-clock time (us) of a telemetry tick, or None.

    None (the caller then publishes valid=False and a zero stamp) when:

    * ``time_synced`` is false: the bridge has no wall anchor (or it went stale),
      so its clock reads uptime, a 1970-era value;
    * ``t_teensy_us`` is missing or zero: no tick time was latched;
    * ``abs(t_teensy_us - now_jetson_us) > max_skew_us``: the clock is not on the
      Jetson epoch even though the flag is set (a step, a stale anchor). A skew of
      exactly ``max_skew_us`` is accepted.
    """
    if not time_synced:
        return None
    if not t_teensy_us:
        return None
    t = int(t_teensy_us)
    if t <= 0 or abs(t - int(now_jetson_us)) > int(max_skew_us):
        return None
    return t
