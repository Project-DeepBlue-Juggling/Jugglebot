"""Measured toss-record fixtures — DO NOT hand-edit.

The hand-mined ground truth for 2026-08-10_16-30-44, and the acceptance
this phase is gated on: the miner has to reproduce it from the bag alone.

Regenerate with:
    python tools/probes/toss_record_miner.py --bag 2026-08-10_16-30-44 --emit-fixture

Consumed by tests/ros/test_toss_record_miner.py. Schema: toss_record/1
(jugglebot/toss_record.py). Plan: plans/active/toss-selftuning.md § 3.3.

The bag itself is machine-local and gitignored, so the bag-backed test skips on
any machine without it — including a fresh clone. These constants are what
carries the evidence across that boundary, which is the whole point of the
tests/ros/possession_fixtures.py pattern this file follows.
"""

from __future__ import annotations

#: The reference sitting. Every number below is measured FROM it.
REFERENCE_BAG = '2026-08-10_16-30-44'

#: The whole-bag sensor census. n_catches = empty->held edges,
#: n_departures = held->empty, both on the DEBOUNCED verdict over
#: VALID samples. Hand-mined 2026-08-10: 39 departures, 38 catches.
LEDGER = {'n_samples': 70666, 'n_valid': 70666, 'n_catches': 38, 'n_departures': 39, 'n_segments': 39}

#: Held segments shorter than this are seat-then-leave events.
QUICK_DROP_S = 1.5

#: The three sub-second seat-then-leaves. Phase 1 sized
#: JB_BD_RETENTION_WINDOW_S on these.
QUICK_DROP_DURATIONS_S = (0.5695, 0.9995, 0.9883)

#: MEASURED hand-sensor poll cadence (median ball_held_stamp advance),
#: against a CONFIGURED JB_BD_CHECK_INTERVAL_MS of 20. The gap is the
#: reason sensor_poll_dt_ms_median is a mined field and not an assumption.
POLL_DT_MS_MEDIAN = 70.998

#: Announcements with thrower_name == target_id == the robot.
N_SELF_TOSSES = 31

#: label_from_sensor over every self-toss in the bag.
LABELS = {'CAUGHT': 19, 'MISSED': 12}

#: RAW held->empty edge minus the announced throw_time, per self-toss.
#: This is the band DEPARTURE_WINDOW_S is sized against.
DEPARTURE_DT_MS = (148.3, 150.0, 151.3, 151.6, 152.1, 153.5, 154.5, 157.0, 161.1, 161.8, 162.5, 162.8, 163.8, 164.1, 166.5, 171.7, 174.3, 177.5, 178.1, 180.0, 185.0, 186.8, 197.2, 199.8, 200.5, 200.6, 200.7, 201.2, 203.2, 209.2, 213.0)

#: Debounced departure minus RAW departure. THE D12 measurement: the
#: debounce is asymmetric (0 ms on a rise, ~240 ms on a fall), so a
#: timing fit taken off the debounced bit carries a systematic lag
#: comparable to the uptime dispatch shift it would be measuring.
DEBOUNCE_FALL_LAG_MS = (232.5, 232.5, 232.9, 235.3, 239.3, 239.3, 239.3, 239.5, 239.6, 239.6, 239.7, 239.8, 239.8, 239.9, 240.0, 240.1, 240.2, 240.3, 240.4, 240.7, 240.7, 240.8, 240.8, 241.0, 241.5, 241.7, 243.0, 243.6, 244.4, 244.7, 250.5)

#: This bag predates /toss/record, so every row is mined-only — which is
#: exactly the degraded-form capability § 3.3 requires of the miner.
RECORD_PROVENANCE = ('mined-only',)
