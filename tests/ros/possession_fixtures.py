"""Measured ball-possession fixtures — DO NOT hand-edit.

Every CAUGHT estimate the ball tracker published for a
``destination='jugglebot'`` track during the 2026-07-27 validation sitting,
read from ``/home/jetson/Desktop/rosbags/2026-07-27_15-39-38``.

Regenerate with:
    python tools/probes/possession_verdict_bag_check.py \
        --bag /home/jetson/Desktop/rosbags/2026-07-27_15-39-38 --emit-fixtures

Consumed by tests/ros/test_ball_possession.py. Contract:
ros_ws/docs/ball_possession_contract.md (C-POSSESS-1).
"""

from __future__ import annotations

# (ball_id, x_mm, y_mm, z_mm) at the FIRST CAUGHT sample of each track.

# Ball-Butler reload tracks. Every one is a split track whose Kalman filter is
# fed by the WRONG marker (NOT "got no measurements" — all 18 reach
# tracking=CONFIRMED; corrected 2026-07-28, see the contract's section 4), so its
# estimate tracks something that is not the ball. Of the eighteen: THIRTEEN were real catches,
# THREE were eye-confirmed bounce-outs (see BOUNCE_OUT_IDS), and TWO
# (balls 31 and 62) are goals whose BB throw aborted YAW NOT_SETTLED, so no
# ball ever left the Butler. The estimates are indistinguishable across all
# three groups, which is why the honest verdict for all of them is refusal.
RELOAD_CAUGHT = (
    (6, -636.6858, -349.7732, -532.1082),
    (9, -612.2336, -344.7602, -621.6210),
    (11, -632.0651, -347.0029, -701.0236),
    (14, -627.5940, -346.1477, -646.6386),
    (19, -664.1972, -354.5837, -483.6544),
    (24, -625.9678, -346.1709, -705.0714),
    (27, -598.9097, -340.0202, -690.1190),
    (31, -736.1618, -127.1689, -392.2510),
    (32, 189.1707, 78.7024, -286.7407),
    (41, 242.0865, 86.8773, -416.3021),
    (42, -621.0748, -343.1788, 97.9271),
    (47, -647.1094, -351.0450, -581.4246),
    (50, -590.4651, -336.4910, -598.0641),
    (53, 314.9558, 98.4140, -396.5144),
    (55, -532.0648, -322.7197, -518.2298),
    (62, -543.6915, -326.8458, -490.8698),
    (63, -651.3349, -352.2407, -473.0504),
    (67, -635.2224, -346.9034, -523.9222),
)

# Self-toss tracks. Measurement-driven; every one is a catch the operator
# watched land, and every one was reported MISSED by the pre-2026-07-28 gate.
SELF_TOSS_CAUGHT = (
    (80, 1.4934, -2.2970, 310.6558),
    (82, 1.1611, -1.8727, 432.9296),
    (87, 1.1176, -1.3070, 354.8539),
    (89, 0.1636, -0.2573, 440.2679),
    (90, 0.2931, -0.2526, 448.5202),
    (96, 1.2919, -2.2548, 435.4645),
    (99, 1.2875, -0.7863, 276.5662),
    (101, 1.2996, -1.4694, 434.9208),
    (102, 2.6365, -2.0893, 154.4267),
    (107, 2.4955, -2.9645, -15.2176),
    (114, 1.7566, -3.2848, 39.1893),
    (119, 2.8025, -2.1538, -184.7527),
    (123, 1.3967, -1.4640, -198.0552),
    (128, 1.9721, -2.2547, -8.6848),
    (132, 1.9636, -1.4688, 396.2233),
    (136, 2.1002, -2.5433, 442.7249),
    (137, 2.4046, -2.1836, 504.0534),
)

# The three eye-confirmed bounce-outs (reload attempts 1-3), operator
# testimony 2026-07-28. They are a SUBSET of RELOAD_CAUGHT: the tracker
# reported CAUGHT for all three.
BOUNCE_OUT_IDS = (6, 9, 11)

# Where the three bounce-outs actually crossed z = 1050 mm on their way in,
# from the sitting's mocap analysis (logbook/2026-07-28-anomaly-fixes-validation-sitting.md
# "The three drops"). NOT tracker output — this is the ball's TRUE arrival,
# i.e. what a FIXED tracker would report, and it is the fixture that pins the
# trap: all three entered the cup region at small xy before departing.
BOUNCE_OUT_TRUE_ARRIVAL_XY_MM = (
    (6, -7.2, 16.1),
    (9, -18.8, 3.6),
    (11, -21.5, 7.4),
)

# The 2026-07-23 corrupt track quoted in the original gate comment, kept as a
# second-session negative so the fixture set is not single-capture.
CORRUPT_2026_07_23 = (-539.0, -323.0, -532.0)
