# Jugglebot records

<!-- jugglebot-records v2 -->

The most Catches in one continuous Attempt on the robot (never the sim), per Pattern. A record is beaten only by strictly more Catches. Each time a record is beaten a row is appended to that Pattern's table, so a table is the history of its record, newest last. The Feed is not a Pattern here.

`skill_node` writes this file live at the end of each Attempt (ROS parameter `records_path`, set by `jugglebot_launch.py`) and the Sitting's close-out commits it. The tables are rendered and parsed by `ros_ws/src/jugglebot/jugglebot/records.py`; a Note cell may be edited by hand, nothing else. "Parameters" is what that run used (`key=value` pairs; empty = not recorded). "How it ended" is completed, stopped, drop (a missed or lost ball), compute (a solve or limit refusal) or other, with the raw end code in parentheses.

## Drops to date

<!-- drops: live=0 since=2026-11-11 history=819 history_caught=6485 history_from=2026-07-23 history_through=2026-11-10 -->

Drops to date: 819 (0 counted live since 2026-11-11; 819 from history, 2026-07-23 to 2026-11-10). 7,304 Throws and Feeds to catch, 88.8 % caught, 11.2 % dropped. A drop is a ball thrown to be caught by Jugglebot that was not caught.

## 1 ball

<!-- pattern: columns_1ball balls: 1 -->
### Columns — Ball 1

| When | Throws | Catches | How it ended | Note | Parameters |
|---|---:|---:|---|---|---|
| 2026-10-04 19:54 | 3 | 3 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-04 19:55 | 6 | 5 | compute (LIMIT_JERK) | seed: log |  |
| 2026-10-04 19:55 | 10 | 9 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-04 19:56 | 10 | 10 | completed (COMPLETED) | seed: fed goal |  |

<!-- pattern: columns_1ball_fed balls: 1 -->
### Columns — Ball 2 (fed)

| When | Throws | Catches | How it ended | Note | Parameters |
|---|---:|---:|---|---|---|
| 2026-10-11 15:20 | 2 | 1 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-11 22:19 | 3 | 2 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-13 23:17 | 5 | 4 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-28 14:31 | 5 | 5 | completed (COMPLETED) |  |  |

<!-- pattern: hop balls: 1 -->
### Hop

| When | Throws | Catches | How it ended | Note | Parameters |
|---|---:|---:|---|---|---|
| 2026-09-29 00:02 | 1 | 1 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-29 19:17 | 2 | 2 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-29 19:19 | 5 | 5 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-29 19:20 | 6 | 6 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-29 19:21 | 10 | 10 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-29 19:21 | 28 | 24 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-11-02 18:31 | 61 | 61 | drop (MISSED_CATCH) |  | apex_m=0.9; separation_mm=250; dwell_s=0.27; planner=pose |

<!-- pattern: oval_1ball balls: 1 -->
### Oval — Ball 1

| When | Throws | Catches | How it ended | Note | Parameters |
|---|---:|---:|---|---|---|
| 2026-10-23 11:08 | 4 | 4 | completed (COMPLETED) | seed: log |  |
| 2026-10-23 11:09 | 10 | 10 | completed (COMPLETED) | seed: log |  |
| 2026-10-23 17:40 | 32 | 32 | completed (COMPLETED) | seed: log |  |
| 2026-10-27 23:06 | 35 | 35 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-28 21:09 | 50 | 50 | completed (COMPLETED) |  |  |

<!-- pattern: oval_1ball_fed balls: 1 -->
### Oval — fed

| When | Throws | Catches | How it ended | Note | Parameters |
|---|---:|---:|---|---|---|
| 2026-10-23 11:08 | 4 | 4 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-23 17:32 | 6 | 6 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-23 17:36 | 32 | 32 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-28 09:56 | 35 | 35 | completed (COMPLETED) | seed: fed goal |  |

<!-- pattern: self_toss balls: 1 -->
### Self-throw

| When | Throws | Catches | How it ended | Note | Parameters |
|---|---:|---:|---|---|---|
| 2026-09-16 14:18 | 4 | 1 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-16 16:24 | 4 | 3 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-17 18:54 | 5 | 4 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-18 13:29 | 5 | 5 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-18 16:18 | 20 | 8 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-22 23:28 | 10 | 9 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-22 23:29 | 25 | 23 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |
| 2026-09-22 23:32 | 25 | 24 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |  |

## 2 balls

<!-- pattern: columns balls: 2 -->
### Columns

| When | Throws | Catches | How it ended | Note | Parameters |
|---|---:|---:|---|---|---|
| 2026-10-02 12:57 | 1 | 1 | compute (LIMIT_VEL) | seed: fed goal |  |
| 2026-10-02 18:19 | 3 | 2 | compute (WINDOW_TOO_SHORT) | seed: fed goal |  |
| 2026-10-02 18:21 | 3 | 3 | compute (WINDOW_TOO_SHORT) | seed: fed goal |  |
| 2026-10-04 20:04 | 5 | 4 | drop (ABORTED_NO_RELEASE) | seed: fed goal |  |
| 2026-10-11 22:21 | 6 | 5 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-11 22:21 | 9 | 8 | compute (LIMIT_JERK) | seed: fed goal |  |
| 2026-10-13 00:21 | 10 | 9 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-13 00:22 | 11 | 11 | drop (MISSED_CATCH) | seed: fed goal |  |
| 2026-10-13 00:24 | 15 | 15 | drop (MISSED_CATCH) | seed: fed goal |  |
| 2026-10-13 23:20 | 32 | 31 | completed (COMPLETED) | seed: fed goal |  |
| 2026-10-22 20:00 | 48 | 47 | drop (MISSED_CATCH) | seed: fed goal |  |
| 2026-10-28 14:33 | 49 | 49 | completed (COMPLETED) |  |  |
| 2026-10-28 21:04 | 59 | 57 | drop (ABORTED_NO_RELEASE) |  |  |
| 2026-11-02 18:20 | 58 | 58 | other (LANE_REFUSED) |  | apex_m=0.95; separation_mm=125; dwell_s=0.27; planner=pose |
| 2026-11-02 18:21 | 65 | 64 | compute (LIMIT_VEL) |  | apex_m=0.95; separation_mm=125; dwell_s=0.27; planner=pose |
| 2026-11-06 14:59 | 73 | 73 | other (LANE_REFUSED) |  | apex_m=1; separation_mm=125; dwell_s=0.27; planner=pose |

<!-- pattern: columns_highlow balls: 2 -->
### Columns — high-low

| When | Throws | Catches | How it ended | Note | Parameters |
|---|---:|---:|---|---|---|
| 2026-11-01 22:14 | 8 | 8 | other (GUARD_LATCHED) |  | apex_m=1; separation_mm=100; dwell_s=0.27; apex_m_ball2=0.9; planner=pose |
| 2026-11-01 22:15 | 13 | 12 | compute (LIMIT_ACC) |  | apex_m=1; separation_mm=100; dwell_s=0.27; apex_m_ball2=0.9; planner=pose |
| 2026-11-03 12:52 | 17 | 16 | drop (MISSED_CATCH) |  | apex_m=1; separation_mm=100; dwell_s=0.27; apex_m_ball2=0.9; planner=pose |
| 2026-11-03 12:53 | 18 | 18 | drop (MISSED_CATCH) |  | apex_m=1; separation_mm=100; dwell_s=0.27; apex_m_ball2=0.9; planner=pose |

<!-- pattern: columns_turned balls: 2 -->
### Columns — turned

| When | Throws | Catches | How it ended | Note | Parameters |
|---|---:|---:|---|---|---|
| 2026-11-01 22:15 | 1 | 1 | drop (MISSED_CATCH) |  | apex_m=0.95; separation_mm=125; dwell_s=0.27; turned_angle_deg=45; planner=pose |
| 2026-11-01 22:16 | 2 | 2 | drop (MISSED_CATCH) |  | apex_m=0.95; separation_mm=125; dwell_s=0.27; turned_angle_deg=45; planner=pose |
| 2026-11-01 22:17 | 4 | 4 | drop (MISSED_CATCH) |  | apex_m=0.95; separation_mm=125; dwell_s=0.27; turned_angle_deg=90; planner=pose |
| 2026-11-01 22:18 | 16 | 16 | other (GUARD_LATCHED) |  | apex_m=0.95; separation_mm=125; dwell_s=0.27; turned_angle_deg=90; planner=pose |
| 2026-11-03 12:59 | 19 | 19 | completed (COMPLETED) |  | apex_m=0.95; separation_mm=125; dwell_s=0.27; turned_angle_deg=45; planner=pose |
| 2026-11-06 16:03 | 20 | 20 | completed (COMPLETED) |  | apex_m=1; separation_mm=120; dwell_s=0.27; turned_angle_deg=90; planner=pose |
| 2026-11-06 16:04 | 35 | 35 | drop (MISSED_CATCH) |  | apex_m=1; separation_mm=120; dwell_s=0.27; turned_angle_deg=90; planner=pose |
