# Jugglebot records

<!-- jugglebot-records v2 -->

The most Catches in one continuous Attempt on the robot (never the sim), per Pattern. A record is beaten only by strictly more Catches. Each time a record is beaten a row is appended to that Pattern's table, so a table is the history of its record, newest last. The Feed is not a Pattern here.

`skill_node` writes a gitignored runtime copy of this file, `temp/runtime/records.md`, at the end of each Attempt (ROS parameter `records_path`, set by `jugglebot_launch.py`); the Sitting debrief copies it back here (`tools/runtime_records.py merge`) and commits it. The tables are rendered and parsed by `ros_ws/src/jugglebot/jugglebot/records.py`; a Note cell may be edited by hand, nothing else. "Parameters" is what that run used (`key=value` pairs; empty = not recorded). "How it ended" is completed, stopped, drop (a missed or lost ball), compute (a solve or limit refusal) or other, with the raw end code in parentheses.

## Drops to date

<!-- drops: live=72 since=2026-11-11 caught=598 history=819 history_caught=6485 history_from=2026-07-23 history_through=2026-11-10 -->

Drops to date: 891 (72 counted live since 2026-11-11; 819 from history, 2026-07-23 to 2026-11-10). 7,974 Throws and Feeds to catch, 88.8 % caught, 11.2 % dropped. A drop is a ball thrown to be caught by Jugglebot that was not caught.

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
| 2026-11-13 12:03 | 20 | 20 | completed (COMPLETED) |  | apex_m=1; separation_mm=100; dwell_s=0.27; apex_m_ball2=0.9; planner=pose |

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

## Landing scores

A landing score is the distance in mm from the Fit's landing to the target Site, one per Throw (Throw order); "Without a Fit" counts the Throws with no landing, which have no score. A Feed's score is kept apart. Written at the end of every Attempt, whether or not a record is beaten.

<!-- scores: pattern=oval_1ball -->
### Oval — Ball 1

| When | Scored | Median mm | Without a Fit | Scores mm |
|---|---:|---:|---:|---|
| 2026-11-13 11:52 | 32 | 22 | 0 | 23 24 13 32 17 10 12 31 17 18 28 27 12 51 23 20 10 19 28 52 49 24 29 10 18 23 32 18 6 11 38 7 |
| 2026-11-13 11:58 | 0 | - | 1 | - |
| 2026-11-13 11:58 | 1 | 54 | 0 | 54 |

<!-- scores: pattern=oval_1ball_fed -->
### Oval — fed

| When | Scored | Median mm | Without a Fit | Scores mm |
|---|---:|---:|---:|---|
| 2026-11-13 11:46 | 7 | 33 | 1 | 30 60 66 38 29 15 33 |
| 2026-11-13 11:47 | 32 | 23 | 0 | 14 64 51 13 21 34 26 24 9 32 30 23 27 3 23 9 22 38 37 17 25 17 15 33 2 21 9 14 28 28 17 5 |
| 2026-11-13 11:48 | 1 | 27 | 1 | 27 |
| 2026-11-13 11:49 | 2 | 19 | 1 | 16 21 |
| 2026-11-13 11:49 | 5 | 36 | 1 | 25 55 71 36 27 |
| 2026-11-13 11:50 | 32 | 28 | 0 | 9 27 28 44 35 20 30 7 22 42 6 4 38 20 28 49 16 26 24 37 9 17 40 24 53 45 38 47 50 19 34 39 |
| 2026-11-13 11:53 | 3 | 20 | 1 | 5 20 71 |
| 2026-11-13 11:53 | 4 | 26 | 1 | 30 51 14 22 |
| 2026-11-13 11:53 | 1 | 12 | 1 | 12 |
| 2026-11-13 11:53 | 3 | 35 | 2 | 35 34 44 |
| 2026-11-13 11:54 | 5 | 38 | 1 | 21 26 38 64 39 |
| 2026-11-13 11:55 | 2 | 23 | 1 | 19 26 |
| 2026-11-13 11:55 | 25 | 35 | 1 | 2 43 38 58 6 21 63 49 35 56 40 25 14 58 23 60 63 29 42 17 12 3 8 30 55 |
| 2026-11-13 11:56 | 1 | 32 | 1 | 32 |
| 2026-11-13 11:57 | 0 | - | 1 | - |
| 2026-11-13 12:52 | 0 | - | 1 | - |
| 2026-11-13 12:52 | 1 | 23 | 1 | 23 |
| 2026-11-13 12:52 | 1 | 14 | 1 | 14 |
| 2026-11-13 12:53 | 2 | 25 | 1 | 13 36 |
| 2026-11-13 12:53 | 32 | 23 | 0 | 7 35 6 63 37 39 31 47 30 28 10 6 12 27 22 21 44 11 24 1 7 5 18 13 20 34 3 12 35 45 36 24 |
| 2026-11-13 12:55 | 1 | 26 | 1 | 26 |
| 2026-11-13 12:55 | 1 | 30 | 1 | 30 |
| 2026-11-13 12:56 | 1 | 26 | 1 | 26 |
| 2026-11-13 12:56 | 1 | 12 | 1 | 12 |
| 2026-11-13 12:57 | 1 | 35 | 1 | 35 |

<!-- scores: pattern=self_toss -->
### Self-throw

| When | Scored | Median mm | Without a Fit | Scores mm |
|---|---:|---:|---:|---|
| 2026-11-13 13:24 | 1 | 38 | 0 | 38 |
| 2026-11-13 13:24 | 1 | 17 | 0 | 17 |
| 2026-11-13 13:25 | 1 | 30 | 0 | 30 |
| 2026-11-13 13:25 | 1 | 30 | 0 | 30 |
| 2026-11-13 13:25 | 1 | 15 | 0 | 15 |
| 2026-11-13 13:26 | 1 | 26 | 0 | 26 |
| 2026-11-13 13:26 | 1 | 25 | 0 | 25 |
| 2026-11-13 13:26 | 1 | 34 | 0 | 34 |
| 2026-11-13 13:27 | 1 | 13 | 0 | 13 |
| 2026-11-13 13:27 | 1 | 32 | 0 | 32 |
| 2026-11-13 13:27 | 1 | 27 | 0 | 27 |
| 2026-11-13 13:27 | 1 | 3 | 0 | 3 |
| 2026-11-13 13:28 | 1 | 19 | 0 | 19 |
| 2026-11-13 13:28 | 0 | - | 1 | - |
| 2026-11-13 13:28 | 1 | 25 | 0 | 25 |
| 2026-11-13 13:28 | 1 | 22 | 0 | 22 |
| 2026-11-13 13:28 | 1 | 20 | 0 | 20 |
| 2026-11-13 13:29 | 1 | 7 | 0 | 7 |
| 2026-11-13 13:29 | 1 | 28 | 0 | 28 |
| 2026-11-13 13:29 | 1 | 22 | 0 | 22 |
| 2026-11-13 13:29 | 1 | 2 | 0 | 2 |
| 2026-11-13 13:29 | 1 | 3 | 0 | 3 |
| 2026-11-13 13:30 | 1 | 29 | 0 | 29 |
| 2026-11-13 13:31 | 1 | 17 | 0 | 17 |
| 2026-11-13 13:32 | 1 | 34 | 0 | 34 |
| 2026-11-13 13:32 | 1 | 15 | 0 | 15 |
| 2026-11-13 13:32 | 1 | 17 | 0 | 17 |
| 2026-11-13 13:32 | 1 | 21 | 0 | 21 |
| 2026-11-13 13:32 | 1 | 25 | 0 | 25 |
| 2026-11-13 13:33 | 0 | - | 1 | - |
| 2026-11-13 13:33 | 1 | 18 | 0 | 18 |
| 2026-11-13 13:33 | 1 | 28 | 0 | 28 |
| 2026-11-13 13:34 | 1 | 27 | 0 | 27 |
| 2026-11-13 13:34 | 1 | 16 | 0 | 16 |
| 2026-11-13 13:34 | 1 | 21 | 0 | 21 |
| 2026-11-13 13:34 | 1 | 44 | 0 | 44 |
| 2026-11-13 13:34 | 1 | 35 | 0 | 35 |
| 2026-11-13 13:35 | 1 | 20 | 0 | 20 |
| 2026-11-13 13:35 | 1 | 43 | 0 | 43 |
| 2026-11-13 13:35 | 1 | 20 | 0 | 20 |
| 2026-11-13 13:35 | 1 | 32 | 0 | 32 |
| 2026-11-13 13:35 | 1 | 26 | 0 | 26 |
| 2026-11-13 13:36 | 1 | 25 | 0 | 25 |
| 2026-11-13 13:36 | 1 | 28 | 0 | 28 |
| 2026-11-13 13:36 | 1 | 39 | 0 | 39 |

<!-- scores: pattern=columns -->
### Columns

| When | Scored | Median mm | Without a Fit | Scores mm |
|---|---:|---:|---:|---|
| 2026-11-13 11:45 | 20 | 26 | 0 | 22 40 32 24 22 13 9 17 44 31 49 31 25 11 26 30 21 39 21 57 |
| 2026-11-13 13:00 | 7 | 35 | 1 | 39 18 60 55 18 22 35 |
| 2026-11-13 13:00 | 5 | 10 | 2 | 13 10 21 9 2 |
| 2026-11-13 13:01 | 20 | 23 | 0 | 17 38 10 43 47 18 29 25 8 33 21 56 15 9 16 34 28 18 5 31 |
| 2026-11-13 13:01 | 11 | 17 | 1 | 17 28 28 4 15 22 32 22 6 13 4 |
| 2026-11-13 13:01 | 5 | 13 | 1 | 12 13 9 34 45 |
| 2026-11-13 13:02 | 20 | 29 | 0 | 13 19 16 19 23 34 42 30 35 24 35 41 12 39 17 39 7 28 33 33 |

<!-- scores: pattern=columns_highlow -->
### Columns — high-low

| When | Scored | Median mm | Without a Fit | Scores mm |
|---|---:|---:|---:|---|
| 2026-11-13 12:00 | 3 | 15 | 1 | 36 15 11 |
| 2026-11-13 12:00 | 15 | 22 | 2 | 13 18 31 24 34 42 1 49 5 18 15 22 25 27 2 |
| 2026-11-13 12:01 | 2 | 28 | 0 | 33 23 |
| 2026-11-13 12:01 | 7 | 19 | 1 | 19 26 35 22 14 18 14 |
| 2026-11-13 12:02 | 4 | 24 | 1 | 23 28 25 18 |
| 2026-11-13 12:02 | 7 | 19 | 1 | 26 9 7 19 13 22 46 |
| 2026-11-13 12:02 | 2 | 35 | 0 | 29 41 |
| 2026-11-13 12:03 | 8 | 22 | 1 | 27 8 24 17 16 19 42 41 |
| 2026-11-13 12:03 | 20 | 18 | 0 | 16 17 36 20 25 37 14 16 9 44 18 22 2 9 11 35 30 26 17 15 |
| 2026-11-13 12:06 | 18 | 26 | 1 | 27 63 45 41 23 18 25 30 40 33 41 4 7 19 16 53 8 17 |
| 2026-11-13 12:06 | 5 | 32 | 1 | 32 32 21 28 32 |
| 2026-11-13 12:06 | 2 | 42 | 0 | 42 42 |
| 2026-11-13 12:07 | 9 | 25 | 2 | 52 20 25 39 9 62 20 28 11 |
| 2026-11-13 12:07 | 9 | 17 | 1 | 23 13 16 25 41 13 33 17 8 |
| 2026-11-13 12:08 | 6 | 27 | 2 | 36 40 28 12 12 26 |
| 2026-11-13 12:08 | 0 | - | 2 | - |
| 2026-11-13 12:09 | 14 | 26 | 2 | 29 27 21 45 9 41 26 12 26 36 14 28 20 15 |

<!-- scores: pattern=columns_turned -->
### Columns — turned

| When | Scored | Median mm | Without a Fit | Scores mm |
|---|---:|---:|---:|---|
| 2026-11-13 13:15 | 2 | 28 | 1 | 22 34 |
| 2026-11-13 13:15 | 3 | 47 | 1 | 20 51 47 |
| 2026-11-13 13:15 | 2 | 32 | 1 | 23 41 |
| 2026-11-13 13:16 | 7 | 22 | 1 | 16 22 35 38 15 29 14 |
| 2026-11-13 13:16 | 7 | 36 | 1 | 53 17 48 59 36 11 9 |
| 2026-11-13 13:16 | 9 | 15 | 2 | 15 49 36 33 1 10 5 12 44 |
| 2026-11-13 13:17 | 9 | 22 | 1 | 43 14 37 5 22 30 71 13 15 |
| 2026-11-13 13:17 | 20 | 19 | 0 | 19 11 44 46 32 17 25 17 11 19 19 19 44 34 4 19 13 30 28 42 |
| 2026-11-13 13:18 | 15 | 23 | 1 | 13 30 28 49 19 13 18 11 18 32 23 47 30 22 35 |
| 2026-11-13 13:18 | 7 | 20 | 1 | 39 43 18 20 15 31 20 |
| 2026-11-13 13:18 | 2 | 42 | 0 | 41 42 |
| 2026-11-13 13:18 | 20 | 33 | 0 | 10 34 13 56 32 40 52 37 30 38 42 27 50 20 8 21 38 31 22 37 |
| 2026-11-13 13:20 | 1 | 21 | 1 | 21 |
| 2026-11-13 13:20 | 2 | 32 | 0 | 25 39 |
| 2026-11-13 13:20 | 2 | 32 | 0 | 11 53 |
| 2026-11-13 13:20 | 1 | 25 | 0 | 25 |
| 2026-11-13 13:21 | 2 | 51 | 0 | 68 33 |

<!-- feed-scores -->
### Feeds

| When | Pattern | Score mm |
|---|---|---:|
| 2026-11-13 11:45 | columns | 39 |
| 2026-11-13 12:00 | columns_highlow | 44 |
| 2026-11-13 12:00 | columns_highlow | 14 |
| 2026-11-13 12:01 | columns_highlow | 46 |
| 2026-11-13 12:02 | columns_highlow | 19 |
| 2026-11-13 12:02 | columns_highlow | 49 |
| 2026-11-13 12:03 | columns_highlow | 22 |
| 2026-11-13 12:03 | columns_highlow | 39 |
| 2026-11-13 12:06 | columns_highlow | 31 |
| 2026-11-13 12:06 | columns_highlow | 43 |
| 2026-11-13 12:07 | columns_highlow | 49 |
| 2026-11-13 12:07 | columns_highlow | 42 |
| 2026-11-13 12:08 | columns_highlow | 45 |
| 2026-11-13 12:08 | columns_highlow | 17 |
| 2026-11-13 12:09 | columns_highlow | 26 |
| 2026-11-13 13:00 | columns | 19 |
| 2026-11-13 13:00 | columns | 18 |
| 2026-11-13 13:01 | columns | 16 |
| 2026-11-13 13:01 | columns | 21 |
| 2026-11-13 13:01 | columns | 42 |
| 2026-11-13 13:02 | columns | 21 |
| 2026-11-13 13:15 | columns_turned | 27 |
| 2026-11-13 13:15 | columns_turned | 27 |
| 2026-11-13 13:15 | columns_turned | 23 |
| 2026-11-13 13:16 | columns_turned | 37 |
| 2026-11-13 13:16 | columns_turned | 31 |
| 2026-11-13 13:16 | columns_turned | 32 |
| 2026-11-13 13:17 | columns_turned | 44 |
| 2026-11-13 13:17 | columns_turned | 38 |
| 2026-11-13 13:18 | columns_turned | 24 |
| 2026-11-13 13:18 | columns_turned | 14 |
| 2026-11-13 13:18 | columns_turned | 33 |
| 2026-11-13 13:20 | columns_turned | 44 |
| 2026-11-13 13:20 | columns_turned | 40 |
| 2026-11-13 13:20 | columns_turned | 44 |
| 2026-11-13 13:21 | columns_turned | 26 |
