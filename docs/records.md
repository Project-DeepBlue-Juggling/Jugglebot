# Jugglebot records

<!-- jugglebot-records v1 -->

The most Catches in one continuous Attempt on the robot (never the sim), per Pattern. A record is beaten only by strictly more Catches. Each time a record is beaten a row is appended to that Pattern's table, so a table is the history of its record, newest last. The Feed is not a Pattern here.

`skill_node` writes this file live at the end of each Attempt (ROS parameter `records_path`, set by `jugglebot_launch.py`) and the Sitting's close-out commits it. The tables are rendered and parsed by `ros_ws/src/jugglebot/jugglebot/records.py`; a Note cell may be edited by hand, nothing else. "How it ended" is completed, stopped, drop (a missed or lost ball), compute (a solve or limit refusal) or other, with the raw end code in parentheses.

## 1 ball

<!-- pattern: columns_1ball balls: 1 -->
### Columns — Ball 1

| When | Throws | Catches | How it ended | Note |
|---|---:|---:|---|---|
| 2026-10-04 19:54 | 3 | 3 | completed (COMPLETED) | seed: fed goal |
| 2026-10-04 19:55 | 6 | 5 | compute (LIMIT_JERK) | seed: log |
| 2026-10-04 19:55 | 10 | 9 | completed (COMPLETED) | seed: fed goal |
| 2026-10-04 19:56 | 10 | 10 | completed (COMPLETED) | seed: fed goal |

<!-- pattern: columns_1ball_fed balls: 1 -->
### Columns — Ball 2 (fed)

| When | Throws | Catches | How it ended | Note |
|---|---:|---:|---|---|
| 2026-10-11 15:20 | 2 | 1 | completed (COMPLETED) | seed: fed goal |
| 2026-10-11 22:19 | 3 | 2 | completed (COMPLETED) | seed: fed goal |
| 2026-10-13 23:17 | 5 | 4 | completed (COMPLETED) | seed: fed goal |

<!-- pattern: hop balls: 1 -->
### Hop

| When | Throws | Catches | How it ended | Note |
|---|---:|---:|---|---|
| 2026-09-29 00:02 | 1 | 1 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-29 19:17 | 2 | 2 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-29 19:19 | 5 | 5 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-29 19:20 | 6 | 6 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-29 19:21 | 10 | 10 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-29 19:21 | 28 | 24 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |

<!-- pattern: oval_1ball balls: 1 -->
### Oval — Ball 1

| When | Throws | Catches | How it ended | Note |
|---|---:|---:|---|---|
| 2026-10-23 11:08 | 4 | 4 | completed (COMPLETED) | seed: log |
| 2026-10-23 11:09 | 10 | 10 | completed (COMPLETED) | seed: log |
| 2026-10-23 17:40 | 32 | 32 | completed (COMPLETED) | seed: log |
| 2026-10-27 23:06 | 35 | 35 | completed (COMPLETED) | seed: fed goal |

<!-- pattern: oval_1ball_fed balls: 1 -->
### Oval — fed

| When | Throws | Catches | How it ended | Note |
|---|---:|---:|---|---|
| 2026-10-23 11:08 | 4 | 4 | completed (COMPLETED) | seed: fed goal |
| 2026-10-23 17:32 | 6 | 6 | completed (COMPLETED) | seed: fed goal |
| 2026-10-23 17:36 | 32 | 32 | completed (COMPLETED) | seed: fed goal |
| 2026-10-28 09:56 | 35 | 35 | completed (COMPLETED) | seed: fed goal |

<!-- pattern: self_toss balls: 1 -->
### Self-throw

| When | Throws | Catches | How it ended | Note |
|---|---:|---:|---|---|
| 2026-09-16 14:18 | 4 | 1 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-16 16:24 | 4 | 3 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-17 18:54 | 5 | 4 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-18 13:29 | 5 | 5 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-18 16:18 | 20 | 8 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-22 23:28 | 10 | 9 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-22 23:29 | 25 | 23 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |
| 2026-09-22 23:32 | 25 | 24 | other (UNKNOWN) | seed: reconstructed from per-throw lines, no summary line (approximate) |

## 2 balls

<!-- pattern: columns balls: 2 -->
### Columns

| When | Throws | Catches | How it ended | Note |
|---|---:|---:|---|---|
| 2026-10-02 12:57 | 1 | 1 | compute (LIMIT_VEL) | seed: fed goal |
| 2026-10-02 18:19 | 3 | 2 | compute (WINDOW_TOO_SHORT) | seed: fed goal |
| 2026-10-02 18:21 | 3 | 3 | compute (WINDOW_TOO_SHORT) | seed: fed goal |
| 2026-10-04 20:04 | 5 | 4 | drop (ABORTED_NO_RELEASE) | seed: fed goal |
| 2026-10-11 22:21 | 6 | 5 | completed (COMPLETED) | seed: fed goal |
| 2026-10-11 22:21 | 9 | 8 | compute (LIMIT_JERK) | seed: fed goal |
| 2026-10-13 00:21 | 10 | 9 | completed (COMPLETED) | seed: fed goal |
| 2026-10-13 00:22 | 11 | 11 | drop (MISSED_CATCH) | seed: fed goal |
| 2026-10-13 00:24 | 15 | 15 | drop (MISSED_CATCH) | seed: fed goal |
| 2026-10-13 23:20 | 32 | 31 | completed (COMPLETED) | seed: fed goal |
| 2026-10-22 20:00 | 48 | 47 | drop (MISSED_CATCH) | seed: fed goal |
