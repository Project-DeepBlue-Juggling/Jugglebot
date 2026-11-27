---
id: TKT-107
kind: ticket
title: 'BUG-154 held on night-shift/2026-11-28-BUG-154: add the second test''s clock freeze and merge, merge as is, or fix both in your Session?'
status: open
created: '2026-11-28'
updated: '2026-11-27T17:38:18Z'
owners: []
priority: high
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-robot-frame-feed-test-ages-its-own-frame-samples-on-the-real
type: question
---

Main's full suite fails on this box: `test_moved_frame_announcement_landing_is_the_robot_frame_prior` keeps the real frame clock, and the Feed start ages its own Platform samples 1.10 s against the 1.0 s window. It fails at every commit back to the tip the 2026-11-27 gate passed, so something on the box changed (unchecked). Until it is fixed every Night Shift gate holds at (a), and the next nightly will likely be RED.

BUG-154 is built on night-shift/2026-11-28-BUG-154: the test freezes the frame clock as the fixture does for frame=True tests (BUG-14), and passes. It is test-only. The audit left one finding open: `test_a_robot_frame_update_mid_attempt_does_not_move_its_landings` has the same shape and needs the same one-line freeze.

## Options

1. Apply the second freeze and merge: the next night adds it, re-audits and gates the branch, and merges on MERGE.
2. Merge as is in your Session: main goes green now; the second test stays exposed until a later fix.
3. Your Session applies the second line and merges both today, which is the fastest way to a green main.

## What each answer unblocks

1: a merge the next night, then BUG-150 can be re-gated. 2 or 3: main's suite green before the next nightly.

Recommendation: 3, because main's suite is red until this lands and the night cannot merge before the next nightly runs.
