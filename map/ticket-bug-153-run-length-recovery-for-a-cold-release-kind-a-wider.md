---
id: TKT-102
kind: ticket
title: 'BUG-153: run-length recovery for a cold release kind, a wider prior in config, a widening gate, or leave it pinned?'
status: complete
created: '2026-11-28'
updated: '2026-11-28T00:45:06Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: learner
blocked_by: []
links: []
parent: bug-plant-model-a-release-kind-with-no-rows-rejects-every-row
type: question
---

BUG-153: a release kind with no rows measures the plant model's 100 mm lateral gate against its prior offset of 0, so a cold carried kind rejects every oval row (the oval's offset is 179 mm) and never learns. The live Memory does not hit it (carried accepted 1083 of 1085 rows on 2026-11-27); a strict xfail in `tests/motion/test_plant_model.py` pins the 179 mm case.

## Options

1. Run-length recovery: accept a row after about five consecutive rejections whose innovations agree within the band; one wild row resets the run. A warm kind never sees five agreeing rejections, so its commands are unchanged.
2. A wider prior sd on b for the carried kind. One number, but it lives in config, which the night cannot merge, and it loosens the gate for every cold start.
3. A widening term on the lateral gate like the Apex gate's. Alone it reaches only about 112 to 127 mm, short of 179.
4. Leave it pinned by the xfail until a Memory reset is planned.

## What each answer unblocks

1: the next night builds it, flips the 179 mm xfail to a pass, and replays the live Memory copy to show every command unchanged; it merges if the gate says MERGE. 2: a day Session edits the config. 3: built as 1, but it does not fix the 179 mm case. 4: the Item stays open as accepted.

Recommendation: 1, because it fixes any cold kind without widening the gate for warm ones, and the replay can show it is invisible on today's Memory.

## Answer

Agreed. Go with Option 1
