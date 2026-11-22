---
title: "Bench Hand Sitting 2026-11-22 debrief: the feedforward offset law holds at 10 rev/s (0.284 against 0.286 rev); the stall-guard row did not fly because the checkout reused a stale local branch, and an unpowered dry run found and fixed one more blocker"
type: investigation
date: 2026-11-22
status: complete
phase: "Sitting debrief"
related_plan: ""
related_issues: []
files_changed:
  - logbook/2026-11-22-sitting-debrief-bench-hand.md
  - logbook/INDEX.md
  - map/sheets/2026-11-22-bench-hand.results.yaml
  - tests/hardware/hand_stall_stage.py
  - tests/hardware/hand_stream_bench.py
  - tests/motion/test_hand_stream_bench_stall.py
subsystem: [hand, tooling]
---

## Summary

Sheet `2026-11-22-bench-hand`, flown 22:02 to 22:14 on the bench Hand. No launch.

| Item | step | verdict | measured |
|---|---|---|---|
| BUG-105 | 9 | inconclusive | 10 rev/s up: settled 0.2844 rev above the firmware's last command against 0.2857 expected, drift 0.0007 rev. 20 rev/s up: the disarm landed after the feedforward had decayed to 0, offset -0.0016 against 0. Down cut and E-stop button check not flown |
| BUG-123 | 11 | inconclusive | not flown: the old stage ran and crashed on `KeyError: 'stall'` |

Merged: nothing. Held: `night-shift/2026-11-20-BUG-123`, now at `4c5e78ff`. Both Items are on sheet `2026-11-22-bench-hand-2`.

## Big picture

What went well: the fixed triangle stage flew first time. The hand-back after arming worked, the Hand tracked inside the new belt (0.21 rev at 10 rev/s, 0.45 at 20), and the 10 rev/s cut gave the cleanest number this Item has had: the Hand settles 0.284 rev past the firmware's last command, against the 0.286 the law predicts.

What did not: the stall-guard row crashed again with the same error. The fix was real but was never on the bench: the sheet's `git checkout` switched to a local copy of the branch left from 2026-11-20, which `git fetch` does not move. The old stage ran, and the bridge was flashed with that commit's firmware. The restore step was then skipped, so the bridge and the checkout were both left on the branch.

What it means: the 20 rev/s cut showed the offset is not a fixed property of a cut. When the disarm lands late the firmware has already wound the feedforward down to 0 and the Hand ends where it was told, after running on 2.1 rev. The residual offset is what is left of the feedforward at the instant the disarm lands. Separately, a sheet that checks out a branch must bring the local branch up to the pushed one and check the commit before flashing.

Next: BUG-123 flies on the follow-up sheet with the checkout fixed. An unpowered dry run tonight carried the real stage as far as the bring-up, so only the push itself is untested. BUG-105 needs the downward cut and one 20 rev/s cut that lands inside the decay.

## Discussion

**The repeat crash.** `git reflog` on the live checkout shows `checkout: moving from main to night-shift/2026-11-20-BUG-123` landing on `9e715165`, 99 commits behind the pushed branch. The step's `see` text named the expected log line, but nothing forced a stop, and a wrong line is easy to pass over. The follow-up sheet and the Item's section now run `git merge --ff-only origin/<branch>` after the checkout and tell the operator to stop unless the log line starts with the expected SHA.

**The bridge was left on the wrong build.** The flashed `9e715165` build numbers itself 47, the same number main uses for the Platform IMU relay build, so the version check alone could not have told them apart on the robot. During this debrief the bridge was flashed with the branch's current build, firmware 48 (launch down, bench bus at 0 V), which main's check refuses. The follow-up sheet's step 7 flashes main's build back. Until then the robot must not be launched.

**Unpowered dry run (owner's request).** With the bench supply off (vbus 0.04 V over USB) and firmware 48 on the bridge:

- `python tests/hardware/hand_stream_bench.py --stage stall` no longer crashes. It refused with three reasons: the dead bus's fault and error bits, and "motor temperature nan C".
- The third is a real blocker: the bench drive has no motor thermistor and reported NaN while powered on 2026-11-20 too, so the stage would have refused for ever. Fixed (`4c5e78ff`): a motor reading that is not finite falls back to the FET temperature under the same 50 C bound, the stage says so, and with neither reading it refuses. This weakens the cool-motor check on the bench to the FET reading plus the operator's hand; it is the owner's to veto.
- With only the two dead-bus refusals dropped (a scratch wrapper, not committed), the real stage ran: limit 70 to 45 A written over CAN and read back over USB in the same process as the bridge link, frame shifted +0.650 rev, bring-up attempted and failed as it must at 0 V, limit restored to 70 A and read back, frame restored to +0.571 rev. The Hand cannot move at 0 V; nothing else was commanded.

Not verified: the push itself, the guard's trip, and the timing.

**BUG-105 at 20 rev/s.** The cut lets the 10 Hz heartbeat carry the disarm, as a Ctrl-C does, so it lands anywhere in the next 100 ms. At 10 rev/s it landed with the feedforward still 10; at 20 rev/s it landed after the decay. A cut that sends the disarm at once would test the law deterministically; that is a tool change not made tonight.

**Owner's standing notes from the sheet.** Push-check and torque-check are not needed by default on this bench Hand; the top stop is 10.4 rev. The owner will say when the Hand changes.

## Verification

- 2026-11-22, `pytest tests/motion/test_hand_stream_bench_stall.py tests/motion/test_hand_stream_bench_order.py tests/motion/test_hand_stream_bench_stroke_cut.py tests/teensy_link/test_hand_stream_bench_tri_cut.py tests/teensy_link/test_hand_stream_bench_gap.py tests/motion/test_hand_foc_scope.py -q` on the branch: 243 passed in 26.08 s.
- 2026-11-22, the unpowered runs above, on the live checkout at `4c5e78ff`: `ABORT: stall stage refused` with only `fault_state=ODRIVE_FATAL` and `active_errors=513`.
- 2026-11-22, map and logbook tests on this commit: `pytest tests/sim/test_map_items.py tests/sim/test_mapkit_sheet.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_glossary_avoid_words.py -q`: 122 passed in 8.86 s. No full gate: this commit is documentation and map files; the code fix is on the held branch with its scoped run above.
