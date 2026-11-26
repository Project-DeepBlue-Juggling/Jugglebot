---
title: "BUG-75 replay tooling: chained re-aims, timing shift, per-row record selection and a committed row labeller; control per angle and the bump on second re-aims"
type: feature
date: 2026-11-26
status: complete
phase: ""
related_plan: "late-catch-reaim-bump.md"
related_issues: []
files_changed:
  - tools/probes/late_reaim/bump_replay.py
  - tools/probes/late_reaim/label_rows.py
  - tests/motion/test_late_reaim_replay.py
  - logbook/2026-11-26-bug75-chained-reaim-replay.md
subsystem: [tooling, skill-stack]
---

## Summary
Tooling only; no planner, executor or skill code touched. The 2026-11-25 replay applied each logged re-aim once to a pristine sim record and ignored its timing shift, but 57 of the 72 `columns_turned` refusals were SECOND re-aims spliced 0.1 to 0.3 s after an accepted first one (diagnosis in the BUG-75 Progress of 2026-11-26). `bump_replay.py` gains `--chain` (a Catch's re-aims in logged order, each on the record and committed aim the previous accepted one produced; a refused one leaves both as they were, as on the robot), `--record ANG:APEX:SEP=PKL` (the record is chosen per row from its label), and `--bump-last` (all but the last re-aim of a Catch raw, the last through the production bump on the chained record). `--raw-ds` already applies the timing shift. New `label_rows.py` turns a launch log into per-Catch ordered re-aim rows with angle, Apex, separation, flown verdict and `ds`. With none of the new flags the script runs the old path unchanged.

## Verification
- 2026-11-26, `pytest tests/motion/test_late_reaim_replay.py -q -p no:xdist`: 7 passed in 0.4 s. Each test failed under one mutation (chain not passing the new record; refused re-aim also updating the record, found after the stub returned a changed record on refusal; `ds` dropped; record chosen by default file; rows not sorted by seq; refused rows numbered 1).
- 2026-11-26, `python tools/probes/late_reaim/label_rows.py out.json d1=2026-11-06-15-53-05-334499-jetson-1580225 d2=2026-11-07-16-15-21-645789-jetson-2503132`: 298 rows, equal to the diagnosis rows (first re-aims 213 accepted / 15 refused, second 13 / 57).
- Readers enumerated with `grep -rln "bump_replay\|late_reaim\|raw_reaim" tests tools`: only `tests/motion/test_skills_bump.py` (comment) and `tools/probes/README.md` (prose).
- 2026-11-26, `bump_replay.py --rows rows_labeled_2026-11-06_07.json --patterns columns_turned --raw --raw-ds --chain --max-catches 1 --record ...` (one sim Catch, four diagonal directions), outputs `temp/probes/late_reaim/chain_ctl_d1_a90_ap1p0.txt` and `chain_ctl_d2_a45_a90.txt`: second re-aims flown-refused refuse in replay 77 % (2026-11-06, 90 deg), 86 % (2026-11-07, 45), 78 % (2026-11-07, 90); flown-accepted second re-aims accept 75 %, 88 %, 67 %. First re-aims flown-accepted accept 88 %, 79 %, 72 %. Per-row, a direction exists that reproduces the flown verdict for 88 to 100 % of refused second re-aims. Direction is not logged, so the case-level figures are a lower bound.
- 2026-11-26, same with `--bump-last --retarget --hold-s 0.05 --lead-knots 5|6 --dirs diag|axis`, 2026-11-06 / 90 deg only: `chain_bump_d1_a90_ap1p0_<dirs>_lead<5|6>.txt`. The 2026-11-07 runs are in the handoff, not run.
- 2026-11-26, `bump_replay.py` gained `--launch-bound MM_S` (probe-local override of the 50 mm/s `BUMP-LAUNCH` bound; `bump.py` untouched; no new test, argument plumbing only). Same flags as the 2026-11-06 diagonal lead-5 run plus `--launch-bound 80`: output `temp/probes/late_reaim/chain_bump_d1_a90_ap1p0_diag_lead5_lb80.txt`. The 2026-11-07 step 4 runs: `chain_bump_d2_{diag,axis}_lead{5,6}.txt`. Default mode regression: 2026-11-26 the 2026-11-25 command (45 deg record, 72 re-parsed rows, diag, lead 0.125 s) gave 937/1152 = 81.3 %, equal to the recorded figure. `pytest tests/motion/test_late_reaim_replay.py tests/motion/test_skills_bump.py -q -p no:xdist`: 32 passed in 2.2 s.
