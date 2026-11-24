---
title: "BUG-75 steps 1-3: late_reaim probe scripts re-pointed (turned-pair angle flag, log dirs as arguments, raw re-aim control) and run on the 2026-11-06/07 refusals"
type: feature
date: 2026-11-25
status: complete
phase: ""
related_plan: "late-catch-reaim-bump.md"
related_issues: []
files_changed:
  - tools/probes/late_reaim/capture_records.py
  - tools/probes/late_reaim/parse_resends.py
  - tools/probes/late_reaim/bump_replay.py
  - logbook/2026-11-25-late-reaim-replay-tooling.md
subsystem: [tooling, skill-stack]
---

## Summary
Tooling only; no flown code touched. `capture_records.py` gains `--angle` (passed to `SkillsGateConfig.angle_deg`; the pickle name gets `_a<angle>`) and now also stores each install's terminal, seed rest, cfg and the record before it. `parse_resends.py` takes `<out.json> tag=logdir ...` instead of the three hardcoded 2026-10-16 directories. `bump_replay.py` gains `--raw` / `--raw-ds`: replay the unbumped raw re-send (install_segment with the terminal moved by the row's delta) as the "before" control. Nothing had rotted: the scripts ran unchanged against today's code.

## Verification
- 2026-11-25, `python tools/probes/late_reaim/parse_resends.py out.json d1=2026-11-06-15-53-05-334499-jetson-1580225 d2=2026-11-07-16-15-21-645789-jetson-2503132`: 121 RESEND-REFUSED lines found across all patterns; 72 are columns_turned (the replay rows).
- 2026-11-25, `python tools/probes/late_reaim/capture_records.py --angle 45|90 --tag columns_turned`: 10 installs, 8 CATCH accepted each.
- 2026-11-25, `python tools/probes/late_reaim/bump_replay.py --rows ... --patterns columns_turned --records <a45|a90 pkl> --retarget --hold-s 0.05 --dirs diag|axis [--lead-knots 6]`: results are in the BUG-75 Progress text (orchestrator).
- 2026-11-25, `pytest tests/motion/test_skills_bump.py -q -p no:xdist` (the only test naming `late_reaim`, by comment only): 25 passed in 39.46 s.
