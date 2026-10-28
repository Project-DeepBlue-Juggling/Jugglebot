---
title: "JB-84: Ball Butler over-CAN flash hook repointed from the retired Jugglebot-skills worktree to ~/Desktop/Jugglebot"
type: bugfix
date: 2026-10-28
status: resolved
phase: "inbox-wall"
files_changed:
  - map/bug-ball-butler-over-can-flash-calls-the-retired-jugglebot-skills-worktree.md
  - docs/project-map.md
subsystem:
  - ball-butler
tags:
  - ball-butler
  - flash
---

## What changed (BallButler repo, not this one)

`ball_butler_main/platformio.ini` (`upload_command`, line 94, and its comment block) and
`ball_butler_main/scripts/bb_link_check.py` (the `JUGGLEBOT_DIR` default) called
`~/Desktop/Jugglebot-skills/tools/teensy_link_bridge.py`. That worktree was retired on
2026-10-23 when `skill-stack` merged into `main` and the single checkout became
`~/Desktop/Jugglebot`, so the next over-CAN Ball Butler flash would have failed. Both now
point at `~/Desktop/Jugglebot`; `tools/teensy_link_bridge.py` exists there. The
accuracy-testing sitting script's `WS` default and the one runbook command in
`LOCAL_CALIBRATION.md` got the same edit; the runbook's historical sentences naming the old
worktree are left as written. Found by the JB-34 logbook survey (2026-09-30 r5-sitting-1
residue), pushed to BallButler `main` 2026-10-28.

## Verification

- 2026-10-28, `grep -rn "Jugglebot-skills" .` in the BallButler repo (excluding `.pio`,
  `node_modules`): only the three historical runbook sentences remain.
- 2026-10-28, `ls ~/Desktop/Jugglebot/tools/teensy_link_bridge.py`: present.
- Not flown: the next over-CAN flash is the real test; `bare pio run -e teensy41 -t upload`
  from `ball_butler_main/` with the launch down.
