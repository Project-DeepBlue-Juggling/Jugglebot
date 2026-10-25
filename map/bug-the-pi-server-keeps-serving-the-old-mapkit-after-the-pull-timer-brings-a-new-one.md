---
kind: bug
title: The Pi server keeps serving the old mapkit after the pull timer brings a new one
status: complete
created: '2026-10-24'
updated: '2026-10-25T05:29:54Z'
owners: []
priority: medium
blocked_by: []
links:
- logbook/2026-10-24-project-map-p3-p5-fork-server-half.md
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map/commit/711e413
parent: project-map
filed_by: claude
---

The jugglebot-map server imports the checkout's `mapkit` once at startup. The Pi's two-minute pull timer fast-forwards `~/Jugglebot`, so a `mapkit/` change on `main` (the libyaml speed-up of 2026-10-24 is the first) reaches the disk but is not served until someone runs `sudo systemctl restart jugglebot-map`; a change to the item schema would silently diverge. Fix in the fork's `deploy/jugglebot-map-pull.service`: after the pull, restart the server when `git diff --quiet ORIG_HEAD HEAD -- mapkit` says the parser changed (an `ExecStartPost` script), with a README line.

## Progress

- 2026-10-25: fixed in the Jetson hosting: the pull unit runs deploy/jetson/pull.sh, which restarts jugglebot-map when mapkit/ changed (fork 711e413, tests/test_pull_restart.py); the Pi is retired
