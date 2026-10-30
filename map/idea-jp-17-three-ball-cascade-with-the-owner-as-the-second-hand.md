---
id: JB-113
kind: idea
title: JP-17 Three-ball cascade with the owner as the second Hand, Platform held tilted toward them
status: inbox
created: '2026-10-30'
updated: '2026-10-30T09:07:33Z'
owners: []
priority: medium
effort: xl
hardware: robot
owner: decision
radius: cross-cutting
area: skill-stack
blocked_by: []
links: []
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
---

Owner 2026-10-30: a plain three-ball cascade, every throw at an equal Apex and timing, the Platform held tilted toward the owner the whole Attempt, catch and throw sites free. Timing: each ball's period is 3 Beats = flight + Dwell for both hands, so the robot's Dwell equals the owner's; the robot's empty gap 2 Beat - Dwell must clear about 0.30 s (transit-free if it catches and throws at one Site, which drops the floor toward 0.15 s). At a human-comfortable Dwell of 0.5 s that gives Beat 0.40-0.45 s, flight 0.70-0.85 s, apex 0.60-0.89 m. Pass distance at a held tilt: g t_f^2 tan(theta) / 2 = 0.51 m at 12 deg and flight 0.70, 0.75 m at flight 0.85; 9 deg gives 0.38 / 0.56 m. A cup tilted toward the owner faces the incoming ball squarely (its velocity reversed is the cup axis), so one held tilt serves both the throw and the catch. Blockers in order: (1) perception of a human throw (detect_human_throws exists but is off and spawned 158 phantom tracks on a static marker 2026-09-15; R5 Block C registered 0 lobs 2026-09-30, undiagnosed; a human track has no throw_time so no identity latch resolves it); (2) a reactive CATCH anchored to the observed flight, as compile_columns(feed=...) anchors to the Ball Butler's release, with the robot's THROW time derived from the owner's tempo and the catch re-aimed after the fit; (3) a held-tilt planner mode (the aimed-throw reach inside a catch-and-throw window measured only 250-300 mm with the tilt acquired inside the window; untraced). First rung is a tracker diagnosis replaying the 2026-09-30 bag through the parabolic detector.
