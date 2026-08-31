---
title: "Unified 7-DoF planner probe 3 flown — the 7th frame costs 5.5 pp of bus and the leg-bus drop rate does not follow it"
type: feature
date: 2026-08-30
status: resolved
phase: "unified-7dof-planner — Phase 0"
related_plan: unified-7dof-planner.md
files_changed:
  - tests/hardware/session_unified7_bus_headroom.md
  - plans/active/unified-7dof-planner.md
  - plans/active/leg-bus-frame-drops.md
  - logbook/2026-08-30-unified7-bus-headroom-sitting.md
  - logbook/INDEX.md
subsystem:
  - can
  - motion
tags:
  - firmware-bench
  - performance
  - trajectory
---

# Unified-7DOF bus headroom — probe 3's bench arm

**What.** The operator flew `tests/hardware/session_unified7_bus_headroom.md`
rows 11–14 on one boot (bench7 off/on/off/on; an 11-move battery was briefed per
arm — only three ran, arm A none, see the qualifications) with the BENCH image
aboard. This entry records the verdict; the
runbook's Results section carries the full table and every qualification.

**Headroom: comfortable.** The 7th frame flowed — `can1_tx` steps
**3150 → 3650 fps, exactly +500**, twice, and back to 3150 in between, which
closes the row-20 unseen-hand hazard from the wire side. `can1_util_pct`
**56.7 → 62.2 %, +5.5 pp**, *below* the pinned ~65 % prediction and well under
the ~70 % line (operator's GUI read ~62.5 %). `defer jb` increment **0** and
`txq jb` **0** in every arm — the software txBuffer was never entered.
Validity clean throughout: `leak_* ≡ 0`, fifo overflows/warns 0, all fifteen
`can3_errors` fields 0 (`ack=0`), `rx_cap_hits_* ≡ 0`, `decode_short` 0,
`seen_mask` 127, `interp_deadline_misses` 0 and `interp_max_jitter_us` peaking
at **2 µs in every arm**, 6- and 7-frame alike, `latency_monitor` OK on all
3358 samples. One boot, `uptime_ms` 61.5 → 397.2 s.

**Decision rule: the drop rate does NOT scale with bridge TX rate, so the
`leg-bus-frame-drops` source fix does not sequence before Phase 3.** Per-axis
encoder deficit pooled: **−0.76 ± 5.68 at 6 frames/tick (12 episodes / 135 w)**
vs **−0.28 ± 3.73 at 7 frames/tick (2 / 124)** — *smaller* at the higher rate.
Battery-moving windows only: **−1.06 ± 7.06 (3 eps / 21 w)** at 6 frames vs
**−0.02 ± 0.25 (0 eps / 41 w)** at 7. Judged against the A-vs-A′ within-boot
spread — the runbook's designed drift control, here used as the yardstick for
"materially" — that spread (quiet windows −0.78 vs 0.00) *exceeds* the
6-vs-7 difference of 0.48 and the difference points the wrong way for the
hypothesis. § 4.1's falsification branch, not its conviction branch.

**Three qualifications, so it is not over-read.** (a) Only three of the four
batteries are in the bag and **arm A carries none** — motion sits in three
~20 s blocks at t 242–262 (arm B), 280–299 (A′), 313–332 (B′), so the moving
comparison is one 6-frame battery against two 7-frame ones. (b) This plant is
~5× cleaner than the 2026-08-15 reference (−0.76 vs −3.6 ± 10.3), so a +16 %
(6→7 frames/tick; 3150→3650 fps) arm had little dynamic range — which is the argument for row 16's −50 % lever.
(c) The only hand-axis (ax 6) episodes in the bag fall in arm B (4 windows,
worst −49), but B′ had none and the same shape hits axes 0–5 in arm A with no
7th frame, so it reads as the ordinary episodic pattern landing on axis 6.
**Arms C (drives quiet) and D (250 Hz) were not flown** — C deferred by the
operator, D's companion build never authorised.

**FW 16 is now aboard.** `bridge_fw_version` reads `16 (proto 5)` with
`install_skew` 0 on all 3358 samples — the `15 (SKEW — expected v16)` advisory
that has run since ~2026-08-20 is gone. Its console-side acceptance half (the
ball-sensor poller's ~42 → ~50 Hz) is still unread; the bench had no ball.
Whether the stock image was reflashed after close-out is not determinable from
the bag (runbook rows 25–26) — confirm the boot banner before any non-bench
session.

**Verification.**

- Bag: `~/Desktop/rosbags/2026-08-30_15-59-52/` (46 MB `.mcap`, 337 s,
  169 514 msgs, recorded 2026-08-30 15:59:52–16:05).
- Analysis (2026-08-30):
  `python /tmp/probe_bag_unified7_headroom.py ~/Desktop/rosbags/2026-08-30_15-59-52`
  → the four-arm table in the runbook's § Results. Ad-hoc seed for the § 2.4 tool the runbook's
  Artifacts § flags as missing; promote to `tools/probes/` before the next sitting.
- Row-1 gate: `./run_tests.sh --full`, run 2026-08-30 → **6676 passed,
  4 skipped, 3 xfailed (+ 9 serial) in 580 s, PASS**.
- Cross-check: 6-leg deficit vs `can1_rx` deficit **r = 0.453, slope 0.670**
  (2026-08-15 reference 0.62 / 0.82) — the loss still localises before the
  bridge's CAN peripheral.
