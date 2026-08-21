---
title: "Bridge temporal trustworthiness arc opened — the uptime latency drift finally gets an owner, and the clock plan gets sequenced behind it"
type: refactor
date: 2026-08-11
status: in-progress
phase: "bridge-temporal-trustworthiness — plan kickoff (pre-P0)"
related_plan: bridge-temporal-trustworthiness.md
files_changed:
  - plans/archived/bridge-temporal-trustworthiness.md
  - plans/active/bridge-clock-frequency-discipline.md
  - plans/active/INDEX.md
  - logbook/2026-08-11-bridge-temporal-trustworthiness-kickoff.md
  - logbook/INDEX.md
subsystem:
  - can
  - motion
tags:
  - docs
  - performance
  - IPC
---

# Bridge temporal trustworthiness arc opened

Planning change only, no code. `plans/archived/bridge-temporal-trustworthiness.md`
is created and takes ownership of the can-bridge uptime command-latency drift —
until now owned solely by the open
`logbook/2026-07-18-teensy-uptime-tracking-degradation.md` — and sequences the
dormant `plans/active/bridge-clock-frequency-discipline.md` clock-precision
phases behind it. Same change-set: a new row in `plans/active/INDEX.md`, and a
sequencing status note added to the clock plan (which stays the authoritative
design document for its half).

**Why now.** The drift has spread past leg tracking: 2026-07-28 measured it in
the throw dispatch shift (+14.5 ms at 0.24 h uptime → +54–63 ms at ~1.6 h), and
2026-08-10 sized the possession ARRIVAL windows *around* it rather than around a
fresh-boot plant. The operator wants the Teensy able to stay up indefinitely
with zero drift, and the four-arm isolation experiment pre-registered on
2026-07-18 has still never run.

**Owner decisions (harrison, 2026-08-10/11).** Route **B then A** — telemetry
before experiment. Both halves in one arc. **S1 (the aged-bridge sitting) runs
BEFORE the FW 11 flash**, because a flash reboots the Teensy and resets the aged
state the experiment exists to study; the reboot-before-every-session standing
rule is suspended for that one sitting.

**One new technical finding, recorded in the plan.** The TOD anchor's `rtt/2`
compensation assumes path symmetry, so a transport-side latency drift can bias
the *clock* by up to half the one-way delay — the two "complementary halves" are
coupled, which is why the clock servo (P4) must follow the latency fix (P3).
Relatedly: kernel-RX stamping alone does not delete the server-processing jitter
term, it flips its sign; P2 returns the midpoint `(t2+t3)/2` instead.

**Phases.** P0 LEG_CMD echo published + `/profile` bagged (Jetson-only, so
bridge aging is preserved) → P1 FW 11 `CLOCK_DIAG` + interp-occupancy counters,
written NOT flashed → P2 midpoint TOD stamp → S1 aged sitting + the four arms →
P3 fix + the alarmed latency monitor the 2026-07-24 closure contract requires →
P4 clock plan Phases 3–5.

## Verification

Docs-only change, but inside the test surface: `tests/sim/test_plans_index.py`
pins the plans INDEX ↔ `plans/active/` correspondence this change-set touches,
and the logbook front-matter/search tests parse this entry. Gate
(`./run_tests.sh`, run 2026-08-11): **4974 passed in 232 s (parallel 224 s
rc=0, serial phase empty), RESULT: PASS.** `/audit --unstaged` ran before
commit (multi-document narrative gate): 1 WARNING + 2 NOTEs, all three fixed
in this change-set (stale `Last touched` cell; an unverifiable Python-version
rider replaced with the measured claim; a quote pointer widened to the section
that actually contains the quoted sentences).
