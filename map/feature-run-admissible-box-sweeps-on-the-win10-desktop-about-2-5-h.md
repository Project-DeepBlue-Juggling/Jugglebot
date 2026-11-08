---
id: FEAT-38
kind: feature
title: 'Run admissible-box sweeps on the Win10 desktop: about 2.5 h on the Jetson, and a sweep blocks Sittings'
status: in_progress
created: '2026-11-02'
updated: '2026-11-03T02:42:08Z'
owners:
- jugglebot-50
priority: medium
effort: m
hardware: none
owner: decision
radius: subsystem
area: tests-tooling
blocked_by: []
links:
- bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms
- bug-box-at-hand-acc-4500-traded-hop-landing-width-for-apex-band
filed_by: claude
---

Filed 2026-11-02 at the owner's request, from the Sitting debrief Session (logbook/2026-11-02-sitting-debrief.md). Not scheduled; the owner will take it up when there is time.

## Problem

An admissible-box re-sweep (`tools/admissible_sweep.py`) takes about 2.5 h on the Jetson. Each gated-file change (feasibility.py, segments.py, cup_cycle.py, cup_realize.py, unified_cycle.py, tilt_geometry.py, hardware_config.py, skills/sites.py) re-hashes the box and needs one.

The runner is 26-29 job families x two rounds (a and b, compared byte for byte), 6 at a time on the Jetson's 6 cores, BLAS 1 thread, about 75 min per round (examples: temp/sweeps/sweep_h4500.sh; the peer's 29-family runner in temp/sweeps/h70/).

While a sweep runs it holds every core at about 85 %, so no Sitting can fly (BUG-58: the setpoint stream starves even under ordinary load). Sweeps and Sittings therefore compete for the same box and the same evenings.

## Idea: run the sweep on the owner's Win10 desktop

The Win10 PC is on the LAN (it hosts QTM, and the owner already keeps a clone there at E:\PDJ\Github\Jugglebot for the MuJoCo viewer). The sweep is pure Python + numpy, so it ports. A desktop x86 core is likely 3-5x an Orin Nano core and there are probably more of them, so the estimate is roughly 15-30 min per sweep, with the Jetson left free.

## Things to get right

1. **Line endings.** gate_hash is a sha256 over the gated files' TEXT, so a Windows checkout with core.autocrlf rewriting to CRLF gives a different hash and a box the loader rejects. Use an LF checkout (`git config core.autocrlf false`, or a `.gitattributes` `eol=lf` for the gated files), and have the runner assert that `admissible.gate_hash()` on the PC equals the Jetson's for the same commit.
2. **Same dependency versions.** Pin numpy (and anything else the planner imports) to the Jetson venv's versions; record them in the box file's metadata.
3. **Cross-platform reproducibility.** x86 and aarch64 floating point (SIMD paths, libm, BLAS) can tip a borderline feasibility sample, so a cell bound may differ by one bisection step from a Jetson sweep. The a/b byte-identity check only proves determinism within one platform.
4. **Jetson-side certification.** Before a PC-swept box is committed, run a certification pass on the Jetson: re-check each cell's chosen rectangle corners and band ends with the Jetson's own `validate_cycle` (a fraction of the search cost). Any corner that fails on the Jetson shrinks that cell or fails the run.
5. **QTM shares the PC.** Fine between Sittings. During a Sitting either do not sweep, or run at BelowNormal/Idle priority with an affinity mask that leaves QTM its cores.
6. **Transport.** Either the Jetson drives the PC over SSH (enable Windows' built-in OpenSSH Server; the runner pushes the commit, runs the jobs, pulls the merged YAML back), or the owner starts it on the PC (PowerShell or WSL2) and the result comes back by git or scp. WSL2 gives a Linux userland (bash runner reused as is) at a small overhead.

## Cheaper speed-ups that need no PC (can land first)

- **Sample round b.** Rounds a and b have been identical on every sweep so far (e.g. the 2026-11-01 82-cell re-sweep). Comparing 3 rotating families instead of all 29 roughly halves the wall time.
- **Profile `admissible_sweep.py`.** It has never been profiled; a Python-level hot loop may hide a 2x.
- **Pause, don't fight.** If a sweep must run on the Jetson during a Sitting, SIGSTOP its workers for the Sitting and SIGCONT after (results unaffected, ETA shifts).

## Open questions for the owner

- The PC's CPU model and core count (sets the worker count and the real speed-up).
- SSH server on the PC, or start the sweep by hand there?
- Is the Jetson-side certification pass the acceptance rule, or must a box always be swept on the Jetson for the record?

## Options (2026-11-03)


1. Cheaper speed-ups first, as a Night Shift unit: compare a rotating sample of 3 families in round b instead of all 29 (rounds a and b have been identical on every sweep), and profile `tools/admissible_sweep.py`. This roughly halves the wall time with no new machine.
2. Sweep on the PC over SSH: Windows OpenSSH server, an LF checkout, pinned numpy, and a Jetson-side certification pass before a box is committed.
3. Sweep on the PC by hand (PowerShell or WSL2), with the result back by git, and the same certification pass.
4. Leave it until a sweep blocks a Sitting again.

Recommendation: 1 now, then 2 if the halved sweep still blocks Sittings. Option 1 needs only your yes. Options 2 and 3 also need the PC's CPU and core count, and your call on whether a Jetson certification pass is enough for the record.

## Option 1 landed (2026-11-03)

- `tools/admissible_sweep_run.py` reproduces the a/b flow; round b re-runs a rotating 3-family sample keyed by the gate_hash (`--sample all` restores the full round). Estimated full sweep 150 min to about 85 min; not yet measured on a real sweep.
- The profile found all hot code inside gated files, so no speed-up was applied (candidates listed in the logbook entry).
- Owner rules: a Jetson-side certification pass is enough for the record; the Win10 PC (option 2) is the backup if the halved sweep still blocks Sittings.
- Next: the first real re-sweep through the runner, once with `--sample all`, to prove round-a parity. Logbook 2026-11-03-jb150-sweep-round-b-sample-and-profile.

## Progress

- 2026-11-03: Owner call (TKT-33): option 1 landed as tools/admissible_sweep_run.py (round b = rotating 3-family sample); a Jetson certification pass is enough for the record; the PC is the backup (detail below)
