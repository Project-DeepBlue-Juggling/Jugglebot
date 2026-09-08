# Bench session — hand geometry re-validation (G3)

Bench checklist for `plans/active/hand-geometry-correction.md` phase G3 (see
that plan's "G3 — Bench re-validation" phase and its "Confirm before Part B"
section above, and the full rationale in
`logbook/2026-09-08-hand-geometry-flown-position-audit.md`'s Diagnosis
section). The
owner's acceptance criterion is explicit: the legacy `x2` (release) and `x3`
(catch prime / stroke top) points get re-validated on the **current** gain
**before** the geometry correction ships, so that any later difference is
attributable to the gain fix and nothing else. Run this **after** the FW 18
bench sitting (`session_fw18_flash.md`) — do it on the same bench visit if
convenient.

Harness: `tests/hardware/toss_trace_recorder.py` (`record` then `check`) —
observes only, never commands. Writes `temp/logs/toss_trace_<stamp>.jsonl` +
a `_meta.json` (git SHA, per-topic counts, bridge `uptime_ms`).

## Part A — baseline, current branch (mvp-trajectory-bringup)

Run this on the branch you are already on after the FW 18 flash. Nothing
here touches the geometry correction; it exists to freeze what the machine
is doing *before* that correction lands.

**Pre-flight**
- [ ] `git status -sb` — confirm you are on `mvp-trajectory-bringup` and the
      FW 18 flash has landed (`session_fw18_flash.md` step "Bring-up check").
- [ ] Whole sitting is BENCH. No ball anywhere near the hand. E-stop within
      reach and tested before arming.

**Dry-run rehearsal — no power**
- [ ] `python tests/hardware/toss_trace_recorder.py check <an existing
      trace>.jsonl --dry` in the venv. Confirm it prints per-invariant
      PASS/FAIL lines and exits 0 before the robot is waiting on it.

**Bring-up**
1. Home the hand (`HOME(6)`, or the normal launch home sequence). Confirm
   `is_homed` and (per FW 18) `ctrl_mode`==3 / `input_mode`==1 on axis 6.
2. Log `uptime_ms` with every measurement below (habit, not a live hazard —
   the uptime-lag arc closed 2026-08-15).

**Step 1 — the metal-stop reading.** Drive the hand slowly to the top stop
and read the encoder. **Expect 10.701 rev ± 0.02.** Same at the bottom:
**expect −0.107 rev.** This is the single measurement the whole correction
is built on. **If either is off by more than 0.05 rev, STOP** — record the
readings and do not proceed to Part B.

**Step 2 — x3 / catch prime, static.** Command the catch prime
(`JB_OP_HAND_CATCH_PRIME_REV` = 9.9594 rev) via the legacy `smooth_move_hand`
path. Record:
- settled encoder position (expect 9.9594 ± 0.10 rev),
- physical cup height if a ruler or mocap can see it — **expect ≈1003.97 mm
  in Part A too**: this height is a property of the mechanism (9.9594 rev ×
  the measured 32.5685 mm/rev) and does not change with which software model
  is running. Part A's own model still *predicts* ≈994.6 mm at this rev (the
  old, wrong gain) — that is a label, not a measurement. If the ruler reads
  near 994.6 mm instead of ≈1003.97 mm, the MEASUREMENT is suspect (wrong
  reference plane, parallax), not "the hand doing the old gain" — the
  mechanism has no gain model to run,
- hand `iq_rms` at rest.

**Step 3 — x2 / release, static then dynamic.** Command a legacy kind-0
throw at a LOW tier. Record with `toss_trace_recorder.py record`. Note:
- commanded release position (expect 5.9138 rev; physical cup z ≈872.21 mm),
- commanded `event_vel` vs. the achieved release velocity read off the
  encoder,
- ball apex.

Walk the tier ladder upward one tier at a time, scoring each before the
next, and repeat Step 3 at **two or three tiers total**. This is the
pre-correction baseline the audit's ~11 % excess-throw-velocity number was
measured against — without it, nothing in Part B has anything to compare to.

**Stop rules (any one → power down, do not continue):**
- peak hand position > 10.501 rev (FW 18's clip; 10.60 was the pre-FW-18
  ceiling) — E-STOP per the runbook's HAND-4 / HAND-7 rows;
- `dip_below_x3` > 0.100 rev;
- top-stop reading off 10.701 by more than 0.05 rev;
- audible contact with either end stop;
- any hand fault (a HAND fault E-STOPs the legs).

## Confirm before Part B

These are orchestrator assumptions, not owner-confirmed facts (full
rationale in the logbook entry above). Read them back to the owner before
switching branches:

- **D1** — unified cup-z throw/catch sites re-set to the physically-flown
  865.37 / 834.47 mm (not left at the round 860 / 830).
- **D2** — the MJCF hand-joint clip is emitted as the derived
  travel-above-encoder-zero (348.524 mm), not the raw stop-to-stop stroke.
- **D3** — `teensy_trajectory.hand_stroke_m` is re-based (0.355 →
  0.3643707) so commanded rev at x2/x3/x5/prime does not move.
- **D4** — the measured coast ladder's v-axis is left untouched (reads
  ~5.7 % conservative afterwards, not re-scaled).
- **D5** — `sim/cycle_gate.py`'s 0.690 / 0.985 m literals follow the re-base
  to 0.6896 / 0.9940.
- **D6** — the C-HAND-2 open-loop undershoot ceiling is NOT raised: the
  corrected gain lifts the worst open-loop commanded undershoot at the band
  floor to 0.629 rev against the landed 0.60 rev ceiling
  (`test_wire_quantisation_cannot_produce_a_visible_undershoot`), and the
  test stays `xfail(strict=True)` pending an owner decision (raise the
  ceiling, document-first, or reduce the feedforward) — not a bench item,
  flagged here only so Part B's checklist matches the plan's D1-D6 set.

## Part B — switch to the correction

`~/Desktop/Jugglebot-geometry` is a **separate worktree checkout** of branch
`hand-geometry-correction` — building or flashing from it does **not**
change anything in `~/Desktop/Jugglebot` (the `mvp-trajectory-bringup`
checkout you used for Part A).

1. `cd ~/Desktop/Jugglebot-geometry`
2. `git status -sb` — confirm branch `hand-geometry-correction`, confirm G2
   has actually landed (the YAML re-base + regenerate + code edits commit;
   check the plan's own status line if unsure).
3. `cd ros_ws && colcon build --packages-select jugglebot && source
   install/setup.bash` — build and source **this worktree's** install, not
   the one on `mvp-trajectory-bringup`.
4. Flash the Platform Teensy to **FW 5** — Arduino IDE only, the `pio` image
   is CAN-MUTE. Flash
   `ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino` from
   this worktree. Receipt: serial boot banner `[boot] jugglebot-platform v5`,
   or `platform_fw_version` on `/link_status` reading `5` after the next
   authoritative read.
5. Swap in the re-fitted ILC artifact — **staged at
   `temp/probes/toss_ilc_g2a3_staged.yaml`** in this worktree (re-fit
   content ready; NOT dropped at the live `config/toss_ilc.yaml` path — see
   below for why). Copy it into place immediately before this step:
   `cp temp/probes/toss_ilc_g2a3_staged.yaml config/toss_ilc.yaml` — that
   makes `toss_ilc.resolve_toss_ilc_path()` find it automatically (confirmed
   2026-09-08, no env var needed). ⚠ **`tests/motion/test_toss_ilc.py::
   test_the_repo_ships_no_ilc_artifact_yet` asserts `config/toss_ilc.yaml`
   does NOT exist** — it is `critical-point-ilc.md`'s own Phase-3 A/B
   shipping gate, unrelated to this plan, and its own failure message says
   to delete it in the same commit that ships the artifact for real, with a
   logbook entry. Copying the staged file into `config/` for THIS bench
   session will fail that test the next time `./run_tests.sh` runs on this
   branch; `git checkout -- config/toss_ilc.yaml` (or just `rm` it, it is
   untracked) after the session, before any commit, restores it. Do not
   proceed to Step 6 with the old (`event_vel_trim ≈ −0.1076`) artifact
   still loaded; confirm the loaded trim is the corrected value
   (`event_vel_trim = −0.081044`, ≈ −0.0810) before any throw — the fit
   itself is gain-blind (re-running it on the corrected config reproduces
   −0.1076 unchanged, because `d(flight_time_err)/d(event_vel_trim) = 2/g`
   never touches the rev/m gain), so this artifact's trim was hand-rescaled
   via `(1 + old_trim) × 1.029748 − 1 = −0.081044` (the measured
   `TEENSY_LINEAR_GAIN` ratio applied to the trim's multiplicative factor —
   NOT `old_trim × 1.029748`, which gives the wrong −0.1108) rather than
   re-derived from the fit; see the staged file's own `captured.note` field
   for the full arithmetic. If the staged file is ever missing, re-derive it
   from `tests/hardware/ilc_corpus_fixture.py` per that same note.

   Note: the trim is keyed to the ILC corpus's captured cell (z=160 mm,
   flight ≈0.9 s) — if Part B's tiers don't land in that cell, the applied
   trim is 0 (not −0.0810), and the release-velocity comparison below reads
   differently (see Compare).
6. Repeat Steps 1–3 above exactly, same tiers, same recording harness.

**Compare Part A vs. Part B:**
- **Commanded rev must be IDENTICAL** at the metal-stop reading, x3, and x2
  (that identity is the re-base's whole promise — a difference here means
  D3 did not land as designed, not that the machine moved).
- **The achieved-vs-commanded velocity ratio should drop from ~1.03 to
  ~1.00** — that is the fix. Whether *absolute* release velocity moves
  depends on whether the ILC cell (z=160 mm, flight ≈0.9 s) admits: **if it
  admits, the re-fitted trim cancels the ratio change by design and absolute
  release velocity is unchanged from Part A** (trimmed = event_vel ×
  (1 + trim), 0.9190 × base = A); **if it does not admit, trim is 0 and
  absolute release velocity reads ~2.9 % lower than Part A.** Record which
  case obtained.
- **Physical cup-z readings (Step 2/3) should be unchanged between A and
  B** — ≈1003.97 mm in both, since the commanded rev and the mechanism are
  identical in both parts. What changes is the *model's* stated prediction,
  from ≈994.6 mm (Part A, the old wrong gain) to ≈1003.97 mm (Part B, the
  corrected gain): the model-vs-ruler gap should close from ~9.4 mm in
  Part A to ~0 in Part B.

## Part C — pass / fail

**Pass** means: rev positions bit-identical across A/B, the
achieved-vs-commanded velocity ratio measurably closer to 1.00 in B
(absolute release velocity unchanged from A if the ILC cell admitted,
~2.9 % lower if it did not — either is a pass, record which), mm/cup-z
readings unchanged from A and agreeing with the corrected model's
prediction in B, no stop rule triggered in either part. Merge
`hand-geometry-correction` into `mvp-trajectory-bringup`.

**Fail** means: any rev position differs between A and B, or a stop rule
triggered, or the mm/cup-z readings still disagree with the model in B. Do
**not** merge. Bring back: the exact rev readings from both parts, the
metal-stop reading from Step 1 of Part B (confirms whether the bench
geometry itself is stable), and the release-velocity numbers from both
parts' Step 3.
