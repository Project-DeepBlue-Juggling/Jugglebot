---
title: "The mine flavor becomes part of the selection — a shared drop-box stops swapping populations under a fit"
type: bugfix
date: 2026-08-27
status: resolved
phase: "toss-pipelined-preamble — Phase B1 residual"
related_plan: toss-pipelined-preamble.md
files_changed:
  - tools/probes/toss_record_miner.py
  - tools/probes/seat_edge_decomposition.py
  - tests/motion/test_ilc_fit.py
  - tests/ros/test_toss_record_miner.py
subsystem:
  - ros
tags:
  - toss
  - corpus
  - instrument
  - ilc
---

# The mine flavor becomes part of the selection — a shared drop-box stops swapping populations under a fit

## Summary

`temp/probes` is a shared drop-box: any probe may mine any bag for its own
purpose. On 2026-08-27 `tools/probes/seat_edge_decomposition.py` legitimately
re-mined the four post-FW-14 bags **without** `--sensor-only` (it needs the mocap
arc), and `tests/motion/test_ilc_fit.py` — which ranks mines of a bag by stamp
alone — adopted them. Five assertions went red without a line of production code
changing. That is a corpus bug wearing a test failure's clothes: the two flavors
are two **populations** of one bag, not two versions of one corpus (a
`--sensor-only` run returns before `/mocap_data` is read, so every row lands
`excluded_reason='no_mocap_fit'` with every `usable_for_*_fit` flag false).

The change-set:

* **`_corpus_paths` selects newest-per-bag WITHIN ONE FLAVOR**
  (`tests/motion/test_ilc_fit.py`). The mine stamp is a chronological order, not a
  quality one; ranking two flavors by it makes "which population does this module
  fit?" depend on which probe ran last, and it flips silently both ways.
* **`write_outputs` records the flavor in the `_meta.json` sidecar**
  (`tools/probes/toss_record_miner.py`). **Sidecar, not row**: a per-row flavor
  field would have to join `toss_record.FIELDS`, which is the wire schema shared
  with the node's `/toss/record` publisher — a schema change to carry a fact about
  the *mine*, not about the *toss*.
* **A structural fallback for pre-marker artifacts**: does any row carry any of
  five mocap-pass-only columns (`mocap_gap_ms_max`, `apex_z_mm`, `arc_fit_n`,
  `coverage_asym_s`, `release_pos_track_mm`)? Five rather than one because any
  single column is null on a bag whose arc fits all failed — on the 2026-08-20 bag
  exactly **1 of 7** rows carries an arc column at all.
* **Fails closed.** An unreadable file or a malformed row is reported as NOT this
  module's flavor, so a corrupt artefact drops out of the corpus rather than
  joining it.
* **`seat_edge_decomposition.newest_mined` gained the mirror guard** — it skips a
  mine marked `sensor_only`, so a later sensor-only mine can no longer hand the
  arc probe a corpus with nothing to decompose. Absent/unreadable sidecar admits
  the file there (every pre-marker mine in its four bags is whole-arc, and `load`
  still refuses arc-less rows downstream).
* **Four stale prose references re-spelled.** `tools/probes/toss_record_miner.py`
  and `tests/ros/test_toss_record_miner.py` named `_toss_release_state` /
  `_toss_release_cmd` in comments (not code); they now name
  `TossCycleState.release_state` / `.release_cmd`. This closes the one residual
  Phase B1 handed forward.

Two follow-ons from the audit of the above landed with it: the four whole-arc
sidecars from the 00:09–00:17 re-mine were **backfilled** with
`sensor_only: false` (they predate the marker, so both flavor guards were running
on the thinner structural inference — the branch the marker exists to take), and
`tests/ros/test_toss_record_miner.py` gained a parametrised
`test_the_mine_flavor_lands_in_the_meta_sidecar` pinning the new sidecar key in
both states.

## Verification

`pytest tests/motion/test_ilc_fit.py -q`, run 2026-08-27: **5 failed, 64 passed**
before → **66 passed, 3 skipped in 2.36 s** after. The three skips are the
corpus-absent guards and are expected on a tree without the bags.

Wider scoped set, run 2026-08-27:
`pytest tests/motion/test_ilc_fit.py tests/motion/test_cadence_rung_check.py
tests/ros/test_toss_record_miner.py tests/sim/test_plans_index.py
tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q` →
**224 passed, 3 skipped in 34.99 s**. The full gate is the orchestrator's.

`python tools/probes/seat_edge_decomposition.py --csv --json`, run 2026-08-27:
exit **0**, split unchanged at **(a) −1.6 / (b) +102.1 / (c) +85.9 ms**, n = 25 of
33 CAUGHT rows.

**Scratch-tree invariant**, measured 2026-08-27 against a copy of `temp/probes`
with `_REPO` re-pointed at it: a later re-mine of the OTHER flavor — **marked or
unmarked** — and a corrupt artefact (truncated `.jsonl`, malformed sidecar) each
leave `_corpus_paths()`'s 17-file selection **bit-identical**. A later re-mine of
this module's OWN flavor *is* adopted, which is the intended newest-per-bag rule
and the one case that must still move the corpus.
