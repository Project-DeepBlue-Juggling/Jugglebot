---
title: "Config freshness contract — codegen --check drift gate, launch-time banner, boot-time config identity"
type: feature
date: 2026-08-01
status: resolved
phase: "refactor-2026-07 Phase 5"
related_plan: refactor-2026-07.md
files_changed:
  - config/generate_config.py
  - tests/firmware/test_config_drift.py
  - tests/ros/test_config_identity.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - logbook/INDEX.md
subsystem:
  - config
  - ros
  - testing
tags:
  - codegen
  - observability
  - launch
---

# Config freshness contract (Phase 5)

## What changed and why

Production stays **BUILD-FROZEN** — nodes import the colcon-*installed*
generated modules, so a YAML edit changes nothing until `generate_config.py`
*and* `colcon build` have both run. That is the safe direction (staler-than-
expected fails safe; fresher-than-expected on an actuator path does not), so
the plan's approved answer is to kill the *confusion*, not the frozenness.
Three additions, in increasing distance from the source of truth:

1. **`config/generate_config.py --check`.** `main()` was split into
   `build_artifacts()` (renders everything in memory, returns an ordered
   `(dest, content, label)` plan) + a write path and a check path that consume
   the *same* plan — so a future artifact cannot be added to one and forgotten
   in the other. `--check` diffs the 16 planned artifacts (5 generated + 11
   delivered copies), skips absent parent dirs exactly as the emit path does,
   prints `DRIFT: <file> — <reason>` per drifted file, and exits 1. It performs
   **no writes at all** — not even the `OUTPUT_DIR.mkdir()`, which moved into
   the write path. Three properties of the gate are deliberate and each closes
   a way it could have become useless:
   - **The exit code covers the 14 IN-REPO destinations only.** The other two
     live in `../BallButler/ball_butler_main/` — a *separate git repo*, on its
     own branch, at its own commit. Letting them fail `./run_tests.sh` would
     mean this repo's pre-commit gate goes red because of the working-tree
     state of a repository that is not this one (and the remediation it prints
     *writes into* that external repo); at bring-up it would raise the full
     "this launch is NOT running the source config" banner over two Teensy
     headers no node in this launch reads. Their drift still prints, as
     `EXTERNAL DRIFT:`, which does not touch the exit code.
   - **Anything not checked is announced** (`SKIPPED:` / `NOT CHECKED:`).
     A green verdict over half the surface is the exact trap this feature
     exists to close: with `hardware_config.yaml` absent, 9 of the 16 planned
     artifacts — the entire tuning surface — drop out of the plan, and the
     first version printed `CONFIG FRESH: 7` and exit 0 for it. The launch
     folds any such line into `PARTIAL`.
   - **It compares text, not bytes.** `Path.write_text(..., encoding="utf-8")`
     has `newline=None`, which translates `\n` to `os.linesep`, so on Windows
     every artifact is written CRLF while the rendered string holds LF — a byte
     compare reports 16/16 permanent drift on a perfectly fresh Windows
     checkout, and this repo *is* run from a Win10 clone for sim work. The
     sibling `test_udp_protocol_xlang.py` gate compares text for the same
     reason.
2. **Launch-time drift banner** (`jugglebot_launch.py`). Two independent links,
   reported separately because the fixes differ: (A) source YAML → repo
   artifacts, via `--check`; (B) repo artifacts → **installed** copies, a plain
   byte compare against the package found on `AMENT_PREFIX_PATH`. Link B is the
   one `--check` structurally cannot see and the one that actually bites
   ("I regenerated, why is it still the old gain?"). Fresh → one INFO line.
   Drift → a `!`-bordered banner naming the two commands and RELAUNCH. It
   **never blocks**: bounded 10 s timeout, every failure path (checker missing,
   crash, timeout, no install tree, a module missing from either side of link B)
   degrades to one line, and a check that could not *run* reports `PARTIAL`,
   never a green `OK` — a green line for a check that never happened is worse
   than no line. The timeout is 30× the measured 0.33 s cost and deliberately
   not generous: `ros2 launch` prints nothing until
   `generate_launch_description()` returns, so the bound is the length of an
   unexplained frozen terminal. The repo root is resolved by walking up from
   the launch file itself (`config/generate_config.py` as the marker) before
   falling back to the canonical path, so a build made from a `git worktree`
   compares against *its own* source tree rather than reporting drift that is
   purely an artifact of the diagnostic.
3. **Boot-time config identity** (`teensy_bridge_node`). `hardware_config_identity()`
   logs the resolved path + sha256 + mtime (with a `%z` UTC offset — an
   offset-less local timestamp is ambiguous across a DST change, and the whole
   point is reconciling a bag *later*) of the `jugglebot.hardware_config`
   module the process *actually imported* (read off `hw.__file__`, not
   recomputed), once, at the end of construction. On the Jetson it correctly
   names the installed copy, so a bag or screenshot can be reconciled against a
   specific artifact after the fact. Deliberately scoped to `hardware_config`:
   `friction_ff_params.py`'s env → ament-share → source-tree resolution order is
   the landed 2026-06-24 crash fix in motor_guard's import chain and is untouched
   (plan Phase 5 item 3, owner decision pending).

Both new log sites are single-severity by construction (the launch has only
`LogInfo` available in Foxy anyway; the bridge line is its own `info` call
site) — Foxy's rcutils caches severity per source line and raises on a flip,
which is what killed the node on the first STANDBY arm (`971d12c`).

**The bespoke per-constant stale checks are deliberately NOT deleted.** The
generic gate has to survive a few real config edits before anything is removed
on its authority.

**What Phase 5 item 1 still owes.** The plan asks that *every node* log "its
effective tuning values + a hash of the config it loaded, at boot". Delivered
here is **one node** (`teensy_bridge_node`) logging **identity only** — path,
hash, mtime — not the effective tuning values, and not the other nodes. That
covers the process that owns the leg/hand actuation path, which is where the
confusion has actually bitten, and it is the piece the launch banner pairs
with; the rest is real remaining work, not a silent descope. Item 3
(`friction_ff_params.py` resolution order) is likewise untouched pending the
owner decision, so **Phase 5 is partially delivered** — the drift gate (the
plan's own parenthetical scope for this phase) is complete.

**Honest scope**, restating the plan: this would *not* have caught `24608bb`'s
stale firmware binary. That was an incremental-build artifact, not a codegen
artifact; the fix for it is the pio-clean forcing in Phase 7.

`return LaunchDescription([*_config_freshness_actions(), ...])` uses list
unpacking rather than list concatenation on purpose:
`tests/ros/test_launch_nodes.py` slices the launch body with a
`return LaunchDescription\(\s*\[(.*?)\]\s*\)` regex, and the concatenation form
broke its collection.

## Verification

- Deliberate-drift red run (`python config/generate_config.py --check`, run
  2026-08-01): clean → `CONFIG FRESH: 14 artifact(s)`, exit 0. Appending one
  line to the delivered `ros_ws/src/jugglebot/jugglebot/hardware_config.py` →
  1 `DRIFT:` line, exit 1; also dirtying `config/generated/geometry-config.js` →
  2 DRIFT; additionally `mv`-ing away `config/generated/protocol_config.h` →
  3 DRIFT with the third reported as `missing`. All restored → `CONFIG FRESH`,
  exit 0, tree clean.
- External-partition red run (2026-08-01): appending a line to
  `../BallButler/ball_butler_main/hardware_config.h` →
  `EXTERNAL DRIFT: … (separate checkout; does not fail this gate)`,
  **`CONFIG FRESH: 14`, exit 0**, and `pytest tests/firmware/test_config_drift.py
  -q` still **21 passed**. Restored; the BallButler checkout was verified clean
  afterwards (`git status -s` empty).
- Partial-run announcement (2026-08-01): with `HW_YAML` redirected to a
  nonexistent path (in-process, no writes), `--check` prints
  `NOT CHECKED: Skipped hardware config (not found): …` and
  `CONFIG FRESH: 6` — the launch folds that into `PARTIAL`. Before the fix it
  printed a bare green `CONFIG FRESH: 7` covering half the surface. Pinned as
  `test_check_mode_announces_what_it_did_not_check`.
- No-write proof (2026-08-01): `mtime_ns + st_size + sha256` snapshotted for all
  16 destinations before and after a `--check` run — **16/16 identical**. Pinned
  as `test_check_mode_reports_fresh_and_writes_nothing`.
- Launch-interpreter budget (2026-08-01): `--check` runs under `/usr/bin/python3`
  (3.8.10, system site-packages, `VIRTUAL_ENV`/`PYTHONPATH` scrubbed) in
  **326 ms**; PyYAML is the only non-stdlib import. Its verdict and its rendered
  bytes are both asserted equal to the venv's (system PyYAML 5.3.1 vs venv
  6.0.2 — a major version apart, agreeing today with nothing pinning it, which
  is why `test_both_interpreters_render_identical_bytes` exists).
- Launch branches exercised offline against the INSTALLED launch file
  (2026-08-01): fresh → `[config] freshness check OK — hardware/protocol/
  geometry codegen and the installed copies agree.`; a dirtied delivered
  `protocol_config.py` → the full banner naming BOTH links (`DRIFT:` and
  `STALE INSTALL:`); an installed module moved aside → `PARTIAL` naming it and
  suggesting `colcon build`; bogus `JUGGLEBOT_REPO` → `PARTIAL`, no banner, no
  raise. `_repo_root()` resolved to `/home/jetson/Desktop/Jugglebot` from BOTH
  the source and the installed launch file (the worktree case).
- `pytest tests/firmware/test_config_drift.py tests/ros/test_config_identity.py -q`,
  run 2026-08-01: **26 passed in 2.38 s**. Gate red-check, same file alone
  (`pytest tests/firmware/test_config_drift.py -q`, run 2026-08-01): clean
  **21 passed**; with one line appended to the delivered `protocol_config.py`,
  **2 failed, 19 passed** — the parametrised id naming the exact file, plus the
  no-write/freshness test. Restored, tree clean.
- `colcon build --packages-select jugglebot`, run 2026-08-01: 1 package finished
  (2.49 s). **The launch runs the INSTALLED copy** — a relaunch is required for
  the banner and the identity line to appear; all launch branches above were
  driven against the installed file, not the source tree.
- Full gate `./run_tests.sh --full`, run 2026-08-01 on the Jetson under
  `~/Desktop/PDJ_venv/venv`: **parallel 4339 passed, 3 xfailed in 439.28 s;
  serial 9 passed in 40.13 s; total 485 s; RESULT: PASS** (exit 0).
