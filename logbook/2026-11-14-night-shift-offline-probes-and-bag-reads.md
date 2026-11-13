---
title: "Night Shift runs probes and bag reads offline: nonet binds the evidence read-only and masks serial"
type: feature
date: 2026-11-14
status: complete
phase: "night-shift close-out"
subsystem: tools
files_changed:
  - tools/night_shift_nonet.sh
  - tools/night_shift_settings.json
  - tools/night_shift_bash_guard.py
  - .claude/commands/night-shift.md
  - tests/sim/test_night_shift_settings.py
  - logbook/INDEX.md
---

## What

Close-out C7(a). The owner wants the night to run offline probes over the rosbags and
launch logs (TKT-68's nominate/tilt-back sweep, BUG-75's capture/replay), which the deny
list refused outright (`*tools/probes/*`, `*ros2 bag*`).

- `tools/night_shift_nonet.sh` now also binds `~/Desktop/rosbags` and `~/.ros/log`
  read-only, binds `/dev/null` over every `/dev/ttyACM*`, `/dev/ttyUSB*`, `/dev/ttyTHS*`
  (Jetson UARTs) and `/dev/hidraw*` (the Teensy HID path), mounts an empty read-only
  tmpfs on `/dev/serial` and `/dev/bus/usb` (libusb's nodes), and points an unset
  `ROS_LOG_DIR` at a temp dir (rcl would otherwise fail on the read-only log dir). Any
  failed mount exits 125 without running the command.
- `tools/night_shift_bash_guard.py`, a PreToolUse hook on the Bash tool wired in
  `tools/night_shift_settings.json`, is the ONE enforcement point. A command whose
  normalised text (quotes and backslashes dropped, whitespace collapsed) mentions
  `probes`, `ros2 bag`, `rosbag` or `.ros` runs only if, after leading whitespace, its
  first word is a wrapper file byte-identical to `origin/main:tools/night_shift_nonet.sh`
  (by any path: `tools/`, `./tools/`, `D/gate-base/tools/`) and the line holds none of
  `; & | < > $(`, backtick or newline. `ros2 bag play|record|reindex|convert` never run.
  A refusal is exit 2 with the accepted shape on stderr.
- The hook command runs MAIN's guard, `g=$(timeout 20 git -C "$CLAUDE_PROJECT_DIR" show
  origin/main:tools/night_shift_bash_guard.py) && /usr/bin/python3 -I -c "$g" || exit 2`,
  so a branch checked out in the clone cannot weaken it before the merge gate, and a
  guard that cannot be read or started refuses every Bash call. `Edit`/`Write` of any
  `.claude/settings*.json` is denied: the docs say settings edits are picked up live, and
  `disableAllHooks` would switch the guard off.
- The ~40 spelling denies the first cut of this commit added are back to a small
  defence-in-depth set: the four bag verbs (`*ros2 bag play*` etc.) and five line-start
  rules (`python*tools/probes/*`, `tools/probes/*`, `./tools/probes/*`, `cd tools/probes*`,
  `ros2 bag*`), plus `Edit`/`Write` denies on `~/.ros/**`. Invariant 2 of the protocol
  states the accepted shape.

## Discussion

**Why a hook and not more deny patterns.** The first cut anchored deny rules on the
unwrapped spellings (line start, after `&&`, `;`, `|`). Its own residual list showed the
class: `FOO=1 python tools/probes/x.py`, `/usr/bin/python3 tools/probes/x.py`,
`bash -c "..."`, `cd tools && python probes/x.py`, `xargs`, and `nonet x && python
tools/probes/y` all ran unwrapped, and each fix was one more pattern. A permission rule
can refuse a spelling but cannot demand a prefix; a hook can state the invariant
positively ("the whole line is one wrapper call") and refuse everything else, so the
whole class closes at one point with one table-driven test.

**Strict over clever.** The guard does not parse shell. Any operator that could start a
second command or write outside the namespace by redirection is refused on a guarded
line, even inside quotes (`nonet bash -c "a; b"`), and reads count (`ls ~/Desktop/rosbags`
must be wrapped). Guarding the bare word `probes` rather than `tools/probes` catches the
`cd tools` then `probes/x.py` spelling at the price of also guarding `temp/probes`. A
refused legitimate command is rewritten by the night; a missed one reaches the evidence.

**Trust by content, not location.** Accepting any path ending in
`/tools/night_shift_nonet.sh` would let a branch-supplied fake through, and the clone's
own `tools/` copy IS branch-supplied (the night switches branches in the clone). Pinning
one location (`D/gate-base/`) needs the guard to know D. Comparing the file's git blob
hash with `origin/main`'s does neither: whatever passed the merge gate is trusted, at any
path. The guard script itself is trusted the same way.

**What the hook contract required** (hooks reference, code.claude.com/docs/en/hooks,
read 2026-11-14): exit 2 blocks and feeds stderr to the model; every other non-zero exit
and a timeout are NON-blocking (hence `|| exit 2` and the guard catching its own errors);
the command runs under `sh -c` with `CLAUDE_PROJECT_DIR` exported; subagents' tool calls
fire the same hooks. That a project `.claude/settings.local.json` hook fires in a headless
`claude -p` run was not in the docs excerpt and was checked live (below).

## Why a nested namespace

The mounts are made in the outer user namespace and the command runs in a nested one.
In a single namespace the command is root there and simply `umount`s the read-only bind
and writes (control probe below); a nested user namespace makes the kernel lock the
inherited mounts, so `umount` and `remount,rw` are refused.

## Verification

- Probe 2026-11-14, `/tmp/probe_c7_nonet.sh 1..3` (uncommitted; stand-in dirs for every
  write, a non-mutating `access(W_OK)` + mountinfo check on the real dirs, `stat` only on
  `/dev/ttyACM0`, TEST-NET 192.0.2.1 for the network): 3/3 trials, 14/14 checks each
  (EROFS on write, append and mkdir; umount/remount refused; ttyACM0 is 1:3; `/dev/serial`
  empty; only `lo`; host stand-ins untouched; no mount leaked). Control, single namespace:
  `umount` succeeded and the write landed.
- Probe 2026-11-14, `NIGHT_SHIFT_NONET_EXTRA_MASK='/dev/hidraw*:/dev/ttyTHS*'
  NIGHT_SHIFT_NONET_EXTRA_EMPTY=/dev/bus/usb tools/night_shift_nonet.sh bash -c '...'`:
  hidraw0-3 and ttyTHS0/3/4 all 1:3, `/dev/bus/usb` empty and EROFS, `umount` refused,
  `import mujoco, numpy, serial, usb` clean, a pytest subset 4 passed. Then made default.
- Live hook, 2026-11-14, a throwaway repo (guard + wrapper committed, `origin/main` set by
  `update-ref`, the settings file as `.claude/settings.local.json`), `claude -p --model
  haiku --permission-mode acceptEdits --output-format json --max-turns 3 "Run exactly
  this bash command ...: <C>"`, three calls: `FOO=1 python3 tools/probes/x.py` refused by
  the hook with the accepted-shape message, and listed under `permission_denials`;
  `echo HARMLESS-$((6*7))` printed `HARMLESS-42`; `tools/night_shift_nonet.sh python3
  tools/probes/x.py` printed `PROBEWORD`. Model reported: claude-haiku-5-5 each time. (Run
  before `timeout 20` was added to the hook command; the wired-hook tests cover the final one.)
- 2026-11-14, `python -m pytest tests/sim/test_night_shift_settings.py -q`: 144 passed in
  15.27 s (the guard table: 40 refused spellings, 11 accepted, 8 malformed inputs each
  with and without a guarded token, and the settings' own hook command run through `sh
  -c`, including a branch-edited guard that exits 0 and a project with no `origin/main`).
- 2026-11-14, every test file that reads a changed file (`grep -rln` for the four changed
  tools/protocol paths in `tests/` and `tools/`), through the new wrapper with Foxy
  sourced: `tools/night_shift_nonet.sh python -m pytest
  tests/sim/test_night_shift_{settings,gate,runner,candidate}.py
  tests/sim/test_sitting_debrief_protocol.py tests/sim/test_no_legacy_item_ids.py
  tests/ros/test_launch_{console,nodes}.py -q -n 4 --dist loadfile`: 473 passed in
  284.59 s. After that run the hook command gained `timeout 20` and the settings-edit
  denies; `python -m pytest tests/sim/test_night_shift_settings.py
  tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py -q` then: 160
  passed in 12.53 s (2026-11-14).
  The default gate is run on the stacked branch.

## Runner check (same day)

`tools/night_shift.sh` step 6 now ends `NO_SETTINGS` before claude starts when the installed settings do not wire the Bash guard hook, or when `origin/main` has no `tools/night_shift_bash_guard.py`. Without it a missing guard would refuse every Bash call for the whole night and look like a quiet run. Verification (2026-11-14): `python -m pytest tests/sim/test_night_shift_runner.py -q -n 4`: 71 passed, two of them new (guard missing on main; hook block removed). The protocol's probe paragraph also says the guard refuses redirections and a `git commit -m` naming a guarded word.
