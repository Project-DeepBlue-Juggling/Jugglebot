---
title: "Night Shift reach widened: the firmware freeze fixed, mapkit and the map app opened, the app deployed at the end of each night with automatic rollback"
type: feature
date: 2026-11-07
status: complete
phase: night-shift
related_plan:
  - night-shift.md
files_changed:
  - tools/night_shift_gate.py
  - tools/night_shift.sh
  - tools/night_shift_settings.json
  - tools/night_shift_deploy.py
  - tools/night_shift_nonet.sh
  - tools/night_shift_app_suite.sh
  - .claude/commands/night-shift.md
  - .claude/commands/sitting-debrief.md
  - mapkit/sitting.py
  - CLAUDE.md
  - docs/night-shift/README.md
  - docs/runbook.md
  - plans/active/night-shift.md
  - tests/sim/test_night_shift_gate.py
  - tests/sim/test_night_shift_runner.py
  - tests/sim/test_night_shift_deploy.py
  - tests/sim/test_night_shift_settings.py
  - tests/firmware/test_wave_native.py
subsystem:
  - night-shift
tags:
  - night-shift
  - merge-gate
  - map-app
---

# Night Shift reach widened

**What.** The owner asked whether the report's ten `out of reach` Items should be opened. Eight open, JB-86 builds but holds, JB-134 stays closed. Six changes make that safe, plus a deploy at the end of each night (plan § 7).

**The bug found on the way.** The gate forbade `Teensy_code_canbridge/*`, but the firmware lives at `ros_ws/src/jugglebot/Teensy_code_canbridge/`: 0 of 71 files matched, so condition (f) never froze firmware. `test_forbidden_paths_pinned` pinned the same wrong string, and the (f) test built a fixture at the wrong path too, so both passed. No damage: none of the ten firmware-tree commits since 2026-11-01 is in a report's Merged table, and the seven held branches touch no firmware. Hardware ratings and the protocol's dispatch had kept firmware out by themselves. The deny list also missed `platformio run -t upload` ("platformio" does not contain "pio") and bare `pio test`, whose default environment is the ESP32.

The class fix: `test_every_forbidden_path_matches_a_tracked_file` and `test_every_firmware_tree_is_frozen` check the real tree, and the app's list checks itself at the base on every gate run, since one test cannot pin two repos.

## Discussion

**Why the runner deploys, not Claude.** The owner asked for Claude to deploy and roll back. The orchestrator is denied `systemctl` and `~/map-live` on purpose, and it can die mid-night with commits merged (the 2026-11-02 credit-out). A script run after it exits covers those nights and behaves the same every time; nothing in a rollback needs judgement.

**Why deploy first, then blame.** `pull.sh` puts a `mapkit` change live within about 30 s, so by the deploy step the old app is already running on the new `mapkit`. An Item spanning both repos can need both halves, so judging `mapkit` against the old app could revert good work. The tool checks new app with new `mapkit` first, and only on failure goes back to separate them. `test_a_pair_that_works_together_is_kept` pins this.

**Why a revert needs proof (the audit reshaped this).** The first version reverted on any failed check, and the audit's scenarios showed how wrong that goes: a `degraded` health from one bad map file reverting a good `mapkit`, an owner-stopped service doing the same, a broken older commit getting tonight's good one blamed, a build's long output in a Progress note turning `main` red, and a `git revert` that conflicts on `logbook/INDEX.md` on every real night. So: going back to the last good commit is safe and needs no proof, and a revert changes `main` for everyone and does. Proof is: healthy before the night, data clone caught up, rollback worked, every commit marked `Night-Shift: <date>`, and for the app, old is the night's start and `main` has not moved. Otherwise the app stays on the last good commit and a Session decides, which costs a morning, while a wrong revert costs real work. Reverts are new commits, because `main` is never rewritten. The cost accepted: rare real breakages wait for a Session instead of cleaning up by themselves.

**Why the mapkit undo restores files instead of `git revert`.** Every real night's report commit touches `logbook/INDEX.md` after the fix did, so a revert conflicts every time and the fallback would be the normal path. Restoring the files those commits changed (code and tests, not logbook, map or docs) to the night's start is deterministic. A file a later commit also changed is left alone and named, since that work is not ours to lose.

**Why `mapkit` opened but the gate tests did not.** The freeze existed so a branch could not edit code that judges it. Running the gate from an extract of the base closes that directly, which the freeze only approximated: until today the night had to add `PYTHONPATH=.` because the protocol's command did not import, which loaded the working tree's `mapkit`. The byte comparison catches a wrong invocation; it is not a barrier, because imported code could say anything (the audit proved it with a `print("MERGE")` in a branch's `ids.py`), so the extract is what protects and `/sitting-debrief` now uses one too. The tests stay frozen, and so do the conftests and root pytest configs that decide whether they run, because they are what a `mapkit` change has to pass.

**Why a network namespace instead of more deny rules.** A deny rule reads spelling, and beats every allow, so `*teensy_link*` blocked greps and the bridge's own tests while `./run_tests.sh --full` ran those same tests every night with no check at all. The namespace makes the board unreachable whatever a new test does. It is not enforced on every command: the protocol requires it and the deny list cannot demand it. Narrowing the rules accepts that a heredoc `python - <<E` importing `teensy_link` is no longer refused; with the robot unpowered, the exposure is a client opening the idle bridge link.

**Why `jugglebot_interfaces` stays frozen.** Considered: allow `colcon build` in the clone. `tests/ros/conftest.py` mocks that package unconditionally, so the suite still would not test a node against its interface. A day Session runs the real build instead. Both Items it affects are low priority.

**JB-86 was protected by accident.** It was rated `hardware: none` and only "out of reach" kept it from merging. `holds_ball=False` removes the cup-contact floor from a REST entirely, and live schedules send `False` (`schedule.py:2688`), so wiring it changes flown motion. rosidl also defaults a new bool to `false`, which would drop the floor for any sender that omits it. Re-rated `robot`, with the default noted on the Item.

**An open question for the owner.** `/sitting-debrief` re-gates a branch with `--waive c`, and the gate requires the branch to contain the base, so once `main` moves at all it says `[pre] does not contain base`. That is older than this work and fails closed, and the audit surfaced it; changing it is a decision about what a Sitting pass means, so it is not changed here.

**The namespace found a network dependency in a new test.** The full gate under `tools/night_shift_nonet.sh` passed everything except `test_wave_native`, which hit its 600 s limit at 60 s per case. A fresh build directory resolves Unity from the PlatformIO registry; that works while the registry cache is warm, so it passed alone and had passed earlier. With a cold cache and no network it hangs, reproduced 2026-11-07. Unity now lives in one stable directory, so only the first run ever needs the network. Every other test in the suite passed with no network.

**What the deploy check cannot see.** A feature that works badly. A failed build, a server crash, API or page errors, missing assets and a white screen are all caught; the headless render tells a mounted page from an empty root (a throwing page was probed, 2026-11-07). JB-151-style behaviour still needs eyes.

## Fix

`tools/night_shift_gate.py`: real paths, `config/` whole, invariants, interfaces; `mapkit/` off the list; base-copy precondition; (h); `--app-repo`. `tools/night_shift.sh`: app clone preparation, `--app-clone`, deploy step, deploy line. New: `tools/night_shift_deploy.py`, `tools/night_shift_nonet.sh`, `tools/night_shift_app_suite.sh`. Deny list narrowed and extended. Protocol: nonet everywhere, base extract in 2g, new 2m. Rebuild line is two-package in the Runbook, the debrief and `CLAUDE.md`.

## Verification

- Full gate, in the namespace, on bd73bdbb (2026-11-07, `tools/night_shift_nonet.sh ./run_tests.sh --full`): parallel 8972 passed, 59 skipped in 530.05 s; serial 10 passed in 38.12 s; `RESULT: PASS`. The run before it, same command, failed 2: `test_mapkit_no_ros` (the Runbook line named a ROS package) and `test_wave_native` (the Discussion's network dependency); both fixed.
- Cold, no network (2026-11-07, `PLATFORMIO_CACHE_DIR=<empty> tools/night_shift_nonet.sh python -m pytest tests/firmware/test_wave_native.py`): 1 passed in 19.44 s. Before the fix, `pio test -e native -f test_word_filter` the same way: ERRORED after 60.66 s.
- Deploy under a systemd user unit, against the real app, with no code change (2026-11-07, `systemd-run --user --wait` of the tool, then `Live.deploy` and `Live.rollback` directly; that rehearsal ran the version before the audit reshaped the decision, and the check, `deploy` and `rollback` paths it used are unchanged in substance): NOOP correct, real build and restart 27 s, restoring the last good build 3 s, healthy, tree clean.
- App suite on the night clone (2026-11-07, its CI steps run one by one): server pytest 78 s, lint 7 s, typecheck 15 s, vitest 37 s, build 23 s, name check 1 s, contract check about 14 s; all rc 0. About 3 minutes per app Item.
- One independent audit (`audit-reporter`, Opus, 2026-11-07): no false-MERGE path through the real 2g invocation; behaviour findings in the deploy, the debrief invocation and the protocol, applied except for the one under "An open question" and one small gap left as is: (h) does not compare the app log's `HEAD=` with the app's `origin/main`, so a suite run with the app clone on a night branch would count. A stated limit too: nothing can make a unit use the wrapper.
- Deny list against the real matcher (2026-11-07, `claude -p --model haiku` in the night clone with the installed settings, same permission flags as the runner): all 10 forbidden commands in `permission_denials` (`platformio run`, `pio test`, `python -m platformio`, `python -m teensy_link`, `python -c` naming `teensy_link`, `python tools/night_shift_deploy.py`, `git -C` with `--force` and with `HEAD:main`, `systemctl`, `ls ~/map-live`); all 4 that must work ran (two greps, `tools/night_shift_nonet.sh true`, `git -C ... log`).
- Runner end to end with a stand-in `claude` (2026-11-07, `NIGHT_SHIFT_CLAUDE=<fake> tools/night_shift.sh --dry-run`, the live checkout's runner at 31136680): `DRY_OK`, settings installed, `--app-clone` in the prompt. The unit's `TimeoutStartSec` is 8h, installed and reloaded.

Not verified: a real night. A real failed deploy has only run against the test rig, not the live app.
