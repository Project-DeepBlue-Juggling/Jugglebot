---
description: Build one ordered, step-by-step Sitting sheet for a rig from every open Item's ## Sitting section, check it against the Items, and publish it to the map app's Next Sitting page. Invoke just before a Sitting with /sitting-sheet <robot|bench-leg|bench-hand> [<date>].
---

# Sitting sheet

The operator runs this just before powering up. It produces one page, stepped through from top to
bottom by someone who has **not read ahead**. Anything a step needs must come before it, and
each step must be doable from its own card.

Arguments: `$ARGUMENTS` = `<rig>` (`robot` includes Ball Butler) and an optional `<date>`
(default today, local).

## 1. Read

1. `git fetch && git status -sb`. A dirty tree you didn't make, or `origin/main` ahead: stop and
   say what you see. Work in a scratch worktree of `origin/main` and push `HEAD:main`.
2. `python -m mapkit sheet scope <rig>` lists the Items in scope with their `do:` steps numbered,
   and `python -m mapkit sheet next-name <date> <rig>` gives the file name. The sheet is
   `map/sheets/<name>.yaml`. A sheet that has results is never overwritten; `next-name` moves on.

## 2. Check what the Items can't know

An Item's `## Sitting` section was written when it was filed. Before copying from it, compare:

- **Firmware**: `EXPECTED_BRIDGE_FW_VERSION` in `teensy_link/rpc_args.py` against any version a row
  names. The sheet states the real one as `can-bridge v<N>`, and says so when the row disagrees; the
  check fails on any other `can-bridge v<N>`, and the app re-runs it on every load, so a flash after
  the sheet was written shows as a stale-sheet banner.
- **Rebuild**: `git log` on `ros_ws/` against the install tree's time, and the latest
  `docs/night-shift/` report. A `branch:` means the candidate branch
  (`tools/night_shift_candidate.sh --date <date> --push`), then a checkout and a build.
- **Field values**: GUI values (Pattern, Sep., Apex, Dwell, Throws) as the Item gives them, and
  checked against the latest launch log's dispatch line. Never typed from memory.
- **Physical changes** since the last Sitting that make a row's stop condition matter (the last
  debrief, open Items on the same mechanism).

## 3. Order the steps

**The prerequisite chain, in this order, in every launch: Boot > Home > Level > Activate > goal.**
The operator does not read ahead, so no step may need a state a later step creates.

- **Launch and Homing** come first; a launch step ends `launched` only once Homing has finished.
- **Level before anything that plans or flies a Pattern.** The skill executor refuses every
  dispatch with `REJECTED_NOT_LEVELLED` until this launch has a levelling offset. Put the Level
  step straight after the launch's own checks, before rehearsals, parameters and Activate.
- **Activate (and TRAJECTORY mode) before any Juggle goal.** `python -m mapkit sheet check` fails a
  step that sends `send_goal /jugglebot/juggle` unless it starts from `launched, level, armed`.
- **A dress rehearsal is `/skills/nominate`, never a goal.** One call per Pattern and setting:
  `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: '<pattern>,apex_m=..,separation_mm=..,dwell_s=..[,apex_m_ball2=..][,columns_angle_deg=..]'}"`.
  It plans the Pattern at the live Sitting limits, lists every refusal at once, and cannot move
  anything; the extra keys replace `ros2 param set` for that call. A Juggle goal is NOT a
  rehearsal: with the Cup empty and Ball Butler off it is still accepted once the Platform is
  Active, the Platform does its pre-level move and the Hand homes (2026-11-10: a sheet called
  three such goals rehearsals and put them before Level). If an Item's row rehearses with a goal,
  fix the Item's `## Sitting` section in the same commit.
- **One rehearsal step per launch**, all its nominate calls together, right after Level. Do not
  give each Item its own rehearsal step.

1. **Launches first.** Group by what forces a new launch: launch arguments, the branch, a drive
   setting changed launch-down. A new launch is its own step with `new_launch: true`.
2. **Before arming, then armed.** The rehearsal step and anything marked "do NOT arm" come after
   Level and before Activate. The launch's `auto_arm` arms on entering Active.
3. **Highest priority earliest** within what the launches allow. A row with `first:` opens the rig.
   Dependencies between rows win ("right after IDEA-10, same settings").
4. **Isolate what would contaminate others**: a `plant_id` keeps a row's Throws out of another
   row's Memory, so put the plant back before the next row, even when the Item forgets to say so.
   A trial on one drive or a diagnostic launch flag goes last or gets its own launch.
5. **Merge only what is the same action.** The same goal with the same fields serving two Items is
   one step naming both. Never merge steps that merely look alike.
6. **Every parameter set gets its restore.** Teardown is a step: IDLE, Ctrl-C, Cup empty.

## 4. Write each step

`map/sheets/<name>.yaml`:

```yaml
schema: 1
date: "2026-11-10"
rig: robot
omitted:                 # an Item, or some of its do steps, deliberately left off
  - {item: BUG-1, do: [2], reason: why}   # leave out do: to omit the whole Item
steps:
  - n: 1                 # 1, 2, 3... no gaps
    kind: run            # setup | run | restore
    title: Short imperative title
    new_launch: true     # only on a step that starts a launch; its actions must hold the ros2 launch command
    from: [launched, level]   # state words: launched level tilt armed bb-fed bridge-cycled
    to: [launched, level]     # must equal the next step's from; the last step must not be armed
    actions:
      - {where: Terminal 2, command: ros2 param set /skill_node x 1.0}
      - {where: GUI, text: "Juggle panel: Columns, Sep. 125, Apex 0.95, Dwell 0.27, Throws 21."}
    see: What the operator should see.
    stop_if: What makes them stop, and what to do next if it matters.
    record: What to write down (omit when nothing).
    serves:
      - {item: IDEA-10, do: [1, 2], pass: true}
```

- `where` names one place: `Terminal 1` (the launch), `Terminal 2` (parameters, goals), `GUI`,
  `Hands`. The first use of Terminal 2 shows its full `source` line, and any working directory a
  later command relies on is established by an earlier step.
- `serves[].do` cites the Item's `do:` numbers, so a step that serves no Item's `do:` has
  `serves: []` (a build, a Level, a teardown). A `run` step serves at least one Item.
- `pass: true` goes on exactly one `run` step per Item: the last flown step that decides its
  criterion. It gets one verdict per such Item in the app, and the app shows the Item's own `pass:`.
  Its `from` must include the Item's `setup:` words, unless that serve carries `setup_waived: <why>`
  (an Item judging what a relaunch restores reads it before the new launch re-establishes the state). Put a "type pending" hint in `record:` when
  the Item's `pass:` is computed at the debrief.
- Never invent a command. Quote the Item's, with `<date>` filled in. `reorder: <why>` on a `serves`
  entry is for an Item whose `do:` states a precondition after the steps it governs.
- `optional` isn't written: the app marks a step optional when every Item it serves is priority low.

## 5. Check, read, publish

1. `python -m mapkit sheet check map/sheets/<name>.yaml` until it prints `sheet check: OK`.
2. One fresh `model: sonnet` agent reads the sheet cold, given only the sheet and the
   `sheet scope` output: walk it tracking parameters, ball, Ball Butler, launch and checkout, and
   name any step needing something not established before it, any value changed from its Item, and
   anything misreadable. Fix what it finds and re-run the check.
3. Commit only `map/sheets/<name>.yaml` as `map: Sitting sheet <name>`, no trailer, push `HEAD:main`,
   then in one short message give the operator: the page
   (`http://192.168.20.34:8765`, Next Sitting), the step count and launches, the Items covered,
   anything `omitted` with its reason, each place a row disagreed with reality, and how to close out
   (`/sitting-debrief <date>`).

Nothing here powers hardware. The operator runs every motion command.
