---
title: Surface the ODrive firmware versions the hand-sensor endpoint id is pinned against
type: feature
date: 2026-07-29
status: in-progress
phase: "Hand ball-present sensor — Phase 0 (fw-version surfacing)"
related_plan: "hand-ball-sensor.md"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_version.py
commits: []                # backfill the SHA of the Phase 0 commit
subsystem:
  - ros
tags:
  - testing
---

# Surface the ODrive firmware versions the hand-sensor endpoint id is pinned against

## Summary

The bridge has always decoded every axis's `Get_Version` reply and then thrown
the numbers away: `_version_check_poll` fed them to `MotorStateTracker.
record_version` / `validate_group` and neither the PASS log line nor
`/link_status` ever showed a version (only `platform_fw_version`, which is the
*Platform Teensy*, not the drives). Phase 0 makes them observable — a
`_odrive_fw_versions_str()` rendering appended to **both** the PASS and FAIL
firmware-check log lines, and a new `odrive_fw_versions` KeyValue on
`/link_status` — **including `Get_Version`'s fourth byte (`fw_unreleased`),
previously discarded as `_unrel`**, which is the only wire evidence that could
carry the `-1` of `0.6.11-1`. No behaviour changes: nothing new is asserted,
refused, or faulted on. This is instrumentation that the later sensor phases
read, and its live confirmation is Phase 7 step 1.

## Motivation

The hand ball-present sensor is polled through an RxSdo function-invoke of
`get_gpio_states`, whose **endpoint id is firmware-build-specific**. The plan
pins **726** against the hand ODrive **Pro** at fw `0.6.11` / `0.6.11-1`. Two
facts make "what firmware is actually running" load-bearing rather than
curiosity:

1. **The existing "firmware check PASSED" line is not evidence of a version.**
   `validate_group` checks firmware only for *internal consistency within a
   hardware group* — seven axes agreeing on the wrong version passes. So the
   one log line an operator would reach for cannot answer "are the drives on
   0.6.11?".
2. **The wrong id fails silently.** Per the plan's blocking finding, the id the
   repo carries today (700, the S1 value) *exists* on Pro 0.6.11 as
   `encoder_estimator1.status` — a read-only byte that answers with a
   well-formed TxSdo reply and no timeout. A commissioning check of the form
   "no reply ⇒ wrong endpoint" is therefore worthless, and the fallback is to
   confirm the firmware the id was chosen for.

Approved-decisions **row 6** of the plan states the expectation — no `-1` is
expected to appear in CAN `Get_Version` frames, the 0.6.11 and 0.6.11-1
endpoint trees are identical (CRC 55416), so 726 stands either way — and
requires Phase 0 to surface the fourth byte so that expectation is *confirmed
empirically at Phase 7 step 1* rather than assumed.

## Design

**The tracker is untouched.** `MotorStateTracker` keeps storing the
`(major, minor, rev)` triple it validates on; the fourth byte lands in a
node-local `self._fw_unreleased` dict written inside the same decode loop.

**One renderer, two consumers.** `_odrive_fw_versions_str()` walks
`odrive.JUGGLEBOT_AXES` in order and emits `axis:major.minor.rev-unreleased`
per axis, joined by spaces. Both the PASS and the FAIL firmware-check log lines
append `(fw <rendering>)`; `_publish_link_status` emits the same string as an
`odrive_fw_versions` KeyValue placed immediately after — and deliberately
distinct from — `platform_fw_version`.

**Two rendering decisions, both about not lying:**

- **Absent axes render as `axis:?`, never omitted.** A partial bench rig is
  legitimate (the Teensy sweep only queries axes that have heartbeated), and a
  silently short list reads as a formatting gap rather than a rig fact.
- **The fourth byte is rendered even when it is zero.** `6:0.6.11-0` says *the
  drive reported no suffix*; a suppressed-when-zero rendering would make that
  indistinguishable from *the bridge does not surface the byte* — which is
  exactly the ambiguity row 6 exists to remove.

## Discussion

### A node-local dict, not a wider tracker record

The obvious alternative was widening `MotorStateTracker.record_version` to take
a 4-tuple, so the byte lives beside the triple it came from. Rejected: the
tracker has callers outside this node, and `validate_group` — the only thing
that consumes what the tracker stores — has no use for the byte whatsoever. A
node-local dict is the smallest change that gets the evidence out, and it keeps
the tracker's job (**validation**) separate from this phase's job
(**rendering evidence**). The tracker's API is bit-unchanged; nothing outside
`teensy_bridge_node` learns a new field.

### How the two Phase 0 tests split the plan's "Done when"

The plan's "Done when" asks one unit test, driven by a mocked tracker, to
cover the all-seven-axes and partial-rig cases and to assert both the log
string and the KeyValue content. It is split across two tests because **a
partial rig and a PASS/FAIL log line are mutually exclusive by construction**:
`_version_check_poll` returns at the `all_jugglebot_versions_received()` gate,
*before* either log call, precisely because the sweep is incomplete. A
partial-rig test that asserted a log string would either be testing a state
the code cannot reach or quietly relaxing the gate.

So `test_fw_versions_render_absent_axes_on_partial_rig` asserts the shared
rendering helper directly, plus the `/link_status` row — the surface that
actually exists in that state. Nothing is lost: the all-seven-axes test covers
the log line, and the helper is the single source of both strings.

A reviewer proposed closing the gap the other way — add a throttled "sweep
incomplete" log line so the `?` rendering becomes log-observable on a partial
rig. Rejected because the information is already available where it is
needed: `/link_status` carries the row at 10 Hz on the bench today, and the
Phase 6 runbook will point operators at it. Adding a new periodic log line to
a boot path to make a test's shape prettier is not a trade worth making.

### The write-order rule, and why the renderer never defaults to `0`

`self._fw_unreleased[axis] = unrel` is written **before**
`self._versions.record_version(...)`, and the renderer's fw-is-known branch
reads the byte with `.get(axis)` and renders `-?` when it is missing — never
a defaulted `-0`.

The value that must never appear is a fabricated `-0`: on any future path
that populated the tracker without the byte, a `.get(axis, 0)` default would
read as positive evidence ("the drive reports no suffix") for a suffix nobody
had actually measured — a *plausible-but-wrong* value for exactly the byte
row 6 exists to confirm. `-?` is as honest as absence can be rendered, and it
stays distinguishable from a measured zero.

An earlier draft indexed `self._fw_unreleased[axis]` directly, on the theory
that a future desync should fail as a "loud `KeyError`". The pre-commit audit
showed that claim was false at both call sites: inside `_publish_link_status`
the exception would be swallowed by the publisher's blanket `except` and the
whole `/link_status` topic would go dark (the silent-degradation signature the
anomaly-fixes runbook warns about for this node), and at the PASS/FAIL log
call site it would propagate out of the 1 Hz timer and take the bridge node
down. A rendering helper must not carry a node-death path; the total renderer
keeps the never-fabricate property without one.

The write order still matters: it makes `-?` unreachable today (the tracker
never knows an axis's fw without the byte having been stored first), so a
`-?` against a known fw triple in a real log is itself evidence of a new
writer that skipped the byte. No concurrent reader exists — the version-poll
timer (`teensy_bridge_node.py:857`) and the `_publish_link_status` timer
(`:849`) share the node-default `MutuallyExclusiveCallbackGroup`, so they
cannot run at the same time, and the stale comment claiming the tracker was
touched on "the version-poll timer thread" was corrected to say so
truthfully. The ordering is written anyway so the invariant survives a future
callback-group change.

## Implementation

- **`teensy_bridge_node.py`** — the decode loop keeps the fourth byte
  (`_unrel` → `unrel`) into `self._fw_unreleased[axis]`, written before
  `record_version`; new `_odrive_fw_versions_str()`; `(fw <rendering>)`
  appended to the PASS and FAIL log lines; `odrive_fw_versions` KeyValue added
  to `_publish_link_status` after `platform_fw_version`.
- **`tests/ros/test_teensy_bridge_node_version.py`** — `_raw_version` /
  `_wire_versions` gained an `unrel` byte (default 0, so every pre-existing
  caller is unchanged). New `test_fw_versions_render_on_pass_log_and_link_status`
  drives all seven axes with `unrel=1` on the hand axis and asserts the PASS log
  line and the KeyValue against a **literal oracle string**. New
  `test_fw_versions_render_absent_axes_on_partial_rig` populates three axes
  directly into the tracker (the plan's "mocked tracker") and asserts the
  `3:? 4:? 5:? 6:?` rendering plus the KeyValue. The pre-existing
  `test_version_mismatch_forces_fault` gained a mocked logger and one assertion,
  so the FAIL line's rendering is pinned too.

**Process.** Implemented by one agent, then three parallel read-only reviewers
on distinct lenses (correctness / plan-conformance / minimalism). **Zero
blocking findings**; six review fixes applied at finalize: the write-order swap
plus direct indexing (which killed the only path that could fabricate `-0`), the
truthful concurrency comment, the partial-rig test simplified to a direct
tracker populate, a vacuous "the two rows are distinct" assertion deleted, and a
test helper that re-derived the expected format from the code replaced with
literal oracles.

## Verification

**Full suite** — `python -m pytest tests/ -q`, run 2026-07-29:
**4256 passed, 3 xfailed in 1421.88 s (0:23:41)**. That run predates the
pre-commit audit's renderer fix (direct indexing → total `.get`/`-?`); the
delta was re-checked scoped — `python -m pytest
tests/ros/test_teensy_bridge_node_version.py tests/sim/test_logbook_search.py
-q`, run 2026-07-29: **32 passed in 2.32 s**. Per the operator's gating
direction of 2026-07-29, later phases of this plan run scoped checks per
commit and the full suite once at the end of the plan.

**Not verified**: nothing here has run on powered hardware. Whether the hand
drive's `Get_Version` reply actually carries a non-zero fourth byte — the whole
point of surfacing it — is unknown until Phase 7 step 1 reads the line on the
bench. A partial bench rig showing `?` for un-heartbeated axes is expected
there and is not a defect.

## Deployment

**`colcon build --packages-select jugglebot` + `source install/setup.bash` +
RELAUNCH `jugglebot_launch.py`.** The launch runs the *installed* copy, so an
edit to `ros_ws/src` is invisible without the rebuild. No
`jugglebot_interfaces` rebuild (no message or action changed — `/link_status`
gains a KeyValue *row*, not a field) and no firmware flash.

## Open questions

1. **Does the hand Pro report a non-zero `fw_unreleased`?** Row 6 expects `-0`
   on the wire even for a `0.6.11-1` build. Phase 7 step 1 settles it; either
   answer leaves endpoint id 726 standing (identical endpoint trees), so this
   is confirmation, not a fork.
2. **The `?` rendering is only observable via `/link_status` on a partial
   rig**, not in a log line — see Discussion. If bench experience shows
   operators reaching for the log first, the throttled "sweep incomplete" line
   is the deliberate follow-up.

## Related

- `plans/active/hand-ball-sensor.md` — Phase 0 and Approved-decisions row 6.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/version_check.cpp` — the
  once-per-Teensy-boot sweep whose cached replies this decodes.
- `logbook/2026-07-28-anomaly-fixes-validation-sitting.md` — the sitting that
  approved the sensor plan.
