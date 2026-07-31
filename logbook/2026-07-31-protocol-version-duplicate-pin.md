---
title: The 4→5 protocol bump went green on the canonical pin and left a duplicate literal stale — one constant, two pins, one procedure
type: bugfix
date: 2026-07-31
status: resolved
phase: "Developer workflow — test-suite hygiene"
files_changed:
  - tests/teensy_link/test_protocol_codec.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
# backfilled after the commit lands
commits:
  - PENDING
subsystem:
  - controller
  - ros
tags:
  - testing
  - docs
---

# The 4→5 protocol bump left a duplicate version pin stale

## Summary

`tests/teensy_link/test_protocol_codec.py::test_constants_match_firmware_spec`
failed with `assert 5 == 4` on `PROTOCOL_VERSION`, leaving the branch red. The
bump itself (`bf1e9a5`, "cone PROFILE slot (UDP v5, FW8)") was done **correctly** —
it updated the canonical pin, the generator spec, the generated Python module and
the committed C++ header. What it missed was a *second, undocumented literal* of
the same constant in a different test file.

The fix removes the duplicate rather than bumping it.

## Problem

Surfaced by the full-suite gate on 2026-07-31 as the only failure in 4297 tests.
It reproduced identically under plain serial `pytest`, so it was not a
parallelism artefact from the concurrent test-parallelisation work.

## Root cause

`PROTOCOL_VERSION` was pinned in **two** places:

1. `tests/firmware/test_udp_protocol_xlang.py::test_protocol_version_frozen` — the
   designed tripwire. It pins the generator spec (`assert spec_ver == 5`) with a
   full bump-history comment, cross-checks that against **both** the generated
   Python module and the committed C++ header, and its assertion message carries
   the procedure: *"update this pin, re-pin `test_wire_layout_frozen`, and reflash
   the whole fleet"*.
2. `tests/teensy_link/test_protocol_codec.py:14` — a bare `assert
   p.PROTOCOL_VERSION == 4` with its own parallel bump-history comment.

Pin 2 is fully redundant with pin 1: `p.PROTOCOL_VERSION` is imported from the
generated module, and pin 1 already asserts generated == spec == C++ header. But
pin 2 is not named anywhere in the bump procedure that pin 1's failure message
prints, so an author following that checklist to the letter still ships a red
suite.

## Fix

Deleted the redundant literal and replaced it with a comment naming the canonical
pin and stating why there is no literal here. The remaining assertions in that
test are kept — `MAGIC`, `HEADER_SIZE` and `CRC_SIZE` are covered by the wire
layout hash, but `PORT_STREAM`, `PORT_RPC`, `NUM_LEGS`, `NUM_AXES` and
`HEARTBEAT_HZ` are **not**, so that test still earns its place.

Also de-stamped a stale comment at `teensy_bridge_node.py:216` that still read
`(generated udp_protocol, PROTOCOL_VERSION 4)`.

## Verification

- Scoped: `pytest tests/teensy_link/ tests/firmware/test_udp_protocol_xlang.py -q`
  (run 2026-07-31): **231 passed in 3.97 s**.
- Full gate: see the Outcome below.

## Outcome

The branch is green again. Full gate (`./run_tests.sh`, run 2026-07-31, with only
this change in the working tree): **parallel 446 s (4287 passed, 3 xfailed) |
serial 32 s (7 passed) | total 478 s** — `RESULT: PASS`, zero failures. That is
the same 4297 collected as before, with the previously-failing assertion now
absent rather than merely passing.

The duplicate is gone, so the next incompatible bump has exactly one pin to
update and one procedure that names it.

## Discussion

### Bump it, or delete it?

Bumping the literal 4→5 is a thirty-second fix and it is what the failure message
invites you to do. It was rejected because it rebuilds the trap: the next
incompatible bump would follow the same documented checklist, update the same
canonical pin, and leave the same second literal stale. The failure would recur
with a different number.

The class here is **a single fact pinned in two places, with only one of them
named in the procedure that maintains it.** A duplicated pin is not twice the
protection — it is one real check plus one time bomb, because the redundant copy
carries no independent signal (it re-asserts a value already derived from the same
generated module) while carrying a full maintenance cost. Removing it makes the
invariant true by construction rather than by vigilance.

Worth being precise about what pin 2 *could* have caught that pin 1 cannot:
nothing. Both ultimately read `config/generated/udp_protocol.py`. Had pin 2
instead asserted something pin 1 does not cover — the wire ports, say — it would
have been worth keeping. It did not, which is what makes deletion the right call
rather than a judgement about which file "owns" the constant.

### The comment that rotted the same way

`teensy_bridge_node.py:216` said `PROTOCOL_VERSION 4` while describing
`DIAGNOSTIC.flags` bits that are unchanged at v5. Version-stamping a comment about
something that is not version-specific creates exactly the same maintenance debt
as the duplicate pin, minus the test that eventually catches it — nothing at all
fails when that comment goes stale. It is now written without a version stamp.

Noted while there and deliberately **not** fixed, because it is a different change
with a different risk profile: `_DIAG_FLAG_HB_STALE = 0x1` / `_DIAG_FLAG_HB_SEEN =
0x2` are hand-mirrored from the generated spec rather than imported from it. That
is the same duplication class again, one level down. It is stable today and the
fix (import the generated names) touches a live ROS node on a red-hot branch, so
it wants its own change and its own bench check.
