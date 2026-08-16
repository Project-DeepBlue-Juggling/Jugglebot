---
title: Clapboard Phase 4 — one shared wire fixture, so the two repos cannot drift apart quietly
type: feature
date: 2026-08-16
status: resolved
phase: "clapboard-can3-integration Phase 4"
related_plan: clapboard-can3-integration.md
files_changed:
  - tests/fixtures/clapboard_wire_vectors.json
  - tests/fixtures/clapboard_wire_vectors.json.sha256
  - tests/fixtures/README.md
  - tests/ros/test_clapboard_wire_vectors.py
  - tests/firmware/test_clap_wire_dlc_lint.py
  - tools/probes/clapboard_wire_vectors_gen.py
  - tools/probes/README.md
  - plans/active/clapboard-can3-integration.md
subsystem:
  - testing
  - can-bridge
  - ros2
tags:
  - clapboard
  - protocol
  - cross-repo
---

# Clapboard Phase 4 — the shared wire fixture

## Summary

Jugglebot and Electronic-Clapboard each had their own clapboard test file,
asserting their own encoders against their own reading of `protocol.md` §8, with
**no shared artefact**. Both suites can stay green while drifting apart, and the
first evidence would be a bench sitting where the panel silently ignores every
frame.

`tests/fixtures/clapboard_wire_vectors.json` is now that artefact: CRC goldens
(`"123456789"` → `0x29B1`, empty → `0xFFFF`), six field-set CRC vectors,
byte-exact layouts for `CLAP_FIELD`/`CLAP_COMMIT`/`CLAP_LINK`/`CLAP_ACK`/
`CLAP_HEARTBEAT`/`CLAP_FIRE_EVENT`/`0x7DD`, and three whole-transaction goldens
(41-frame full slate, 6-frame single-field patch, 1-frame repaint). Every vector
whose name starts `peer/` is carried across verbatim from that repo's own Unity
assertions — field 5 / seq 3 → `d[0] == 0x35`; template 3 / mask `0b00001011` /
flags `0x01` / txn 200 / crc `0xBEEF` → `d[4]=0xEF, d[5]=0xBE` — so the two
suites agree by construction rather than by coincidence.

This repo owns it; the clapboard repo vendors a byte-identical copy against the
published SHA-256 (owner decision, plan §6 — this repo has the stricter gate).
The clapboard repo was **not** edited; what it must do is written into the JSON's
own `_README`, because the JSON is what it receives.

`tests/ros/test_clapboard_wire_vectors.py` asserts the live
`jugglebot.clapboard_slate` and `jugglebot.can.clapboard` reproduce every vector,
and `tests/firmware/test_clap_wire_dlc_lint.py` covers the two bridge-side
emitters. The DLC-8 rule — the one the clapboard enforces in code and
`protocol.md` states nowhere — is now pinned in both directions: everything this
repo emits is 8 bytes, and every decoder it owns refuses anything else.

## Discussion

**The fixture needed a committed recipe, and finding that out was the self-audit
of this phase.** The first cut hand-maintained the JSON. `tools/probes/README.md`
already forbids that shape generally — *"a fixture nobody can re-baseline is only
half a live reference"* — and here it bites unusually hard: nobody hand-computes
a CRC-16 over eight 32-byte NUL-padded buffers, or 41 frames of chunked text. So
the only practical way a future session could have changed a vector was *"run
`clapboard_slate.build_transaction` and paste the output"* — which silently
converts the artefact from a **second opinion** into a snapshot of our own
encoder agreeing with itself, i.e. destroys the single property that makes it
worth having. `tools/probes/clapboard_wire_vectors_gen.py` closes that: it
imports nothing from this repo, transcribes `protocol.md` §8 and the peer's
`can_frames.h`, and refuses to emit unless it first reproduces that repo's own
literal goldens. A test compares the committed JSON against a fresh generation,
so a hand-edit fails the gate.

**Why a vendored file with a published checksum, and not literals in both
suites.** A literal in two codebases is two literals; nothing connects them. The
checksum makes "these repos disagree" a test failure on *both* sides. Its second
job matters more than the first: editing the JSON without re-publishing the
digest fails here, and that failure is the only mechanism that reliably converts
"I changed a shared wire vector" into "go tell the other repo" — an unavoidable
prompt rather than a good intention.

**A peer-tree cross-check was considered and rejected.** A test that read
`~/Desktop/Github/Electronic-Clapboard/src/can_frames.h` directly would catch
drift the moment it landed over there, which is tempting. It was not added: it
makes this repo's gate depend on a sibling checkout's *working tree*, so an
unrelated edit in that repo reddens a commit in this one, on a box where the
operator cannot fix it from here — and that checkout is not present on every box
the suite runs on. The vendored-copy checksum is the sanctioned direction for the
same signal, and it puts the obligation on the side making the change.

**`0x7DD` is in scope even though it is not a clapboard-block id.** The bridge
emits it on the same segment at 100 Hz, and the peer's `decode_time_sync`
requires DLC 8 for it exactly as for the clapboard block. The consequence of
getting it wrong is not a degraded timestamp — the clapboard emits **no** fire
event at all until it has a wall clock, so a short DLC there costs the device its
entire reason for existing, quietly. Excluding it because of its id would have
been filing by allocation rather than by failure mode.

**A text lint beside the compiled one, not instead of it.**
`native/test_clap_link.cpp` already drives the real `clap_link.cpp` and asserts
`len == 8`; that is the stronger check and it stays. But it needs `g++`, and
`time_sync_master.cpp` compiles in no native binary at all. The new lint is
compiler-free and covers both emitters everywhere the suite runs — the
`test_cone_rx_role_lint.py` precedent.

## Fix

- `tools/probes/clapboard_wire_vectors_gen.py` — the recipe. `--emit-fixture`
  required to write; a bare run reports drift. Pure stdlib, imports nothing from
  this repo, anchored on the peer's own literal goldens.
- `tests/fixtures/clapboard_wire_vectors.json` (+ `.sha256`, + `README.md`) — the
  shared artefact, its published digest, and the directory's ownership rule.
- `tests/ros/test_clapboard_wire_vectors.py` — 105 checks: the fixture against
  the live encoders/decoders, the DLC-8 invariant both directions (emission swept
  across the shapes where a length bug hides; rejection over every decoder found
  by introspection, so a decoder added later inherits it), the commit-last rule,
  and anti-rot guards that fail on a fixture section, frame kind, constant, id or
  repo path with no consumer.
- `tests/firmware/test_clap_wire_dlc_lint.py` — the `CLAP_LINK` and `0x7DD`
  emitters, compiler-free.

Four rules that `protocol.md` §8 never states but code enforces are recorded in
the fixture's `unstated_rules` block, each with a vector that fails if either
side forgets it: strict DLC-8 on receive; `CLAP_FIRE_EVENT` carrying the **low 48
bits** of a 51-bit Unix-µs clock; all five chunks for every present field; and an
empty present-field set CRCing to the `0xFFFF` init value.

## Verification

Guards mutation-tested before the gate, each reverted immediately (2026-08-16):
swapping the `CLAP_FIELD` nibble packing, trimming instead of NUL-padding the CRC
input, byte-swapping the commit CRC, editing one fixture byte, and setting the
`CLAP_LINK` / `0x7DD` DLC to 1 / 4 each failed the intended test and only that
class of test.

`PROTOCOL_VERSION` verified still **5**, and `test_protocol_version_frozen`
verified still asserting 5 (T-R3) — neither touched by this phase.

`./run_tests.sh --full`, run 2026-08-16: **PASS** — parallel phase 5960 passed,
3 xfailed in 489.82 s; serial phase 9 passed in 41.46 s; total 537 s.

No firmware, no flash, no codegen, **no `colcon build`** — the phase is `tests/`
plus one `tools/probes/` entry.
