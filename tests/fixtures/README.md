# `tests/fixtures/` — shared, cross-repository test data

Committed data files that are **contracts with something outside this repo**, and
the checksums that let the other side prove it holds the same bytes.

This is deliberately narrow. Fixture data used by exactly one test belongs beside
that test (or in `tmp_path`); goldens emitted by a native binary live next to it
in `tests/firmware/native/`. What lands here is data that a **second codebase**
reads.

| File | Contract | Peer |
|---|---|---|
| `clapboard_wire_vectors.json` | Electronic Clapboard CAN wire vectors — CRC goldens, byte-exact frame layouts, whole-transaction goldens, and the four rules `protocol.md` §8 never states | `~/Desktop/Github/Electronic-Clapboard` |
| `clapboard_wire_vectors.json.sha256` | The published digest the peer vendors against | — |

**The JSON is generated, not hand-written.** Its recipe is
`tools/probes/clapboard_wire_vectors_gen.py`, which transcribes `protocol.md` §8
and the peer's `src/can_frames.h` and imports nothing from this repo — so the
tests that check our encoders against it are a second opinion, not a snapshot of
our own code agreeing with itself. `test_fixture_is_exactly_what_the_committed_recipe_produces`
rejects a hand-edit.

## The ownership rule

**This repo owns `clapboard_wire_vectors.json`; the clapboard repo vendors a
byte-identical copy and pins it with a checksum test.** Owner decision
2026-08-16, recorded in `plans/active/clapboard-can3-integration.md` §6. The
reason is gate strength, not seniority: `./run_tests.sh --full` runs here on
every commit, so a drifted vector is caught in this tree first.

The rule is restated inside the JSON's own `_README` block, because the peer
receives the JSON and not this file — and `test_clapboard_wire_vectors.py`
asserts that block still says it.

## Changing a vector

A vector change is a change to `Electronic-Clapboard docs/protocol.md` §8, which
is normative and **jointly owned — neither side may change it unilaterally**. So:

1. Agree the change with the clapboard repo first.
2. Edit `tools/probes/clapboard_wire_vectors_gen.py`, not the JSON, then:
   ```bash
   python tools/probes/clapboard_wire_vectors_gen.py --emit-fixture
   ```
   (A bare run writes nothing and reports whether the committed JSON still
   matches the recipe.)
3. Re-publish the digest:
   ```bash
   sha256sum tests/fixtures/clapboard_wire_vectors.json \
     | sed 's|tests/fixtures/||' > tests/fixtures/clapboard_wire_vectors.json.sha256
   ```
4. `./run_tests.sh` — `test_fixture_matches_published_checksum` is the step that
   refuses to let (2) happen without (3), which is what makes "go tell the other
   repo" an unavoidable prompt rather than a good intention.
5. Tell the clapboard repo to re-vendor and re-pin.

## Who reads these

- `tests/ros/test_clapboard_wire_vectors.py` — the host encoders
  (`jugglebot.clapboard_slate`) and decoders (`jugglebot.can.clapboard`) against
  every vector, plus the DLC-8 invariant in both directions.
- `tests/firmware/test_clap_wire_dlc_lint.py` — the two bridge-side emitters
  (`clap_link.cpp`, `time_sync_master.cpp`) against the same DLC rule.

Both files fail on an unrecognised fixture section or frame kind, so a vector
with no consumer cannot sit here rotting.
