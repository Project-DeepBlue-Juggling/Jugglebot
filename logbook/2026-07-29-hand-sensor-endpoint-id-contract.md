---
title: Board+fw-qualified ODrive endpoint ids — the class fix for a silently-wrong hand endpoint
type: refactor
date: 2026-07-29
status: in-progress
phase: "Hand ball-present sensor — Phase 2 (endpoint-id contract)"
related_plan: "hand-ball-sensor.md"
files_changed:
  - config/protocol_config.yaml
  - config/generate_config.py
  - config/generated/protocol_config.h
  - config/generated/protocol_config.py
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - ros_ws/src/jugglebot/Teensy_code/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h
  - ros_ws/src/jugglebot/jugglebot/can/odrive.py
  - ros_ws/src/jugglebot/jugglebot/archived/can_interface.py
  - tests/ros/test_odrive.py
  - tests/motion/test_endpoint_id_contract.py
  - tests/firmware/test_odrive_protocol_xref.py
  - tests/firmware/native/test_odrive_protocol.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/native/odrive_protocol_golden.json
commits: []                 # to be backfilled after the Phase 2 commit lands
subsystem:
  - config
  - can
tags:
  - testing
---

# Board+fw-qualified ODrive endpoint ids — the class fix for a silently-wrong hand endpoint

## Summary

The repo carried ODrive endpoint ids as a **flat, unqualified table** —
`endpoints: { GPIO_STATES: 700, commutation_mapper_pos_abs: 488 }` — which are
**S1 0.6.11** values with nothing in the name, the comment, or the type system
saying so. Jugglebot's hand drive is an ODrive **Pro** 0.6.11, where
`get_gpio_states` is **726**.

Phase 2 replaces the flat table with (board, fw)-qualified groups in
`config/protocol_config.yaml`, teaches both emitters the nested shape, and
migrates every consumer in the same change. No runtime behaviour changes —
nothing in production sends an SDO to either id today. What changes is that an
endpoint id can no longer be *quoted without its board*.

## Motivation

**700 on a Pro does not fail — it answers.** On Pro 0.6.11 endpoint 700 is
`encoder_estimator1.status`, a read-only `uint8`. An RxSdo read of it returns a
well-formed TxSdo reply, on time, forever, carrying a value that never changes.
So the commissioning check an implementer would naturally reach for — *"if the
endpoint is wrong we'll see no reply"* — is worthless here: the wrong id
produces a **live-looking sensor that is always in the same state**. That is
the failure mode Phase 0 exists to backstop with fw-version evidence, and it is
the same silent-plausible-answer shape Phase 1's flashed-pin drift test guards
from the other direction.

The one-line fix would have been to change `700` to `726`. That closes today's
bug and leaves the class open: the next person to add an endpoint id copies a
number out of *some* `flat_endpoints.json` into a flat table that has no slot
for which board and which firmware it came from, and the next wrong id also
answers plausibly. So this is the contract pattern instead — a normative
structure (the YAML shape), a single enforcement point (the two emitters, which
can only emit qualified names), and a test that fails if the invariant drifts.

## The new contract

```yaml
endpoints:
  # ODrive endpoint ids are firmware-build-specific (flat_endpoints.json).
  # Qualified by (board, fw). A consumer MUST verify Get_Version before use.
  odrive_pro_0_6_11:            # hw 4.4.58, fw 0.6.11 and 0.6.11-1 (tree CRC 55416)
    get_gpio_states: 726
  odrive_s1_0_6_11:             # BallButler hand (hw 5.2.0) — values proven in production on BB
    get_gpio_states: 700
    commutation_mapper_pos_abs: 488
```

Emitted surface, pinned so Phase 3 and cross-repo consumers can quote it
verbatim:

| Language | Name |
|----------|------|
| C++ | `EndpointId::odrive_pro_0_6_11::get_gpio_states` (nested namespace) |
| Python | `ENDPOINT_ODRIVE_PRO_0_6_11_GET_GPIO_STATES` |

The name carries the board and the firmware, so a mismatched pairing is
visible at the *call site* rather than only in the YAML comment nobody re-reads.

## Discussion

### The dangling Jugglebot name is deleted, not re-pointed

`can/odrive.py`'s `ENDPOINT_IDS` had a `'commutation_mapper.pos_abs'` entry.
Re-pointing it at the S1 group was the obvious mechanical migration, and it is
exactly wrong: `can/odrive.py` is a **Pro-facing module** (its
`encode_sdo_read` / `encode_sdo_write` are exercised only against Jugglebot
axes), so re-pointing would hand an S1 endpoint id to Pro axes — **the class
defect this phase exists to close, re-created one line below the fix**.

The entry has no production caller at all: the table's only readers are those
two encoders, and their only callers are tests driving axis 0 (a Jugglebot Pro
leg). A name with no production caller is not worth binding to *either* board's
id — binding it means picking a board on no evidence and leaving a loaded
mis-pairing for whoever wires up the first real caller. Deleted.
`ENDPOINT_IDS` is left with one entry, `'get_gpio_states'` → the Pro constant.

### 488 moves verbatim; the Pro's 461 is not emitted

`commutation_mapper_pos_abs: 488` is **production-proven** — BallButler's S1
encoder-search runs it on the shipping S1 0.6.11 firmware every session — so it
moves under `odrive_s1_0_6_11` unchanged, value and all.

The Pro's value for the same endpoint (**461**) is deliberately **not** added,
even though adding it would make the table look complete and cost one line.
The contract qualifies ids **by evidence, not by table completeness**: 461 has
no Jugglebot consumer, has never been exercised on this hardware, and shipping
it would put an unverified number in the one place the whole phase is trying to
make trustworthy. A future reader who needs it can add it *and* commission it
in the same change. Symmetry in the table is not a goal; every number in the
table being one somebody has actually seen work is.

### The native test uses the emitted symbol, not the 726 literal

`tests/firmware/native/test_odrive_protocol.cpp` passed the endpoint as a bare
literal (`488`, now `726`). It now passes
`EndpointId::odrive_pro_0_6_11::get_gpio_states`.

The tempting argument for keeping a literal is independence: a test that
re-derives its expectation from the same generated header it is testing can't
catch a bad header. But that anchor already exists and is stronger — the
**committed `odrive_protocol_golden.json`** is a byte-level record checked by
`test_odrive_committed_golden_matches_live_firmware`, and the Python xref side
(`test_odrive_protocol_xref.py`) already resolved the constant through the
generated module rather than a literal. So the C++ literal was **pure
duplication of an anchor that already existed in a better form**: both designs
fail loudly if the YAML drifts, but only the symbol version fails with a
compile error naming the missing endpoint instead of a byte diff you have to
decode by hand.

### The emitters are nesting-mandatory, not dual-mode

A reviewer proposed a shape guard in `generate_config.py` — detect a flat
scalar under `endpoints:` and either handle it or raise a friendly error. Not
adopted (the minimalism lens's call, taken over the correctness lens's
suggestion).

After the restructure **every** entry is nested; a flat scalar cannot appear
except by someone hand-editing the YAML back into the old shape. In that case
the emitters already fail loudly — `.items()` on an `int` aborts codegen with a
traceback pointing at the endpoints loop — and the new round-trip test fails
too. A defensive branch would buy a nicer message for a state the
repo-controlled YAML cannot reach, at the cost of a second supported shape that
future readers must keep working. Smallest code, loudest failure.

### Why BallButler lands in two commits, not one

The BB side needs `CanInterface.h:95-96`'s two `EndpointId::` aliases
re-pointed at `EndpointId::odrive_s1_0_6_11::*`, in lockstep with the Jugglebot
commit or the BB build breaks. But BB's generated `hardware_config.h` also
carried roughly **60 lines of pre-existing uncommitted drift** from earlier,
already-committed Jugglebot config work (flagged in the Phase 1 entry):
`MOTOR_KT` 0.0624 → 0.057 plus the `TORQUE_FF_*` block, `LEG_VEL_LIMIT_RPS`
4 → 12, the `JBOp` toss/reload block, `STOP_SETTLE_MS`, the entire new
`TrajOp` namespace (leg vel/acc/jerk limits and their ceilings, `KNOT_DT_S`,
timed-lead bounds, `SPACEMOUSE_HORIZON_S`, `LEAN_GAIN`, the four `CATCH_*`
reach/settle constants, `RETIME_MODEL`), five new `TeensyTraj` constants
(`THROW_DECEL_REFLECTED_INERTIA_KGM2`, `QUINTIC_H_MAX`, `QUINTIC_H2_MAX`, the
two `SMOOTH_MOVE_*`), the derived `JBOp::ACTIVATE_POSITION_REVS[6]` block,
and Phase 1's new `JBBallDetect` namespace. (The audit measured the full
delta at 73 insertions / 3 deletions including Phase 1's 13-line block —
the enumeration above is what BB commit 1's message must carry.)

Landing that in one commit with the alias migration would make `git blame` on
BallButler say the **endpoint-id contract phase changed the motor constant and
the leg velocity limit** — flashed-firmware constants, in a commit whose
message is about CAN endpoint addressing. That is the kind of attribution error
that costs an hour on a future bench day. So: commit 1 is the drift-resync of
the regenerated `hardware_config.h`, with the swept-in changes enumerated in
its message; commit 2 is the lockstep endpoint change, touching only
`CanInterface.h` and `protocol_config.h`.

## Changes

- **`config/protocol_config.yaml`** — flat `endpoints:` replaced by the two
  qualified groups above, with the "MUST verify `Get_Version` before use"
  normative comment.
- **`config/generate_config.py`** — both flat emitters taught the nested shape:
  the C++ path (`generate_cpp`) emits an inner `namespace <group>` inside
  `namespace EndpointId`; the Python path (`generate_python`) emits
  `ENDPOINT_{GROUP}_{NAME}`. Regenerated artifacts committed:
  `config/generated/protocol_config.{h,py}`, the ROS copy
  (`ros_ws/src/jugglebot/jugglebot/protocol_config.py`), and the three
  Jugglebot firmware dirs.
- **`ros_ws/.../can/odrive.py`** — `'commutation_mapper.pos_abs'` **deleted**;
  `'get_gpio_states'` re-pointed at
  `proto.ENDPOINT_ODRIVE_PRO_0_6_11_GET_GPIO_STATES`.
- **`ros_ws/.../archived/can_interface.py`** — left code-as-is with a
  self-anchored `STALE (2026-07-29)` comment. The pre-commit audit corrected
  the comment's first draft, which claimed importing the module "now raises
  `AttributeError`" — untraced and false: the module was **already
  unimportable before this change** (its line 47 imports
  `jugglebot.ball_butler_states`, which exists only under `archived/`), so
  the removed names are unreachable and no new failure mode was introduced.
  The comment now states exactly that. Rewriting archived code to keep it
  notionally runnable is maintenance nobody asked for.
- **`tests/ros/test_odrive.py`** — 4 sites re-targeted; 2 test method names
  changed to say `get_gpio_states` (they keyed on the deleted name).
- **`tests/firmware/test_odrive_protocol_xref.py`** — `_SDO_PARAM` →
  `"get_gpio_states"` / 726, which moves the bytes the xref reproduces.
- **`tests/firmware/native/test_odrive_protocol.cpp`** — endpoint passed as
  `EndpointId::odrive_pro_0_6_11::get_gpio_states`.
- **`tests/firmware/native/test_rpc_dispatch.cpp:189`** — `488` → `726` on a
  *rejection* path (the value never reaches the wire). Cosmetic, done so no
  bare `488` aimed at a Pro axis survives in the firmware/test tree.
- **`tests/firmware/native/odrive_protocol_golden.json`** — regenerated
  (`python tests/firmware/native/build.py --odrive-golden
  tests/firmware/native/odrive_protocol_golden.json`). **Exactly 2 of 19 rows
  changed**, both `sdo_read`/`sdo_write` data bytes `e801` → `d602`
  (LE `uint16` 488 → 726); every other row byte-identical. That 2/19 blast
  radius is itself the evidence that the regeneration changed only what it
  should have.
- **BallButler `ball_butler_main/CanInterface.h:95-96`** — the two aliases
  re-pointed at `EndpointId::odrive_s1_0_6_11::*` (separate repo, lockstep
  commit; see Discussion for the two-commit split). **Not yet committed at
  the time of writing** — the BB working tree carries the edits; BB commits 1
  and 2 land immediately after this Jugglebot commit, and their SHAs are
  backfilled here with it.

### New tests

`tests/motion/test_endpoint_id_contract.py` — a review finding against the
plan's Testing-plan row *"codegen round-trip for the nested endpoint shape"*,
which nothing else covered. Two tests:
`test_endpoint_yaml_roundtrips_to_generated_constants` walks the YAML
`endpoints:` block and asserts every `(group, name)` resolves to
`ENDPOINT_{GROUP}_{NAME}` on the generated module with the right value,
asserting at least 3 entries were seen (so an emptied block can't pass
vacuously) — pinning **codegen freshness** and the **name-mangling contract**
that Phase 3 and the BB header quote verbatim.
`test_ros_delivered_copy_matches_generated` asserts the ROS-delivered
`protocol_config.py` is byte-identical to `config/generated/` (the delivered
copy is what the round-trip test and the live nodes actually resolve
through). The audit moved this file out of its first home in
`test_odrive_protocol_xref.py`: that module sits below
`pytest.importorskip("can")`, so on boxes without python-can (the Win10 sim
clone) the codegen-freshness pin would silently vanish — an ungated
`tests/motion/` module is the sibling home Phase 1's drift test set.

### Grep-zero

The removed flat names — `ENDPOINT_GPIO_STATES`,
`ENDPOINT_COMMUTATION_MAPPER_POS_ABS`, `EndpointId::GPIO_STATES`,
`EndpointId::commutation_mapper_pos_abs` — return **zero hits in both repos**,
excepting `jugglebot/archived/can_interface.py` (the deliberately-flagged stale
block) and plan prose describing the pre-change state.

Two known out-of-scope survivors of the *literal-id* class, recorded so the
grep-zero is not read as covering them:

- `BallButler/zTesting/sensored_hand_testing/CanInterface.h` has its own
  hardcoded `488`/`700` struct and **never includes the generated header**.
  Untouched, and correct as-is — that sketch talks to an S1.
- `experimenting/platform_calibration/measuring_leg_mapping/can_interface.py:79`
  hardcodes `488` against Jugglebot legs (Pro axes, where the id is 461) and
  never imports `proto`. A standalone calibration script with no production
  caller — out of this phase's consumer list, left untouched (461 is
  deliberately unverified/unemitted), recorded here as a known-wrong literal
  should anyone resurrect that script.

### Process

One Opus implementer, then three parallel read-only reviewers on distinct
lenses (correctness / plan-conformance / minimalism), then the pre-commit
audit. **Zero blocking findings.** Orchestrator-applied fixes: the round-trip
tests above (review), the native-test literal → emitted-symbol substitution
(review), the archived-module comment (review draft corrected again by the
audit — see the Changes bullet), the BB drift enumeration extended to the
full sweep (audit), and the out-of-scope `488` survivor recorded (audit). One
suggestion **rejected with reason** — the emitter shape guard (see
Discussion).

## Verification

**Scoped gate** — `python -m pytest tests/motion/test_endpoint_id_contract.py
tests/ros/test_odrive.py tests/firmware -q`, run 2026-07-29: **423 passed in
17.61 s** (native binaries hash-cached from the earlier run), on the
post-audit tree, including the regenerated golden and both new round-trip
tests. Earlier post-review-fixes run with a full native rebuild:
`python -m pytest tests/ros/test_odrive.py tests/firmware -q`, run
2026-07-29: **422 passed in 178.62 s (0:02:58)**.

**Full suite** — `python -m pytest tests/ -q`, run 2026-07-29 by the
implementer on the **pre-review-fixes** tree: **4258 passed, 3 xfailed in
1435.84 s (0:23:55)**. A bonus data point beyond the scoped-gate policy, not
the gate itself; the post-fix delta is covered by the scoped run above.

**BallButler build** — `cd /home/jetson/Desktop/BallButler/ball_butler_main &&
pio run -e teensy41` (`pio` from `~/Desktop/PDJ_venv/venv/bin`), run 2026-07-29:
**SUCCESS in 5.68 s**, `firmware.hex` built — against the migrated aliases and
the regenerated header.

Per the operator's gating direction of 2026-07-29, phases of this plan run
scoped checks per commit and the **full suite once at the end of the plan**.

**Not verified**: that **726 is the right number**. Nothing here has run on
powered hardware, and no code sends an SDO to it yet — the first live read is
Phase 3's poller, confirmed at Phase 7. This phase makes the id *impossible to
quote without its board*; it does not prove the id.

## Deployment

**Nothing to flash and nothing to deploy this phase.** The regenerated firmware
headers have no consumer until `gpio_poll.cpp` exists (Phase 3), and the BB
build was run only to prove the aliases compile. The ROS copy of
`protocol_config.py` did change, so the *installed* package keeps the old flat
names until the next ordinary `colcon build --packages-select jugglebot` —
which first matters when Phase 4 code reads the new constant.

## Open questions

1. **Does any other repo-carried ODrive constant need the same qualification?**
   This phase qualified `endpoints:` only. Command ids and opcodes are stable
   across the board families in play, but that was asserted from the protocol
   docs, not swept.
2. **When 461 (the Pro's `commutation_mapper.pos_abs`) is eventually wanted**,
   it must arrive with a commissioning read, not as a table-completion edit —
   see Discussion.

## Related

- `plans/active/hand-ball-sensor.md` — Phase 2, and the endpoint-trap
  paragraphs in § Notes for collaborators.
- `logbook/2026-07-29-hand-sensor-fw-version-surfacing.md` — Phase 0, the
  fw-version evidence that makes "which firmware is this id pinned against"
  answerable on the bench.
- `logbook/2026-07-29-hand-sensor-ball-detect-config.md` — Phase 1, whose
  "Phase 2 must enumerate the BallButler drift" open question this entry
  discharges.
