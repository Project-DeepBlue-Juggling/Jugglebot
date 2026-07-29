---
title: Config groundwork for the hand ball-present sensor — jugglebot_ball_detect and its flashed-pin drift test
type: feature
date: 2026-07-29
status: in-progress
phase: "Hand ball-present sensor — Phase 1 (config groundwork)"
related_plan: "hand-ball-sensor.md"
files_changed:
  - config/hardware_config.yaml
  - config/generate_config.py
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - tests/motion/test_hand_ball_detect_config.py
commits:
  - 2b3ab78                # feat(hand-sensor phase-1): jugglebot_ball_detect config block + flashed-pin drift test
subsystem:
  - config
tags:
  - testing
---

# Config groundwork for the hand ball-present sensor — `jugglebot_ball_detect` and its flashed-pin drift test

## Summary

Phase 1 adds the config block the rest of the plan reads from and the drift
test that keeps it honest. `config/hardware_config.yaml` gains a new section 23,
`jugglebot_ball_detect` — a direct sibling of `ball_butler_ball_detect` —
registered in `generate_config.py`'s section table so codegen emits `JB_BD_*`
Python constants and a C++ `namespace JBBallDetect`. A new
`tests/motion/test_hand_ball_detect_config.py` pins the generated pin against
the flashed hand-ODrive config **and** against the YAML.

**Nothing consumes any of it yet.** The first reader is Phase 3's `gpio_poll`;
there is no behaviour change, no new runtime code path, and nothing to flash.

## Motivation

The sensor's ODrive-side pin config is **already flashed and NVM-persisted**
(recorded in commit `64d2a8f`), and nothing in the system ever reads that
setting back off the drive — the repo's `config/ODrive config Files/
odrive_pro_hand_config.json` is a *mirror* of the flash, not a source that is
verified against it. That makes silent divergence the default failure mode, and
this particular divergence is nasty: if G02 is not flashed
`DIGITAL_PULL_UP`, the `get_gpio_states` bit still answers — it just carries
whatever the pin's other function drives. A live-looking, never-changing sensor
**with no timeout to diagnose it** is exactly the failure shape Phase 0's
motivation section documents for the wrong endpoint id, arriving through a
different door.

The config block itself is the ordinary prerequisite: Phase 3's poller is
firmware (C++) and Phases 4–6 are Python, so the pin, the poll rate, the
debounce depth and the expected firmware triple all have to exist in both
languages from one source.

## Design

The block, verbatim in intent:

| Key | Value | Why |
|-----|-------|-----|
| `enabled` | `true` | Build-time kill switch — `false` compiles the Phase 3 poller out entirely (requires a reflash). The Phase 7 A/B uses the **runtime** serial toggle (`gpio_poll on\|off`), not this flag. |
| `gpio_pin` | `2` | G02 on the hand ODrive Pro; switch shorts to GND, internal pull-up. |
| `check_interval_ms` | `20` | 50 Hz continuous poll (operator decision 2026-07-28). |
| `max_missing_samples` | `5` | HELD→EMPTY debounce — a 100 ms window at 50 Hz. |
| `check_timeout_ms` | `100` | SDO reply timeout per request. |
| `expected_fw` | `[0, 6, 11]` | The fw triple the endpoint-id table is pinned against; Phase 3's `Get_Version` gate compares against it. |

Registration is one row in `generate_config.py`'s `HW_SECTIONS` table:
`("jugglebot_ball_detect", "JB_BD_", "JBBallDetect", "Jugglebot Ball Detection")`.

**No generator change was needed.** A 3-int list already emits as ints in both
languages — the pre-existing precedent is `ODriveVer::AXIS_0` — so `expected_fw`
lands as `JB_BD_EXPECTED_FW` in Python and
`constexpr uint32_t EXPECTED_FW[3] = {0, 6, 11}` in C++ with the generator
untouched.

Emitted surface: `JB_BD_*` Python constants, and C++
`namespace JBBallDetect { ENABLED, GPIO_PIN, CHECK_INTERVAL_MS,
MAX_MISSING_SAMPLES, CHECK_TIMEOUT_MS, EXPECTED_FW[3] }`.

## Discussion

### `JBBallDetect`, not the plan's literal `JbBallDetect`

The plan's Phase 1 text spells the C++ namespace `JbBallDetect`. Shipped as
`JBBallDetect`, and the deviation is deliberate: **all 26 namespaces in the
generated header uppercase their initialisms** — `JBOp`, `BBGeom`, `BBHb`, and
most pointedly the direct sibling this block was modelled on, `BBBallDetect`.
The plan's lowercase-`b` spelling is a typo-class inconsistency rather than a
design choice, and this commit was the **last free moment to fix it**: zero
consumers exist until Phase 3, so the rename costs nothing now and would cost a
cross-language sweep later. Behaviour is unchanged either way; the plan text is
corrected in the backfill commit.

### A Phase 3 constraint the review surfaced, recorded so the firmware implementer does not trip on it

`JBBallDetect::ENABLED` emits as a **`constexpr bool`, not a preprocessor
macro**. The plan's phrasing — "`enabled: false` compiles the TU out" — therefore
**cannot be implemented as a literal `#if`**. The mechanism has to be
constexpr-gated code: early returns on `!ENABLED`, letting the compiler
eliminate the dead body.

The trap is specific and silent. The tempting `#if JB_BD_ENABLED` evaluates
an **undefined identifier to 0** — the preprocessor does not see C++
constants — so the poller compiles *out* even when `enabled: true`, with no
diagnostic. (The `#if JBBallDetect::ENABLED` spelling is not the silent one:
`::` is a hard preprocessor error, verified with `g++ -std=c++17` on
2026-07-29.) That is a working-firmware-that-does-
nothing outcome discovered on the bench, and it is cheap to avoid if the
implementer knows before writing the file. The plan's Phase 3 bullet gets this
clause in the backfill commit.

### Why the drift test has two legs rather than one

The plan asks for one assertion: `gpio2_mode == 1` in the flashed hand JSON,
"keyed off the new YAML block's `gpio_pin`". Keyed off the **generated**
constant, which is what a Python test can actually read, that single leg has a
hole — the *stale-codegen* divergence. Move the pin in the YAML and forget to
re-run `generate_config.py`, and the generated constant still says `2`, so the
test still checks `gpio2_mode` and still passes, green, while the YAML says
something else and the firmware that Phase 3 builds from the header polls the
wrong bit.

So there are two tests:

1. `test_ball_detect_pin_is_flashed_as_a_pulled_up_input` — the plan's leg:
   the flashed JSON's `gpio{JB_BD_GPIO_PIN}_mode` must be `1`
   (`DIGITAL_PULL_UP`). A hand-config re-dump from a drive that lost the
   setting, or a pin move, breaks loudly.
2. `test_generated_pin_matches_the_yaml` — the YAML↔generated leg, mirroring
   the repo's canonical stale-codegen test
   (`test_trajectory_planner_catch.py::test_generated_seat_rate_matches_the_yaml`).

Two tests, not one compound assertion, so **a failure names its own class**:
wrong flash versus stale codegen are different repairs by different people, and
the test name should say which one you have.

## Implementation

- **`config/hardware_config.yaml`** — new section 23,
  `jugglebot_ball_detect`, placed as the sibling of `ball_butler_ball_detect`,
  with the six keys above and the house-form header
  `Source: Teensy_code_canbridge (gpio_poll.cpp — NEW)`.
- **`config/generate_config.py`** — one `HW_SECTIONS` row; the following
  Catching Cone comment renumbered 23 → 24. No generator logic touched.
- **Regenerated artifacts, all committed this phase** —
  `config/generated/hardware_config.{h,py}`, the ROS copy
  (`ros_ws/src/jugglebot/jugglebot/hardware_config.py`), and the three
  Jugglebot firmware dirs (`Teensy_code`, `Teensy_code_canbridge`,
  `CatchingCone_code`).
- **`tests/motion/test_hand_ball_detect_config.py`** — the two tests above,
  with the failure they exist to kill written into the module docstring.

### The BallButler artifact is deliberately *not* committed here

`generate_config.py` also writes `../BallButler/ball_butler_main/
hardware_config.h`, which lives in a **separate git repo**. That file already
carried roughly **60 lines of pre-existing uncommitted drift** (the file's
current 73-insertion / 3-deletion delta minus the 13-line `JBBallDetect`
block this phase added) from earlier,
already-committed Jugglebot config work: `MOTOR_KT` 0.0624 → 0.057,
`LEG_VEL_LIMIT_RPS` 4 → 12, the `TORQUE_FF_*` block, the `TOSS_*` block,
`HAND_CATCH_PRIME_REV`, `STOP_SETTLE_MS`. This phase stacked only the
`JBBallDetect` namespace on top of that pile.

Nothing was committed on the BB side this phase. **Phase 2's lockstep
BallButler commit carries the whole resync, and must enumerate the swept-in
pre-existing drift rather than attributing it to Phase 2** — otherwise a
`git log` on the BB repo will read as though the endpoint-id contract phase
changed the motor constant and the leg velocity limit.

### Process

Implemented by one Opus agent, then two parallel read-only reviewers on
distinct lenses (correctness + plan-conformance / minimalism). **Zero blocking
findings.** Five orchestrator-applied fixes at finalize: the YAML↔generated pin
assertion added (closing the stale-codegen leg the plan's "keyed off the YAML"
text asks for), the C++ namespace renamed `JbBallDetect` → `JBBallDetect`, a
redundant comment deleted from the generator table, a dead `int()` cast dropped,
and the YAML `Source:` line normalised to the house
`(gpio_poll.cpp — NEW)` form.

## Verification

**Scoped gate** — `python -m pytest tests/motion/test_hand_ball_detect_config.py
tests/firmware -q`, run 2026-07-29: **357 passed in 184.08 s (0:03:04)**.

**Broader sweep by the implementer** (pre-review-fixes tree) —
`python -m pytest tests/motion tests/ros/test_gui_geometry.py
tests/sim/test_motor_kt_canonical_source.py tests/sim/test_ball_butler_sim.py
tests/sim/test_hand_throw_decel_ff.py tests/firmware -q`, run 2026-07-29:
**1324 passed in 526.95 s**.

**Logbook surface** — `python -m pytest
tests/motion/test_hand_ball_detect_config.py tests/sim/test_logbook_search.py
-q`, run 2026-07-29: **26 passed in 0.43 s**. Weak evidence for the INDEX row
specifically: `logbook_search.py` skips `INDEX.md` (`_SKIP_FILES`) and
`continue`s past title-less entries, so a broken INDEX row would still pass.

Per the operator's gating direction of 2026-07-29, phases of this plan run
scoped checks per commit and the **full suite once at the end of the plan**.

**Not verified**: nothing here has run on hardware, and nothing here can — no
code reads these constants until Phase 3. The drift test asserts that the
repo's *mirror* of the flashed hand config says `gpio2_mode == 1`; it cannot
assert that the drive's NVM agrees. That remains a Phase 7 bench observation.

## Deployment

**Nothing to deploy this phase.** No firmware flash (the regenerated headers
have no consumer until `gpio_poll.cpp` exists) and no `colcon build` is
*required* — though the ROS copy of `hardware_config.py` did change, so the
installed package will not carry `JB_BD_*` until the next ordinary rebuild.
That first matters at Phase 4.

## Open questions

1. **How Phase 3 gates on `ENABLED`.** A `constexpr` early return is the
   recommendation above; whether the compiler's dead-code elimination is
   sufficient to satisfy the plan's "compiles out entirely" intent (versus,
   say, wrapping the poller registration) is a Phase 3 call.
2. **The BallButler resync.** Phase 2 must land it *and* enumerate the
   pre-existing drift it sweeps in. If Phase 2 slips, the BB working tree
   stays dirty in the meantime — a known, deliberate state, not a surprise.

## Related

- `plans/active/hand-ball-sensor.md` — Phase 1 (and the Phase 3 bullet this
  entry adds a constraint to).
- `logbook/2026-07-29-hand-sensor-fw-version-surfacing.md` — Phase 0 sibling,
  the fw-version instrumentation that `expected_fw` will be checked against.
- Commit `64d2a8f` — the flashed, NVM-persisted `gpio2_mode = 1` record that
  this drift test now guards.
- `tests/motion/test_leg_torque_ff.py::test_yaml_kt_odrive_config_matches_the_flashed_odrive_json`
  — the flashed-mirror drift-test pattern this follows.
