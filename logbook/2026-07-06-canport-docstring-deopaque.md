---
title: De-opaque CAN-port docstrings/comments — strip plan-document vocabulary from the code
type: refactor
date: 2026-07-06
status: resolved
phase: "post-port code-clarity cleanup"
related_plan: teensy-can-offload.md
related_entries:
  - 2026-07-06-phase13-socketcan-decommission
  - 2026-06-27-can-node-teensy-parity-audit
files_changed:
  - config/generate_udp_protocol.py
  - controller/teensy_link/
  - ros_ws/src/jugglebot/Teensy_code_canbridge/
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - ros_ws/src/jugglebot/jugglebot/can/__init__.py
  - ros_ws/src/jugglebot/jugglebot/motion/friction_ff_params.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/launch/teensy_bridge_launch.py
  - ros_ws/src/jugglebot/setup.py
  - tests/firmware/
  - tests/teensy_link/
  - tests/ros/
  - tests/hardware/
  - tests/motion/
  - tools/probes/
  - tools/teensy_link_bridge.py
commits:
  - 63feaef
  - f13be2b
  - b60e0c6
  - eeb76a9
  - f1e5ac3
  - 235514d
subsystem:
  - can
  - ros
  - firmware
  - controller
  - tooling
tags:
  - refactor
  - documentation
  - code-clarity
  - migration-cleanup
---

# De-opaque CAN-port docstrings/comments

## Summary

Across the ~6-week port of all CAN comms off the Jetson onto the can-hub Teensy,
code comments/docstrings accumulated references to the *plan documents'* private
vocabulary — `Phase 6`, `Tier-2`, `U3-iv`, `Flash-A item 4`, `item 14`,
`canbridge-foundation-coldstart-parity`, `decision D9`, `(Fable-5 …)`, the `α`/`β`
path codenames. Those phrases are meaningful inside the plans but **completely
opaque to a collaborator reading only the code**. This sweep removes them from the
code and replaces each with self-contained functional wording, keeping the plan
vocabulary where it belongs (the plans and the logbook).

Scope: all recently-modified (CAN-port arc) code — files last touched on/after
2026-06-03. ~130 files (131 unique; `fault_logic.py` edited in both batch 2 and batch 6),
~825 rewrites, delivered in six gated, path-scoped commits:

1. `63feaef` — the UDP-protocol generator (`generate_udp_protocol.py`) + all its
   regenerated copies (`config/generated/udp_protocol.{h,py}`, `docs/teensy-udp-protocol.md`,
   the firmware `udp_protocol.h`, the profiling-harness copy). Fixed at source, regenerated.
2. `f13be2b` — the `controller/teensy_link/` Jetson library (11 modules).
3. `b60e0c6` — the `Teensy_code_canbridge/` firmware (32 files) + 3 in-pass comment-accuracy
   fixes (`rpc.h` staleness fix, `can_buses.h` D7 reword, `profiling.h` straggled-tag drop).
4. `eeb76a9` — `teensy_bridge_node.py` + launches + `setup.py`.
5. `f1e5ac3` — the CAN-port test suite (58 files; 3 concurrently-edited files deferred — see below).
6. `235514d` — probes, tools, bench harnesses (18 files) + 2 straggler fixes.

Comment/docstring/string text only — no wire format, control logic, topic, service,
or behaviour change anywhere.

## Discussion

**Why a scope boundary at 2026-06-03, not the whole repo.** The same opaque-token
disease exists in the pre-port May MPC-robustness plan files (`controller/mpc.py`,
`scheduler.py`, the `tests/sim/` *tier* tests) and the 2026-05-08 friction-FF
integration plan (`§3.1`/`§7.1` refs in `friction_ff_params.py`,
`test_motor_guard_friction_ff.py`). Those belong to *different, older* plans; the
date-sorted history shows a clean cut (last-modified jumps 2026-05-21 → 2026-06-03,
the day `teensy_link/`, `Teensy_code_canbridge/`, and `teensy_bridge_node.py` first
appeared). Folding them in would have doubled the surface with unrelated vocabulary,
so they are explicitly deferred as an optional follow-up batch. The friction-FF
`§`-refs additionally *name* their archived plan doc, so they are less opaque than a
bare `Phase 6`.

**REWRITE vs PRESERVE — the litmus.** Not every `Phase N` is opaque. Local
step-markers that the surrounding code fully explains (e.g. `conftest.py`'s
"Phase 1: inject mocks / Phase 2: fixtures" banner enumerating the file's own two
halves; `teensy_bridge_node.py`'s `_reboot_odrives` "Step 1/Step 2" hook order) read
fine to a code-only reader and were **kept**. Only references that point *outside the
file* (to a plan, tier, numbered unit/item, flash batch, decision id, or the AI-model
attribution) were rewritten. 22 sites were preserved on this basis.

**The four operator decisions that shaped the wording** (asked up front, applied
uniformly): (a) the `α`/`β` codenames → `α` = "Jetson-relay" (the legacy Jetson
500 Hz motor_guard relay), `β` = "Teensy-side" (the on-Teensy interpolator knot path);
(b) `ADR-XXXX` architecture-decision records → **kept** (durable, navigable, unlike
ephemeral plan phases); (c) logbook pointers added **only at defining/banner sites**,
strip-only for inline repeats, to keep them useful not noisy; (d) handoff-doc decision
ids (`D1…D12`, `HANDOFF Dx`) and `Fable-5` attributions → stripped, substance inlined.
Root-cause for (b)/(c): traceability already exists structurally via `git blame` →
commit → its `Logbook-Entry:` trailer, so inline plan-phase refs are redundant clutter;
an ADR is the one external reference durable enough to keep.

**Two staleness bugs fixed in-pass** (the sweep surfaced comments that were not just
opaque but *wrong*): `rpc.h` claimed the RPC arg layouts were "to be hoisted at
Phase 10" — they are already hoisted into `JbUdp::RpcArgs` (a few lines below);
`setup.py` claimed `teensy_bridge_node` is "NOT in jugglebot_launch by default" — it
is now a launched Node there. `can_buses.h`'s `HANDOFF D7` clause was reworded to keep
its accurate technical point (the FlexCAN rxBuffer hop vs. no application-level queue).
Fixed in the same session per the "fix surfaced issues when the diagnosis is clear"
norm — these are comment-accuracy fixes, same spirit as de-opaquing.

**Completeness: the gathering regex under-caught, so a ground-truth backstop was
mandatory.** The initial per-file inventory (a `Phase[-\s]?\d`-style pattern) missed
several token classes: lowercase `phase-10b`, letter-suffix `Phase A`, bare
sub-phases (`9a`/`9b`/`in 10b`/`when 9a`), `HANDOFF`/`(row N)`/`coverage gap N`/`§N`,
and — because those files contained *only* missed tokens — **five whole files that
were never inventoried** (`test_teensy_bridge_node_cone.py`, `shutdown_stow.py`,
`profile_monitor.py`, native `test_ball_butler_protocol.cpp`, `test_odrive_protocol.cpp`).
A final complete-pattern sweep over the entire surface caught every straggler; ~22
were fixed by hand after the batch agents. Lesson for the next such sweep: the
line-level ground-truth grep (broad pattern, whole directory tree, not a curated file
list) is the authority for *completeness*; the per-file proposals are only the
first-draft *wording*.

**Concurrent-session handling.** A parallel session was developing an unrelated
feature (a `SET_LEAD_ACCEL_LIMIT` RPC for the leg-interpolator lead clamp) in the same
shared working tree. Its edits were additive on top of the committed de-opaquing (my
work was never reverted), but it held uncommitted changes in three files that overlap
this sweep's batch-5 scope (`native/test_leg_interp.cpp`, `native/test_rpc_dispatch.cpp`,
`test_udp_protocol_xlang.py`). Because git stages whole-file content, de-opaquing those
would have entangled the two changes — so all six commits were staged by **explicit
path with `:!` exclusions** (never `git add -A`), and those three files are **deferred**
to a follow-up once the feature lands.

## Verification

Full suite (`pytest tests/ -q`, run 2026-07-07): **1955 passed, 1 xfailed in 639.44 s**,
with one **load-flaky** failure (`tests/motion/test_motor_guard.py::test_decay_boundary_continuity`)
that passes in isolation (0.43 s) and whose files are unmodified by this work — a known
timing/boundary flake, not a regression. Batches 1, 5, and 6 were gated on the full suite
(batch 1 `63feaef`, 2026-07-06: 1956 passed, 1 xfailed, 635.99 s; batches 5+6 share the
2026-07-07 full run cited above); batches 2–4 on scoped subsets during iteration
(`tests/teensy_link tests/ros` 811 passed; `tests/firmware` 165 passed incl. native
firmware compile; `tests/ros tests/motion/test_friction_ff_params.py` 630 passed). Native firmware compiles cleanly (comment-only edits) and the firmware
xref/lint parsers pass.

## Remaining work

- **Three deferred files** (`native/test_leg_interp.cpp`, `native/test_rpc_dispatch.cpp`,
  `test_udp_protocol_xlang.py`) — de-opaque once the concurrent `SET_LEAD_ACCEL_LIMIT`
  feature is committed, to avoid entangling the two changes.
- **Optional pre-port batch** — the same cleanup for the May MPC-robustness plan
  vocabulary in `controller/mpc.py`, `scheduler.py`, `plant.py`, `params.py`, and the
  `tests/sim/` tier tests, plus the 2026-05-08 friction-FF plan `§`-refs in
  `friction_ff_params.py` and `test_motor_guard_friction_ff.py` (~50 sites total), if desired.
