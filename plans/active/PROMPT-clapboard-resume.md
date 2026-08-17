---
title: Resume prompt — electronic clapboard over CAN3
created: 2026-08-18
status: active
related_plan: clapboard-can3-integration.md
---

# Resume prompt: electronic clapboard over CAN3

**This branch is PARKED as of 2026-08-18.** It is not abandoned and it is not
finished. Read this file end to end before touching anything.

## How to get here

```bash
git worktree add ~/Desktop/Jugglebot-clapboard 2026-08_clapboard-can3
cd ~/Desktop/Jugglebot-clapboard
source ~/Desktop/PDJ_venv/venv/bin/activate
```

The branch is `2026-08_clapboard-can3`, pushed to origin. `mvp-trajectory-bringup`
carries **only** `plans/parked/clapboard-can3-integration.md` — no clapboard
code, no clapboard tests, no firmware change. Everything real is here.

The plan itself is `plans/active/clapboard-can3-integration.md` **on this
branch** (it is the parked copy on the main branch — same filename, as the
never-rename rule requires). Read it in full; it is the specification this work
implements.

## The single most important thing to know

**The final whole-diff audit never ran.** The workflow that built this had one
fresh auditor scheduled over the complete `38892ed..HEAD` diff, and it died on a
usage limit before starting. Each phase audited *its own* changes, so what is
unverified is precisely what a per-phase audit structurally cannot see: whether
the pieces fit each other.

**First action on resume, before writing any new code:** run that audit. The
brief is in the workflow script at
`.claude/projects/.../workflows/scripts/clapboard-can3-build-wf_979a0720-286.js`
(the `final-audit` agent), or reconstruct it from §5 and §6 of the plan. It must
verify, directly in the final code and not from any report:

1. `s_cone_last_rx_us` is **still written unconditionally** in `on_cone_rx`. If
   a phase gated it behind the ID check, the TX presence gate silently closes,
   time-sync stops, and the clapboard never leaves screensaver. Highest-severity
   check in the whole workstream.
2. Every emitted frame is DLC 8 — especially `CLAP_LINK`, whose natural 1-byte
   implementation the clapboard silently drops.
3. `PROTOCOL_VERSION` is still 5.
4. `FW_VERSION == EXPECTED_BRIDGE_FW_VERSION`, both 15, bumped exactly once.
5. No field was added to `BusRxHealth` without a matching line in `snapshot_bus`.
6. Firmware never retries a gated TX and never bypasses the gate.
7. Frames are paced one per tick with a **named** divisor.
8. `tests/ros/test_teensy_bridge_node_cone.py` has no logic change (one
   docstring line was touched by the doc sweep; that is the only edit and it is
   acceptable — the byte-identical proof for the cone path still holds).
9. No new pytest marker (vocabulary is exactly `serial` + `nightly`).

## What is done

| Commit | Phase | State |
|---|---|---|
| `8eaa6ec` | 0 — test-surface prep | Gated, self-audited, complete |
| `86d864a` | 1 — uplink, Jetson only | Gated, self-audited, complete |
| `336f5a0` | 2a — honest `cone_health` + `CLAP_LINK` | Gated, self-audited, complete |
| `9b6e653` | 2b — `CLAP_SEND` + paced drain + `CLAP_DIAG` + FW 14→15 | Gated, self-audited, complete |
| `de1cbae` | 3 — `SetSlate` action | Gated, self-audited, complete |
| `d7989b9` | 4 — cross-repo wire fixture | Gated, self-audited, complete |
| `e650448` | 6 — bus-role doc sweep | **WIP. NEVER GATED. NEVER AUDITED.** |

Each completed phase carries its own logbook entry with a (date, command,
result) verification triple; read those rather than trusting this table.

## What is NOT done

- **The final whole-diff audit.** See above.
- **Phase 5, the bench soak.** An operator sitting with the clapboard attached.
  Acceptance criterion is pre-registered: ≥2 h with a slate push every 60 s, and
  cone-bus `bit0_cnt`/`bit1_cnt`/`ack_cnt` must show no increase over the idle
  baseline. If they do, the response is **already decided** — halve the drain
  rate (a one-line change to the named divisor), not an investigation.
- **Phase 6.** Committed mid-flight, never run against the suite. The
  `docs/adr/0013-three-can-buses.md` amendment needs particular review: an ADR
  is a historical record and must not be rewritten to read as though it always
  said CAN3.

## Firmware state — read before any powered session

**FW 15 exists in source on this branch and has NEVER BEEN FLASHED.** The
can-bridge Teensy is running FW 14.

`EXPECTED_BRIDGE_FW_VERSION` was bumped to 15 here, so launching from this
branch against the real bridge logs `BRIDGE_FW_CHECK: FAIL`. That check is
advisory and never enforced — commands are not refused — but it will look
alarming if unexpected.

When the flash does happen:

- Pass `-e teensy41` **explicitly**. A bare `pio run -t upload` builds and
  flashes every env in file order, and the bench-sysid variant — *"NEVER flash
  to the assembled robot"* — lands last and wins, with zero wire-format change
  to give it away.
- **Confirm the flash on the `BRIDGE_IDENTITY` frame** (`/link_status`
  `bridge_fw_version` must read 15), never by inference. FW 9 through 14 were
  all wire-identical, so a healthy link proves nothing about which build is
  aboard.
- Rollback is all-or-nothing back to FW 14. The owner chose one flash over
  rollback granularity on 2026-08-16, so there is no intermediate image.
  Bisection means reverting 2b in the tree and rebuilding.

## Open question needing the owner, before Phase 2b ships

The clapboard repo's `docs/can-integration-handoff.md` §4 instructs *"Bump
PROTOCOL_VERSION (currently 5). Both sides validate it."* **The plan declines
that bump**, and the Phase 0 agent assessed the disagreement against the
normative document and concluded the plan wins: `docs/protocol.md` §8 governs
only the CAN wire contract and never mentions `PROTOCOL_VERSION`, which is the
Jetson↔Teensy **UDP** protocol version the clapboard never sees. The handoff
doc is explicitly non-normative. The plan's own reasoning — additive
MsgType/RpcArg, six in-tree precedents, exact-size unpack as the real bump
trigger — is sound.

This is almost certainly correct, but it is a cross-repo contract call and it
was never put to the owner. **Confirm it before shipping**, because a wrong
answer here takes the whole UDP link dark in both directions, which is the
`24608bb` failure mode that once cost a session.

## Standing context that does not change

- The clapboard repo's `docs/protocol.md` §8 is **normative and may not be
  changed unilaterally by either side**. Never edit that repo from here.
- The clapboard **accepts no fire commands, by design, on either transport**.
  *"ROS2 gets a complete log of every clap; it does not get to clap."*
- The clapboard must **transmit first**. The bridge's TX gate is closed for the
  first 5 s and after any 5 s of silence, so a device that waits to be spoken to
  will never be spoken to — and the failure presents as "the bridge is broken".
- CAN3's analog drive path is known-degraded. The logbook discriminator is drive
  current from node count, not frame rate, and a clapboard is one node — so the
  risk is modest, but the soak is what proves it.
- Owner decisions, all resolved 2026-08-16 and recorded in §"Owner decisions" of
  the plan: fallback accepted, 12 V validated as adequate, this repo owns the
  golden-vector fixture, harness already wired, single flash.

## A known coverage observation worth acting on

Phase 0 surfaced that the `ConeFrame` gap it closed was one instance of a class:
several generated messages still have no round-trip test, and more are absent
from the frame encode/decode list. A completeness guard asserting that every
entry in `MESSAGES` appears in both parametrize lists would close the class
rather than the instance — the repo's own "climb one level of abstraction"
principle. Out of scope when it was found; still worth doing, and it is
independent of the clapboard.
