---
title: GUI CAN TRAFFIC panel rebuilt — per-bus (CAN1/CAN2/CAN3) traffic + health from the can-hub /profile + /link_status uplink
type: feature
date: 2026-07-11
status: resolved
phase: GUI + can-hub observability
related_entries:
  - 2026-07-11-gui-leg-setpoint-echo-poscmd
  - 2026-07-06-phase13-socketcan-decommission
  - 2026-07-06-can-node-parity-reconcile-decommission-precheck
  - 2026-07-05-canhub-marginal-can3-diagnosis
files_changed:
  - ros_ws/gui/js/can-traffic.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/theme.js
  - ros_ws/gui/index.html
  - ros_ws/gui/css/panels.css
  - tests/ros/test_gui_geometry.py
  - tools/probes/gui_dom_probe.py
  - tools/probes/gui_synthetic_stack.py
  - tools/probes/README.md
commits:
  - b91f0cc
subsystem:
  - gui
  - can
  - ros
tags:
  - observability
  - testing
---

## Summary

The GUI's **CAN TRAFFIC panel was DEAD** — its topic (`can_traffic`) lost its
only publisher when `can_node` was deleted in the 2026-07-06 SocketCAN
decommission ([[2026-07-06-phase13-socketcan-decommission]]; the decommission-precheck parity audit ([[2026-07-06-can-node-parity-reconcile-decommission-precheck]], Row 63)
accepted the loss, naming `/profile` as successor). The metric itself was also
obsolete: a single-bus count as seen by the Platform Teensy on the old shared
bus, while the CAN network is now **three buses through the can-hub Teensy**
(CAN1 = Ball Butler, CAN2 = catching cone, CAN3 = Jugglebot core). Rebuilt
(user-approved design) as a per-bus panel — rows CAN1/CAN2/CAN3 with msg/s,
kbit/s, util%, and a health dot, plus one uPlot msg/s chart with 30/60/120 s
window presets — fed entirely by **existing bridge topics** (`/profile` 1 Hz +
`/link_status` 10 Hz; zero backend work). Two hazards dominate the design: the
**firmware slot remapping** (wire slot `can1_*` = physical CAN3, `can2_*` =
physical CAN1 — labeling rows off field names would swap the two most important
buses; triple-guarded), and a review-confirmed **HIGH**: the bridge republishes
cached `/profile` + frozen bus healths forever after the can-hub uplink dies,
so naive message-arrival watchdogs would fabricate a live-looking panel during
the exact outage class the panel exists to surface — fixed in-session with an
explicit three-cause staleness state keyed on the in-band `bridge_link` signal.
New ES module `ros_ws/gui/js/can-traffic.js`; the old `panels.js` CAN section
is deleted and the dead `can_traffic` subscription removed (the topic now falls
through to the discovery spy).

## Motivation

Operator-facing bus observability regressed to nothing: the panel rendered but
never updated, and no current-topology publisher exists for its topic. Beyond
being dead, the old panel answered the wrong question — one aggregate count
from the Platform Teensy's vantage on the retired shared bus. The current
topology routes everything through the can-hub Teensy across three physical
buses with very different roles (CAN1 = Ball Butler, CAN2 = catching cone —
often physically disconnected, CAN3 = Jugglebot core: 6 legs + hand + Platform
Teensy), and the can-hub already profiles per-bus traffic and health onto the
UDP uplink. The GUI just had to consume what was already on the wire.

## Design

**Per-bus rows** CAN1/CAN2/CAN3, each with:

- a **clickable label** that toggles that bus's chart series — readouts keep
  updating while the series is hidden (explicit user ask);
- **msg/s**, **kbit/s**, **util%** readouts and a **health dot**.

One **uPlot msg/s chart** below, with **30/60/120 s** window presets — user
chose these over the originally-floated 5/10/30 s once the 1 Hz data cadence
was on the table (a 5 s window at 1 Hz is 6 points).

**Data sources (zero backend work):**

- `/profile` — DiagnosticStatus, 1 Hz, from `teensy_bridge_node`; KeyValues
  `can1_rx`/`can1_tx`/`can1_util_pct` + `can2_rx`/`can2_tx`/`can2_util_pct`.
- `/link_status` — 10 Hz; `bus1_health`/`bus2_health` + `bridge_link`
  (UP/LOST/NO_HEARTBEAT) + `heartbeat_age_ms`.

**Derivations (documented approximations):**

- msg/s = (rx + tx) / 1.0 s against the firmware's ~1 s LED-cadence window —
  there is **no window-duration KeyValue on the wire**, so the 1.0 s divisor is
  an accepted, documented approximation.
- kbit/s = msg/s × `CAN_BITS_PER_FRAME_APPROX` (111) / 1000 — the **same
  pre-bit-stuffing approximation the firmware itself uses**.
- util% rendered **verbatim** (firmware-computed over the exact window).
- Constants imported from the GENERATED `geometry-config.js`
  (`protocol_config.yaml` provenance) — not re-hardcoded.

**CAN2 (user decision):** rendered permanently greyed **"n/a — not on uplink"**
(tooltip: needs protocol v4 + a can-hub flash). Rows and chart series are
generated from a registry, so v4 lights CAN2 up as a two-field change.

## Discussion

### The slot trap (the load-bearing hazard of this panel)

The uplink's PROFILE slots do **not** match physical bus numbers: wire slot
`can1_*` = **physical CAN3** (Jugglebot core) and `can2_*` = **physical CAN1**
(Ball Butler) — a deliberate firmware remapping (`profiling.cpp:24-28`; the
health slot semantics originate in `Teensy_code_canbridge.ino:100-101`, with
the bridge's `_publish_link_status` a verbatim relay). Labeling rows off the
field names would silently **swap the two most important buses** on an
operator-facing panel. Triple-guarded:

1. a code comment at the `BUSES` registry in `can-traffic.js` citing the
   firmware origin lines;
2. a **KeyValue drift-pin test** in `tests/ros/test_gui_geometry.py` — producer
   keys ⊇ consumer keys, regex-extracted from both sides, comment-stripped;
3. an **end-to-end synthetic-stack probe assertion** publishing distinct
   per-slot values and asserting the CAN3 row shows the `can1_*` numbers.

The slot mapping itself was independently verified correct through all four
layers (`can_buses.cpp` → `profiling.cpp` → `udp_protocol` → bridge KeyValues).

### Adversarial review — 1 HIGH confirmed (independently by two lenses), fixed in-session

The bridge **republishes cached `/profile` at 1 Hz and frozen bus healths at
10 Hz FOREVER after the can-hub Teensy uplink dies**
(`teensy_bridge_node.py:922` set-only + `:2101-2107` unconditional; `:898` +
`:2045-2050`). So message-arrival watchdogs alone — the obvious GUI staleness
design — would fabricate a live-looking flat traffic line with green health
dots during the exact outage class the panel exists to surface.

**Why the fix is GUI-side, not bridge-side:** the bridge's republish semantics
serve other consumers, and the honest signal **already exists in-band** — the
same `/link_status` message carries `bridge_link` UP/LOST/NO_HEARTBEAT +
`heartbeat_age_ms`. No bridge change needed; the GUI just has to believe the
link state over message arrival.

**Fix:** an explicit three-cause staleness state `{profileStale, healthStale,
linkDown}` derived in one place (`applyStaleUI`) — `linkDown` drops cached
profile payloads, injects **exactly one NaN chart gap per episode** (latched,
re-armed at UP), and forces `--` readouts + UNKNOWN dots + a badge with a
cause-naming tooltip. `profileStale` alone no longer stomps the health dots —
they have their own source.

### CAN2 tri-state (user decision)

The PROFILE uplink has two slots; the cone slot is an acknowledged firmware
TODO. Adding it = `PROTOCOL_VERSION` 3→4 + wire-freeze re-pin + a lockstep
flash (the [18A] precedent). Chosen: permanently-greyed "n/a — not on uplink"
row now, registry-driven so v4 lights it up as a two-field change. Rejected:
**full-stack-now** (v3 was flashed only yesterday, and the cone bus is often
physically disconnected anyway) and **omitting the row** (the physical topology
should be visible — an operator should see that a third bus exists).

### 1 Hz cadence → window presets

The panel's data cadence is the 1 Hz `/profile` tick, so the originally-floated
5/10/30 s presets would render 6–31 points; user chose 30/60/120 s once that
was on the table. `RING_CAP` sized 132 (see review fixes) so the 120 s window
is fully backed.

### Approximation honesty

msg/s and kbit/s are documented approximations (no window-duration on the wire;
pre-bit-stuffing 111-bit frame estimate) rather than silently-precise-looking
numbers; util% is the one field the firmware computes over its exact window and
is passed through verbatim. The kbit/s approximation is deliberately the
firmware's own, so the two ends of the wire never disagree about methodology.

### Other review fixes (accepted and fixed in-session)

- **STALE badge light-mode contrast** ~2.2:1 → filled amber + dark text
  (~9.8:1, the `.state-badge.idle` pattern).
- **`RING_CAP` 120→132** — a 120 s window needs 121 samples, plus jitter
  headroom.
- **Isolated single sample between two outages was invisible** (points off) →
  uPlot `points.filter` shows markers only for isolated finite samples (filter
  support verified against the vendored build; 17/17 probe assertions).
- Provenance comment corrected to firmware-origin citations.
- **`nanGaps` hook EXPORTED from `telemetry-charts.js`** (single source, not
  copied into `can-traffic.js`) — `tools/probes/uplot_nan_gap_probe.js` still
  21/21 after the export.

## Implementation

**New ES module** `ros_ws/gui/js/can-traffic.js`: the `BUSES` registry (slot →
physical-bus mapping with firmware citations, CAN2 tri-state flag), `/profile`
+ `/link_status` consumers, the derivations, `applyStaleUI` three-cause
staleness, the uPlot msg/s chart (imports the shared `nanGaps` hook; NaN-gap
latch per link-down episode; `points.filter` for isolated samples), window
presets, and `rebuildCanChart` for theme swaps.

**GUI wiring**: `index.html` panel markup; `css/panels.css` per-bus rows,
health dots, badge styles (incl. the light-mode amber STALE fix); `main.js`
module hookup + dead `can_traffic` subscription removed (the topic falls
through to the discovery spy); `panels.js` old CAN section deleted;
`telemetry-charts.js` exports `nanGaps`; `theme.js` calls `rebuildCanChart()`
on theme change.

**Tests**: `tests/ros/test_gui_geometry.py` +2 drift-pin tests (producer
KeyValues ⊇ consumer keys, both sides regex-extracted and comment-stripped).

**Probe harness (NEW, reusable — `tools/probes/` rule)**:
`tools/probes/gui_dom_probe.py` + `tools/probes/gui_synthetic_stack.py` — a
synthetic-stack end-to-end harness (real rosbridge + scripted rclpy fake stack
+ headless-chromium CDP DOM assertions), including the keystone slot-mapping
assertion and a can-linkdown stage pinning the HIGH fix. Registered in
`tools/probes/README.md`.

## Verification

- **GUI geometry / drift-pin suite** (`pytest tests/ros/test_gui_geometry.py -q`,
  run 2026-07-11): **48 passed** (46 pre-existing + 2 new drift-pin tests).
- **GUI JS syntax**: `node --input-type=module --check` on all modified/new JS —
  clean.
- **uPlot gaps-hook probe** (`node tools/probes/uplot_nan_gap_probe.js`, run
  2026-07-11): **21/21 assertions** after the `nanGaps` export refactor.
- **Synthetic-stack DOM probe** (`python3 tools/probes/gui_dom_probe.py
  --scenario scenario1`, run 2026-07-11, three green runs — twice consecutively
  plus once after the audit's harness fixes): **17/17 assertions PASS every
  run, teardown verified clean** — includes the keystone
  slot-mapping assertion (distinct per-slot values; CAN3 row must show the
  can1_* numbers) and the `can-linkdown` stage pinning the HIGH fix (fresh
  cached messages with `bridge_link=LOST` ⇒ `--` readouts, UNKNOWN dots,
  cause-naming badge tooltip). An earlier run against the pre-fix fixture
  (12/14) correctly exposed the fixture's missing always-present `bridge_link`
  key — the fixture, not the GUI, was wrong; it now models the real bridge.
- **Full suite** (`pytest tests/ -q`, run 2026-07-11): **2472 passed, 1 xfailed
  in 582.61 s** (+3 vs the sibling entry's 2469: two drift-pin tests + the
  parametrized EXPECTED_FILES case for `can-traffic.js`). Post-suite audit
  fixes touched only `tools/probes/*` and markdown — outside `pytest tests/`
  collection scope — so the run remains valid for the committed code.

## Outcome

The CAN TRAFFIC panel is live again, on the current three-bus topology, with
zero backend changes — and it is honest under the failure mode it exists to
observe: a can-hub uplink death now renders as a named outage (gap, `--`,
UNKNOWN dots, cause tooltip) instead of a fabricated flat line. The slot trap
is pinned at three layers so a future firmware or bridge rename cannot silently
swap bus labels. CAN2 lights up as a two-field change when protocol v4 adds the
cone slot. Working-tree complete; DOM probe green twice (see Verification); commit +
final full-suite run pending (one placeholder in Verification).

## Related

- [[2026-07-11-gui-leg-setpoint-echo-poscmd]] — sibling entry from the same
  arc; source of the shared `nanGaps` hook and the uPlot probe this change
  re-validated.
- [[2026-07-06-phase13-socketcan-decommission]] — deleted `can_node`, orphaning
  `can_traffic`.
- [[2026-07-06-can-node-parity-reconcile-decommission-precheck]] — Row 63 is
  the parity-audit acceptance of the `can_traffic` loss, naming `/profile`
  (`can1_rx`/`can1_util_pct`) as the successor this panel now consumes.
- [[2026-07-05-canhub-marginal-can3-diagnosis]] — context for reading the new
  panel: the still-flying 0x7DF Platform-Teensy TRAFFIC_REPORT (2 Hz
  `bad_axis`, benign, accepted residue) is **not** resurrected by this panel —
  it simply counts toward CAN3 traffic.
- Memory: `project_canhub_tier2_validated` — protocol v3 flash state and the
  lockstep-flash constraint behind the CAN2 tri-state decision.
