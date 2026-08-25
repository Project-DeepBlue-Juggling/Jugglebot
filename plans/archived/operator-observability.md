---
title: Operator observability quartet — chart units, UDP message rates, ODrive error propagation, QTM calibrate gate
created: 2026-08-21
status: completed
completed: 2026-08-25
archived: 2026-08-25
owner: harrison
last_updated: 2026-08-25
related_logbook:
  - 2026-08-21-operator-observability-plan.md
  - 2026-07-11-gui-can-traffic-per-bus-panel.md   # the CAN panel — the end-to-end template for F2
  - 2026-07-12-gui-bb-stale-calibrated.md          # where calibration truth may and may not live — bounds F4's layering
  - 2026-08-10-tilt-cal-c0-blockers-level-noise-and-leg0-spinout.md   # the spinout only a bag autopsy could name — F3's motivating incident
  - 2026-05-18-mocap-qtm-connection-logging.md     # the QTM connection state machine F4 reads from
related_config:
  - config/hardware_config.yaml → teensy_trajectory.hand_spool_radius_m / linear_gain_factor   # hand mm/rev source
  - config/hardware_config.yaml → ball_butler_trajectory.hand_spool_radius_m / linear_gain_factor   # BB hand mm/rev source
  - config/hardware_config.yaml → mocap.qtm_host / qtm_port
related_code:
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/panels.js
  - teensy_link/client.py::LinkStats
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/can/odrive.py::error_names
---

# Operator observability quartet

> **Status 2026-08-25: ARCHIVED — completed.** All four features shipped
> (F3 `68da188`, F4 `56424c9` + Phase-A audit fixes `4c33a90`, F1 `cddd670`,
> F2 `f59eb9b` + Phase-B audit fixes `395b7dd`; both phase audits done, both
> `--full` closures green — Phase A 2026-08-22 total 571 s, Phase B 2026-08-22
> total 541 s, RESULT: PASS each) and **merged to `mvp-trajectory-bringup` as
> `b705a21`** on 2026-08-25, colcon-built. The merged quartet was
> **bench-validated at the 2026-08-25 sitting** — owner verdict: all four
> features landed cleanly. The § 8 operator bench checklist has been run and
> every item dispositioned; see **Archival note — 2026-08-25** at the foot of
> this file for what shipped, where it lives, and where the residue was
> re-homed.

Four owner-requested features, one branch. All four are **pure software — no
firmware change, no Teensy flash, no PROTOCOL_VERSION bump anywhere**. Ordered
by operational value: F3 (ODrive errors) → F4 (QTM calibrate gate) → F1 (chart
units) → F2 (UDP rates).

> **Line-number anchors** below were surveyed 2026-08-21 on
> `mvp-trajectory-bringup@6d226f6`; this branch is based at `5de4a1c` (one merge
> earlier), so expect small drift. Treat anchors as locators, not gospel —
> re-grep before editing.

## 0. Branch ground rules

- **Worktree** `~/Desktop/Jugglebot-obs`, branch `2026-08_operator-observability`,
  based at `5de4a1c` — the commit that closed the hand-tuning/throw-envelope
  unit, immediately before the ILC fold-in merges (`e329ffb`/`6d226f6`). Chosen
  so this work is fully independent of the live ILC session in the main tree.
  Nightly GREEN 2026-08-21 tested `d0b3cca`; the delta to `5de4a1c` is docs
  plus two full-suite-gated sim commits.
- **All work happens in this worktree.** Nothing is written to
  `~/Desktop/Jugglebot` (the ILC session owns it) until merge-back.
- **Merge-back** to `mvp-trajectory-bringup` is an owner-timed event after both
  pairs land. Expected conflicts: `plans/active/INDEX.md`, `logbook/INDEX.md`
  (trivial, additive), and nothing else — the ILC arc lives in `controller/`,
  `sim/` and toss files this plan does not touch, except `config/generate_config.py`
  (F1 adds constants; envelope changes are already in our base).
- **Deploy**: F1 + F2-frontend are static JS (browser reload). F3, F4 and F2's
  `/udp_diag` publisher touch `ros_ws` →
  `colcon build --packages-select jugglebot` + relaunch on merge-back. F2's
  `teensy_link/client.py` change needs **relaunch only** — `teensy_link` runs
  from the live tree via the launch files' PYTHONPATH injection, deliberately
  not installed (CLAUDE.md § architectural boundaries).
  No hardware sitting is required to land this plan
  software-complete; § 8 lists the operator validation checks for the next
  powered session.
- **Phases for gating purposes**: Phase A = F3 + F4 (ROS-side pair),
  Phase B = F1 + F2 (GUI-side pair). Each phase closes with an independent
  audit over the pair's combined diff, audit fixes applied, and one
  `./run_tests.sh --full` run. Per-feature commits run the default gate.

## 1. Scope decisions (owner, 2026-08-21)

1. **F2 counts the UDP link only** (Jetson ↔ can-bridge Teensy). ZMQ channel
   rates are a follow-up (§ 8) — different process, no counter infrastructure.
2. **F3 ships the full ladder** — launch shell AND GUI event log.
3. **F4 ships gate + calibration hardening** (arc-span floor, mid-sweep
   invalidation, collection timeout) — the connect-time gate alone does not
   close the partial-data garbage path.
4. **BB pitch charts display absolute barrel degrees** (`deg = 90 + 360·rev`),
   matching the BB panel's `pitch_deg` and the configured 12–90° range.
5. The `main.js:385` hand-factor bug (leg factor applied to the hand, ~2.2×)
   is fixed inside F1.

---

## 2. F3 — ODrive error propagation

**Current state.** The full 32-bit `active_errors` + `disarm_reason` bitmasks
already flow end-to-end: ODrive `Get_Error` (0x03) cyclic @ 50 Hz → Teensy axis
cache (`can_buses.cpp:194-198`, BB at `:305-315`) → UDP `DIAGNOSTIC` on-change
(`telemetry.cpp:54-104`) → `teensy_bridge_node._latest_diag` (`:1970`) →
`/robot_state.motor_states[i].active_errors/disarm_reason` (`:2882`, BB `:2940`)
→ rosbags and the browser. Detail is destroyed on the Jetson only:
`:3148/:3205` collapse to booleans + fixed strings; `_guard_fault_leg_hint`
(`:4249-4289`) reads the mask and prints only `(leg 3)`; the GUI emits a bare
"ODrive Error" row on a boolean rising edge (`main.js:316-346`). The decode
table `ERROR_CODES` / `error_names()` already exists at
`jugglebot/can/odrive.py:83-107, 341-356` (unknown bits → one aggregate
`UNKNOWN(0x…)`) with **zero consumers in the launched node graph** — four
`tests/hardware/` harnesses do import it (`bench_leg_sysid.py:173`,
`torque_step_test.py:76`, `cogging_bench_test.py:97`, `tilt_cal_grid.py:238`)
and three more hand-vendor their own copy (`tilt_cal_grid.py:241`, pinned by
`tests/motion/test_tilt_cal_grid.py:636`; `supported_platform_test.py:192` and
`single_leg_test.py:75`, both unpinned) — the deleted `can_node.py`
(`7c7f61b^`, ~lines 419-447) is the reference implementation for what we are
restoring, and its throttle scaffolding survives unused in
`can/motor_state.py:76-78, 245-249`. Known parity regression:
`ros_ws/docs/can-node-teensy-parity.md` rows 35 and 62.

**Changes (all Jetson-side):**

- **C1 — guard-latch decode** (`teensy_bridge_node.py`): `_guard_fault_leg_hint`
  decodes both masks via `error_names()` and covers **all axes present in
  `_latest_diag`**, not `range(_NUM_LEGS)`. Axis labels `leg0..leg5, hand,
  bb_pitch, bb_hand`. Output shape:
  `(leg 3: active=[SPINOUT_DETECTED] disarm=[] 0x4000000/0x0)`.
  Existing tests: `tests/ros/test_teensy_bridge_node_read.py:477-598`.
- **C2 — per-axis throttled error logging**: new method on the bridge, called
  from the existing `_publish_link_status` 10 Hz timer (`:1760`) — not the
  100 Hz `_publish_robot_state` (`:1758`), and not `_health_check`, which is
  1 Hz (`:1776`) — porting the `can_node` loop: for each axis, for each set
  bit in either mask,
  `get_logger().error("ODrive error on <label>: active=[...] disarm=[...]")`
  throttled 10 s per (axis, code) via
  `MotorStateTracker.last_error_log_times()` (`can/motor_state.py:245`,
  throttle constant `:69`). NB the tracker already at `:1433` is bound to
  `self._versions` (the firmware-version handshake) — instantiate a second one
  as `self._error_log` rather than overloading that name. Covers axes 0–8.
- **C3 — decoded names in `robot_state.error[]`** (`:3205-3219`): append
  per-axis decoded lines **only when `has_fatal_odrive_error` is already
  true** — `orchestrator_node.py:291` force-FAULTs on any non-empty `error[]`,
  so decoded names for non-fatal bits must never create a new FAULT path.
- **C4 — orchestrator FAULT-entry logging**: log `ctx.errors` once on entry to
  FAULT (`orchestrator_node.py` FAULT dispatch or `state_machine.py`
  FaultHandler `on_enter`), so the shell shows *why* next to
  `[SM] Entering FAULT`.
- **C5 — GUI decode table**: new `ros_ws/gui/js/odrive-errors.js` —
  hand-mirrored `ERROR_CODES` map + a `errorNames(mask)` helper (unknown bits →
  hex, same contract as Python). **Drift-pinned** by a new test in
  `tests/ros/test_gui_geometry.py` that regex-parses the JS map and asserts
  equality with `jugglebot.can.odrive.ERROR_CODES` (helpers `_extract_js_*`
  at `:44-63` are the template). Extend the same pin to the two currently
  unpinned vendored copies (`tests/hardware/supported_platform_test.py:192`,
  `tests/hardware/single_leg_test.py:75`) so all five copies of the one table
  sit behind pins — otherwise F3 ships a fifth copy behind a single pin.
- **C6 — GUI event log per-axis rows** (`main.js:316-346`): replace the
  boolean rising-edge with a per-axis change-detector over
  `(active_errors, disarm_reason)` tuples across `motor_states`. On any change
  to a non-zero state emit
  `{type: FAULT, label: 'ODRIVE ERROR: leg 3 SPINOUT_DETECTED', detail: 'active=SPINOUT_DETECTED (0x4000000); disarm=—'}`
  (the trailing `…` is appended only when the chosen mask decodes to MORE THAN
  ONE name — a single active `SPINOUT_DETECTED` gets no ellipsis;
  `main.js:409-410`).
  Keep the existing three boolean-flag events (they cover CAN/undervoltage).
  Handle the 7→9→7 axis-count shrink the same way the fault dots do
  (`main.js:351-365`).
- **C7 — parity-doc reconciliation**: update
  `ros_ws/docs/can-node-teensy-parity.md` rows 35 and 62 in the F3 commit —
  nothing pins this doc, so a missed edit is silent. Row 62 →
  ported+validated once C3 lands. Row 35 → PARTIAL still, restated as
  "decode + throttled per-code logging ported; the explicit disarmed-axes
  list (`7c7f61b^:can_node.py:1687`) stays dropped" — unless C3 also restores
  that list, in which case row 35 closes too (implementer's call; either way
  the row must state the truth).

**Caveats (non-negotiable):** decode **both** masks everywhere — the 2026-08-10
spinout had `active_errors == 0` and the truth in `disarm_reason`; `disarm_reason`
is sticky until CLEAR_ERRORS, `active_errors` self-heals. Deferred, not in
scope: an `odrive_errors` KeyValue on `/link_status` (§ 8).

**Done means:** a bench-inducible ODrive error appears (a) decoded by name +
axis in the launch shell within its throttle window, (b) as a decoded per-axis
row in the GUI event log, (c) named in `robot_state.error[]` when fatal — plus
the drift-pin tests and updated guard-hint tests green, and the parity doc
rows restated truthfully (C7).

## 3. F4 — QTM-gated BB calibration + hardening

**Current state.** The calibrate *command* path (GUI button `panels.js:438-460`
/ HOMING `orchestrator_node.py:432-442` → `bb/calibrate` Trigger →
`teensy_bridge_node._svc_bb_calibrate:6607-6622` → RPC → BB sweeps) and the
*data* path (QTM → `mocap_node` accumulates while `bb/heartbeat` reports
CALIBRATING → `bb_calibration.run_calibration` on the state-exit edge) share no
edge. The service gates only on `_bb_present()` (`:6366-6375`). There is **no
ROS-observable QTM-connected signal** — `mocap_interface.is_receiving()`
(`mocap_interface.py:291-294`) never leaves the process. With QTM down the
sweep runs blind and fails cleanly afterwards; with a **mid-sweep dropout** the
solver can accept a 0.25 s arc (min_points=50 @ 200 Hz) and publish plausible
garbage that `ball_butler_node` then uses for every throw; a heartbeat frozen
at CALIBRATING wedges collection forever (no timeout, `mocap_node.py:213-243`).

**Changes:**

- **Q1 — `mocap/status` publisher** (`mocap_node.py`): DiagnosticStatus @ 5 Hz
  with KeyValues `qtm_receiving` (bool), `bb_markers_visible` (count of
  non-NaN BB fiducials in the latest frame), `marker3_visible` (bool — the
  solver hard-requires index 2), `aligned`, `qtm_synced`. Do **not** overload
  `bb/markers` (200 Hz) or `/qtm_clock_offset_sec` (goes silent on disconnect).
- **Q2 — service gate** (`teensy_bridge_node.py`): subscribe `mocap/status`;
  `_qtm_ready()` = last status msg age ≤ 1.0 s AND `qtm_receiving` AND
  `bb_markers_visible ≥ 3` AND `marker3_visible`. Gate at the head of
  `_svc_bb_calibrate`, **after** the existing `_bb_present` silent-success
  branch (BB-absent stays a skip — pinned by
  `tests/ros/test_teensy_bridge_node_bb.py:186-207`). Refuse fail-closed with
  distinct codes, e.g.
  `success=False, message='QTM_STALE: no mocap status for 3.2 s — calibration refused'`
  vs `'BB_MARKERS_NOT_VISIBLE: 1/5 visible (need ≥3 incl. Marker 3)'` —
  distinct codes per the toss-ladder rationale (`toss_sequencer.py:1039-1048`).
- **Q3 — HOMING skips, never faults** (`orchestrator_node.py:432-442`): the
  orchestrator subscribes `mocap/status` and, when `_qtm_ready()` fails,
  mirrors the existing service-not-ready branch verbatim: WARN
  `"QTM not delivering BB markers — skipping calibration"`,
  `ctx.bb_calibration_skipped = True`, `ctx.operation_result = True`. A
  refusal must never reach `HomingHandler`'s
  `operation_result is False → FAULT` edge (`state_machine.py:357-358`).
  Safety net: `tests/ros/test_orchestrator_conduit_integration.py`.
- **Q4 — GUI button**: extend the enable rule (`panels.js:586`) to also
  disable Calibrate when mocap is disconnected (state already tracked,
  `main.js:505-514`), with the reason in the button tooltip.
- **Q5 — hardening** (`bb_calibration.py` + `mocap_node.py`):
  (a) **arc-span floor** — `run_calibration` refuses when the accumulated yaw
  span < `MIN_ARC_DEG` (module constant, **default 20°** — conservative: the
  partial-data failure mode is ≪10°; ⚠ the true sweep span is BB-firmware-owned
  and not in this repo, so the owner confirms/raises this after the first
  hardware calibrate — flagged in § 8);
  (b) **mid-sweep invalidation** — if `is_receiving()` goes false at any point
  while `_calibrating`, latch invalid and publish
  `success=False, 'QTM_DROPOUT_MID_SWEEP'`;
  (c) **collection timeout** — wall-clock cap (60 s) on the CALIBRATING window
  → finalize as `'CALIBRATION_TIMEOUT'` and unlatch `_calibrating`.
- **Q6 — choreography**: regenerate `ros_ws/docs/choreography.md`
  (`python tools/gen_choreography_map.py`) — new pub/subs otherwise redden
  `tests/ros/test_choreography_map.py`.
- **Amendment (2026-08-22, adjudicated at F4 landing)**: the `_qtm_ready()`
  predicate and both thresholds live in ONE shared pure module,
  `jugglebot/mocap_status.py`, consumed by bridge and orchestrator (each keeps
  a thin `_qtm_ready()` over its own cached snapshot). Root cause: two nodes
  independently evaluating the same predicate drift silently and
  asymmetrically — raise the marker floor in one place only and HOMING starts
  skipping while the GUI button still dispatches, or the reverse, which lands
  on a bridge refusal that HOMING would FAULT on. One enforcement point
  supersedes the per-node phrasing in Q2/Q3 above. (Topic name stays a string
  literal at the call sites — `gen_choreography_map.py` resolves endpoints by
  AST and refuses attribute expressions.)

**Tests:** first-ever `tests/ros/test_mocap_node.py` (Q1 publisher, Q5b/c —
mocked ROS per `tests/ros/conftest.py`); refusal + ordering tests in
`test_teensy_bridge_node_bb.py`; skip-not-fault test in
`test_orchestrator_node.py` beside `test_bb_calibrate_skips_when_not_ready`;
arc-span cases in `ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py`
(note its case 9 "partial visibility still succeeds" may need re-scoping under
the floor).

**Done means:** with QTM absent, the GUI button is disabled, a forced service
call refuses with a named code and **no RPC is dispatched**, HOMING completes
with a WARN skip; the three hardening refusals each have a test; choreography
map regenerated.

## 4. F1 — chart physical units

**Current state.** Nine independent uPlot charts
(`ros_ws/gui/js/telemetry-charts.js`, ~1983 lines) plot raw motor revs / rev/s
for all axes; `/robot_state`, `/leg_setpoint_echo`, `/hand_telemetry` all carry
revs. Legs already have per-leg `MM_TO_REV` in generated
`ros_ws/gui/js/geometry-config.js`; hand/BB-hand gains exist only in Python
derived config (`TEENSY_LINEAR_GAIN` = 31.617 rev/m, `BB_LINEAR_GAIN` = 30.319
rev/m, `config/generate_config.py:589-591, 627-629`); BB pitch is affine:
`deg = 90 + 360·rev`, `deg/s = 360·rev/s` (docstring at
`teensy_bridge_node.py:2324`; firmware `PitchAxis.h`, not in this repo).

**Changes:**

- **U1 — codegen** (`config/generate_config.py`, `generate_gui_js` near
  `:864`): emit `HAND_MM_PER_REV = 1000/TEENSY_LINEAR_GAIN`,
  `BB_HAND_MM_PER_REV = 1000/BB_LINEAR_GAIN`, `BB_PITCH_DEG_PER_REV = 360.0`,
  `BB_PITCH_DEG_OFFSET = 90.0`. Run `python config/generate_config.py`; commit
  both generated copies (`config/generated/` + `ros_ws/gui/js/`).
- **U2 — per-chart unit table** (`telemetry-charts.js`): `axisUnitsFor(chartIdx)`
  beside `highlightTargetFor` (`:1919-1925`) returning
  `{posUnit, posScale, posOffset, velUnit, velScale, decimals, padFloor}`:
  legs `1/MM_TO_REV[i]` mm; hand `HAND_MM_PER_REV`; BB pitch
  `360·rev + 90` deg / `360` deg/s; BB hand mm.
- **U3 — convert at ingestion** (`onTelemetryData`, `:1068-1103`):
  `pos_measured`, `vel_measured` **and `pos_commanded`** with the identical
  per-axis mapping (mismatch = fake constant tracking error). One stroke fixes
  axes, callouts, Δ pills, y-ranges and CSV. `NaN` survives conversion, so gap
  semantics hold.
- **U4 — per-chart labels/format**: thread `chartIdx` through `buildUPlotOpts`
  (`:774`, sole call site `:974`), the axis label (`:847`), and
  `formatCalloutValue` (`:1860`, used `:1597`, `:1906`). Per-unit pad floors
  replacing the hardcoded 0.5 rev (`:809`): mm 2.0, deg 1.0, mm/s 5.0,
  deg/s 5.0. Decimals: mm/mm/s 1 dp, deg/deg/s 2 dp. Widen y-axis `size` for
  3-digit mm labels. Update `SIGNAL_TOOLTIPS` (`:85-95`) and the
  click-to-copy/"raw value" wording (`:1832`, `charts.css:339`).
- **U5 — CSV headers** carry units (`leg_0.pos_measured_mm`,
  `bb_pitch.pos_measured_deg`, `:1661-1667`); no in-repo consumer exists.
- **U6 — `main.js:385` bugfix**: hand 3D extension uses `HAND_MM_PER_REV`,
  not `MM_TO_REV[0]`.
- **Constraint**: the committed probe `tools/probes/uplot_nan_gap_probe.js`
  string-pins `function nanGaps(...)` and `gaps: nanGaps,` — keep both
  literals verbatim; `can-traffic.js:34` imports `nanGaps`.
- **Non-goal**: `panels.js` motor-grid / tracking-error units stay as-is
  (legs mm, hand rev) — separate decision, § 8.

**Tests:** new class in `tests/ros/test_gui_geometry.py` pinning the four new
JS constants against `hardware_config.yaml`-derived values (template
`TestBallButlerGeometry:245-270`).

**Done means:** every chart shows its physical unit on axis, callouts, Δ pills
and CSV; commanded and measured convert identically; constants drift-pinned.

## 5. F2 — UDP message-rate view on the topics panel

**Current state.** The "ROS2 Topics" panel is pure frontend
(`panels.js:940-1187`, 1 Hz repaint from `main.js:129`) counting
browser-received messages; spy subscriptions are throttled to 200 ms
(`ros-bridge.js:350`), so its Hz column saturates at 5 Hz for spied topics —
display-side only, no bearing on true rates. The UDP link (ports 5005/5006) has
**21 message types** (3 downlink, 18 uplink, RPC ids bidirectional — inventory
in `config/generate_udp_protocol.py:159-196`); it is the only UDP in the
system. `teensy_link.LinkStats` already counts **RX per type**
(`client.py:527-529`) and per-type seq gaps (`:539-549`); nothing publishes
them; aggregates already ride `/link_status` (`teensy_bridge_node.py:4685-4688`).
The CAN-traffic panel (`can-traffic.js`) is the proven counter→topic→GUI
template, including its three-cause staleness honesty.

**Changes:**

- **R1 — TX per-type counters** (`teensy_link/client.py`): add
  `tx_count_by_type` to `LinkStats` + `snapshot()`; one increment in each of
  `send_stream`/`send_rpc`/`send_to` (`:335, :344, :358` — `msg_type` already
  in hand). **Pre-seed both per-type dicts with all `MsgType` keys at
  construction** — kills the unlocked-dict snapshot race and renders
  never-seen types as honest 0 rows. Do not rename `rx_count_by_type`
  (used as a sync barrier by two bridge tests).
- **R2 — `/udp_diag` publisher** (`teensy_bridge_node.py`): DiagnosticStatus
  @ 1 Hz modeled on `_publish_profile` (`:4781-4820`): KeyValues
  `rx_<TYPE_NAME>` / `tx_<TYPE_NAME>` (cumulative counters, `MsgType` enum
  names) + `gap_<TYPE_NAME>` (nonzero only) + aggregates
  (`rx_frames/tx_frames/crc_errors/decode_errors/drain_capped`). Counters,
  not rates — the GUI differentiates; counters are the honest rosbag artifact.
- **R3 — GUI module** `ros_ws/gui/js/udp-traffic.js` (do not grow
  `panels.js`): subscribes via a `main.js` route beside `profile`/`link_status`
  (`:276-279`, `:474-483`); stores counter samples with arrival timestamps;
  rate = Δcount/Δt over the selected window; column header **`msg/s`** (true
  rate), never "Hz", with a tooltip naming the difference from the ROS view's
  received-rate semantics. Window presets for UDP mode: 5/10/30/60 s (1 Hz
  cadence makes a 1 s window one sample — the CAN panel's lesson).
- **R4 — panel toggle** (`index.html:51-57` + the new module): a two-state
  `<button>` segmented control in the `.panel-header` (a button is inert to
  `initCollapsiblePanels`' interactive-element guard, `panels.js:409-410`);
  sibling `#udp-table-container`; mode persisted in localStorage beside
  `jugglebot-hidden-topics`; `#panel-topics` stays the single panel so
  `theme.css` flex/scroll rules hold.
- **R5 — staleness honesty**: gate the UDP view on `bridge_link` from
  `/link_status` (already routed in `main.js:479-483`), three-cause style from
  `can-traffic.js` — a dead uplink must render as "link down", not a plausible
  table of zeros.
- **R6 — contract + choreography**: drift-pin producer⊇consumer KeyValues
  (copy `TestCanTrafficKeyValueContract`, `test_gui_geometry.py:340-395`);
  add `js/udp-traffic.js` to `EXPECTED_FILES` (`:215-240`); TX-counting test
  beside `tests/teensy_link/test_client.py:104-112`; regenerate
  `choreography.md` for `/udp_diag`.

**Done means:** toggling the panel shows live per-type `msg/s` both directions
(TELEMETRY ~100, HEARTBEAT 10, SETPOINT 40-when-streaming…), gaps surfaced,
link-down rendered honestly, contracts pinned.

## 6. Verification & process

- Per-feature commit: `./run_tests.sh` (default gate) + a short-form logbook
  entry (+ `logbook/INDEX.md` row) in the same commit, `Logbook-Entry:` trailer.
- Phase closure (A after F4, B after F2): independent audit over the pair's
  combined diff → audit fixes applied and committed → `./run_tests.sh --full`.
  Audits are deliberately **two total** (owner asked for sparing usage).
- No `config/*.yaml` edits are planned. F1's U1 does edit
  `config/generate_config.py`, so that commit runs
  `python config/generate_config.py` and stages both regenerated
  `geometry-config.js` copies (`config/generated/` and `ros_ws/gui/js/`, the
  only outputs `generate_gui_js` writes). If a YAML edit does appear:
  edit YAML → regenerate → stage the regenerated artifacts → run tests → commit.
- No `controller/` or `sim/` files are touched, so the default gate suffices
  between phase closures.
- Push after every commit (`git push -u origin 2026-08_operator-observability`
  first time), with the standing fetch-first check.

## 7. Orchestration

Owner-directed: implementation via **Workflow with Opus agents** working in
`~/Desktop/Jugglebot-obs` (venv `~/Desktop/PDJ_venv/venv`). Shape: one small
workflow per feature (implement agent, tests included in its brief), the main
session gating and committing between workflows; one audit workflow per phase
closure (reporter → fixer). The main session does not implement — it briefs
from this plan's sections, adjudicates diffs, runs/verifies gates, commits.

## 8. Deferred / follow-ups

- ZMQ endpoint rates (six sockets, `motion/ipc.py:66-81`) — needs net-new
  counting in another process; revisit after F2 proves the panel.
- `odrive_errors` KeyValue on `/link_status` for the minimap.
- `panels.js` unit consistency (motor grid hand → mm, velocities → physical).
- `config/generate_config.py` delivers rendered headers into the **sibling
  `~/Desktop/BallButler` checkout** with no way to suppress it — both 2026-08-22
  sessions (this one at F1, the catch-robustness one independently) tripped
  over the surprise external write. Owner decision wanted: a `--no-external`
  flag or an explicit delivery step.
- `tests/hardware/supported_platform_test.py`'s vendored `error_names()` still
  drops unknown bits (its table is now drift-pinned by F3, but the function
  diverges from the `UNKNOWN(0x…)` contract) — align when next touching that
  harness.
- **Owner hardware checks, next powered session**: (1) induce/observe an ODrive
  error → confirm shell + event-log decode; (2) with QTM off, press Calibrate →
  confirm refusal code, then run HOMING → confirm WARN skip, no FAULT;
  (3) eyeball chart magnitudes (legs ~0–300 mm, BB pitch 12–90°);
  (3b) toggle the topics panel to UDP mode — expect TELEMETRY ~100 msg/s,
  HEARTBEAT 10 both directions, SETPOINT ~40 only while streaming; then check
  the LINK DOWN banner renders (not a table of zeros) when the bridge drops.
  The F2 GUI half was never exercised in a real browser (the DOM probe needs
  an empty ROS graph this box didn't have) — the CSS sticky-banner/scroll
  interaction is the part most worth the eyeball;
  (4) confirm/raise `MIN_ARC_DEG` from the **BB yaw span** a real calibrate
  logs (`mocap_node`: "BB yaw span swept: N°") — that is the encoder-derived
  primary gate; the per-marker `arc_span` lines beside it are noise-inflated
  and are NOT the number to read.

## Archival note — 2026-08-25

Archived **completed**. All four features shipped, merged to
`mvp-trajectory-bringup` as **`b705a21`**, and bench-validated at the
2026-08-25 sitting — owner verdict: all four features landed cleanly.

### What shipped, and where it lives

| Feature | Where it lives now |
|---------|--------------------|
| **F3** — ODrive errors decoded by name + axis | `teensy_bridge_node._log_odrive_errors` and `_guard_fault_leg_hint`, plus `ros_ws/gui/js/odrive-errors.js`. Includes the 2026-08-25 owner-requested headline refinement (`ODRIVE ERROR: <axis> <NAME>`) — `logbook/2026-08-25-odrive-event-headline.md` |
| **F4** — QTM-gated BB calibrate | the shared `mocap_status.py` predicate, the `mocap/status` topic, and the `bb_calibration.py` hardening (arc-span floor, mid-sweep invalidation, collection timeout) |
| **F1** — charts in physical units | `axisUnitsFor` in `ros_ws/gui/js/telemetry-charts.js`, fed by the four generated constants |
| **F2** — true per-type UDP rates | the 1 Hz `/udp_diag` publisher, `ros_ws/gui/js/udp-traffic.js`, and `LinkStats.tx_count_by_type` |

### § 8 dispositions

- **Bench checks 1–3 passed** at the 2026-08-25 sitting.
- **3b exposed the per-type Gaps artifact.** Root cause **pre-dates F2 by
  eleven weeks**: `client.py::_track_seq` (`2b605af`) tracks last-seq per
  message *type*, but the wire carries ONE shared sequence counter per
  (socket, direction), so any interleaved frame of another type scores a gap.
  Diagnosis in `logbook/2026-08-25-udp-gap-column-artifact.md`; the fix is
  scoped as P1–P4 of `plans/active/udp-channel-health.md`; and the misleading
  column has **already been removed from the panel** (close-out,
  `logbook/2026-08-25-observability-closeout-fixes.md`).
- **Item 4 RESOLVED.** The first real calibrate swept **118.8°**, so
  `MIN_ARC_DEG` was raised **20° → 60°** the same day.
  `MIN_MARKER_RADIUS_MM` stays **20**, confirmed by that passing calibrate.
  (Only the sweep gate moved: the per-marker INCLUSION threshold was split off
  as `MIN_MARKER_ARC_DEG = 20` — unchanged behaviour — because a marker
  occluded inside a sweep that legally cleared 60° fits a short arc and must
  still be folded into the axis average.)
- **`generate_config --no-external` RESOLVED.** The flag landed 2026-08-25,
  the default is unchanged, and external writes are now announced with an
  `EXTERNAL:` line naming the absolute destination.
- **C5 count corrected.** The archival review found a **sixth** unpinned
  vendored table (`tests/hardware/free_platform_test.py`), and the fix closed
  the whole vendored-decoder **class** rather than the one instance —
  `supported_platform_test.py`'s disarm-blindness and `single_leg_test.py`'s
  (disarm-blind *and* dropping unknown bits) went with it;
  `tests/hardware/tilt_cal_grid.py` was checked and **exonerated**. Result:
  **6/6 tables pinned, and the decoders pinned on behaviour**, not merely on
  their tables.

### Named but not scheduled (seen, not forgotten)

- ZMQ endpoint rates (six sockets, `motion/ipc.py`) — needs net-new counting
  in another process.
- `odrive_errors` as a KeyValue on `/link_status`, for the minimap.
- `panels.js` unit consistency (motor grid hand → mm, velocities → physical).

### Watch item, next bench run

`tests/hardware/supported_platform_test.py` B4 / `test_estop` **and**
`tests/hardware/single_leg_test.py` `test_estop` may now **fail loudly** if the
firmware records a disarm reason for a user-requested IDLE. Both report and
score it the same way: the named `fault_summary` is printed and folded into the
pass/fail terms, so it is a scored **FAIL**, not a raised `RuntimeError` that
aborts the run before the other criteria are reported. The faithful both-masks
gate was kept **deliberately** — judge from the printed `fault_summary` before
touching the criterion.
