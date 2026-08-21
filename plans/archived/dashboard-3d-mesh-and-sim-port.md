---
title: Dashboard 3D Mesh Upgrade + Sim Port
created: 2026-05-27
status: superseded   # Phase 0 shipped; Phases 1-5 never started (see Archival note)
completed: 2026-08-01
archived: 2026-08-01
---

# Dashboard 3D Mesh Upgrade + Sim Port

## Context

### Why this plan exists

The MuJoCo passive viewer (`sim/juggle_demo.py --viewer`) requires an X11
display, which makes it unusable over plain SSH from a Windows dev box —
the typical iteration loop. The existing browser dashboard
(`sim/juggle_demo.py --dashboard`, served at port 8082) renders 2-D
telemetry only; there's no way to see the platform or balls move.

A native 3-D Three.js + uPlot single-page app already exists at
[ros_ws/gui/](ros_ws/gui/) — the production hardware GUI. It runs in any
browser, uses zero-build ES modules from CDN (Three.js 0.170), and
renders the Stewart platform from auto-generated constants in
[ros_ws/gui/js/geometry-config.js](ros_ws/gui/js/geometry-config.js). The
prod GUI currently uses **primitive geometry only** (hex base, line legs,
hex platform, line hand axis) — the actual STL meshes used by MuJoCo
([sim/model/meshes/](sim/model/meshes/)) are unused on the GUI side.

This plan does two things, in order:

1. **Upgrade prod GUI** to optionally render the STL meshes that MuJoCo
   already uses, giving the hardware GUI a nicer-looking robot.
2. **Fork the upgraded prod GUI into `sim/viz/dashboard/`** so
   `sim/juggle_demo.py --dashboard` shows the same 3-D scene driven by
   the sim's SSE feed, plus rendered balls.

### What this plan achieves

- A browser-native 3-D view of the simulator that works over any SSH
  connection, no X server required.
- Visual parity between prod GUI and sim dashboard — same layout, same
  chart toolbar, same scene controls.
- STL meshes (`base.stl`, `platform.stl`, `leg_inner.stl`,
  `leg_outer.stl`, `hand.stl`) rendered in both UIs, with a wireframe
  fallback toggle.
- Two correctly-scaled ball spheres in the sim scene, driven by the
  existing two-ball trajectory broadcast (currently CSV-only).
- Sim-specific telemetry surfaced (catch count, throw-director phase,
  realtime rate, computed leg accelerations) without polluting the prod
  GUI.

### When to do this

The browser-native 3-D view is the blocker for productive remote
iteration on `sim/juggle_demo.py`; that motivates the work now. Phase 1
(prod STL upgrade) is independently valuable on hardware and can land
before any sim work — so the sim port automatically inherits the upgrade
when it forks.

---

## Architecture

### Current

```
sim/juggle_demo.py --dashboard
  -> sim/viz/dashboard/server.py (HTTP + SSE on :8082)
       -> static/index.html  (uPlot charts only)
       -> static/dashboard.js (consumes SSE /events)
       -> static/dashboard.css

ros_ws/gui/  (prod GUI, served by gui_server.py)
  -> Three.js viewer with primitive Stewart geometry
  -> uPlot 9-chart grid
  -> ROSLIB WebSocket to rosbridge:9090

sim/model/meshes/  (STL files — used by MuJoCo only)
  base.stl, platform.stl, leg_inner.stl, leg_outer.stl, hand.stl, ...
```

### Proposed

```
ros_ws/gui/                                          (PROD, Phase 1)
  -> js/mesh-renderer.js     (NEW — STL loader, leg-mount transforms)
  -> js/viewer.js            (small edit — wireframe/meshed toggle)
  -> Toggle in scene menu: "Wireframe" vs "Meshed"
  -> Meshes served from a new /meshes/* route in gui_server.py
       referencing sim/model/meshes/ (shared single-source-of-truth)

sim/viz/dashboard/                                   (SIM, Phases 2-5)
  static/  (forked from ros_ws/gui/ after Phase 1 lands)
    index.html              (ROS-only panels stripped)
    js/sim-bridge.js        (NEW — SSE -> events; replaces ros-bridge.js)
    js/ball-renderer.js     (NEW — two spheres parented to scene)
    js/main.js              (sim-fork of prod's main.js)
    js/{viewer,stewart-model,stewart-fk,telemetry-charts,
        panels,camera-presets,theme,geometry-config,
        mesh-renderer}.js   (straight copies from prod)
    css/                    (copied from prod)
    lib/uPlot.*             (copied; roslib.min.js dropped)
  server.py
    -> Serves the new static tree
    -> Streams an extended SSE record (balls, accels, throw phase)
    -> /meshes/* static route mirroring prod
```

### Branch flow

Phase 1 lands on `refactor` (touches `ros_ws/gui/` + `config/`). The sim
work happens in the existing worktree at `~/Desktop/Jugglebot-bb` on
`demo/bb-led-two-ball-juggle`, which merges `refactor` to pick up
Phase 1 before starting Phase 2. Plan document itself lives on
`refactor` and is merged forward.

### Single-source-of-truth notes

- Geometry constants: [config/hardware_config.yaml](config/hardware_config.yaml)
  is canonical. [config/generate_config.py](config/generate_config.py) emits
  `ros_ws/gui/js/geometry-config.js` (and the Python / C / sim mirrors).
  Any new constant (e.g. `BALL_RADIUS_MM`) goes in YAML first, then
  `python config/generate_config.py`.
- Mesh files: [sim/model/meshes/](sim/model/meshes/) is canonical (used by
  MuJoCo's [sim/model/jugglebot.xml](sim/model/jugglebot.xml)). Both GUIs
  serve the same files via a `/meshes/*` static route — no copies.

---

## Implementation Phase Summary

| Phase | Scope                                                       | Branch                          | Risk   | Validates                                              |
|-------|-------------------------------------------------------------|----------------------------------|--------|--------------------------------------------------------|
| 0     | Telemetry feed: balls + accels + throw phase in `StepRecord`| `demo/bb-led-two-ball-juggle`    | Low    | New SSE fields land without breaking prod consumers    |
| 1     | Prod GUI: STL mesh rendering (wireframe/meshed toggle)      | `refactor`                       | Medium | Hardware GUI renders correct mesh transforms on Jetson |
| 2     | Sim dashboard: fork upgraded prod GUI; strip ROS-only panels| `demo/bb-led-two-ball-juggle`    | Low    | Sim dashboard opens; viewer + charts driven by SSE     |
| 3     | Sim dashboard: render two balls as correctly-scaled spheres | `demo/bb-led-two-ball-juggle`    | Low    | Trajectories visually match CSV/replay                 |
| 4     | Sim-specific overlays: catch count, throw phase, RT rate    | `demo/bb-led-two-ball-juggle`    | Low    | Phase transitions match logs; readouts update at 40 Hz |
| 5     | Polish: chart-grid retuning for sim signals; entry-point sync| `demo/bb-led-two-ball-juggle`   | Low    | `sim/main.py --dashboard` also opens the new UI        |

---

## Implementation Phases

### Phase 0: Extend the telemetry feed

Self-contained backend change. Lands before any front-end churn so
Phase 2+ have data to consume.

#### 0.1 Add fields to `StepRecord`

Files: [controller/telemetry.py](controller/telemetry.py),
[controller/telemetry_test_fixtures.py] (if present — grep first).

Add to the `StepRecord` dataclass:

- `ball0_x, ball0_y, ball0_z, ball0_held` (float, float, float, int 0/1)
- `ball1_x, ball1_y, ball1_z, ball1_held`
- `leg_acc_0 .. leg_acc_5` (mm/s²; numerical derivative computed where
  `leg_velocities` are already written — central or backward difference,
  one tick of lag is fine at 40 Hz)
- `throw_phase` (string: priming / throw0 / catch0 / throw1 / catch1 /
  idle — exact set comes from `controller.demo.timeline.MasterTimeline`)
- `catches_total` (int)

Update `record_from_arrays` to accept these.

#### 0.2 Plumb through the broadcast

Files: [sim/juggle_demo.py:826-846](sim/juggle_demo.py#L826-L846).

`_broadcast_dashboard_record` already calls `plant.get_ball_state(0)` /
`get_ball_state(1)` from the CSV writer — refactor so both writers use a
single ball-state snapshot per tick. Pass through to
`record_from_arrays`.

#### 0.3 Extend `_record_to_payload`

File: [sim/viz/dashboard/server.py](sim/viz/dashboard/server.py).

Add to the SSE JSON payload:

```python
"balls": [
    {"pos": [bx, by, bz], "held": bh, "vel": [vx, vy, vz]},   # ball 0
    {"pos": [bx, by, bz], "held": bh, "vel": [vx, vy, vz]},   # ball 1
],
"leg_acc": [a0, a1, a2, a3, a4, a5],
"throw": {"phase": <str>, "catches": <int>},
```

**Go:** sim runs unchanged; `curl -N http://localhost:8082/events` shows
the new fields with sane values; existing dashboard.js still parses
(unknown fields ignored).
**Abort:** any test in `tests/sim/` regresses; any subscriber crashes on
the new payload shape.

#### 0.4 Tests

Add a small unit test exercising `_record_to_payload` with a populated
`StepRecord` — asserts the new keys are present and JSON-serialisable.

Run `pytest tests/ -q` per the project workflow rule.

---

### Phase 1: Prod GUI — STL mesh rendering

This phase lands on `refactor` and is independently valuable on
hardware. It adds STL rendering as a toggleable layer over the existing
primitive geometry; the wireframe view remains the default until Phase 1
is verified on a real robot.

#### 1.1 Mesh server route

File: [ros_ws/gui/gui_server.py](ros_ws/gui/gui_server.py).

Add a static route `/meshes/<filename>` that serves files from
[sim/model/meshes/](sim/model/meshes/). Reuse MuJoCo's filename
convention (`base.stl`, `platform.stl`, `leg_inner.stl`,
`leg_outer.stl`, `hand.stl`). HTTP cache headers: `max-age=86400` —
meshes are immutable per geometry-config regeneration.

Decide whether to also expose this from the production launch
(`ros_ws/jugglebot/launch/jugglebot_launch.py` mounts the GUI; verify
where `gui_server.py` is started).

#### 1.2 `mesh-renderer.js`

New file: [ros_ws/gui/js/mesh-renderer.js](ros_ws/gui/js/mesh-renderer.js).

Exports:

- `initMeshRenderer({ onLoaded })` — loads all 5 STLs via
  `three/addons/loaders/STLLoader.js` (importable via the existing
  import map; no bundler change). Caches geometries.
- `getBaseMesh()`, `getPlatformMesh()`, `getLegMesh(side)` (`'inner' |
  'outer'`), `getHandMesh()` — `THREE.Mesh` factories.
- `setMeshMode('wireframe' | 'meshed')` — global toggle.

Material: `MeshStandardMaterial` with the same colour palette already
used by primitives (`BASE_COLOR`, `PLAT_COLOR`, `HAND_COLOR` from
`stewart-model.js`). No shadows. Add a single hemisphere light + one
directional light in `viewer.js` if not already present.

#### 1.3 Wire into `stewart-model.js`

File: [ros_ws/gui/js/stewart-model.js](ros_ws/gui/js/stewart-model.js).

For each existing primitive (base, platform, legs, hand), add a
`*Mesh3D` sibling. In meshed mode, hide the primitive and show the
mesh; in wireframe mode, the inverse. The transform code that updates
primitive positions/rotations is reused for the mesh — just `.copy()`
the matrix.

**Leg transform math** (the fiddly bit):

Each leg's STL is authored with its axis along `+Z`. To place leg `i`:

1. Anchor point on base = `BASE_NODES_MM[i]`.
2. Anchor point on platform = `currentPlatNodes[i]` (world frame).
3. Direction `d = (platAnchor - baseAnchor) / |…|`.
4. Length `L = |platAnchor - baseAnchor|` mm.
5. `leg_outer` mesh: positioned at base anchor, rotated to align `+Z`
   with `d`, scaled to its authored length (no length change — the
   outer tube is rigid).
6. `leg_inner` mesh: positioned at `baseAnchor + d * (L - innerLen)`,
   same rotation, scaled to authored length.

`THREE.Quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1), d)` does
the rotation. Author-length comes from `INIT_LEG_LENGTHS_MM` per
`geometry-config.js`.

Verify against the MuJoCo XML — if MuJoCo authors the leg meshes with a
different axis (`+Y` is also common), follow the convention in
[sim/model/jugglebot.xml](sim/model/jugglebot.xml).

#### 1.4 Scene-menu toggle

File: [ros_ws/gui/js/main.js](ros_ws/gui/js/main.js) and the existing
"View" dropdown (`#scene-menu-dropdown`).

Add a "Mesh: Wireframe / Meshed" toggle. Persist to `localStorage` so
the choice survives reload. Default = `wireframe` (matches prod's
current behaviour; flip to `meshed` after operator validation).

#### 1.5 Verification on hardware

Run the prod GUI against the real robot. Confirm:

- Meshes load (no 404s in DevTools network tab).
- Meshed leg lengths match primitive leg lengths within ~1 mm at five
  test poses (home, +X jog, +Z jog, +RX jog, +RY jog).
- Frame rate ≥ 30 fps on the operator workstation (Chromium DevTools
  Performance panel). Jetson is server-only here, no rendering load.
- Hand mesh tracks the carriage along the central column.
- No console errors during a 60-second mocap streaming session.

**Go:** all five poses meshed-vs-wireframe align to < 1 mm at end-points;
frame rate ≥ 30 fps; no console errors; toggle persists.
**Abort:** mesh transforms visibly diverge from primitives (anywhere
> 5 mm at endpoints) → return to wireframe-only and debug the
leg-orientation maths separately before unblocking.

---

### Phase 2: Fork prod GUI into the sim dashboard

Phase 1 must be merged into `demo/bb-led-two-ball-juggle` first.

#### 2.1 Copy the prod GUI tree

```bash
cp -r ros_ws/gui/{index.html,css,js,lib,favicon.svg} \
      sim/viz/dashboard/static/
```

Delete the existing `sim/viz/dashboard/static/{dashboard.js,
dashboard.css,index.html}` — the fork replaces them entirely.

#### 2.2 Replace ROS bridge with SSE bridge

New file: `sim/viz/dashboard/static/js/sim-bridge.js`.

Reuses the same public surface that `ros-bridge.js` exposes
(`subscribe(topic, msgType, cb)`, `getConnectionState()`,
state-change listeners) so the existing modules
(`stewart-model.js`, `panels.js`, `telemetry-charts.js`, etc.) consume
its events without modification. Internally:

- One `EventSource('/events')`.
- On each message, parse JSON and dispatch to "topics" the modules
  expect — e.g. `robot_state` (leg lengths, leg vels, platform pose),
  `bb/heartbeat` (omitted in sim; no-op), etc.
- Map sim's `actual_pose` / `actual_ext` arrays to the same shape
  `stewart-model.js` already consumes from prod telemetry.

#### 2.3 Strip ROS-only DOM

File: `sim/viz/dashboard/static/index.html` (the fork).

Remove (or hide with `style="display:none"` if simpler to maintain):

- Topic monitor sidebar (`#left-sidebar` → `#panel-topics`,
  `#panel-can`)
- CAN traffic panel
- Ball Butler calibration panel (sim has no BB)
- Jog panel (`#panel-jog`)
- Speed-limits panel (`#panel-speed-limits`)
- Command-history panel (`#panel-history`)
- Command overlay (`#command-overlay`)
- Mocap info (`#mocap-info`)
- Connection-status pill stays but binds to SSE readyState
- Fault flags panel — sim has no faults; remove

Keep:

- 3-D viewer pane (`#viewer-container`) with scene menu and connection
  status
- Motor grid (state-only, no commands)
- Levelling / tracking-error / motion readouts (driven by sim data)
- Chart grid + chart toolbar + signal toggles + time-window selector
- Theme toggle, font-size, sidebar resize handles

#### 2.4 Sim-fork `main.js`

File: `sim/viz/dashboard/static/js/main.js`.

Imports `sim-bridge.js` instead of `ros-bridge.js`. Skips initialisation
of stripped modules (`initJogPanel`, `initSpeedLimitsPanel`,
`initCommands`, `initCommandHistory`, `initMocapMarkers`,
`initBallButlerModel`, etc.). Keeps `initViewer`, `initStewartModel`,
`initTelemetryCharts`, `initMotorGrid`, `initTheme`, `initCameraPresets`,
and the new `initBallRenderer` (Phase 3).

Be ruthless about dead-code removal — this is a fork, not a feature
flag. Leave a comment at the top pointing back to the prod GUI commit
hash that was forked, to support occasional 3-way merges later.

#### 2.5 SSE static route + mesh route

File: [sim/viz/dashboard/server.py](sim/viz/dashboard/server.py).

Add the same `/meshes/<filename>` route Phase 1 added to
`gui_server.py`. Update the `_STATIC_DIR` glob if needed to serve the
new tree.

#### 2.6 Verification

```bash
cd ~/Desktop/Jugglebot-bb
python sim/juggle_demo.py --dashboard --duration 30
# Open http://<jetson-ip>:8082 in browser on Win10
```

**Go:** browser shows the prod-GUI-style layout; Stewart platform
renders and moves; charts populate; no JS console errors; SSE stays
connected; meshes load (verify in DevTools Network tab).
**Abort:** any uncaught JS error in `main.js`, `stewart-model.js`, or
`telemetry-charts.js` — debug before Phase 3.

---

### Phase 3: Render the balls

#### 3.1 Add `BALL_RADIUS_MM` to config

File: [config/hardware_config.yaml](config/hardware_config.yaml).

Locate the section that owns ball geometry (likely under a `ball:`
key — confirm by `grep -n 'ball' config/hardware_config.yaml`). Add or
expose `radius_mm`. Run:

```bash
python config/generate_config.py
```

Confirm the new constant appears in `ros_ws/gui/js/geometry-config.js`
(prod) and the sim's copy.

#### 3.2 `ball-renderer.js`

New file: `sim/viz/dashboard/static/js/ball-renderer.js`.

- Creates two `THREE.Mesh(SphereGeometry(BALL_RADIUS_MM * S, 24, 16),
  MeshStandardMaterial)`. Parents to a new `sceneGroups.balls` group
  (add to `viewer.js` if not already there).
- `updateBalls([{pos:[x,y,z], held:bool}, ...])` — updates positions
  per tick. Colour by held: held = amber (matches hand colour),
  in-flight = matte white. Hide if `pos` is NaN.
- Position mapping uses the same `r2t(rx,ry,rz)` mm→m convention as
  `ball-butler-model.js`.

#### 3.3 Wire into `main.js`

Subscribe to the sim-bridge's "balls" event (added in Phase 0). Call
`updateBalls` per SSE message.

#### 3.4 Scene-menu toggle

Add "Balls" to the scene visibility menu (`scene-menu-dropdown`). On by
default.

#### 3.5 Verification

Run the juggle demo. Compare ball trajectories against the CSV
post-hoc plot (the existing PNG report). They should visually match.

**Go:** balls appear, follow physically-correct parabolas, change
colour at catch/throw events, run for the full 30 s without flicker.
**Abort:** noticeable jitter or wrong scale → check `S = 0.001` and
`BALL_RADIUS_MM` units.

---

### Phase 4: Sim-specific overlays

Small additions surfacing data not present in prod.

#### 4.1 Top-bar readouts

File: `sim/viz/dashboard/static/index.html` (sim fork) + a small JS
module.

Add to the top bar (where the prod GUI has the orchestrator state
badge): sim time, realtime rate, throw-director phase, catch count.

#### 4.2 Throw/catch event markers on charts

Use the existing `flashChart(idx)` machinery in `telemetry-charts.js`.
On each throw/catch event from the SSE feed, draw a vertical marker at
that timestamp on the per-leg charts.

#### 4.3 Leg-acceleration chart

Add an acceleration chart to the 9-cell grid (one of the existing cells
can be repurposed — pick the cell prod uses for an
unused-in-sim signal). Data comes from Phase 0's `leg_acc_*` fields.

**Go:** phase transitions on the readout match the timeline log;
catch-count increments at the exact frame ball state transitions to
held; markers align with the throw timestamps in the CSV.

---

### Phase 5: Polish and cross-entry-point sync

#### 5.1 Chart layout for sim

Tune the 9-chart grid to sim-most-useful signals:

- 0–5: per-leg cmd vs. actual extension (matches prod)
- 6: per-leg velocity (overlay, 6 series)
- 7: per-leg acceleration (overlay)
- 8: hand cmd vs. actual / tracking error / MPC solve time
  (one chart, picked per session usefulness)

#### 5.2 Cross-entry-point sync

File: [sim/main.py](sim/main.py).

Hook `--dashboard` to the same `DashboardServer` and static tree, so
both sim entry points show the same UI. Verify the StepRecord schema
populates correctly for `sim/main.py` (which uses MPC; juggle demo
doesn't).

#### 5.3 README / docs

Update [sim/README.md](sim/README.md) (or create one) with:
- screenshot of the dashboard
- `--dashboard` flag, port, mesh/wireframe toggle
- note on serving meshes from `sim/model/meshes/`

Per the project's docs convention, also reference this from the top
`README.md` if visible there.

#### 5.4 Verification

```bash
pytest tests/ -q                       # workflow rule
python sim/main.py --mpc --dashboard --duration 20 --pose 0,0,170,0,0,0
python sim/juggle_demo.py --dashboard --duration 30
```

Both should open the same UI; both should render Stewart + meshes;
juggle demo additionally renders balls.

**Go:** all of the above plus `pytest tests/ -q` clean.
**Abort:** any sim test regression — investigate before merging.

---

## Risk and tradeoffs

| Item                             | Cost / risk                                                                  |
|----------------------------------|------------------------------------------------------------------------------|
| Leg mesh orientation             | Half-day of fiddly-but-tractable rotation maths in Phase 1.3; biggest unknown. |
| Frame rate                       | Low risk — Jetson only streams SSE; client (Win10 box) renders. Prod GUI ships 40 Hz × 9 uPlot charts on Jetson Chromium today. |
| STL bundle size                  | ~2-3 MB total, one-shot. Negligible after initial load.                       |
| Bundle drift from prod           | Forked code drifts. Keep `stewart-model.js`, `stewart-fk.js`, `viewer.js`, `panels.js` close enough to prod that 3-way merges remain tractable; avoid renaming exports. |
| hardware-config sync             | Auto-generated — both UIs share via `config/generate_config.py`.              |
| Branch coordination              | Phase 1 lands on `refactor`; sim port consumes via merge. Stay disciplined about the order. |
| Auto-pull on Jetson              | Per [`MEMORY.md`'s project_auto_pull_webhook](.) the Jetson auto-pulls on push for any branch — both worktrees update without manual intervention. |

---

## Out of scope / future work

- **Real URDF.** No URDF file exists today; this plan uses STLs + manual
  mount transforms against `geometry-config.js`. If a URDF is later
  authored, [urdf-loaders](https://github.com/gkjohnson/urdf-loaders)
  drops in cleanly.
- **Mocap visualisation in sim.** Sim has no mocap; the marker rendering
  is dropped. If a synthetic-mocap stream is ever added for testing,
  re-enable `mocap-markers.js` in the sim fork.
- **Shadows / PBR lighting.** Out of scope per user preference.
- **Server-side video stream (MJPEG/WebRTC).** Not needed — primitive
  geometry + STLs at the client side renders smoothly enough.
- **MPC solve telemetry visualisation.** `juggle_demo` doesn't use MPC;
  `sim/main.py --dashboard` does. Phase 5.2 covers the entry-point
  hookup; richer MPC-state UI (predicted trajectory, constraint
  ribbons) is a separate plan.

---

## File touch summary

**Backend (Python):**
- [controller/telemetry.py](controller/telemetry.py) — `StepRecord` fields
- [sim/juggle_demo.py](sim/juggle_demo.py) — pass new fields
- [sim/viz/dashboard/server.py](sim/viz/dashboard/server.py) — extended
  payload, `/meshes/*` route
- [config/hardware_config.yaml](config/hardware_config.yaml) — `ball_radius_mm`
- [ros_ws/gui/gui_server.py](ros_ws/gui/gui_server.py) — `/meshes/*` route (Phase 1)
- [sim/main.py](sim/main.py) — `--dashboard` wiring (Phase 5)

**Front-end (prod, Phase 1):**
- [ros_ws/gui/js/mesh-renderer.js](ros_ws/gui/js/mesh-renderer.js) — NEW
- [ros_ws/gui/js/stewart-model.js](ros_ws/gui/js/stewart-model.js) — STL wiring
- [ros_ws/gui/js/viewer.js](ros_ws/gui/js/viewer.js) — light setup
- [ros_ws/gui/js/main.js](ros_ws/gui/js/main.js) — scene-menu toggle
- [ros_ws/gui/index.html](ros_ws/gui/index.html) — minor

**Front-end (sim fork, Phases 2-5):**
- `sim/viz/dashboard/static/index.html` — fork of prod
- `sim/viz/dashboard/static/js/sim-bridge.js` — NEW
- `sim/viz/dashboard/static/js/ball-renderer.js` — NEW
- `sim/viz/dashboard/static/js/main.js` — sim-fork of prod's main.js
- `sim/viz/dashboard/static/js/{viewer,stewart-model,stewart-fk,
  telemetry-charts,panels,camera-presets,theme,geometry-config,
  mesh-renderer}.js` — copied from prod
- `sim/viz/dashboard/static/css/*` — copied from prod
- `sim/viz/dashboard/static/lib/uPlot.*` — copied (drop `roslib.min.js`)
- Delete: `sim/viz/dashboard/static/{dashboard.js,dashboard.css,
  index.html}`

**Total surface:** ~6 Python touch-ups + 1 large fork + 3 new JS files
+ 1 sim-fork of `main.js` + 1 mesh server route in two places.

---

## Workflow rule reminders (per project CLAUDE.md)

- `pytest tests/ -q` after each phase's code changes and again
  immediately before commit.
- `python config/generate_config.py` after any YAML edit; stage the
  regenerated artefacts.
- Commit trailer: `Logbook-Entry: <slug>` for traceability — open a
  logbook entry when starting Phase 1 (the multi-phase arc warrants
  one).
- `git fetch && git status -sb` before any push; the `refactor` branch
  is shared and the auto-pull webhook means parallel sessions on the
  Jetson may have local changes.

---

## Archival note (2026-08-01)

**Archived as superseded — Phase 0 shipped, Phases 1–5 never started.**
Phase 0 (extend the telemetry feed: ball positions/velocities, per-leg
accelerations, throw-director state on the dashboard SSE payload) landed and is
pinned by `tests/sim/test_dashboard_payload.py`. Phases 1–5 (STL mesh rendering
in the prod GUI, forking that GUI into the sim dashboard, ball rendering,
sim-specific overlays, polish) were never started — there is no mesh loader
under `ros_ws/gui/js/`, which still renders primitive geometry only.

No content edit since the plan was written (2026-05-28, `69644e7`). Nothing in
the current stack depends on Phases 1–5; revive from here if the 3-D sim
dashboard is wanted.

Moved out of `plans/active/` by the 2026-07 refactor programme
(`plans/parked/refactor-2026-07.md` § Phase 1, item 5).
