# Jugglebot Web GUI — Rewrite Plan

## Context

The existing GUI (`ros_ws/gui/`) is a legacy vanilla-JS app with dated styling, hardcoded pixel layouts, and references to old topic names that no longer exist in the refactored codebase. The goal is a modern, stylish dashboard that can be opened from any browser on the network, showing a live 3D view of the robot, system monitoring panels, and basic command controls.

The approach: **rewrite the GUI in-place** using the same technology stack (static files + Three.js + ROSlib.js) but with a modern dark theme, responsive layout, and clean modular JS architecture. No build step, no framework — just ES modules with CDN import maps (same pattern the legacy GUI already uses successfully).

**Critical improvement**: The static files are served by a **standalone lightweight HTTP server** (a tiny Python script) that runs independently of ROS2. The GUI's JavaScript connects to rosbridge via WebSocket with **auto-reconnect** — when ROS2 restarts, the socket silently reconnects and re-subscribes without needing a page refresh.

---

## Architecture

```
  gui_server.py (port 8080, standalone, always-on)
        │ HTTP (static files)
        ▼
  Browser loads index.html + JS modules
        │
  ros-bridge.js ──── WebSocket to rosbridge (port 9090) with auto-reconnect
        │
        ├─→ stewart-fk.js ─→ stewart-model.js ─→ viewer.js (Three.js render loop)
        ├─→ ball-butler-model.js ─────────────────┘
        ├─→ panels.js ─→ DOM status panels (right sidebar)
        ├─→ commands.js ─→ publishes to orchestrator_command
        └─→ telemetry-charts.js ─→ uPlot 3x3 chart grid (bottom panel)
```

**Key design decisions:**
- **Independent HTTP server** — `gui_server.py` (tiny Python script, ~20 lines) serves static files on port 8080, runs as a systemd service or in a tmux session, completely independent of ROS2. No more page refreshes when ROS2 restarts.
- **Auto-reconnect WebSocket** — `ros-bridge.js` reconnects to rosbridge (port 9090) every 2 seconds when disconnected, re-subscribes to all topics on reconnect. Connection status shown in UI.
- **No build tool** — pure static files + ES module import maps (forward-compatible with Vite if needed later)
- **FK computed in JavaScript** — port the Newton-Raphson solver from `ik_solver.py` (55 lines of core logic, trivially fast at 20Hz). When mocap is available, use measured pose instead (bypasses FK).
- **Dark theme** — CSS custom properties, no framework
- **Layout**: main 3D viewer + right sidebar panels + bottom command overlay

---

## File Structure

```
ros_ws/gui/
  gui_server.py               # Standalone HTTP server (~20 lines), runs independently of ROS2
  index.html                  # Main HTML — layout structure, import maps
  favicon.svg                 # Keep existing
  css/
    theme.css                 # Dark theme variables, base typography, grid layout
    panels.css                # Status card styles, motor grid, badges
    viewer.css                # 3D container, command overlay, connection indicator
    charts.css                # Telemetry chart panel: toolbar, signal toggles, 3x3 grid, uPlot overrides
  js/
    main.js                   # Entry: init ROS, viewer, panels, wire callbacks, chart resize handle
    ros-bridge.js             # ROSLIB connection with auto-reconnect, topic subs, throttling
    geometry-config.js        # Hardcoded constants from hardware_config.yaml
    stewart-fk.js             # FK solver + rotation utilities (port from ik_solver.py)
    viewer.js                 # Three.js scene, camera, OrbitControls, render loop
    stewart-model.js          # Stewart platform 3D geometry (base, legs, platform, hand axis)
    ball-butler-model.js      # Ball Butler 3D geometry (yaw/pitch/hand line art)
    panels.js                 # Right sidebar panel creation + data update functions
    commands.js               # Command button handlers → orchestrator_command topic
    telemetry-charts.js       # Live time-series charts: signal defs, data stores, uPlot management
  lib/
    roslib.min.js             # Keep existing local copy
```

Legacy files to remove: `jugglebot_gui.html`, `main.js`, `3dplotter.js`, `package.json`, `package-lock.json`, `convex_hull_points.json`, `convex_hull_points_big.json`

### `gui_server.py` — Standalone HTTP Server

A minimal Python script that serves the `ros_ws/gui/` directory on port 8080. It runs completely independently of ROS2 — start it once (e.g., via systemd service, tmux, or `nohup`) and it stays up forever. When ROS2 restarts, the browser page stays loaded; only the WebSocket to rosbridge reconnects in the background.

```python
# Essentially: python3 -m http.server 8080 --directory ros_ws/gui/
# But as a proper script with bind-to-all-interfaces and CORS headers.
```

---

## ROS2 Topics (subscriptions)

| Topic | Type | Rate | GUI Use | Throttle To |
|-------|------|------|---------|-------------|
| `robot_state` | RobotState | 100Hz | Motor states, errors, homing, voltage, temps, FK fallback | 20Hz |
| `bb/heartbeat` | BallButlerHeartbeat | 10Hz | BB state, yaw/pitch/hand | 10Hz (no throttle) |
| `orchestrator_state` | String | on change | State badge | No throttle |
| `can_traffic` | CanTrafficReportMessage | ~2Hz | CAN bus load chart | No throttle |
| `hand_telemetry` | HandTelemetryMessage | 500Hz | Hand pos/vel detail | 10Hz |
| `rigid_body_poses` | RigidBodyPoses | ~200Hz | Measured platform pose (bypasses FK when available) | 20Hz |
| `mocap_data` | MocapDataMulti | ~200Hz | Raw markers (ball positions, general viz) | 20Hz |
| `leg_lengths_topic` | Float64MultiArray | 500Hz | Commanded leg positions (for tracking error display) | 20Hz |

**Pose source priority**: When `rigid_body_poses` is available, use it directly for the 3D platform pose (most accurate, measured). Fall back to FK from `robot_state` motor positions when mocap is unavailable.

**Publishes to:** `orchestrator_command` (String) — "home", "activate", "deactivate", "spacemouse", "shell", "clear_errors"

---

## 3D Viewer Design

### Stewart Platform (`stewart-model.js`)
- **Base hexagon**: 6 nodes connected by line segments, semi-transparent blue fill — `#3b82f6`
- **Legs**: 6 cylinders from base→platform nodes, color-coded green→amber→red by extension ratio
- **Platform hexagon**: 6 nodes connected by lines, light gray, semi-transparent fill
- **Hand axis**: A single line perpendicular to the platform surface, originating from the platform center offset -100mm along the platform's local Y axis (to account for hand depth). The ball travels along this axis. Line extends by the hand's current extension. Color: amber.

### Ball Butler (`ball-butler-model.js`)
- **Pedestal**: Small cylinder at BB's configurable position offset from base center
- **Yaw turntable**: Ring that rotates around Z by `heartbeat.yaw_deg`
- **Pitch arm**: Cylinder tilting from turntable by `heartbeat.pitch_deg`
- **Hand element**: Slider along pitch arm at `heartbeat.hand_pos_mm`
- **Position**: Configurable constant in `geometry-config.js` (user will provide real coordinates)

### Scene Setup
- Dark background (`#0f172a`), ambient light + soft directional
- OrbitControls, perspective camera, responsive resize
- Scale: all geometry in mm, camera positioned ~1.5m away
- Coordinate swap: Three.js Y-up ↔ robot Z-up (same swap as legacy code)

---

## FK Pipeline (`stewart-fk.js`)

Port from `ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py`:

1. **Motor revs → extensions**: `ext_mm[i] = pos_rev[i] / mm_to_rev[i]`
2. **Extensions → absolute lengths**: `abs_len[i] = init_leg_lengths_with_offset[i] + ext_mm[i]`
3. **Newton-Raphson FK**: Solve for `(pos, R)` such that IK(pos, R) matches target lengths
   - State vector: `[px, py, pz, rx, ry, rz]` (position + rotation vector)
   - Inner loop: compute IK residual, compute Jacobian, solve `J·dx = -residual`, update x
   - Converges in 3-5 iterations for normal poses
   - Warm-start from previous frame's solution for stability
4. **Platform nodes**: `plat_world[i] = (pos + [0,0,574.3]) + R @ plat_nodes[i]`

Functions to port:
- `rotvecToRotMatrix(rv)` — Rodrigues formula
- `rotMatrixToRotvec(R)` — inverse
- `poseToLegLengths(pos, R)` — position IK
- `computeJacobian(pos, R)` — 6x6 analytical Jacobian
- `legLengthsToPose(extensions, initialGuess)` — Newton-Raphson FK
- `solve6x6(A, b)` — Gaussian elimination (no need for a full linear algebra library for a 6x6)

---

## Panel Designs (right sidebar, `panels.js`)

### 1. Connection Status (top bar)
- Green dot + "Connected" / Red dot + "Disconnected" with auto-reconnect

### 2. Orchestrator State
- Large color-coded badge: BOOT (gray), HOMING (blue), IDLE (amber), ACTIVE (green), FAULT (red pulsing)
- Sub-mode shown when ACTIVE: SPACEMOUSE / SHELL

### 3. Motor Status Grid
- 7 compact columns (Leg 0-5 + Hand)
- Per motor: state dot (green=CLOSED_LOOP, yellow=IDLE, red=error), position, velocity, FET temp (color-coded), active errors badge
- Bus voltage: single large readout from `motor_states[0].bus_voltage`

### 4. System Flags
- Encoder search: checkmark / spinner
- Homed: checkmark / X
- Levelled: checkmark / X
- Error flags: fatal ODrive, fatal CAN, undervoltage (red badges when set)

### 5. Ball Butler Panel
- State badge (BOOT/IDLE/TRACKING/THROWING/RELOADING/ERROR)
- Ball-in-hand indicator (green/red dot)
- Yaw, Pitch, Hand readouts
- Shown as "Disconnected" when no heartbeat received

### 6. CAN Traffic
- Small sparkline (canvas chart, lightweight — Chart.js or manual canvas) showing msgs/sec over last 10s
- Current rate as large number

### 7. Position Tracking Error
Displays the delta between commanded and measured positions for each DoF:
- **Legs (6)**: commanded position (from `leg_lengths_topic`) vs measured position (from `robot_state.motor_states[i].pos_estimate`). The error is `commanded - measured` for each leg.
- **Hand**: commanded (`hand_telemetry.pos_cmd`) vs measured (`hand_telemetry.pos_meas`)
- **Display**: compact bar chart or numeric grid, one column per axis. Color-coded: green when |error| < threshold, amber for moderate, red for large.
- **Optional 3D overlay**: Show tracking error as a ghost/wireframe of the commanded platform position overlaid on the measured position in the 3D viewer — makes pose error visually intuitive. The ghost platform is drawn semi-transparent when the error is small and becomes more opaque / red-tinted as error grows.

### 8. Topic Monitor
Discovers all active ROS2 topics via rosbridge `getTopics()` (every 3 seconds). Displays a scrollable table with columns:
- **Topic**: Name (leading `/` stripped for compactness, full name + type in tooltip)
- **Last**: Time since last message ("0.2s", "12s", "3m", or "--" if never received)
- **Hz**: Message rate in Hz within a configurable window (default 5s)

Rate is color-coded: green for active topics, muted for stale/zero, amber for very high rates (>100 Hz). Click the "5s window" label to cycle through 1s/5s/10s/30s. Topics sorted by rate (highest first). Click any topic row to hide it; hidden topics are persisted to `localStorage`. A "N hidden" footer appears — click to expand and unhide topics.

### 9. Resizable Sidebar
The sidebar width is adjustable via a draggable handle between the 3D viewer and sidebar. Default width is 420px. Width is persisted to `localStorage` and restored on page load. Minimum 280px, maximum 50% of viewport.

### 10. Font Size Control
A-/A+ buttons at the top of the sidebar scale all panel text proportionally (10px–22px, default 14px). Uses root `font-size` on `<html>` — all CSS uses `rem` units, so everything scales. Persisted to `localStorage`.

### 11. Telemetry Charts (bottom panel, `telemetry-charts.js`)

Live time-series charts for all 9 ODrive-driven motors. Collapsible bottom panel with a 3x3 grid of synchronized uPlot charts.

**Layout**: Full-width panel below the viewer and sidebar (grid row 3), separated by a draggable horizontal resize handle (row 2).

```
Row 1: [3D Viewer | resize-handle | Sidebar]     ← existing
Row 2: [====== horizontal resize handle ======]   ← chart toggle/resize
Row 3: [=========== chart panel ==============]   ← 3x3 chart grid
```

**3x3 chart grid** (column-major order to match motor numbering):

```
| L0       | L3       | Hand     |
| L1       | L4       | BB Pitch |
| L2       | L5       | BB Hand  |
```

**Charting library**: uPlot (~35KB, MIT, loaded via CDN as global IIFE script — same pattern as ROSLIB).

**Signals** (9 toggleable, persistent to localStorage):

| Signal | Color | Style | Unit | Y-Axis Group |
|--------|-------|-------|------|---------------|
| Position (measured) | `#3b82f6` (blue) | solid | rev | position |
| Position (commanded) | `#60a5fa` (light blue) | dashed | rev | position |
| Velocity | `#22c55e` (green) | solid | rev/s | velocity |
| Current (setpoint) | `#f59e0b` (amber) | solid | A | current |
| Current (measured) | `#fbbf24` (light amber) | dashed | A | current |
| FET Temperature | `#ef4444` (red) | solid | °C | temperature |
| Motor Temperature | `#f87171` (light red) | solid | °C | temperature |
| Bus Voltage | `#a78bfa` (purple) | solid | V | voltage |
| Bus Current | `#c084fc` (light purple) | solid | A | bus_current |

Signals sharing a Y-axis group share a single axis. Axes alternate left/right as groups are added.

**Data flow**: `robot_state` (20Hz) → `onRobotState()` → `onTelemetryData(motors, cmdLegs, handTelem)` → per-motor `ChartDataStore.push()` → `requestAnimationFrame(repaintAllCharts)`. Data is always buffered even when the panel is collapsed — charts have history immediately when expanded.

**Data store**: `ChartDataStore` uses `Float64Array` columns with `copyWithin` for ring buffer shift (zero GC pressure). `getAlignedData()` uses binary search + `subarray()` for zero-allocation reads.

**Time window**: Configurable via dropdown (5s/10s/30s/60s), default 10s. Persisted to localStorage.

**Cursor sync**: All 9 charts share `cursor.sync.key: 'telemetry'` — hovering one chart shows crosshair on all 9.

**Resize handle**: Click "Charts ▼" label to collapse/expand. Drag to resize height (clamped to 150px–60% viewport). Height and collapsed state persist to localStorage.

**localStorage keys**: `jugglebot-chart-height`, `jugglebot-chart-collapsed`, `jugglebot-chart-signals`, `jugglebot-chart-window`.

---

## Command Overlay (`commands.js`)

Semi-transparent bar at the bottom of the 3D viewer:
- **Home** — publishes "home" (only enabled when not homed)
- **Activate** — publishes "activate" (only in IDLE)
- **Deactivate** — publishes "deactivate" (only in ACTIVE)
- **SpaceMouse** — publishes "spacemouse" (only in ACTIVE)
- **Shell** — publishes "shell" (only in ACTIVE)
- **Clear Errors** — publishes "clear_errors" (always enabled)

Context-sensitive enable/disable based on last received `orchestrator_state`.

---

## Implementation Order

### Phase A — Skeleton + ROS connection — DONE (2026-02-24)
1. [x] Create `gui_server.py` — standalone HTTP server
2. [x] Create `index.html` with layout grid and import maps
3. [x] Create `css/theme.css` — dark theme variables, grid, typography
4. [x] Create `js/ros-bridge.js` — ROSLIB connection with auto-reconnect, subscription manager with throttle
5. [x] Create `js/main.js` — wiring entry point
6. [x] Verify: page loads, connects to rosbridge, auto-reconnects when rosbridge restarts

### Phase B — 3D viewer with static geometry — DONE (2026-02-24)
1. [x] Create `js/geometry-config.js` — all constants from `hardware_config.yaml`
2. [x] Create `css/viewer.css` — 3D container styles
3. [x] Create `js/viewer.js` — Three.js scene, camera, OrbitControls, dark background, render loop
4. [x] Create `js/stewart-model.js` — static home-pose Stewart platform geometry
5. [x] **Verify**: static robot renders at home position with orbit controls — needs visual testing

### Phase C — FK + live 3D updates — DONE (2026-02-24)
1. [x] Create `js/stewart-fk.js` — port FK solver from `sp_ik.py`
2. [x] Wire `robot_state` → FK → `stewart-model.js` update methods
3. [x] Wire `rigid_body_poses` as preferred pose source (when mocap available, bypasses FK)
4. [x] Implement leg color coding (green/amber/red by extension ratio)
5. [x] Implement hand axis update from `motor_states[6]`
6. [x] **Verify**: 3D model tracks real robot movement — needs hardware testing

### Phase D — Ball Butler 3D model — DONE (2026-02-24)
1. [x] Create `js/ball-butler-model.js` — yaw/pitch/hand line art
2. [x] Wire `bb/heartbeat` → model updates
3. [ ] **Verify**: BB moves in sync with heartbeat data — needs hardware testing

### Phase E — Status panels — DONE (2026-02-24)
1. [x] Create `css/panels.css` — card styles, motor grid, badges
2. [x] Create `js/panels.js` — all panel creation + update logic
3. [x] Implement: connection status, orchestrator state, motor grid, system flags, BB status, CAN traffic, tracking error
4. [x] **Verify**: all panels update in real-time — needs hardware testing

### Phase F — Command controls — DONE (2026-02-24)
1. [x] Create `js/commands.js` — button handlers
2. [x] Add command overlay to layout
3. [x] Implement context-sensitive enable/disable
4. [x] **Verify**: commands reach orchestrator — needs hardware testing

### Phase G — Polish — DONE (2026-02-24)
1. [x] Responsive resize (sidebar collapse on narrow screens)
2. [x] Smooth transitions on state changes (CSS transitions on all interactive elements)
3. [x] Scene element visibility toggles (menu overlay on 3D view)
4. [x] Tracking error ghost overlay in 3D viewer — red wireframe hex that fades in proportional to error magnitude
5. [ ] Ball rendering (sphere, when ball state data available from future phases) — deferred to Phase 6

### Phase H — Tests + Verification — DONE (2026-02-24)
1. [x] Python unit tests: `tests/test_gui_geometry.py` (40 tests) — validates all JS geometry constants against `hardware_config.yaml`, checks file structure, verifies legacy file removal
2. [x] Browser FK test suite: `ros_ws/gui/test_fk.html` — tests rotation math, IK/FK roundtrips, warm-start, motor rev conversion
3. [x] Mocap-driven 3D pose fully implemented — quaternion → rotation matrix → platform node positions

### Phase I — Telemetry Charts — DONE (2026-03-02)
1. [x] Create `css/charts.css` — chart panel styles: toolbar, signal toggle buttons, 3x3 grid, chart cell titles, uPlot dark-theme overrides
2. [x] Create `js/telemetry-charts.js` — signal definitions, `ChartDataStore` ring buffer, uPlot chart management, rAF-batched repainting
3. [x] Modify `index.html` — add uPlot CDN links (CSS + JS), `charts.css` stylesheet, `#chart-resize-handle` and `#chart-panel` HTML
4. [x] Modify `css/theme.css` — 3-row grid layout, `--chart-panel-height` variable, chart resize handle + panel styles, responsive hide
5. [x] Modify `js/main.js` — import `telemetry-charts.js`, call `initTelemetryCharts()`, route `onTelemetryData()` from `onRobotState()`, add `initChartResizeHandle()` function
6. [x] Signal toggle toolbar — 9 signals with color-coded buttons, localStorage persistence, rebuild charts on toggle
7. [x] Time window selector — 5s/10s/30s/60s dropdown, buffer resize, localStorage persistence
8. [x] Cursor sync — hover one chart shows crosshair on all 9
9. [x] Collapse/expand — click label or drag resize handle, localStorage persistence for height and collapsed state
10. [x] **Verify**: charts render with Y-axes, toolbar works, resize handle works — confirmed on Windows (no live data without ROS2)
11. [x] **Verify**: charts populate with live data at 20Hz — needs hardware testing

---

## Key Source Files Referenced

| File | Purpose |
|------|---------|
| `config/hardware_config.yaml` | Source of truth for all geometry constants |
| `ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py` | FK algorithm to port (lines 126-206 IK/Jacobian, 313-366 FK solver) |
| `ros_ws/src/jugglebot/jugglebot/motion/geometry.py` | StewartGeometry class — how constants are loaded |
| `ros_ws/src/jugglebot/jugglebot/can_node.py` | Topic names, motor state structure |
| `ros_ws/src/jugglebot/jugglebot/orchestrator_node.py` | orchestrator_command topic, state publishing |
| `ros_ws/src/jugglebot_interfaces/msg/RobotState.msg` | Robot state message fields |
| `ros_ws/src/jugglebot_interfaces/msg/MotorStateSingle.msg` | Per-motor fields |
| `ros_ws/src/jugglebot_interfaces/msg/BallButlerHeartbeat.msg` | BB heartbeat fields |
| `ros_ws/src/jugglebot_interfaces/msg/RigidBodyPoses.msg` | Mocap rigid body poses (preferred platform pose source) |
| `ros_ws/src/jugglebot_interfaces/msg/MocapDataMulti.msg` | Raw mocap markers (balls, general viz) |

---

## Verification

Since we're developing on Windows (no ROS2), verification is done in two stages:

1. **Windows dev** — Run `python gui_server.py` and open `http://localhost:8080` in a browser. The 3D viewer should render the static home-pose robot. Panels will show "Disconnected" (no rosbridge). All layout, styling, orbit controls testable offline. The page should NOT require a refresh when rosbridge becomes available later.
2. **Jetson/network** — Run `python3 gui_server.py` as a persistent process. Open `http://<jetson-ip>:8080` from any machine. Start ROS2 with rosbridge. Verify: GUI auto-connects, live data flows, restart ROS2 and confirm the GUI reconnects within a few seconds without page refresh.

---

## Manual Testing Checklist

### Stage 1: Windows Dev (offline, no ROS2)

Run `python ros_ws/gui/gui_server.py` and open `http://localhost:8080`.

**Automated tests (run first):**
- [ ] `python -m pytest tests/test_gui_geometry.py -v` — all 40 tests pass
- [ ] Open `http://localhost:8080/test_fk.html` — all FK tests pass (green "ALL PASSED")

**3D Viewer:**
- [x] Dark background renders (#0f172a)
- [x] Grid and coordinate axes visible on the floor plane
- [x] Stewart platform visible at home position: blue base hexagon, gray platform hexagon, 6 green leg cylinders, amber hand axis line
- [x] Ball Butler model visible: purple pedestal with ring and arm
- [x] OrbitControls: click-drag to rotate, scroll to zoom, right-click to pan
- [x] View menu (top-right): clicking "View" shows checkboxes for Grid, Axes, Platform, Ball Butler, Ghost Overlay
- [x] Toggling checkboxes hides/shows scene elements

**Sidebar panels:**
- [x] "State" panel shows BOOT badge (gray)
- [x] "Motors" panel shows 7 columns (L0-L5, Hand) with "--" placeholders
- [x] Bus voltage shows "--"
- [x] "System" panel shows 6 flags, all in wait state (gray dots)
- [x] "Ball Butler" panel shows "Disconnected" badge
- [x] "CAN Traffic" panel shows "--" msg/s
- [x] "Tracking Error" panel shows 7 columns with "--" placeholders

**Command overlay:**
- [x] Bottom bar shows 6 buttons: Home, Activate, Deactivate, SpaceMouse, Shell, Clear Errors
- [x] All buttons disabled except Clear Errors (which is always enabled)

**Connection status:**
- [x] Red dot + "Disconnected" shown (top-left of viewer)
- [x] No JS console errors (open DevTools → Console)

**Telemetry charts (bottom panel):**
- [x] Chart panel visible at bottom of page with 3x3 grid
- [x] Signal toggle toolbar visible with 9 buttons (colored dots/dashes + labels)
- [x] Time window dropdown visible (5s/10s/30s/60s)
- [x] Click "Charts" label on resize handle → panel collapses/expands
- [x] Drag resize handle → panel height changes smoothly
- [x] Toggle signals on/off → charts rebuild with correct axes
- [x] Chart cell titles visible (L0-L5, Hand, BB Pitch, BB Hand)
- [x] Y-axes render correctly (no data without ROS2)

**Responsive layout:**
- [x] Shrink browser width below 900px → sidebar moves below viewer
- [x] Motor grid wraps to 4 columns

### Stage 2: Jetson + ROS2 (live)

Prerequisites: rosbridge running on the Jetson (`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`), Jugglebot nodes running.

**Connection:**
- [x] Open `http://<jetson-ip>:8080` on any machine
- [x] Connection status changes to green dot + "Connected"
- [x] Kill rosbridge → status goes red "Disconnected"
- [x] Restart rosbridge → auto-reconnects within 2-3 seconds, status goes green again. **No page refresh needed.**

**Live data (BOOT/HOMING):**
- [x] State badge updates: BOOT (gray) → HOMING (blue) during startup
- [x] Motor dots change color: gray (undefined) → yellow (IDLE) → green (CLOSED_LOOP)
- [x] Bus voltage shows actual reading (e.g. ~48V)
- [x] Encoder Search flag changes to checkmark when complete
- [x] Homed flag changes to checkmark after homing

**Live 3D (IDLE/ACTIVE):**
- [x] After homing, state badge shows IDLE (amber)
- [x] Motor positions update in the grid (mm for legs, rev for hand)
- [x] FET temperatures show and are color-coded (green < 50C, amber < 70C, red >= 70C)
- [x] Click "Activate" → state badge shows ACTIVE (green)
- [x] Click "SpaceMouse" → sub-mode shows "SPACEMOUSE"
- [ ] Move spacemouse → 3D platform moves in real-time, legs change colour with extension
- [x] Hand axis line extends/retracts with hand motor

**If mocap is running:**
- [ ] Platform pose driven by measured mocap data (smoother, more accurate than FK)
- [ ] Disconnect mocap → falls back to FK from encoders within 1 second

**Tracking error:**
- [ ] With spacemouse active, tracking error bars show small green values (< 0.5mm)
- [ ] Fast movements cause tracking error to spike (amber/red)
- [ ] Ghost overlay (red wireframe) fades in during high-error periods

**Ball Butler:**
- [x] If BB is connected, state badge shows BB state (IDLE, etc.)
- [x] Yaw/Pitch/Hand readouts update
- [x] Ball-in-hand indicator shows green/red dot
- [x] 3D model rotates/tilts with actual BB movement

**CAN traffic:**
- [x] Rate shows actual msg/s count
- [x] Sparkline chart updates over time

**Commands:**
- [x] Click "Deactivate" → state goes to IDLE
- [x] Click "Home" from IDLE → state goes to HOMING
- [x] Click "Clear Errors" → works from any state

**Telemetry charts (live):**
- [x] Charts populate with live data at 20Hz (9 motors updating)
- [x] Hover one chart → crosshair appears on all 9 (cursor sync)
- [x] Toggle signals on/off → axes appear/disappear, colors match scheme
- [x] Change time window → chart history depth changes
- [ ] Collapse panel → data continues buffering; expand → history immediately visible
- [ ] BB not connected → charts 7-8 show no data (only 7 motors reporting)
- [ ] Commanded position (dashed) tracks measured position (solid) during movement

**Error handling:**
- [x] Trigger an error (e.g. E-stop) → state badge shows FAULT (red, pulsing)
- [x] Error flags light up red in System panel
- [x] "Clear Errors" button enabled
- [x] Clear errors → state recovers (if error was transient)

---

## Risks & Mitigations

| Risk | Mitigation |
|------|-----------|
| FK solver divergence at extreme poses | Warm-start from previous frame; fall back to home pose on divergence |
| rosbridge latency on high-freq topics | Client-side throttle (20Hz 3D, 10Hz panels) |
| CDN dependency for Three.js | Can add local copy to `lib/` if needed |
| BB position offset unknown | Configurable constant in geometry-config.js; user will provide coordinates |
| Motor state array shorter than 7 (BB not connected) | Guard with length checks; BB panel shows "Disconnected" |
| rosbridge restarts kill the page | Standalone HTTP server + auto-reconnect WebSocket. No page refresh needed. |
| Mocap not always available | FK fallback from motor positions; mocap is preferred but optional |

---

## Appendix: Implementation Notes (2026-02-24)

### Files created

| File | Lines | Purpose |
|------|-------|---------|
| `gui_server.py` | ~55 | Standalone HTTP server with CORS, correct MIME types |
| `index.html` | ~185 | Main HTML: layout grid, import maps, sidebar panels, resize handle, font toolbar, chart panel, overlays |
| `css/theme.css` | ~360 | Dark theme variables, 3-row grid layout, resize handles, font toolbar, chart panel, responsive breakpoint |
| `css/viewer.css` | ~120 | 3D container, connection indicator, command overlay, scene menu |
| `css/panels.css` | ~355 | Motor grid, flags, BB, CAN sparkline, tracking error, topic table + hide styles |
| `js/main.js` | ~540 | Entry point: init, subscribe, route data, resize handles, font size, topic discovery, chart data routing |
| `js/ros-bridge.js` | ~230 | ROSLIB auto-reconnect, subscription manager, publisher cache, topic discovery |
| `js/geometry-config.js` | ~100 | All hardware constants from `hardware_config.yaml` |
| `js/stewart-fk.js` | ~220 | Rodrigues rotation, IK, 6x6 Gaussian solver, Newton-Raphson FK |
| `js/viewer.js` | ~110 | Three.js scene, camera, OrbitControls, render loop |
| `js/stewart-model.js` | ~260 | Base/platform hex meshes, leg cylinders, hand axis line |
| `js/ball-butler-model.js` | ~100 | Pedestal, yaw turntable, pitch arm, hand marker |
| `js/panels.js` | ~650 | All sidebar panels: motors, flags, BB, CAN sparkline, tracking, topic monitor + hide |
| `js/commands.js` | ~70 | Command buttons with context-sensitive enable/disable |
| `css/charts.css` | ~150 | Chart toolbar, signal toggles, 3x3 grid, uPlot dark-theme overrides |
| `js/telemetry-charts.js` | ~475 | Signal defs, ChartDataStore ring buffer, uPlot management, rAF batching |
| **Total** | **~3,525** | Complete GUI rewrite |

### Legacy files removed

7 files removed via `git rm`: `jugglebot_gui.html`, `main.js` (root), `3dplotter.js`, `package.json`, `package-lock.json`, `convex_hull_points.json`, `convex_hull_points_big.json`.

### Key design decisions

1. **Coordinate system**: Robot Z-up → Three.js Y-up via `robotToThreeScaled(x, y, z)` → `Vector3(x*s, z*s, y*s)`. Scale factor 0.001 (mm → metres) for natural camera distances.

2. **FK solver**: Newton-Raphson with 6x6 Gaussian elimination (no external linear algebra library). Warm-starts from previous frame for stability. IK ported from `sp_ik.py`, FK is new — the old codebase had no FK.

3. **Pose source priority**: `rigid_body_poses` (mocap) is preferred when available, with a 1-second timeout fallback to FK from motor encoder positions.

4. **Throttling**: `ros-bridge.js` uses ROSLIB's built-in `throttle_rate` parameter. `robot_state` throttled to 20Hz (50ms), `hand_telemetry` to 10Hz (100ms), `leg_lengths_topic` to 20Hz.

5. **ROSLIB loaded as global script**: `roslib.min.js` is not an ES module, so it's loaded via `<script>` tag before the import map. All other JS is ES modules.

6. **Three.js from CDN**: Using import maps with `three@0.170.0` from jsdelivr. Can be replaced with a local copy in `lib/` if needed.

### Items completed since initial implementation

- **Tracking error ghost overlay**: Red wireframe hexagon showing commanded platform pose overlaid on measured pose. Opacity scales with RMS tracking error (invisible below 0.5mm, fully visible above 5mm). Toggle-able via View menu.
- **Mocap-driven 3D pose**: `onRigidBodyPoses` handler now extracts the platform rigid body by name, converts quaternion to rotation matrix via `quatToRotMatrix()`, computes platform node world positions via `poseToPlatNodes()`, and updates the 3D model. Falls back to FK when mocap is unavailable (1-second timeout).
- **Quaternion utilities**: Added `quatToRotMatrix(w,x,y,z)` and `poseToPlatNodes(globalPos, R)` to `stewart-fk.js`.

### Enhancements (2026-02-25)

- **Resizable sidebar**: Draggable splitter handle between the 3D viewer and sidebar. Default width increased from 340px to 420px. Uses Pointer Events API for mouse/touch support. Width persisted to `localStorage` and restored on reload. Clamped to min 280px, max 50% viewport. All panel contents scale automatically (CSS Grid `1fr` columns, `width: 100%` canvases).

- **Topic Monitor panel**: New sidebar panel showing all active ROS2 topics. Uses ROSLIB `getTopics()` for discovery (every 3s). For each discovered topic, a lightweight "spy" subscription (throttled to 200ms) counts messages. Display columns: topic name (with tooltip showing full name + type), time since last message, and rate in Hz (configurable window: 1s/5s/10s/30s, click label to cycle). Sorted by rate descending (highest first). Click any topic row to hide it — hidden topics persisted to `localStorage` and shown via an expandable "N hidden" footer. Topics the GUI already subscribes to share the existing subscription — `recordTopicMessage()` is called in each handler. Spy subscriptions are cleaned up on disconnect and re-created on reconnect via the discovery timer.

- **Font size control**: A-/A+ buttons at the top of the sidebar scale all panel text proportionally. Sets root `font-size` on `<html>` (10px–22px, default 14px) — all CSS uses `rem` units so everything scales together. Persisted to `localStorage`.

### Enhancements (2026-03-02)

- **Telemetry Charts (Phase I)**: Live time-series charts in a collapsible bottom panel. 3x3 grid of uPlot charts (one per ODrive motor: L0-L5, Hand, BB Pitch, BB Hand) with 9 toggleable signals: position (measured + commanded), velocity, current (setpoint + measured), FET/motor temperature, bus voltage/current. Signals sharing the same physical unit share Y-axes (alternating left/right). Data from `robot_state` (already publishes all 9 motor states — no CAN node changes needed). Commanded positions sourced from `leg_lengths_topic` (legs 0-5) and `hand_telemetry` (hand). `ChartDataStore` class uses `Float64Array` ring buffers with `copyWithin` shift and `subarray()` views for zero-GC, zero-allocation data flow. Charts repaint via `requestAnimationFrame` batching (max 60fps regardless of 20Hz data rate). Cursor sync across all 9 charts via uPlot's sync key. Configurable time window (5/10/30/60s). Panel height draggable and collapsible with localStorage persistence. Data buffers continuously even while collapsed. uPlot loaded via CDN (~35KB, MIT). New files: `css/charts.css`, `js/telemetry-charts.js`. Modified: `index.html`, `css/theme.css`, `js/main.js`.

- **Jog Panel (GUI Control Mode)**: Manual platform positioning via 5-DoF jog buttons. New `GUI` control mode added to backend (`state_machine.py` ActiveMode enum, `motion_bridge_node.py` recognized modes). GUI subscribes to `control_mode_topic` to show/hide the jog panel. Panel contains two sliders (translational step 1-50mm, rotational step 0.1-5.0deg, persisted to localStorage) and 10 buttons (+/-X, +/-Y, +/-Z, +/-Rx, +/-Ry). Each click accumulates a delta onto an internal target pose starting from home [0,0,0,0,0,0], converts rotation vector to quaternion, and publishes `PlatformPoseCommand` with `publisher='GUI'`. Home button resets target to origin. Panel auto-hides on disconnect or mode change. New file: `js/jog-panel.js`. Modified: `index.html`, `css/panels.css`, `js/main.js`, `js/commands.js`, `state_machine.py`, `motion_bridge_node.py`.

### Items for future work

- **Ball rendering**: Add a sphere in the 3D viewer when ball state data is available from mocap. Deferred to Phase 6 of the codebase rewrite.
- **BB position calibration**: `BB_POSITION_MM` in `geometry-config.js` is a placeholder `[0, -500, 0]`. Will be updated automatically when the BB calibration position publisher is implemented (see CODEBASE_REWRITE_PLAN Phase 4). The GUI should subscribe to `bb/calibration_result` and update the BB model position dynamically.
- **Levelling status display**: DONE (2026-03-01) — Levelling panel added to sidebar showing levelling status badge (Levelled/Not Levelled) and gravity correction offset as roll/pitch degrees. "Level" button added to command overlay (publishes "level" to orchestrator_command, enabled only in IDLE). LEVELLING state badge renders blue (same as HOMING). `levelling_complete` flag also shown in System Flags panel.

### Test infrastructure

| Test | Type | How to run |
|------|------|------------|
| `tests/test_gui_geometry.py` | Python/pytest | `python -m pytest tests/test_gui_geometry.py -v` |
| `ros_ws/gui/test_fk.html` | Browser JS | Open `http://localhost:8080/test_fk.html` after starting `gui_server.py` |

The Python tests verify 40 properties: all 11 scalar constants, 4 array constants, 3 derived constant cross-checks, 16 file existence checks, and 6 legacy file removal checks.

The browser FK tests verify: geometry constant sanity, rotation math (identity, 90deg, roundtrips), IK (home, Z-translation, symmetry, platform nodes), FK roundtrips (home, Z, arbitrary translation, with rotation), warm-start behaviour, and motor rev conversion.
