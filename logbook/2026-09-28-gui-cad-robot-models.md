---
title: "GUI: articulated CAD robot models with browser-only axis indication"
type: feature
date: 2026-09-28
status: implemented
phase: "GUI — CAD rendering"
subsystem:
  - gui
files_changed:
  - ros_ws/gui/js/robot-meshes.js
  - ros_ws/gui/js/stewart-model.js
  - ros_ws/gui/js/ball-butler-model.js
  - ros_ws/gui/js/viewer.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/mocap-markers.js
  - ros_ws/gui/assets/robots/robot-parts.glb
  - ros_ws/gui/assets/robots/manifest.json
  - ros_ws/gui/test_robot_models.html
  - tools/build_gui_meshes.py
  - tests/ros/test_gui_robot_assets.py
---

# Articulated CAD robot models

Replace the schematic robots with rigid CAD parts, retaining chart selection,
fault indication, mocap markers and triads (triads draw over opaque geometry).
Jugglebot placements match the MuJoCo visual meshes; the platform consumes the
full FK rotation, and inner legs and hands translate without stretching.
The owner selected BallButler's Blender rig over the older STL assemblies.
The authored relative rest transforms of pitch and hand are preserved; the
34.5 mm CAD pivot is registered to the configured 17.5 mm pivot. Its bottom-stroke
slider pose is retained. An initial pitch-only 20-degree correction was removed
after the owner spotted the resulting hand misalignment. The rig's CAD material colours are baked into vertex colours.

## Discussion

The owner accepted a 3.2-second IDLE brightness pulse instead of transparency:
opaque CAD preserves depth and avoids blended surfaces/extra passes. CLOSED_LOOP
is steady bright, faults override red, absent/stale/other states are dim; the
View menu explains this. Indication covers the nine ODrive axes; the BB yaw
servo has no ODrive CLOSED_LOOP state and its mesh retains its CAD colour.
Pulse updates share the existing viewer loop. No subscriptions, ROS messages,
controller changes, server rendering, runtime conversion or Jetson dependencies.

## Assets and rebuild

One 19,197,932-byte static GLB, nine geometries / nineteen instances, 465,417 robot
triangles. Curved surfaces use smooth split normals, with sharp edges above 30 degrees. Leg geometry is shared; CAD materials are collapsed into one draw
call per rigid mesh. No textures, external buffers or decoder downloads.
The existing HTTP server can revalidate its cached response (no-cache does not
mean no-store); a first visit transfers the GLB. The sibling CAD checkout and
Blender are needed only for an offline rebuild on a development workstation.
Current inputs are the owner's GLBs under `temp/gui-robot-source`, replacing
the original STL/Blender inputs:

```powershell
& 'E:/Programs/Blender 5.2/blender.exe' --background --python tools/build_gui_meshes.py
```

Override `--source-dir` after `--` if needed. The committed manifest records
source SHA-256 values, registrations, triangle counts and sampled surface errors. Deploy the GUI asset directory with
the JS files; no server restart or ROS build is needed. A failed asset request
shows a visible error instead of breaking telemetry. Both robots now retain
the GLB material colours through vertex colours.

## Verification

2026-09-28, Windows 10 development workstation (not the Jetson):
- Blender 5.2 offline export completed; source CAD files were not modified.
- `pytest tests/ros/test_gui_geometry.py tests/ros/test_gui_fk_golden.py tests/ros/test_gui_robot_assets.py -q`: 147 passed (UTF-8 mode; temporary Hypothesis dependency).
- GUI/asset tests plus `tests/sim/test_logbook_front_matter.py` and `tests/sim/test_logbook_search.py`: 181 passed.
- `test_robot_models.html`: 26 real WebGL checks for all nineteen meshes, shared
  leg geometry, physical-unit offsets, FK endpoints/orientation, BB pitch/hand
  motion, status priority, freshness and retained mocap overlays.
- Synthetic CAD versus the original HEAD model modules, same browser/viewport,
  60 warmup + 240 measured animated frames each: both median 33.4 ms / p95
  33.9 ms; CAD 23 draw calls, baseline 22 (including grid and sample overlays).
  Final repeat after stale-state changes: median 33.4 ms for both, p95 33.5 ms
  CAD / 33.4 ms baseline.
  The browser was effectively capped around 30 Hz: this demonstrates no observed
  regression on this client, not a GPU headroom guarantee on other clients.
- `bash run_tests.sh` could not run here: Windows checkout CRLF parsing fails;
  the gate also targets the Jetson's Linux virtualenv. Full gate and live
  mocap/model registration still need validation on the Jetson setup.

The standalone browser check opens through the normal GUI server at
`/test_robot_models.html`, uses only synthetic data, and sends no robot commands.
For an A/B run, serve the repository root with the GUI's JS MIME handler and pass
`?baseline=/temp/gui-baseline` after extracting the old model modules with their
viewer/config imports redirected to the current viewer/config. Those temporary
baseline copies are not deployed or required by the production GUI.


## Owner feedback: quality, rig alignment and demo frames

The first reduction was too aggressive and exported flat face normals. Increased
per-part triangle budgets (especially the repeated Jugglebot legs) and exported
smooth normals across curved surfaces while keeping mechanical edges sharp.
Triangle budgets remain offline and GPU geometry remains shared. This increases
the first asset transfer from 4.6 MB to 8.4 MB, without adding runtime conversion,
extra robot messages or production mesh draw calls.

Removed the erroneous pitch-only correction: both BB parts now preserve the
Blender rig's frame-0 transforms exactly before a shared registration shift and
runtime articulation. Verified original source vertex bounds independently;
the pitch's true source top is 441.46 mm (406.96 mm relative to the 34.5 mm pivot),
within 0.4 mm of the reduced result. A geometry regression now distinguishes this
from the incorrectly straightened result, and another checks smooth normals.

The original demo supplied only one Base rigid-body sample, coincident with the
world axes. It now provides Base, moving Platform and BallButler sample poses,
with labels and all three triads visible over the meshes. The production mocap
path already retains real rigid-body poses. Grid moved to global robot Z=-82 mm;
the world origin and robot transforms remain unchanged. Demo starts CLOSED_LOOP
so CAD detail is readily visible; the IDLE button demonstrates breathing.

Jugglebot colours still need a material-bearing export: recommended Fine GLB,
uncompressed, for base/platform/leg_outer/leg_inner/hand with the existing origins
and orientations. OBJ plus its MTL files is also suitable. No new CAD export is
needed merely for the quality improvement; STL geometry already has enough detail.

2026-09-28 follow-up: GUI geometry/FK/asset tests: 148 passed. Browser checks include
the lowered grid and all three triads; refreshed preview median 33.4 ms, p95 33.5 ms.
Live Jetson validation remains pending as above.

Final follow-up verification: 182 automated tests and 27 browser assertions passed.
Same-browser A/B: CAD and schematic median/p95 both 33.4 ms; 28 versus 27 draw
calls including the three demo labels and triads. All 278,758 CAD triangles were
in view. The larger asset does not establish headroom beyond the browser cap.

## Material-bearing GLB exports and homed actuator frames

The owner supplied Onshape GLBs for all nine rigid bodies and then replaced the
inner leg and both hands with bottom-stroke exports. Jugglebot's hand is already
in the Platform frame and receives no registration shift. The inner leg's upper
joint is at source Z=708.13 mm (the new foam homing pad is included); runtime FK
still positions that joint directly, without changing robot calibration.

BB's hand is in the linear-actuator frame. Apply -90 degrees about CAD Z, then
(70.15, 0, 211) mm before the existing runtime lateral offset. The slider aligns
with the pitch rail at X=-53.5 mm, Y=-41 mm; the rail centre is at source
Z=297.5 mm. BB base/yaw shift -69 mm in Z; pitch shifts (0,41,-86.5) mm, placing
the physical pivot at the configured 17.5 mm height. No guessed hand stroke is
subtracted from the replacement exports.

The offline builder joins triangle primitives by material within each CAD
component, welds shading seams, and adaptively reduces geometry. Each candidate
is compared to the original in both directions using deterministic vertex,
face-centre, edge-midpoint and extrema samples. The threshold is 0.75 mm,
providing margin within the requested approximate 1 mm shape accuracy. This is
a sampled check, not a formal maximum-error guarantee. Long triangles on flat
surfaces are intentional. Construction lines are omitted; mechanical surface
geometry and CAD colours remain. Cached component reductions are reused across
instances and builds; these caches and original exports are not deployed.

The final scene has 465,417 triangles and the asset is 19,197,932 bytes. This is
larger than the previous STL-based asset because the new sources include many
additional components, electronics and sensors. GPU geometry remains shared,
with one material and draw call per body instance. All conversion work is offline;
the Jetson only serves a larger static file on first load. No new ROS processing
or subscriptions are introduced.

Verification: 182 automated GUI/FK/asset/logbook tests pass. All 27 browser checks
pass, including hand motion, joint registration, state indications and triads.
The local preview measured median 33.4 ms, p95 33.5 ms with all robot geometry
visible, 28 calls including demo overlays. Live Jetson validation remains pending.

Final GLB A/B check: new CAD and original schematic both measured median 33.4 ms
and p95 33.5 ms in the same browser/viewport (60 warmup + 240 measured frames).
CAD: 28 draw calls including overlays, 465,591 rendered triangles; schematic:
27 draw calls, 4,430 triangles. The roughly 30 Hz browser cap limits conclusions
about GPU headroom or performance on other clients.

## 2026-09-29: production handoff and base-joint alignment investigation

The production main.js already imports the CAD renderer; the standalone page is
only a synthetic verification harness. Ship the changed GUI JavaScript together
with assets/robots/robot-parts.glb and manifest.json. The feature remains
uncommitted on the Windows workstation. Before deployment, run the repository
Linux test gate; then update the checkout/static GUI directory served by the
Jetson and reload clients. No ROS build, firmware update, Blender installation or
CAD conversion on the Jetson is required for these frontend-only changes. Check
live telemetry articulation, mocap overlays, IDLE/CLOSED_LOOP indications and
frame times on the actual viewing client. Three.js and its loader currently use
the existing jsDelivr import map.

The apparent socket misalignment is the nominal CAD versus fitted kinematics:
source Base.glb magnet/spacer centres match the nominal CAD base nodes. Runtime
legs use BASE_NODES_MM, generated from the September 27 accepted calibration.
The six horizontal differences are 6.129, 5.598, 4.557, 6.435, 3.537 and 4.592 mm.
The hardware YAML explicitly documents these fitted nodes replacing CAD nodes.
An independent sphere fit to 5,185 lower-hemisphere source vertices of
Magnetic_Joint_Ball gives centre (0,0,64.0001665) mm and radius 12.5 mm, confirming
the existing -64 mm mesh registration. Simplification's sub-mm sampled error
cannot explain the systematic several-mm mismatch. Platform joints likewise use
fitted attachment points against nominal CAD.

No calibration or renderer changes were made during this investigation. A
presentation-only option is to articulate the visible legs between nominal CAD
attachment points, transformed by the calibrated platform pose; this closes CAD
joints but visual telescoping lengths then represent nominal geometry, not exact
calibrated joint distances. Keep the actual FK/IK and telemetry calculations on
the fitted geometry. The owner has not yet selected that presentation tradeoff.

## 2026-09-29: final leg replacements and owner alignment decision

The owner explicitly chose to retain calibrated leg attachment positions so the
CAD/socket differences remain visible. No nominal-anchor visual correction is
applied. Rebuilt both leg geometries from the replacement GLBs exported on
September 29; their authored relative rotation replaces the earlier pair.
Joint registrations remain -64 mm (outer) and -708.13 mm (inner). Source hashes
in the manifest identify the replacement files.

Final asset: 19,191,820 bytes; 462,837 robot triangles. The focused geometry,
FK, asset provenance, colour, coordinate and logbook suite passed all 182 tests.
Full Linux gate/live robot verification remains outstanding on the Jetson.

## 2026-10-10: QTM-tracked catching cone

Added `temp/gui-robot-source/Catching Cone.glb` to the offline build, with zero
registration offset: the supplied origin is the QTM rigid-body origin. The
existing `rigid_body_poses` stream drives the CAD position and orientation,
accepting `Catching_Cone` and `Catching Cone`. The mesh hides when absent,
invalid, disconnected, or without a valid pose for 1.5 seconds. A separate
View-menu group preserves manual visibility; existing markers and triads remain.
Cone markers have their own purple legend entry.

The cone reduces from 244,672 to 5,486 triangles; maximum sampled surface error
is 0.405281 mm. It adds one opaque draw call and 201,008 bytes to the bundle,
with no new ROS subscription or server-side processing. Total robot geometry
is 468,323 triangles. The original export remains ignored and is not deployed.

Validation: 148 asset/geometry/FK tests pass; whitespace check passes. The demo
now includes cone pose, rotation, loss, recovery, invalid-pose, staleness and
visibility checks, and initializes the now-separate Local Triads group. Browser verification subsequently passed all 35 checks: median 33.4 ms,
p95 33.5 ms, 30 draw calls including overlays. Live QTM alignment still needs
confirmation. The Windows preview server served JavaScript as text/plain due
to registry MIME associations. Added tools/serve_gui.py with explicit JavaScript
MIME types and no-cache preview responses. Vendored the pinned Three.js 0.170.0
module and required loaders/controls with its MIT license; both index.html and
the demo now load it locally, removing their external CDN dependency. A fresh
preview on port 8083 avoids cached incorrect MIME responses.
