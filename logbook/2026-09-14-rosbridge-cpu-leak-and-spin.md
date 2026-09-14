---
title: rosbridge CPU leak (service-client) + 1 ms timer spin — lean launcher fix
type: optimization
date: 2026-09-14
status: in-progress
files_changed:
  - ros_ws/src/jugglebot/jugglebot/rosbridge_websocket_lean.py
  - ros_ws/src/jugglebot/setup.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/gui/js/ros-bridge.js
  - ros_ws/gui/js/main.js
  - tests/ros/test_rosbridge_websocket_lean.py
  - tools/probes/rosbridge_cpu_probe.py
  - tools/probes/README.md
  - tools/gen_choreography_map.py
  - tests/ros/test_choreography_map.py
subsystem:
  - ros
  - gui
  - tools
tags:
  - performance
  - testing
---

# rosbridge CPU leak (service-client) + 1 ms timer spin — lean launcher fix

## Summary

The GUI's rosbridge (`rosbridge_server` 1.3.1, Foxy) measured 48.2% of a core
in a live launch after ~2.6 minutes with one GUI client attached — and grows
further, since one of the two causes is an unbounded leak. `rosbridge_websocket_lean.py`
replaces the stock `rosbridge_websocket` executable with a drop-in that (1)
destroys each `rosapi`-discovery service client after use instead of leaking
it forever, and (2) runs the executor on a dedicated blocking thread instead
of a 1 ms tornado timer. A GUI-side fix (`compression: 'cbor-raw'` on the
monitor's raw spy subscriptions) removes rosbridge's per-message decode cost
for the two 100 Hz topics the monitor watches passively. All three fixes were
implemented and unit-tested in a prior session (this entry's own `git log
--grep` will show the commits that carry it); this entry adds the committed
CPU probe, runs the live acceptance measurements against the real ROS2
stack, and records the numbers.

## Symptom

Live `pidstat` capture, 2026-09-13, `temp/logs/pidstat_r2gate_20260913.txt`
(launch `~/.ros/log/2026-09-13-12-35-28-280381-jetson-723232`): rosbridge
(PID 723249) at 48.2% of a core, rosapi at 2.5%, one GUI client connected,
captured in the first ~2.6 minutes after launch — so the leak (see below)
had barely started accumulating. The number keeps climbing the longer the
process runs.

## Diagnosis

**Where the time goes** (py-spy, 2026-09-13, stock server under a GUI-like
synthetic load — historical measurement, scratch `run_pyspy_residual.sh`,
not committed): 68% of rosbridge's CPU in rclpy's `_wait_for_ready_callbacks`,
~9% taking and deserialising messages, ~4% rosbridge JSON conversion, ~3%
websocket writes. Cost scales with messages-in × wait-set entities, not
message size or browser count — which is why a leaked service client (one
more wait-set entity per `/rosapi/topics` call) is disproportionately
expensive: every one of those entities is rescanned on every `spin_once`.

**Leak mechanism**: `rosbridge_library/internal/services.py::call_service`
(1.3.1) calls `node_handle.create_client(...)` on every service call and
never destroys the client — rclpy keeps it in the node's internal client
list for the life of the process, and the executor iterates that list while
building each wait set. The GUI polls `/rosapi/topics` every 3 s for topic
discovery, so this leaks one client every 3 s indefinitely, and it persists
after the browser disconnects (nothing ever destroys it). `ClientReaper` +
`build_call_service` in `rosbridge_websocket_lean.py` reproduce the 1.3.1
function verbatim and destroy the client in a `finally`, via a guard
condition that runs the destroy on the executor thread — `Node.destroy_client`
mutates the same list the executor iterates, so it cannot safely run on the
`ServiceCaller` worker thread that originates the call. Upstream's `ros2`
branch fixed this the same way (destroy scheduled onto the executor context).

**1 ms timer spin**: stock `main()` drives the executor with
`PeriodicCallback(lambda: executor.spin_once(timeout_sec=0.01), 1)` inside
tornado's IOLoop — a 1 kHz poll regardless of whether there is work. A
dedicated thread blocking in `executor.spin_once()` with no timeout
(`spin_forever` in the lean module) removes the poll entirely; it swallows
and logs any non-shutdown exception rather than letting the thread die, so a
single bad message can't silently kill ROS data flow while the websocket
stays up (matching stock's resilience, which relied on tornado's
`PeriodicCallback` logging-and-continuing).

## Discussion

**Spin thread vs alternatives.** A batched-drain executor (poll every N ms
instead of a dedicated blocking thread) was also measured, 2026-09-13
(scratch `run_batch_probe.sh` / `rosbridge_websocket_batched.py`, not
committed — box under ~3 cores of niced foreign load at the time): thread
control 30.2%, batch-20ms 26.4% (rates unchanged), batch-50ms 21.7% but with
GUI-visible rate loss (robot_state 17.9 → 15.6 Hz, hand_telemetry
9.4 → 7.9 Hz). Rejected: the CPU gain over the plain thread was small, it
would have needed a private rclpy API, and it costs real GUI-visible
latency. The dedicated thread was shipped instead.

**Rejected by the owner (2026-09-14):** dropping the two 100 Hz topics from
the monitor, or making the monitor opt-in — the monitor is always open and
must keep watching them. Source-decimated GUI copies (publish
`gui/robot_state` at 20 Hz and `gui/hand_telemetry` at 10 Hz from
`teensy_bridge_node` instead of letting the GUI subscribe the full-rate
topics) measured lower synthetic CPU on 2026-09-13 (14.2% stock / 10.1% with
the spin thread, against 34.7% / 24.3% with today's subscription set) but
was rejected because it adds ongoing work to the bridge node for a saving
this fix already captures more cheaply. A C++ bridge (`foxglove-bridge` or
similar) is not packaged for ROS2 Foxy — confirmed independently 2026-09-14,
`apt-cache search foxglove` returns only `ros-galactic-foxglove-bridge`
(no `-foxy-` variant), and `apt-cache search rws` returns nothing.

**The raw-subscription sharing caveat.** rosbridge shares exactly one ROS
subscription per topic and fixes it raw-vs-decoded at first creation — a spy
subscribing raw to a topic the GUI also subscribes decoded (or vice versa)
would silently corrupt whichever subscription lost the race. `main.js`'s
`GUI_SUBSCRIBED_TOPICS` set must therefore stay exactly equal to the topics
the GUI subscribes decoded, so a spy never lands on one of them; this is
pinned by the `GUI_SUBSCRIBED_TOPICS` contract test in
`tests/ros/test_rosbridge_websocket_lean.py`.

**Monitor semantics are unchanged by this work**: the panel's displayed
rates are the *throttled delivery rate* (200 ms throttle ⇒ ≈5/s for a 100 Hz
topic), not the underlying publish rate, both before and after — confirmed
live 2026-09-14 (§ Verification, criterion E: 39 callbacks / 8 s ≈ 4.88 Hz
for a 100 Hz synthetic topic under `throttle_rate: 200`).

**Synthetic vs live gap.** The synthetic load's stock+gui number (2026-09-13:
34.7%; 2026-09-14 re-run: 29.2%, see Verification) sits well below the live
48.2% symptom capture. The live stack carries more low-rate topics, larger
messages, the GUI's periodic 3 s discovery calls (which is exactly what
drives the leak — the synthetic `leak` scenario isolates that mechanism
separately and reproduces the live ratio closely: 2.95→59.45% live-adjacent
2026-09-13 vs 3.00→58.70% 2026-09-13-style synthetic, see criterion A), and a
generally busier box. Treat every percentage in this entry as a *ratio*
against its own paired baseline, not an absolute prediction of live rosbridge
CPU. Note also that the 2026-09-14 re-runs of the `load` scenarios came in
systematically lower than their 2026-09-13 counterparts across the board
(stock+gui 29.2 vs 34.7%, lean+gui 23.7 vs 24.3%) while the `leak` scenario
reproduced almost exactly (58.70 vs 59.45% after 1,200 calls) — consistent
with a quieter box today (`uptime` load average 0.6–0.8 at the start of this
session, vs the 2026-09-13 batch comparison's explicit note of "~3 cores of
niced foreign load"). The leak mechanism is close to load-independent (it's
a fixed per-call cost); the spin-loop/decode comparisons are more
sensitive to box load, which is why `compare` interleaves stock and lean in
one invocation rather than trusting two runs made at different times.

**SpaceMouse**: a related always-on idle-CPU fix in a different ROS2 node is
recorded separately —
`logbook/2026-09-14-spacemouse-handler-idle-cpu.md`.

## Fix

- `ros_ws/src/jugglebot/jugglebot/rosbridge_websocket_lean.py` (new): wraps
  stock's `RosbridgeWebsocketNode` with `ClientReaper` (destroys leaked
  service clients on the executor thread via a guard condition) and
  `spin_forever` (a dedicated executor thread replacing the 1 ms tornado
  timer). Stays importable with no ROS2/rosbridge on the path (all such
  imports live inside `main()`), so it loads under the mocked-rclpy test
  gate.
- `ros_ws/src/jugglebot/setup.py`: registers the
  `rosbridge_websocket_lean` console-script entry point.
- `ros_ws/src/jugglebot/launch/jugglebot_launch.py`: the rosbridge `Node`
  now launches `package='jugglebot', executable='rosbridge_websocket_lean'`
  instead of `rosbridge_server`'s own executable — same node name (
  `rosbridge_websocket`), same four parameters (port 9090,
  `retry_startup_delay` 5.0, `websocket_ping_interval` 10,
  `websocket_ping_timeout` 30).
- `ros_ws/gui/js/ros-bridge.js` (`subscribeSpy`): adds
  `compression: 'cbor-raw'` so rosbridge never deserialises a message a spy
  callback only counts. The monitor's displayed rate is unaffected (see
  Discussion).
- `tests/ros/test_rosbridge_websocket_lean.py` (new, 15 tests, ROS-free):
  `ClientReaper` scheduling/draining (incl. concurrent schedulers),
  `build_call_service` success/None-result/exception/unknown-service paths
  (each asserting whether destroy was scheduled), `spin_forever`
  exception-swallow vs shutdown-exception behaviour, import hygiene (no
  `rosbridge_library`/`tornado`/`ament_index_python` at module scope), the
  launch-file pin (package/executable/name/ping params) and the GUI pins
  (`compression: 'cbor-raw'` present; the `GUI_SUBSCRIBED_TOPICS` contract).
- `tools/probes/rosbridge_cpu_probe.py` (new): the committed CPU probe
  documented below.
- `tools/probes/README.md`: new row for the probe in § "Available probes",
  and the § "Probe conventions" carve-out sentence now names both
  `gui_synthetic_stack.py` and `rosbridge_cpu_probe.py`, adding this probe's
  extra containment rail (a private, non-default `ROS_DOMAIN_ID`).

**Operator guidance**: after this lands on the robot, `colcon build
--packages-select jugglebot`, source, relaunch. Then `pidstat -u -p
<rosbridge pid> 1 60` right at launch and again after roughly an hour with
the GUI open — expect both samples to be roughly flat and well below the
48% the un-fixed process reached.

## Verification

All runs 2026-09-14, worktree `/home/jetson/Desktop/Jugglebot-cpu`
(`cpu-efficiency`), `ROS_DOMAIN_ID=87` (never the robot's domain 0), ports
9391–9399/9399 (never 9090). Box quiet at the start (`uptime`: load average
0.63, 0.66, 0.77). Every command below is exactly reproducible from the repo
root with `tools/probes/rosbridge_cpu_probe.py` alone (it self-sources the
ROS2 + venv environment).

**A — leak, stock vs lean, 1,200 calls.**
`python3 tools/probes/rosbridge_cpu_probe.py leak --server stock --calls 1200`:
idle 3.00% → 58.70% after 1,200 calls; latency first-50 mean 7.1 ms, last-50
mean 35.7 ms (reproduces the leak; close to the 2026-09-13 baseline of
2.95→59.45%, 7.3→38.9 ms).
`python3 tools/probes/rosbridge_cpu_probe.py leak --server lean --calls 1200`:
idle 0.00% → 0.05%; latency first-50 mean 6.3 ms, last-50 mean 5.0 ms.
**PASS** (< 5% idle-after threshold; last-50 latency ≈ first-50, no
degradation).

**B — load, stock+gui / lean+gui / lean+rawspy.**
`python3 tools/probes/rosbridge_cpu_probe.py load --server stock --client gui --seconds 35 --warm 8`:
server 29.2% CPU; client Hz: robot_state 17.9, hand_telemetry 9.4, the three
10 Hz topics 10.0 each, spies (leg_cmd_executed, bb/axis_estimates) 4.8 each,
(probe_diag5_a/b) 3.1 each.
`... --server lean --client gui ...`: server 23.7% CPU; Hz: robot_state
17.8, hand_telemetry 9.4, 10 Hz topics 10.0, spies 4.9 / 3.1–3.2.
`... --server lean --client rawspy ...`: server 21.3% CPU; Hz: robot_state
17.9, hand_telemetry 9.4, 10 Hz topics 10.0, spies arrive as one aggregate
binary-frame bucket, 16.0/s (= 4.9+4.9+3.1+3.1, matching the decoded-mode
per-topic sum exactly).
**PASS**: lean+rawspy (21.3%) is below lean+gui (23.7%) — a smaller relative
gain (~10%) than the 2026-09-13 scratch estimate of "-15 to -17%"; see the
Discussion's box-load note. Client Hz per topic matches stock+gui across all
three runs, as required.

**C — service error path under lean.** Ad hoc verification (not a probe
subcommand): a `call_service` for `/this_service_does_not_exist` against a
bare lean server returned `{"op": "service_response", ..., "values":
"Service /this_service_does_not_exist does not exist", "result": false}` in
4.6 ms. Server log shows the `InvalidServiceException` logged and a clean
client disconnect — no hang, and (per the unit-tested `build_call_service`
contract) no client was created on this path, so nothing was scheduled for
destruction.

**D — reconnect under lean.** Ad hoc verification: with the synthetic 100 Hz
publisher running, client 1 subscribed `/robot_state` and received 492
messages over 5 s (98.4 Hz); it disconnected, and a fresh client 2
subscribed and received 499 messages over its own 5 s (99.8 Hz) — data flows
identically after a reconnect.

**D2 — subscription churn under traffic (lean only).**
`python3 tools/probes/rosbridge_cpu_probe.py churn --server lean --cycles 50 --hold 2.0 --steady-seconds 20`:
50 cycles of connect → full GUI subscribe set (spies included) → 2 s hold →
disconnect, then one steady client for 20 s. Result: `server_alive=True`,
**zero** `rosbridge_spin: unhandled exception` lines in the server log
(`{}` — no exception types at all), and the steady client's final Hz
(robot_state 17.9, hand_telemetry 9.5, 10 Hz topics 10.0, spies aggregate
15.8/s) match criterion B closely. **PASS**, and better than the "a few
handle races are tolerable" expectation — none were observed in this run.
This is a stress *measurement*, not a proof of absence for all timings; the
zero count is this run's result, not a guarantee.

**E — roslibjs decode.** Node v22.22.3
(`/home/jetson/.nvm/versions/node/v22.22.3/bin/node`), loading
`ros_ws/gui/lib/roslib.min.js` under a `global.window = global` /
`global.WebSocket = WebSocket` shim (the bundle is a browser UMD build that
attaches to `window.ROSLIB` rather than `module.exports` under plain
`require`), subscribed a `ROSLIB.Topic` with `compression: 'cbor-raw',
throttle_rate: 200` to the synthetic 100 Hz `/robot_state` topic on a lean
server: 39 callbacks over 8 s ≈ 4.88 Hz, matching the expected ≈5/s and
confirming roslib decodes the raw CBOR frames correctly under Node.
**PASS.**

**F — entry point.** In the worktree: `source /opt/ros/foxy/setup.bash &&
source /home/jetson/Desktop/Jugglebot-skills/ros_ws/install/setup.bash && cd
ros_ws && colcon build --packages-select jugglebot` (3.62 s, 1 package,
warns only that `jugglebot_interfaces` is used from the skills install, as
expected). With `ROS_DOMAIN_ID=87`, `ros2 run jugglebot
rosbridge_websocket_lean --ros-args -p port:=9399` logged `Rosbridge
WebSocket server started on port 9399` and was killed after confirming the
log line. `git status` after the build shows only the expected U2a/U2b
source files — `build/`, `install/`, `log/` never appear. **PASS.**

**Logbook + unit tests**: `pytest tests/sim/test_logbook_search.py
tests/sim/test_logbook_front_matter.py tests/ros/test_rosbridge_websocket_lean.py -q`
(run 2026-09-14, before this entry existed): **49 passed in 0.75 s**.

No defect in the U2a implementation was found by any of the above; no U2a
file was modified in this session.


**Post-audit (2026-09-14).** The phase's single `/audit --unstaged` pass found
one latent shutdown race: `main()` destroyed the node without first stopping
the spin thread. Stock could not exhibit this, since it spun and tore down on
one thread. It is now fixed by stopping on an event, calling `executor.wake()`
and joining (2 s) before `destroy_node()`. That path runs only when
`start_hook()` returns normally; a SIGINT raises `KeyboardInterrupt` past it,
as in stock. SIGINT shutdown was compared with both servers launched by
`subprocess.Popen` on domain 87 (a one-off check): stock and lean each exit
rc 0 in 0.16 s, logging "Exiting due to SIGINT". A first attempt launched from
a backgrounded non-interactive bash left lean alive after SIGINT. That shell
starts `&` jobs with SIGINT ignored, so the launch method caused it, not the
node. After the fix, `pytest tests/ros/test_rosbridge_websocket_lean.py -q`:
15 passed. The first full gate then failed 3 `tests/ros/test_choreography_map.py`
tests. The generated topic map (`ros_ws/docs/choreography.md`) scans every
module in the package, and it listed the new module's per-request service
client as `UNRESOLVED(service)`. The map is declared Python-node-only
("GUI/rosbridge consumers are NOT included"), so `tools/gen_choreography_map.py`
now skips that file (`_SKIP_FILES`) rather than regenerating the doc with an
endpoint that has no static name. Two tests pin this.

## Outcome

Software-complete and live-verified against the real ROS2 Foxy stack
(synthetic publisher, real rosbridge/rosapi processes, real websocket
clients, real roslib decode) on this Jetson, domain-isolated from the robot.
**Outstanding**: the operator's live one-hour check with the real GUI open
against the real launch (`pidstat -u -p <rosbridge pid> 1 60` at launch and
again after ~1 h) has not run yet — that is the only remaining gate before
this can be called fully resolved, hence `status: in-progress` (the ladder
reserves `tuned` for work verified on hardware) rather than `resolved`.

## Open Questions

- The live one-hour `pidstat` check (Fix section, Operator guidance) is
  outstanding.
- The lean+rawspy vs lean+gui gap measured smaller here (~10% relative) than
  the 2026-09-13 scratch estimate (-15 to -17%); worth a re-check once the
  box is under its normal live load, since the Discussion's box-load note is
  a plausible but unconfirmed explanation.
