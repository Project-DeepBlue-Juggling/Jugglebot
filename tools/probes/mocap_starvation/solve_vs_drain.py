"""Sweep-solve cost, and whether a worker THREAD would keep mocap_node's 5 ms frame drain alive.

The BB calibration sweep's post-processing (``run_calibration`` + ``estimate_base_pose``, what
``mocap_node._finalize_calibration`` runs inside the bb/heartbeat callback) is timed on one
recorded sweep, then re-run in a worker thread while the main thread imitates the drain timer
(5 ms sleep + 1.2 ms of Python per tick, ~24 % of a core = the node's measured cost) against a
150-frame queue filled at 300 Hz. Reports the solve time and the drain's tick gaps / queue peak /
drops. NOT modelled: rclpy, the asyncio receive thread, DDS publish.

Recipe (repo root, project venv; the fixture is a gitignored run output):
  python tools/probes/bb_calibration_replay.py --bags "$HOME/Desktop/rosbags/2026-10-16_16-10-37" \\
      --emit-fixture 2026-10-16_16-10-37:0 --out temp/probes/mocap_starvation/fixture_2026-10-16_16-10-37_0.json
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python tools/probes/mocap_starvation/solve_vs_drain.py \\
      temp/probes/mocap_starvation/fixture_2026-10-16_16-10-37_0.json
Used by logbook 2026-10-16-mocap-frame-queue-starvation."""
import sys, os, json, time, threading
import numpy as np
sys.path.insert(0, os.getcwd())
import tools.probes.bb_calibration_replay as R
from ros_ws.src.jugglebot.jugglebot.bb_base_frame import estimate_base_pose
f = json.load(open(sys.argv[1])); inp = f['inputs']
template = R.load_marker_template(os.path.join(R.RES_DIR, 'bb_marker_template.json'))
base_model = R.load_base_frame(os.path.join(R.RES_DIR, 'bb_base_frame.json'))
frames = [(t, np.array(p, dtype=float)) for t, p in inp['base_points']]
args = dict(calibration_data={int(k): [np.array(p) for p in v] for k, v in inp['calibration_data'].items()},
    yaw_readings_deg=inp['yaw_readings_deg'], pitch_z_offset_mm=inp['params']['pitch_z_offset_mm'],
    marker_frames=[(t, np.array(p)) for t, p in inp['marker_frames']],
    yaw_samples=[tuple(s) for s in inp['stamped_yaw']], template=template, yaw_source='stamped')
done = threading.Event(); span = {}
def solve():
    t0 = time.perf_counter(); R.run_calibration(**args); estimate_base_pose(frames, base_model.template_mm)
    span['solve'] = time.perf_counter() - t0; done.set()
def busy(sec):
    t = time.perf_counter()
    x = 0
    while time.perf_counter() - t < sec: x += 1
for name, fn in (("run_calibration", lambda: R.run_calibration(**args)), ("estimate_base_pose", lambda: estimate_base_pose(frames, base_model.template_mm))):
    t0 = time.perf_counter(); fn(); print(f"{name} alone {time.perf_counter()-t0:.2f} s")
th = threading.Thread(target=solve, daemon=True); th.start()
ticks = []; queue = 0.0; maxq = 0.0; dropped = 0.0; last = time.perf_counter()
while not done.is_set():
    time.sleep(0.005); now = time.perf_counter(); ticks.append(now - last)
    queue += (now - last) * 300; maxq = max(maxq, queue)
    if queue > 150: dropped += queue - 150; queue = 150
    busy(0.0012); queue = 0; last = time.perf_counter()
g = np.array(ticks) * 1e3
print(f"solve in worker thread {span['solve']:.2f} s; drain ticks {len(g)}; gap p50 {np.percentile(g,50):.1f} p99 {np.percentile(g,99):.1f} max {g.max():.1f} ms; max queue {maxq:.0f} of 150 frames; dropped {dropped:.0f}")
