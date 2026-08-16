# Jugglebot juggle-demo sim — quickstart

> **Status (2026-07-24) — this README describes the PAUSED offline demo.**
> The offline demo documented below (`sim/juggle_demo.py`, driven by the CasADi
> offline optimiser) is **paused, not deleted** — it still runs as described,
> but it is no longer the frontier. The follow-on ONLINE tilt-ladder work
> (per-throw planner, tilt-aimed throws, single-ball self-catch — Rungs 0–2b)
> was merged into `mvp-trajectory-bringup` on 2026-07-24 and now continues on
> the **production trajectory stack** under
> [`plans/active/single-ball-toss.md`](../plans/active/single-ball-toss.md);
> the authority for the two-ball frontier remains
> [`plans/active/bb-online-juggle-tilt-rearchitecture.md`](../plans/active/bb-online-juggle-tilt-rearchitecture.md).
> The `demo/bb-led-two-ball-juggle` branch referenced in §2 is merged and
> retired — everything here now lives on `mvp-trajectory-bringup`.

A standalone MuJoCo simulation of Jugglebot performing a Ball-Butler-led
two-ball oval juggle. The platform plays an offline-optimised periodic
trajectory; the Ball Butler primes the pattern with one throw; Jugglebot
then sustains the pattern for 30+ catches.

This README gets a collaborator from a fresh clone to a working sim
window in ~5 minutes. The deeper architecture, design rationale, and the
in-progress optimiser cuts live in
[`plans/archived/bb-led-two-ball-juggle-demo.md`](../plans/archived/bb-led-two-ball-juggle-demo.md).

---

## 1. Prerequisites

- **Linux or macOS.** Tested on Ubuntu 20.04 (Jetson Orin Nano). The
  MuJoCo viewer needs an X display.
- **Python 3.9+ recommended (3.11+ to match the project's stated sim
  contract in `CLAUDE.md`).** The juggle-demo's own import chain
  works on 3.8.10 (verified on the Jetson venv); the wider `sim/`
  subsystem may need 3.11+ for unrelated modules. The runner has no
  ROS2 dependency.
- **A working CPU.** No GPU required.

System packages on Debian/Ubuntu:

```bash
sudo apt-get install -y python3-venv python3-pip libosmesa6-dev \
                        libgl1-mesa-glx libglfw3 libglew-dev
```

---

## 2. Clone

> *(Historical — the branch below is merged and retired; see the status
> banner. On a fresh clone: `git checkout mvp-trajectory-bringup`.)*

The demo lives on the `demo/bb-led-two-ball-juggle` branch (it merges
back to `refactor` → `main` once the optimiser follow-up cuts land):

```bash
git clone https://github.com/Project-DeepBlue-Juggling/Jugglebot.git
cd Jugglebot
git checkout demo/bb-led-two-ball-juggle
```

---

## 3. Python environment

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r sim/requirements.txt
```

`sim/requirements.txt` pulls in `mujoco`, `casadi` (used by the offline
trajectory optimiser), `numpy`, and friends.

---

## 4. Run the sim

```bash
python sim/juggle_demo.py --viewer
```

A MuJoCo window opens. The optimiser solves the periodic platform
trajectory once at startup (~3 seconds), then the sim runs at real-time
by default. Expect ~33 captures in the default 30-second run with no
drops.

Useful flags (full list: `python sim/juggle_demo.py --help`):

| Flag | Default | What it does |
|---|---|---|
| `--viewer` | off (headless) | Open the MuJoCo passive viewer |
| `--realtime-rate <r>` | `0` (free-running) | Wall-pace the loop. `1.0` = real-time, `0.5` = slowmo, `2.0` = 2x. |
| `--duration <s>` | `30.0` | Simulation duration |
| `--apex-height-mm <mm>` | `1300` | Ball apex above the hand at release. Lower = faster tempo + smaller throws. |
| `--separation-mm <mm>` | `200` | THROW–CATCH horizontal distance. Larger = wider pattern. |
| `--abort-at <s>` | none | Trigger an abort + smooth exit-to-stow at this sim time |
| `--dashboard` | off | Start the live telemetry dashboard (HTTP + SSE on port 8082) |
| `--log <path>` | `temp/logs/juggle_demo_<ts>.csv` | CSV of every 40 Hz tick |
| `--no-log` | off | Disable CSV logging |
| `--capture-tolerance-mm <mm>` | `30` | Strict catch gate: ball-centre within this distance of the cup-opening site. Default on at 30 mm since the cuts #4/#5 optimiser keeps the headline rate under it. Pair with `--no-capture-gate` to disable — see §7. |
| `--no-capture-gate` | off | Disable the capture gate entirely (any cup-rim contact catches, legacy snap-in behaviour). |
| `--scatter-mm <mm>` | `0` | Gaussian sigma on BB landing scatter |
| `--seed <n>` | none | RNG seed |
| `--analytic-baseline` | off | Skip the optimiser, use the un-optimised analytic oval (debugging only) |
| `--record <path>` | none | Render the run offscreen from a **fixed** camera to an H.264 mp4 (needs `ffmpeg` on PATH). Independent of `--viewer`. See §12. |
| `--record-size <WxH>` | `1280x720` | Recording resolution |
| `--record-fps <n>` | `40` | Output fps — `40` = real-time (one frame per tick), lower = slow-motion |
| `--cam-azimuth/-elevation/-distance <v>` | model default | Fixed-camera angle for `--record` (deg / deg / m). Press `C` in the viewer to read your current angle. |
| `--cam-lookat <x> <y> <z>` | model default | Fixed-camera look-at point (m), three space-separated values (each may be negative) |

---

## 5. Viewer keyboard controls

These work when `--viewer` is on. The MuJoCo viewer's built-in mouse
controls (drag to orbit, scroll to zoom, etc.) work normally; the runner
adds:

| Key | Action |
|---|---|
| **`SPACE`** | Pause / resume the sim |
| **`→` (Right)** | Step exactly one 40 Hz tick (when paused) |
| **`[`** | Halve the wall-time playback rate (jumps out of free-running mode to 1.0× first) |
| **`]`** | Double the wall-time playback rate |
| **`C`** | Print the current free-camera angle as ready-to-paste `--cam-*` flags (for `--record` — see §12) |
| **`←` (Left)** | No-op (sim can't run backwards) |

Combine: `--viewer --realtime-rate 0.5` gives a half-speed playback in
the window; then `[` halves it again to ¼ speed for inspecting
individual catches frame-by-frame.

---

## 6. Live telemetry dashboard

```bash
python sim/juggle_demo.py --viewer --dashboard --realtime-rate 0.5
```

On startup the runner prints every LAN IP the dashboard is reachable
from. Open the printed URL in a browser. On the same machine,
`http://localhost:8082` works. From another machine on the same LAN,
use the printed `(LAN)` URL — `http://<jetson-ip>:8082`. **`localhost`
on a remote machine points at the remote machine, not the Jetson**, so
the LAN IP is the one that works for cross-machine viewing.

The dashboard streams platform pose, leg extensions, hand position, and
ball positions live at the 40 Hz tick rate.

---

## 7. The capture-tolerance knob

By default the demo uses a **strict 30 mm capture gate**: a catch only
fires when the ball's centre is within 30 mm of the hand cup-opening
site at contact time. This rejects the "snap-in from cup rim" artefact
visible at lower apex (a ball brushing the rim instantly teleporting
to the cup centre under the looser any-contact rule). The Phase 2
optimiser cuts #4 (leg-jerk² objective) and #5 (leg-velocity
equalisation via minimax `v_max` slack) make the platform's catches
tight enough that the strict gate doesn't cost catch rate — the
headline 33 in 30 s is preserved.

Tighten or loosen the gate:

```bash
python sim/juggle_demo.py --capture-tolerance-mm 15        # stricter
python sim/juggle_demo.py --capture-tolerance-mm 50        # looser
```

Or disable the gate entirely (restore the legacy "any contact catches"
behaviour, which lets you see the snap-in artefact directly):

```bash
python sim/juggle_demo.py --no-capture-gate
```

Note this is *only* the juggle demo's default. The module-level
`BallManager` default in `sim/ball/manager.py` stays "no gate" so
unrelated `MuJoCoPlant` consumers (MPC catch sims, hand-stroke tests)
aren't silently affected.

---

## 8. Common scenarios

```bash
# Default: 30 s run, MuJoCo viewer, real-time
python sim/juggle_demo.py --viewer --realtime-rate 1.0

# Slow inspection of one catch — viewer at ¼ speed, hit SPACE around t≈2 s
python sim/juggle_demo.py --viewer --realtime-rate 0.25

# Lower apex, wider spread, longer run
python sim/juggle_demo.py --viewer --apex-height-mm 1000 \
    --separation-mm 250 --duration 60

# Abort test — start a 5 s run, abort at t=3 s, see the exit transient
python sim/juggle_demo.py --viewer --duration 5 --abort-at 3.0

# Headless benchmark — no window, no log; just measure catch rate
python sim/juggle_demo.py --duration 30 --no-log

# Disable capture gate — see the legacy snap-in-from-rim artefact
python sim/juggle_demo.py --viewer --no-capture-gate

# Reproducible run with BB scatter
python sim/juggle_demo.py --duration 30 --scatter-mm 5 --seed 42

# Record a fixed-camera mp4 from a chosen angle (headless, needs ffmpeg)
python sim/juggle_demo.py --record temp/reports/juggle.mp4 \
    --cam-azimuth -14 --cam-elevation -15 --cam-distance 2.8 --cam-lookat -0.1 0.03 1.11
```

---

## 9. Tests

```bash
# Just the juggle-demo tests (~30 s)
pytest tests/sim/test_demo_juggle_sim.py \
       tests/sim/test_demo_juggle_optimizer.py \
       tests/sim/test_demo_timeline.py \
       tests/sim/test_demo_trajectory.py -v

# Full suite
./run_tests.sh          # the per-commit gate — does NOT run the juggle-demo tests
./run_tests.sh --full   # includes them (they are `nightly`-marked since 2026-08-01)
```

`test_demo_juggle_{sim,optimizer,planner}.py` carry
`pytestmark = pytest.mark.nightly`: the demo is research characterization with no
path to the hardware leg/hand command chain, so it runs at 04:00 via
`tools/nightly_suite.sh` and on `--full`, not on every commit. Naming the files
explicitly (the first block above) still runs them — that is the right loop when
working on the demo. `test_demo_timeline.py` / `test_demo_trajectory.py` are NOT
demoted.

The juggle-demo tests include a 30-second end-to-end run; expect ~30 s
wall time per test.

---

## 10. Troubleshooting

**"This site can't be reached" when opening the dashboard from another
machine.** You used `localhost` on the remote box. Use the `(LAN)` URL
the runner printed, e.g. `http://192.168.20.34:8082`.

**MuJoCo viewer is sluggish / low framerate.** Make sure you're on a
recent commit — shadows and floor reflections were disabled in the
model XML for performance. If still sluggish, try
`--realtime-rate 0` (free-running, no wall-pacing — let MuJoCo render
as fast as it can).

**"Could not find module 'casadi'".** Re-run `pip install -r sim/requirements.txt` — casadi is listed there.

**Optimiser takes >5 s at startup.** First IPOPT call also compiles the
analytic-oval warm-start path. Subsequent runs in the same Python
session reuse the compilation. The standalone CLI doesn't cache between
runs.

**"infeasible tempo" ValueError.** You picked an apex too low for the
two-ball pattern to close (see Phase 1 feasibility study in the plan
doc). The lower bound is about 530 mm with the current hand-trajectory
generator; below that, the platform's idle window between catch and
throw strokes goes negative — the hand finishes catching one ball
after the other ball needs to be thrown.

**Captures dropped when tightening `--capture-tolerance-mm`.** The
default 30 mm gate is calibrated to the current optimiser (cuts #4 +
#5); tighter values may drop catches because the open-loop runner's
tracking error consumes the rest of the margin. Loosen with a larger
value, or `--no-capture-gate` to disable entirely.

---

## 11. What you're looking at

The platform traces an oval-ish closed curve between a THROW point at
`+separation/2` mm and a CATCH point at `−separation/2` mm. Banking
(platform tilt) is enabled by default — the optimiser exploits the
hand-axis tilt to give the ball part of its launch velocity, which lets
the platform move in directions you might not expect (sometimes
*opposite* the ball's flight direction). The Ball Butler stands behind
the platform and throws the priming ball; the hand catches it; Jugglebot
sustains the pattern.

The headline metric is "captures in 30 s" — 33/30 means every scheduled
catch event landed plus the Ball-Butler priming throw. Drops are
counted separately (ball falling below z = −200 mm).

For deeper context, read the plan doc:
[`plans/archived/bb-led-two-ball-juggle-demo.md`](../plans/archived/bb-led-two-ball-juggle-demo.md).

---

## 12. Recording a video from a chosen angle

`--record <path>` renders the run **offscreen from a fixed camera** and
writes an H.264 mp4. It needs `ffmpeg` on your PATH (no extra Python
package — frames are piped straight to ffmpeg) and works headless: no
`--viewer` or display required.

The live viewer always starts its free camera at the model default
(`azimuth=135, elevation=-30`). To record from a different angle:

1. **Find the angle.** Open the viewer, drag / zoom to taste, then press
   **`C`**. The runner prints your current camera as ready-to-paste
   flags:

   ```bash
   python sim/juggle_demo.py --viewer
   # drag the camera... press C →
   # [juggle_demo] camera: --cam-azimuth 90.0 --cam-elevation -20.0 \
   #                       --cam-distance 4.250 --cam-lookat 0.000 0.000 0.700
   ```

2. **Record at that fixed angle** (headless):

   ```bash
   python sim/juggle_demo.py --record temp/reports/juggle.mp4 \
       --cam-azimuth 90 --cam-elevation -20 --cam-distance 4.25 \
       --cam-lookat 0 0 0.7
   ```

Notes:

- Any `--cam-*` flag you omit inherits the model default for that axis,
  so `--record` with no camera flags reproduces the viewer's startup
  angle. `--cam-lookat` takes **three space-separated** values (each may
  be negative); `--cam-azimuth` / `--cam-elevation` / `--cam-distance`
  are single numbers.
- Playback is real-time at `--record-fps` (default `40` = one frame per
  40 Hz tick); a lower fps yields slow-motion. `--record-size` (default
  `1280x720`) sets resolution — the renderer enlarges the model's
  offscreen framebuffer to fit automatically.
- The mp4 captures exactly what the physics produced — platform, hand,
  and both balls — from the post-step state each tick.
