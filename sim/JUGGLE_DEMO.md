# Jugglebot juggle-demo sim — quickstart

A standalone MuJoCo simulation of Jugglebot performing a Ball-Butler-led
two-ball oval juggle. The platform plays an offline-optimised periodic
trajectory; the Ball Butler primes the pattern with one throw; Jugglebot
then sustains the pattern for 30+ catches.

This README gets a collaborator from a fresh clone to a working sim
window in ~5 minutes. The deeper architecture, design rationale, and the
in-progress optimiser cuts live in
[`plans/active/bb-led-two-ball-juggle-demo.md`](../plans/active/bb-led-two-ball-juggle-demo.md).

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
| `--capture-tolerance-mm <mm>` | none | Strict catch gate: ball-centre within this distance of the cup centre. Default off (any rim-contact counts) — see §7. |
| `--scatter-mm <mm>` | `0` | Gaussian sigma on BB landing scatter |
| `--seed <n>` | none | RNG seed |
| `--analytic-baseline` | off | Skip the optimiser, use the un-optimised analytic oval (debugging only) |

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

By default the demo uses MuJoCo's "any contact captures" rule —
whenever the ball touches any hand-cup collision mesh, the catch fires
and a kinematic hold locks the ball to the cup centre. This gives the
demo's headline catch rate (33/30) and looks correct at the
default 1.3 m apex.

At lower apexes (e.g. `--apex-height-mm 600`) the platform's actual
catch position drifts further from the optimised target (the open-loop
runner's tracking error is a larger fraction of the cup geometry), and
the loose rule produces visible "snap-in" — the ball touches the cup
rim and instantly teleports to the centre. To reject that:

```bash
python sim/juggle_demo.py --viewer --capture-tolerance-mm 30
```

With the gate on, a catch only fires when the ball's centre is within
30 mm of the cup-opening site at contact time. Catch rate drops
substantially until the optimiser's Phase 2 follow-up cuts (leg-jerk
objective + leg-velocity bound) constrain the trajectory to something
the actuators track tightly.

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

# Strict capture gate (snap-in rejected, catch rate drops)
python sim/juggle_demo.py --viewer --capture-tolerance-mm 30

# Reproducible run with BB scatter
python sim/juggle_demo.py --duration 30 --scatter-mm 5 --seed 42
```

---

## 9. Tests

```bash
# Just the juggle-demo tests (~30 s)
pytest tests/sim/test_demo_juggle_sim.py \
       tests/sim/test_demo_juggle_optimizer.py \
       tests/sim/test_demo_timeline.py \
       tests/sim/test_demo_trajectory.py -v

# Full suite (~8 min)
pytest tests/ -q
```

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

**Captures dropped after enabling `--capture-tolerance-mm`.** Expected
— see §7. Cuts #4 / #5 of the Phase 2 optimiser will restore the
headline rate with the strict gate on.

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
[`plans/active/bb-led-two-ball-juggle-demo.md`](../plans/active/bb-led-two-ball-juggle-demo.md).
