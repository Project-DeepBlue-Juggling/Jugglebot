# Hermite/Taylor interpolator cross-check

Offline validation that the leg-bridge Teensy's 500 Hz interpolator
(`ros_ws/src/jugglebot/Teensy_code_legbridge/leg_interp.cpp`) reproduces the
production `motor_guard.py` exactly.

## How it works

`leg_interp.cpp` (C++) is a line-for-line transcription of `teensy_interp.py`
(pure Python, no numpy, per-leg loops — written to *be* the C++). `xref.py`
drives **both** `teensy_interp.TeensyLegInterp.tick` and the real
`motor_guard.MotorGuard._interpolate_and_send` through identical inputs and
reports the maximum per-tick commanded-position/velocity divergence.

So the chain of trust is:

```
motor_guard.py  ──(xref.py, 0.0 rev)──  teensy_interp.py  ──(line-for-line)──  leg_interp.cpp
   (truth)                                 (validated)                          (firmware)
```

`motor_guard`'s wall clock is monkeypatched to a deterministic clock; the guard's
interpolation base state is set identically to the port's, isolating the ladder
math (the port target) from motor_guard's separate workspace/deviation/E-stop
machinery (which stays on the Jetson and is not ported).

Coverage: all three ladder modes (Hermite in-segment, Hermite-late `s→1`,
Hermite-without-`u2`, cubic Taylor, velocity-decay) + the lead clamp and stroke
clamp, on synthetic cases **and** a recorded MPC `cmd_ext` stream.

## Run

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
python tools/probes/teensy_link_profiling/hermite_xref/xref.py [recorded_mpc.csv]
# defaults to the newest temp/logs/mpc_*.csv
```

Also runs in the test suite: `pytest tests/firmware/test_hermite_xref.py`.

## Result (2026-06-02)

Synthetic **and** recorded (`mpc_20260524_153227.csv`, 200 rows): **max |Δpos| =
0.0 rev, max |Δvel| = 0.0 rev/s** in float64 — the algorithm is bit-identical to
`motor_guard`. The firmware runs the same math in float32 (single-precision FPU);
the only expected residual on hardware is float32-vs-float64 rounding (plan
Phase 7 risk), to be measured on the bench. `teensy_interp.py` and `leg_interp.cpp`
must be kept in sync — change one, mirror the other, re-run this.

## Files

- `teensy_interp.py` — the C++ translation target (validated Python port).
- `xref.py` — the cross-check driver (runnable + importable by the pytest wrapper).
