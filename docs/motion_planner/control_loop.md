# Motor guard (parked fallback, Hermite xref twin)

`ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py` is **not on the live path**. `jugglebot_launch.py` has not started it since 2026-08-01.

## What it was

A 500 Hz interpolator plus monitor on the pre-cutover leg path: it took low-rate targets from the planner, interpolated them, and gated motor commands on feedback. That job now belongs to the can-bridge Teensy, which does the 500 Hz Hermite interpolation in firmware.

## Why it is retained

- **Hermite xref twin.** It is the validated Python twin that the `hermite_xref` firmware trust chain drives (`tools/probes/teensy_link_profiling/hermite_xref/xref.py`): firmware output is checked against it.
- **Its safety tests run per-commit:** E-stops, NaN rejection, workspace limits and staleness. They keep the twin honest.

## What it is not

- Not on the live path, and its former feeder and consumer are deleted.
- Not the leg-path safety layer. The authority is the Teensy-side `MAX_DEVIATION` guard in the can-bridge firmware; see [CAN Bridge safety](../can_bridge/safety.md).
- Not something to launch for a session. Do not describe it as protecting the robot.

## Where the live path is documented

- [Overview](index.md) and [Architecture](architecture.md) for the skill-stack data path.
- [CAN Bridge control flow](../can_bridge/control.md) for the interpolation ladder and the hand lane.
- The previous operating modes and IPC layer of the guard live at tag `mpc-final`.
