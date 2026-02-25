# tools/ — Standalone Hardware Test & Debug Utilities

Scripts in this directory bypass ROS2 and talk directly to hardware via python-can. They are designed for bench testing, bring-up, and diagnostics.

## single_leg_test.py

Standalone single-leg test harness implementing the four **Phase 2 isolated-leg bench tests** from `MOTION_PLANNER_PLAN.md`. Runs on the Jetson (socketcan) or any machine with a USB-CAN adapter.

### Prerequisites

```bash
pip install python-can numpy
```

The script imports `protocol_config.py` and `hardware_config.py` directly from `config/generated/` (no ROS2 package install required).

### Architecture

```
single_leg_test.py
    |
    +-- python-can Bus (socketcan / USB-CAN)
    |       |
    |       +-- ODrive 0 (axis_id configurable)
    |
    +-- protocol_config.py (CAN IDs, states, modes)
    +-- hardware_config.py (current limits, spool geometry)
```

The harness has three layers:

1. **CAN I/O** — Message encoding/decoding is inlined (mirrors `can/odrive.py` without importing from the ROS2 package). Messages are sent with 2ms inter-message pacing to avoid CAN buffer overflow. All pending messages are polled and dispatched after every send.

2. **AxisState tracker** — A dataclass that accumulates heartbeat, encoder, error, and current data from CAN messages. The heartbeat watchdog will raise an error if no heartbeat arrives within 2 seconds.

3. **Test functions** — Each test is a standalone function that takes the harness and returns `True` (pass), `False` (fail), or `None` (skipped). Every test idles the axis on completion or error.

### Safety Design

- **Conservative current limit**: Set to 50% of rated (10A vs 20A rated) before any test
- **Universal IDLE**: Every test sends IDLE on completion, error, or Ctrl-C (via signal handler)
- **Heartbeat gate**: The harness waits for a heartbeat before sending any commands. If heartbeats stop during a test, it raises an error.
- **No homing required**: Tests use torque mode and position hold from current position — they do not require the leg to be homed first
- **Single-axis only**: The harness only processes messages from the configured axis, ignoring all others

### Tests

#### Test 1: Torque Passthrough Smoke Test (`--test smoke`)

Verifies that the ODrive accepts torque commands and that encoder feedback works.

1. Clears errors, sets safe current limits
2. Enters TORQUE/PASSTHROUGH mode in CLOSED_LOOP
3. Commands a small constant torque (0.02 Nm) for 2 seconds
4. Verifies the encoder position changed (leg moved)
5. Sends IDLE
6. Verifies velocity settles to near-zero (leg stopped cleanly)

**Pass criteria**: Encoder delta > 0.01 rev, post-IDLE velocity < 0.05 rev/s.

#### Test 2: Emergency Stop (`--test estop`)

Verifies that sending IDLE immediately stops the motor.

1. Commands torque to get the leg moving (~1 second)
2. Sends IDLE and timestamps the transition
3. Polls until IDLE state is confirmed
4. Verifies: IDLE confirmed within 100ms, no errors, velocity settled to zero

**Pass criteria**: IDLE latency < 100ms, no active errors, residual velocity < 0.05 rev/s.

#### Test 3: Encoder Sign Check (`--test encoder`)

Determines the sign convention between motor torque and encoder position.

1. Commands a positive torque pulse (0.03 Nm for 1 second), records encoder delta
2. Commands a negative torque pulse (-0.03 Nm for 1 second), records encoder delta
3. Verifies the deltas have opposite signs (consistent)
4. Reports whether the convention matches the Jugglebot model (positive torque = negative encoder = leg extension)

**Pass criteria**: Both pulses cause measurable movement (> 0.005 rev) and the signs are opposite.

#### Test 4: Force Conversion Validation (`--test force`)

Validates the geometric force-to-torque conversion by comparing predicted vs measured motor current under a known static load.

1. Prompts the user to attach a known mass (in kg)
2. Computes predicted torque: `F = m*g`, `tau = F * r_spool`
3. Enters POSITION/PASSTHROUGH mode to hold the current position
4. Collects `iq_measured` samples for 3 seconds
5. Reports mean iq, predicted torque, and effective Kt
6. Instructs user to compare against motor datasheet Kt

**Pass criteria**: Manual — requires comparing measured iq against predicted iq using the motor's torque constant from the datasheet. Discrepancy should be < 10%.

### Usage

```bash
# Run all tests on axis 0 (default)
python tools/single_leg_test.py

# Run a specific test
python tools/single_leg_test.py --test smoke

# Test a different axis
python tools/single_leg_test.py --axis 3 --test encoder

# Use a USB-CAN adapter on Windows/Linux (e.g., PCAN)
python tools/single_leg_test.py --interface pcan --channel PCAN_USBBUS1

# Use slcan (serial-line CAN)
python tools/single_leg_test.py --interface slcan --channel /dev/ttyACM0
```

### Expected Output (smoke test example)

```
Jugglebot Single-Leg Test Harness
  Axis:      0
  Interface: socketcan
  Channel:   can0
  Current limit: 10.0 A (50% of 20.0 A)
Connecting to CAN bus: can0 (socketcan)
Waiting for heartbeat from axis 0...
  Heartbeat received! State: IDLE, errors: 0

============================================================
TEST 1: Torque passthrough smoke test
============================================================
  Errors cleared. Active errors: 0
  Limits set: vel=50.0 rev/s, curr=10.0 A
  Initial position: 0.1234 rev
  Axis in TORQUE/PASSTHROUGH, CLOSED_LOOP
  Commanding 0.02 Nm for 2.0s...
  Position after torque: 0.2567 rev (delta: +0.1333 rev)
  Axis 0 -> IDLE
  Velocity after IDLE: 0.0012 rev/s
  PASS: Leg moved under torque and stopped cleanly on IDLE

============================================================
RESULTS SUMMARY
============================================================
  Torque passthrough smoke test                PASS
```
