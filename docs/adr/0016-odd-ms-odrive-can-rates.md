# ADR-0016: Every cyclic ODrive CAN message runs at an odd number of milliseconds

- **Status**: Accepted
- **Date**: 2026-11-08
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0013](0013-three-can-buses.md) (the Jugglebot bus, CAN3), [logbook: the clock-beat finding](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/logbook/2026-11-06-arrival-pull-and-hand-dropout-probes.md), [logbook: the change](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/logbook/2026-11-08-odrive-odd-can-rates.md), BUG-82 and BUG-50 in the project map

## Context

The Jugglebot ODrives broadcast their cyclic CAN messages at ODrive's round-number defaults: encoder and iq every 10 ms, error 20, heartbeat and temperature 100, bus voltage 500. The can-bridge sends its Leg setpoint burst on a 2 ms tick.

From 2026-09 the bridge saw encoder dropouts of 0.6 to 2 s on single axes. One of them starved the Hand's feedback for 701 ms and faulted it (BUG-50, 2026-11-06). Offline probes of five bags (78 episodes) showed a **clock beat**:
- A drive's 10 ms schedule is a whole multiple of the 2 ms tick. Its frame phase relative to the tick drifts only at the clocks' slip, 7 to 15 ppm.
- When the phase reaches one narrow window, 6 to 30 µs wide, every frame is lost until the drift carries it out again: 0.6 to 3 s.
- On each axis the episodes recur at whole multiples of 2000 µs divided by that drive's slip, which matched within 4 to 14 %.
- Ruled out: bus load, the sensor poller, bridge RX overflow, an electrical cause.
- Still unknown: whether the drive doesn't send the frame or the bridge doesn't receive it.

## Decision

Every cyclic ODrive message on the Jugglebot bus runs at an **odd** number of milliseconds, 1 ms faster than ODrive's default:

| Message | Default | Jugglebot |
|---|---|---|
| encoder | 10 | 9 |
| iq | 10 | 9 |
| error | 20 | 19 |
| heartbeat | 100 | 99 |
| temperature | 100 | 99 |
| bus voltage | 500 | 499 |

A message that is switched off (0) stays off. This applies to all six Legs and the Hand (`config/ODrive config Files/odrive_pro_leg_config.json`, `odrive_pro_hand_config.json`).

**Why odd works.** An odd period is a half-integer number of 2 ms ticks. Successive frames therefore alternate between two phases 1 ms apart, and a 6 to 30 µs window can hold at most one of them. At most every other frame can be lost, so the worst encoder gap falls from 80 to 100 ms to about 18 ms, under the firmware's 30 ms Hand freshness gate.

**Why 1 ms faster rather than slower.** Every firmware and host threshold on these messages is a maximum age (20, 30 and 150 ms; 0.5 s; 2 s). A shorter period can never present a threshold with an older frame than before.

**Enforcement:** `tests/motion/test_odrive_fleet_reflash.py::test_backup_configs_use_odd_ms_can_rates` fails if any non-zero `*_msg_rate_ms` in either backup JSON is even. The drives themselves are written with `tools/odrive_fleet_reflash.py --set ... --include-hand`.

## Consequences

- **Evidence:** at the 2026-11-07 Sitting, Leg 4 at 9 ms had 0 of 2396 `/cache_diag` windows over 25 ms (peak 17.1 ms), while the five Legs still at 10 ms had 15 to 25 windows over 50 ms. The Hand at 9 ms was clean over two Sittings.
- **Bus load:** about 2 % more on the Jugglebot bus.
- **Measurement gap:** only encoder and heartbeat frames are counted by the bridge, so the gain on error, iq, temperature and bus voltage is expected but not measured.
- **Off-by-one numbers are deliberate.** Anyone restoring a drive from ODrive defaults, or "tidying" 9 back to 10, reintroduces the beat. The backup JSONs and the test above are the guard.
- **Other buses:** Ball Butler (ODrive S1, on CAN1) was not changed, because nothing pointed at it. Any new cyclic sender on a bus with a 2 ms tick should follow the same rule.
- **Re-homing:** changing a rate means a drive save, which reboots the drive and wipes its encoder zero. Check the launch log for `homing: 7 axes homed` before arming.

## Alternatives considered

- **Leave the rates and dither the bridge's 2 ms setpoint burst in firmware.** This fixes every message on every drive at once, but it changes interrupt timing on the safety path. Kept in reserve in case odd periods leave a residue.
- **Faster even rates, e.g. 8 ms.** Still a whole multiple of 2 ms, so it stays phase-locked; shorter runs, same failure.
- **Slower odd rates, e.g. 11 ms.** Equally beat-free, but every age threshold would see older frames than today.
- **Change one message per trial.** Each save costs a re-home, and the two counted streams are each judged by their own counter, so the owner chose one save per drive.
