# ODrive config backups

Backups of each drive type's full configuration. Restore one with `odrivetool restore-config`. To write named properties to the Leg drives (and with `--include-hand`, the Hand) without erasing, use `tools/odrive_fleet_reflash.py --set`.

| File | Drives |
|---|---|
| `odrive_pro_leg_config.json` | the six Legs (CAN nodes 0 to 5) |
| `odrive_pro_hand_config.json` | the Hand (CAN node 6) |
| `odrive_s1_bb_hand_config.json` | Ball Butler |
| `*_flat_endpoints.json` | the firmware's endpoint list, read by the fleet tool to type-check `--set` |

## The CAN rates are off by one on purpose

On the Jugglebot drives every cyclic CAN message runs at an odd number of milliseconds:
- encoder and iq: 9
- error: 19
- heartbeat and temperature: 99
- bus voltage: 499

ODrive's defaults are 10, 20, 100 and 500. An even period is a whole multiple of the can-bridge's 2 ms setpoint tick, so a drive's frames drift into a lossy phase and stay there, losing every frame for 0.6 to 3 s. One such dropout faulted the Hand. An odd period alternates between two phases, so at most every other frame can be lost.

Do not round them back. The reasoning and evidence are in `docs/adr/0016-odd-ms-odrive-can-rates.md`, and `tests/motion/test_odrive_fleet_reflash.py` fails if either Jugglebot file holds an even rate.
