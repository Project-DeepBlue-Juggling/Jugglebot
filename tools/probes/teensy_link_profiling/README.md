# teensy_link_profiling

Offline validation + profiling tools for the can-bridge Teensy
(`ros_ws/src/jugglebot/Teensy_code_canbridge/`). See
[`plans/archived/2026-08-15 teensy-can-offload.md`](../../../plans/archived/2026-08-15%20teensy-can-offload.md)
and the firmware-WIP handoff.

## `hermite_xref/` — interpolator validation

Cross-checks the firmware's 500 Hz interpolator against the production
`motor_guard.py`. The C++ `leg_interp.cpp` is a line-for-line transcription of
`teensy_interp.py`, which `xref.py` proves matches `motor_guard` to **0.0 rev**
(synthetic + recorded). See [`hermite_xref/README.md`](hermite_xref/README.md).

```bash
python hermite_xref/xref.py                 # newest temp/logs/mpc_*.csv
pytest tests/firmware/test_hermite_xref.py  # in the suite
```

## `jetson/` — Jetson-side UDP tools

Both import the generated `jetson/udp_protocol.py` (delivered by
`config/generate_udp_protocol.py`). **Run one at a time** — both bind the STREAM port.

### `profile_monitor.py` — diagnostic consumer

Ingests the 1 Hz PROFILE frame (per-task CPU%, CAN1/CAN2 bus utilisation, UDP
RTT/jitter, the 500 Hz interp deadline-miss counter, free heap), logs PROFILE rows
to CSV, and renders matplotlib plots. Single-script.

```bash
python jetson/profile_monitor.py --duration 60          # listen 60 s, then plot
python jetson/profile_monitor.py --bind-ip 192.168.42.1 # the teensy-link interface
```

Outputs to `temp/probes/teensy_link_profiling/` (gitignored): `profile_<ts>.csv`
and `profile_<ts>.png`. The PROFILE `cpu_pct_x100[]` slot order is shared with the
firmware (`profiling.cpp` / `profiling.h`):
`canrx, tsync, net, fault, telem, hb, diag, IDLE, other`. The 500 Hz interpolator
runs in an ISR (not a task), so its health is the `interp_*` fields, not the CPU
slots.

### `setpoint_stub.py` — bench exerciser / Phase-4 stub client

Stands in for the (Phase 10) Jetson UDP bridge: sends a synthetic 40 Hz SETPOINT
sinusoid + 10 Hz heartbeats, answers the time-of-day RPC with `CLOCK_REALTIME`,
and prints the uplink. Lets you exercise the full firmware data path without the
real MPC.

```bash
python jetson/setpoint_stub.py --teensy-ip 192.168.42.2
python jetson/setpoint_stub.py --no-output   # mpc_active off → firmware holds, no CAN TX
```

## Notes

- The firmware emits PROFILE on the STREAM port at 1 Hz (`profiling.cpp`, driven
  by the diag task). Per-task CPU requires `configGENERATE_RUN_TIME_STATS` +
  `configUSE_TRACE_FACILITY` in FreeRTOSConfig (else the CPU fields read 0).
- `jetson/udp_protocol.py` is generated — regenerate with
  `python config/generate_udp_protocol.py`, don't hand-edit.
