# CAN Bridge — Teensy 4.1 Hardware Offload

The can-bridge is a dedicated Teensy 4.1 microcontroller that hosts CAN bus
communication and the leg setpoint interpolator, moving both off the
Jetson's non-real-time Linux scheduler. It talks to the Jetson over a
point-to-point UDP/Ethernet link and owns three isolated CAN buses.

**Related:** [ADR-0001: Offload CAN and the interpolator from the
Jetson](../adr/0001-offload-can-and-interpolator-from-jetson.md) (why this
exists) · [Wire protocol reference](../teensy-udp-protocol.md) ·
[Parent plan](../../plans/archived/teensy-can-offload.md) (current
implementation status) · [Architecture Decisions](../adr/index.md)
(ADR-0002 through ADR-0013 cover the individual hardware/protocol choices)

- **[Control Flow](control.md)** — how the legs and hand are actually
  commanded: the 40 Hz→500 Hz leg interpolation ladder, joint-limit
  clamping, and the hand's separate relay path to the platform Teensy
- **[Safety Mechanisms](safety.md)** — staleness watchdogs, the E-STOP
  fault machine, joint/velocity/deviation limits, and CAN bus health
  monitoring

## Topology

```
                         static /30, UDP only
   Jetson Orin Nano  <───────────────────────>  can-bridge Teensy 4.1
   (192.168.42.1)         192.168.42.2           (FreeRTOS, time-sync master)
                                                          │
                              ┌───────────────────────────┼───────────────────────────┐
                              │                            │                            │
                          CAN1 (1 Mbps)               CAN2 (1 Mbps)               CAN3 (1 Mbps, FD-capable)
                          bb role                     jugglebot role              cone role
                          Ball Butler bus             Jugglebot core bus          Catching-cone bus
                          (throw axis Teensy)         6 leg ODrives + Hand        (often disconnected;
                                                      ODrive + platform           an electronic clapboard
                                                      Teensy 4.0                  may share the segment)
```

> **The jugglebot and cone ROLES are swapped relative to their controller
> numbers, since 2026-07-31.** The bridge's CAN3 analog drive path developed a
> load-dependent fault: it drives the 1-node cone bus cleanly at 100 Hz but fails
> within seconds against the 8-node Jugglebot chain. The roles were swapped and
> the physical plugs moved to match — see
> `logbook/2026-07-31-can3-drive-path-fault-jugglebot-to-can2.md` and the
> authoritative declaration at `can_buses.cpp:18-44`. Wire-slot names
> (`bus1_health`, the `can1_*`/`can3_*` PROFILE slots, the `can3_errors` row) are
> **role-keyed** and did not move; ADR-0013 records the original 2026-06-03
> mapping and carries a marked amendment block recording this swap.

All three buses carry a shared 100 Hz time-sync broadcast (`0x7DD`) from the
can-bridge, which is the sole time-sync master for the system (it
bootstraps and periodically re-syncs its own wall clock against the
Jetson). The platform Teensy 4.0 is unchanged in scope — it still owns hand
trajectory execution and inclinometer/robot-state persistence — but is now
a time-sync slave on the Jugglebot core bus rather than the old Jetson-hosted
stack's peer.

Firmware source: [`ros_ws/src/jugglebot/Teensy_code_canbridge/`](../../ros_ws/src/jugglebot/Teensy_code_canbridge/).

## Current status

!!! warning "Snapshot, not a live status page"
    This section reflects the state as of **2026-07-06**. The authoritative,
    continuously-updated status lives in
    [`plans/archived/teensy-can-offload.md`](../../plans/archived/teensy-can-offload.md)
    (see its "Status snapshot" section) — check there before relying on
    anything below for a decision.

**Live in production today:** heartbeats, telemetry, homing, `ACTIVATE`/
`DEACTIVATE`, encoder search, gain writes, the hand trajectory relay, the
full fault/watchdog machine, and the fully-automated cold-start
orchestrator (BOOT → HOMING → IDLE). The Jetson's old SocketCAN stack
(`can_node.py`) was deleted entirely on 2026-07-06 (Phase 13 of the parent
plan) — the can-bridge is the only CAN path that exists now.

**Not yet live:** the 40 Hz MPC → Teensy leg setpoint stream — the dynamic,
closed-loop leg interpolation described in [Control Flow](control.md). The
Hermite math has two independent validations: a Python port
(`teensy_interp.py`) is checked bit-exact-ish (`<1e-6` rev) against
`motor_guard.py` via `tools/probes/teensy_link_profiling/hermite_xref/`
(`tests/firmware/test_hermite_xref.py`), and the actual compiled
`leg_interp.cpp` firmware is separately behavior-validated (clamps, mode
transitions, deferred-stow completion) on a native host build
(`tests/firmware/native/test_leg_interp.cpp`) — that native test explicitly
does not check bit-exact float equality against the Python mirror (host
float32 vs. Python float64). It has been bench tested with live 40 Hz MPC
streaming at single-leg scale (`logbook/2026-06-24-phase11-bench-cutover.md`);
six-leg testing to date has validated the cold-start ladder (home/activate/
configure via TRAP_TRAJ) but not yet the dynamic Hermite/MPC stream at
six-leg scale — that closed-loop `run_mpc` hold was deferred and later
de-prioritized (`logbook/2026-06-26-phase11-u5-six-leg-cutover.md`,
`logbook/INDEX.md`). The bridge starts DISARMED (`mpc_active=0`) at every
launch; arming is runtime-only via `/set_setpoint_output`, whose
stream-then-arm pre-check is the single safe-to-arm gate — automatic on
ACTIVE entry since the 2026-07-15 arming contract
(`ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md`; the old
`enable_setpoint_output:=true` boot-arm is inert). No leg is currently
driven dynamically through the MPC
path — it is parked pending the parent plan's Phase 12 cutover, and
possibly a redesign of the MPC itself (from a 40 Hz continuous trajectory
generator to a lower-rate/event-triggered replanner, since the Jetson is
compute-marginal at 40 Hz).
