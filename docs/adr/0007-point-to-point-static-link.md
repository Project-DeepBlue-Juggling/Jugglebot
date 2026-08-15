# ADR-0007: Point-to-point Ethernet between Jetson and can-bridge Teensy (no shared LAN)

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-28
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0005](0005-ethernet-over-usb-serial.md), [ADR-0006](0006-udp-not-tcp.md), [ADR-0011](0011-mac-pinned-nmcli.md), [parent plan](../../plans/archived/2026-08-15%20teensy-can-offload.md)

## Context

The Jetson is on the user's house LAN (`eth0` on `192.168.20.0/24`) for SSH,
ROS 2 development, and internet access. The can-bridge Teensy needs an
Ethernet endpoint to talk to. Two ways to wire that up:

1. **Shared LAN.** Plug the Teensy into the user's house network alongside the
   Jetson. Both use the existing LAN segment.
2. **Dedicated point-to-point link.** Add a USB-to-Ethernet adapter on the
   Jetson, run a private cable to the Teensy, configure a tiny static-IP
   subnet that doesn't carry any other traffic.

## Decision

Use a **point-to-point link** with static IPs:

- USB-to-Ethernet adapter on the Jetson, dedicated to this link.
- Cat5e/Cat6 cable directly to the Teensy's PJRC Ethernet kit.
- `/30` subnet `192.168.42.0/30`: Jetson at `192.168.42.1`, Teensy at
  `192.168.42.2`. No gateway. No DHCP. No mDNS. No DNS.
- The Jetson's existing house-LAN connection (`eth0`) is unaffected; the
  dedicated link is a completely separate interface.

## Consequences

**Positive:**

- **Deterministic timing.** The link is not shared with any other traffic.
  Jitter is bounded by the two devices' Ethernet stacks alone, not by
  whatever else is on the LAN.
- **No DHCP races.** Both sides have static configuration. Boot ordering
  doesn't matter — link comes up as soon as both endpoints are powered.
- **No mDNS discovery.** Teensy IP is `192.168.42.2`, hardcoded in
  firmware. Failure modes reduce to "is the cable plugged in?" and "is the
  Teensy alive?" — exactly the modes you want to be debugging.
- **No security surface.** The link is not reachable from anything except
  the two endpoints. No firewall config required.
- **House LAN going down doesn't affect the robot.** The dedicated link is
  a separate L2 segment.
- **Survives Jetson swap.** If the Jetson is ever replaced, only the
  Jetson-side `nmcli` config and USB-Ethernet adapter need to move. The
  Teensy's static config is unchanged.
- **Matches industrial practice.** EtherCAT, PROFINET, etc. all use
  dedicated point-to-point links for the same reasons. This is a
  well-validated pattern.

**Negative:**

- One extra USB-to-Ethernet adapter on the Jetson (~$15).
- The Jetson is now multi-homed (two Ethernet interfaces). Care required
  to avoid setting a default route on the Teensy link — see
  [ADR-0011](0011-mac-pinned-nmcli.md).
- Cannot remote-debug the Teensy from another machine on the house LAN
  without going through the Jetson. Acceptable trade: the Teensy is a
  bench device, and the Jetson is the gateway.

**Neutral:**

- The Jetson can still SSH, ROS 2, and reach the internet via its house
  LAN exactly as before.

## Alternatives considered

- **Shared LAN with static IPs from the user's network.** Rejected because
  it doesn't get the timing-isolation property — the can-bridge timing
  would be coupled to whatever else lives on the user's LAN (printers,
  laptops, video streaming, etc.).
- **Shared LAN with DHCP.** Rejected for the same reason plus DHCP-race
  issues at boot.
- **Switch in between.** Adds a hop, adds a point of failure, no
  compensating advantage at this scale.
