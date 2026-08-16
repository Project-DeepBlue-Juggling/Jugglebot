# ADR-0011: Pin the Jetson's dedicated-link NetworkManager connection by MAC, not by interface name

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-28
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0007](0007-point-to-point-static-link.md), [parent plan](../../plans/archived/teensy-can-offload.md)

## Context

The Jetson is multi-homed once the dedicated Jetson↔Teensy link is added
([ADR-0007](0007-point-to-point-static-link.md)):

- `eth0` — built-in NIC, house LAN, DHCP, has the default route.
- A second interface — USB-to-Ethernet adapter, dedicated link to the
  Teensy, static IP `192.168.42.1/30`, **no default route**.

How that second interface is identified to NetworkManager affects how
robust the config is to:

- Reboots.
- Plugging/unplugging the adapter.
- Adding *additional* USB-Ethernet adapters in the future.
- Different USB ports / hubs / topology changes.

Two common ways to identify an interface:

1. **By kernel-assigned name** — typically `eth1` (traditional naming)
   or `enxAABBCCDDEEFF` (predictable, derived from MAC). On this Jetson
   the kernel happens to assign `eth1` to the USB adapter.
2. **By hardware MAC address** — using nmcli's
   `802-3-ethernet.mac-address` property.

## Decision

Pin the connection by **MAC address**:

```bash
sudo nmcli connection add \
  type ethernet \
  con-name teensy-link \
  802-3-ethernet.mac-address 6c:1f:f7:c6:e4:6b \
  ipv4.method manual \
  ipv4.addresses 192.168.42.1/30 \
  ipv6.method disabled \
  connection.autoconnect yes
```

(The MAC `6c:1f:f7:c6:e4:6b` is the actual adapter on this Jetson, recorded
here for traceability; substitute your adapter's MAC if rebuilding.)

## Consequences

**Positive:**

- **Stable identity through every common failure mode.** Reboot, replug,
  hub-port change, add-another-adapter — the connection binds only to
  the specific physical adapter with that MAC.
- **No `predictable-network-name` gotcha.** The kernel could assign
  `eth1`, `eth2`, `enxAABBCCDDEEFF`, or any other name depending on its
  naming policy and enumeration order. MAC-binding sidesteps the
  question.
- **Future-proof against adapter swaps.** If the adapter ever fails and
  is replaced, exactly one nmcli command (`nmcli connection modify
  teensy-link 802-3-ethernet.mac-address NEW:MAC`) re-binds the connection.

**Negative:**

- The MAC is hardware-specific, so the config file isn't
  copy-paste-portable across Jetsons without a quick edit. Worth it for
  the robustness; the trade-off is documented in the parent plan's
  Jetson network setup section.

**Neutral:**

- The `ifname`-pinned form (`ifname eth1`) also works and is what
  earlier drafts of the plan suggested. MAC-pinning is strictly more
  robust without any operational downside.

## Alternatives considered

- **Pin by interface name (`ifname eth1`).** Works as long as `eth1`
  doesn't move. Rejected because that's a fragile assumption — the
  kernel's naming policy can change between Ubuntu versions, and
  additional adapters re-order names.
- **Use the `enxAABBCCDDEEFF` predictable name.** Functionally
  equivalent to MAC-pinning (the name *is* the MAC). Either works; the
  explicit `802-3-ethernet.mac-address` is more readable in `nmcli
  connection show` output.
- **DHCP from a local DHCP server on the dedicated link.** Way too much
  machinery for a 2-device link; rejected in
  [ADR-0007](0007-point-to-point-static-link.md).
