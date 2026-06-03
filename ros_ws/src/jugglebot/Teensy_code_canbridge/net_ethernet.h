#pragma once
// =============================================================================
//  net_ethernet.h — Ethernet bring-up (QNEthernet, static IP, no DHCP/mDNS)
// =============================================================================
//  Brings the PJRC Ethernet kit (DP83825I PHY) up at the static point-to-point
//  address 192.168.42.2 / 255.255.255.252, no DHCP, no mDNS. The PHY is held in
//  reset until Ethernet.begin() runs here — until then the Jetson-side USB-Eth
//  adapter shows <NO-CARRIER> (expected; see teensy-can-offload.md Phase 1).
// =============================================================================

#include <cstdint>

namespace CanBridge {

// Configure + start the Ethernet interface at the static IP. Returns true once
// the link reports up (or after a bounded wait — caller may proceed regardless).
bool net_ethernet_begin();

// True if the PHY link is currently up (cable connected, peer present).
bool net_link_up();

// Must be pumped regularly to service lwIP (RX, timeouts). Called from the
// UDP RX task. QNEthernet processes received frames in this call.
void net_ethernet_service();

}  // namespace CanBridge
