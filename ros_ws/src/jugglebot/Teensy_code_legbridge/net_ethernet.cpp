// =============================================================================
//  net_ethernet.cpp — Ethernet bring-up implementation (QNEthernet)
// =============================================================================
#include "net_ethernet.h"

#include <QNEthernet.h>
#include "legbridge_config.h"

using namespace qindesign::network;

namespace LegBridge {

bool net_ethernet_begin() {
  // Static configuration — DHCP disabled by passing an explicit IP. We also
  // disable DHCP explicitly so QNEthernet never blocks waiting for a lease.
  Ethernet.setDHCPEnabled(false);

  const IPAddress ip   (TEENSY_IP[0], TEENSY_IP[1], TEENSY_IP[2], TEENSY_IP[3]);
  const IPAddress mask (NETMASK[0],   NETMASK[1],   NETMASK[2],   NETMASK[3]);
  // No gateway on a /30 point-to-point link — pass the local IP as gateway so
  // QNEthernet does not install a default route (mirrors the Jetson-side
  // "no ipv4.gateway" rule in teensy-can-offload.md).
  const IPAddress gw = ip;

  // Begin with the static address. Returns false if the MAC/PHY init fails.
  if (!Ethernet.begin(ip, mask, gw)) {
    return false;
  }

  // mDNS is not started (we never call MDNS.begin) — link is hardcoded.

  // Bounded wait for the link to come up so the first ping is more likely to
  // succeed; not fatal if it times out (cable may be plugged in later).
  const uint32_t deadline = millis() + 4000;
  while (!Ethernet.linkState() && millis() < deadline) {
    delay(50);
  }
  return Ethernet.linkState();
}

bool net_link_up() {
  return Ethernet.linkState();
}

void net_ethernet_service() {
  // Service lwIP: process received frames, run protocol timers. Under the
  // QNEthernet "manual loop" model this must be pumped regularly; we call it
  // from the UDP RX task at its loop cadence.
  Ethernet.loop();
}

}  // namespace LegBridge
