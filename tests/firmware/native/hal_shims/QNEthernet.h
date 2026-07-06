#pragma once
// =============================================================================
//  QNEthernet.h — HOST fake for QNEthernet (native udp_link harness)
// =============================================================================
//  Stands in for ssilverman/QNEthernet when udp_link.cpp (and its 1-line
//  net_ethernet_service forwarder) is compiled on the build host and tested as a
//  real binary. Provides ONLY what udp_link.cpp reaches:
//
//    * IPAddress (Arduino-style, global scope);
//    * qindesign::network::EthernetClass with a global `Ethernet` (loop());
//    * qindesign::network::EthernetUDP with begin/parsePacket/read/beginPacket/
//      write/endPacket/remoteIP/remotePort.
//
//  Every method calls netlockprobe::require_held() at entry: this is the LOCK
//  COVERAGE probe. A call made outside NetLock (the unlocked-pump regression)
//  records a violation the driver asserts is zero — the regression witness.
//
//  The socket also carries a test-only staging queue (bind registers it by port
//  so the driver can push pending packets) to drive the RX drain-budget test.
//  Host-only: validates lock coverage + the drain budget, NOT true preemptive
//  reentrancy (an on-hardware gap — see README.md).
// =============================================================================

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <map>
#include <vector>

#include "net_lock_probe.h"

// ── IPAddress (Arduino-style, global — QNEthernet re-exports the core type) ───
class IPAddress {
 public:
  IPAddress() : a_(0), b_(0), c_(0), d_(0) {}
  IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
      : a_(a), b_(b), c_(c), d_(d) {}
  uint8_t a_, b_, c_, d_;
};

namespace qindesign {
namespace network {

// ── Ethernet (only loop() is exercised, via net_ethernet_service) ─────────────
class EthernetClass {
 public:
  void setDHCPEnabled(bool) {}
  bool begin(IPAddress, IPAddress, IPAddress) { return true; }
  bool linkState() { netlockprobe::require_held(); return true; }
  // The lwIP pump. MUST run under NetLock post-fix; require_held() is the witness.
  void loop() { netlockprobe::require_held(); ++loops_; }
  unsigned loops_ = 0;
};
inline EthernetClass Ethernet;   // C++17 inline var → single instance, header-only

// Port → socket registry so the driver can reach udp_link.cpp's file-static
// s_stream/s_rpc (bound in udp_link_init) to stage packets and inspect TX.
// Declared before EthernetUDP::begin() uses it (pointer-to-incomplete is fine).
class EthernetUDP;
inline std::map<uint16_t, EthernetUDP*>& _registry() {
  static std::map<uint16_t, EthernetUDP*> m;
  return m;
}

// ── EthernetUDP (per-instance staged RX queue + TX capture) ───────────────────
class EthernetUDP {
 public:
  // ―― firmware-facing API (each asserts NetLock coverage) ――
  bool begin(uint16_t port) {
    netlockprobe::require_held();
    port_ = port;
    _registry()[port] = this;
    return true;
  }

  // Advance to the next staged packet and return its size (0 when drained).
  int parsePacket() {
    netlockprobe::require_held();
    if (next_ < pending_.size()) {
      cur_ = pending_[next_++];
      return (int)cur_.size();
    }
    cur_.clear();
    return 0;
  }

  // Copy the current packet into `buf` (a null/zero read discards, as the
  // firmware does for an oversized frame).
  int read(uint8_t* buf, size_t len) {
    netlockprobe::require_held();
    if (!buf || len == 0) return 0;
    const size_t n = (len < cur_.size()) ? len : cur_.size();
    memcpy(buf, cur_.data(), n);
    return (int)n;
  }

  bool beginPacket(IPAddress, uint16_t) {
    netlockprobe::require_held();
    return true;
  }
  size_t write(const uint8_t*, size_t len) {
    netlockprobe::require_held();
    tx_bytes_ += len;
    return len;
  }
  bool endPacket() {
    netlockprobe::require_held();
    ++tx_packets_;
    return true;
  }
  IPAddress remoteIP()  { netlockprobe::require_held(); return IPAddress(192, 168, 42, 1); }
  uint16_t  remotePort() { netlockprobe::require_held(); return 5005; }

  // ―― test-only staging / inspection ――
  void stage(const uint8_t* d, int len) { pending_.emplace_back(d, d + len); }
  int  remaining() const { return (int)pending_.size() - (int)next_; }
  int  consumed()  const { return (int)next_; }
  unsigned tx_packets() const { return tx_packets_; }
  void clear() { pending_.clear(); cur_.clear(); next_ = 0; tx_packets_ = 0; tx_bytes_ = 0; }

 private:
  uint16_t port_ = 0;
  std::vector<std::vector<uint8_t>> pending_;
  std::vector<uint8_t> cur_;
  size_t next_ = 0;
  unsigned tx_packets_ = 0;
  size_t tx_bytes_ = 0;
};

}  // namespace network
}  // namespace qindesign
