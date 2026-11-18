#pragma once
// =============================================================================
//  Wire.h - HOST fake of the Teensy TwoWire, scripted as one ICM-42688-like I2C device
// =============================================================================
//  Just enough for the Platform Teensy's icm42688.cpp driver under test: a register file at the addresses in
//  `present`, a log of every register write and every read transaction (with the host clock at that moment),
//  and injectable faults (NACK everything, NACK from transaction N on, short reads). Counts I2C TRANSACTIONS the
//  way the bus sees them: a write (endTransmission(true)) is one, a register read (endTransmission(false) then
//  requestFrom) is one, an address probe is one.
// =============================================================================
#include <cstdint>
#include <vector>

#include "Arduino.h"

struct WireEvent {
  char kind;          // 'W' register write, 'R' register read, 'P' address probe
  uint8_t addr;
  uint8_t reg;
  uint8_t val;        // W: the value written
  uint32_t t_us;      // host clock (Wire fake's now_us) when it happened
  bool nacked;
};

class TwoWire {
 public:
  // ---- script ----
  uint8_t regs[256] = {};
  bool present[128] = {};
  bool nack_all = false;
  int nack_from_txn = -1;            // NACK every transaction whose index >= this (-1: never)
  int reset_done_after_reads = 1;    // INT_STATUS reads before it reports RESET_DONE (bit4); <0: never
  uint32_t now_us = 0;               // set by the test before each driver call
  // ---- record ----
  std::vector<WireEvent> log;
  int txns = 0;                      // transactions issued so far

  void reset_log() { log.clear(); txns = 0; int_status_reads_ = 0; }
  void set_device(uint8_t addr) { present[addr] = true; regs[0x75] = 0x47; }

  void beginTransmission(uint8_t a) { addr_ = a; n_ = 0; }
  size_t write(uint8_t b) { if (n_ < 4) buf_[n_] = b; n_++; return 1; }
  uint8_t endTransmission(bool stop = true) {
    bool ack = acks();
    if (n_ == 0) {                       // address probe
      txns++;
      log.push_back({'P', addr_, 0, 0, now_us, !ack});
      return ack ? 0 : 2;
    }
    if (n_ == 1) {                       // set the register pointer (a read follows)
      ptr_ = buf_[0];
      pending_read_ = stop ? false : true;
      read_nack_ = !ack;
      if (stop || !ack) {                // a lone pointer write, or a read whose address phase was NACKed
        txns++;
        if (!ack) log.push_back({'R', addr_, buf_[0], 0, now_us, true});
      }
      return ack ? 0 : 2;
    }
    txns++;                              // register write
    log.push_back({'W', addr_, buf_[0], buf_[1], now_us, !ack});
    if (ack) {
      regs[buf_[0]] = buf_[1];
      if (buf_[0] == 0x11 && (buf_[1] & 1)) int_status_reads_ = 0;   // soft reset: RESET_DONE counts from here
    }
    return ack ? 0 : 2;
  }
  uint8_t requestFrom(int a, int n) {
    (void)a;
    int idx = txns;                      // this read is transaction `idx`
    txns++;
    bool ack = !read_nack_ && acks_idx(idx);
    rd_.clear(); rd_i_ = 0;
    if (ack) {
      for (int i = 0; i < n; i++) {
        uint8_t reg = uint8_t(ptr_ + i);
        uint8_t v = regs[reg];
        if (reg == 0x2D) { v = (reset_done_after_reads >= 0 && ++int_status_reads_ >= reset_done_after_reads) ? 0x10 : 0x00; }
        rd_.push_back(v);
      }
    }
    log.push_back({'R', addr_, ptr_, 0, now_us, !ack});
    pending_read_ = false;
    return ack ? uint8_t(n) : 0;
  }
  int available() { return int(rd_.size()) - rd_i_; }
  int read() { return rd_i_ < int(rd_.size()) ? rd_[rd_i_++] : -1; }

 private:
  bool acks_idx(int idx) const {
    return !nack_all && present[addr_ & 0x7F] && !(nack_from_txn >= 0 && idx >= nack_from_txn);
  }
  bool acks() const { return acks_idx(txns); }
  uint8_t addr_ = 0, buf_[4] = {}, ptr_ = 0;
  int n_ = 0, int_status_reads_ = 0, rd_i_ = 0;
  bool pending_read_ = false, read_nack_ = false;
  std::vector<uint8_t> rd_;
};
