/*
 * Proprioception.cpp - Implementation of central state repository
 * 
 * MODIFICATION NOTES (State Machine Update):
 *   - Added simple convenience getters (getYawDeg(), getPitchDeg(), etc.)
 *   - Maintained ISR-safe debug output via ring buffer
 *   - All writers use seqlock pattern for lock-free reads
 */

#include <Arduino.h>
#include "Proprioception.h"
#include <stdarg.h>
#include <stdio.h>

// --------------------------------------------------------------------
// Static debug storage
// --------------------------------------------------------------------
Stream* Proprioception::s_dbg_ = nullptr;
volatile bool Proprioception::s_dbg_enabled_ = false;
char Proprioception::rb_[DBG_CAP_];
volatile uint16_t Proprioception::rb_head_ = 0;
volatile uint16_t Proprioception::rb_tail_ = 0;

// --------------------------------------------------------------------
// inISR_() - Detect if running in ISR context (ARM Cortex-M)
// --------------------------------------------------------------------
inline bool Proprioception::inISR_() {
  uint32_t ipsr;
  __asm__ volatile("MRS %0, IPSR" : "=r"(ipsr));
  return ipsr != 0;
}

// --------------------------------------------------------------------
// Debug stream control
// --------------------------------------------------------------------
void Proprioception::setDebugStream(Stream* s, bool enable) {
  s_dbg_ = s;
  s_dbg_enabled_ = enable;
}

void Proprioception::setDebugEnabled(bool on) {
  s_dbg_enabled_ = on;
}

// --------------------------------------------------------------------
// vdebugf_() - Internal formatted debug output
// If called from ISR, buffers to ring buffer. Otherwise prints directly.
// --------------------------------------------------------------------
void Proprioception::vdebugf_(const char* fmt, va_list ap) {
  if (!s_dbg_ || !s_dbg_enabled_) return;

  char tmp[160];
  vsnprintf(tmp, sizeof(tmp), fmt, ap);

  if (!inISR_()) {
    // Safe: normal thread context - print directly
    s_dbg_->print(tmp);
  } else {
    // ISR context: push bytes into ring buffer (drop on overflow)
    uint16_t h = rb_head_;
    for (const char* p = tmp; *p; ++p) {
      uint16_t next = (uint16_t)((h + 1) % DBG_CAP_);
      if (next == rb_tail_) break;  // Buffer full - drop remaining
      rb_[h] = *p;
      h = next;
    }
    rb_head_ = h;
  }
}

// --------------------------------------------------------------------
// debugf() - Public debug output (safe from any context)
// --------------------------------------------------------------------
void Proprioception::debugf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vdebugf_(fmt, ap);
  va_end(ap);
}

// --------------------------------------------------------------------
// flushDebug() - Drain ISR-buffered messages to Serial
// Call this from loop() to see ISR debug output.
// --------------------------------------------------------------------
void Proprioception::flushDebug() {
  if (!s_dbg_ || !s_dbg_enabled_) return;
  while (rb_tail_ != rb_head_) {
    char c = rb_[rb_tail_];
    rb_tail_ = (uint16_t)((rb_tail_ + 1) % DBG_CAP_);
    s_dbg_->write(c);
  }
}

// --------------------------------------------------------------------
// Constructor
// --------------------------------------------------------------------
Proprioception::Proprioception()
  : seq_(0),
    yaw_deg_(0), pitch_deg_(0), hand_pos_rev_(0), hand_vel_rps_(0), hand_iq_a_(0),
    yaw_ts_us_(0), pitch_ts_us_(0), hand_pv_ts_us_(0), hand_iq_ts_us_(0),
    valid_mask_(0) {}

// --------------------------------------------------------------------
// Writers - All use seqlock pattern
// --------------------------------------------------------------------
void Proprioception::setYawDeg(float yaw_deg, uint64_t ts_us) {
  IRQGuard g;
  seq_++;                              // Start write (now odd)
  yaw_deg_   = yaw_deg;
  yaw_ts_us_ = ts_us ? ts_us : (uint64_t)micros();
  valid_mask_ |= (1u << 0);
  seq_++;                              // End write (now even)
}

void Proprioception::setPitchDeg(float pitch_deg, uint64_t ts_us) {
  IRQGuard g;
  seq_++;
  pitch_deg_   = pitch_deg;
  pitch_ts_us_ = ts_us ? ts_us : (uint64_t)micros();
  valid_mask_ |= (1u << 1);
  seq_++;
}

void Proprioception::setHandPV(float pos_rev, float vel_rps, uint64_t ts_us) {
  IRQGuard g;
  seq_++;
  hand_pos_rev_  = pos_rev;
  hand_vel_rps_  = vel_rps;
  hand_pv_ts_us_ = ts_us ? ts_us : (uint64_t)micros();
  valid_mask_ |= (1u << 2);
  seq_++;
}

void Proprioception::setHandIq(float iq_a, uint64_t ts_us) {
  IRQGuard g;
  seq_++;
  hand_iq_a_     = iq_a;
  hand_iq_ts_us_ = ts_us ? ts_us : (uint64_t)micros();
  valid_mask_ |= (1u << 3);
  seq_++;
}

void Proprioception::publishBulk(const Bulk& b) {
  if (!b.yaw_valid && !b.pitch_valid && !b.pv_valid && !b.iq_valid) return;
  IRQGuard g;
  seq_++;
  if (b.yaw_valid) {
    yaw_deg_   = b.yaw_deg;
    yaw_ts_us_ = b.yaw_ts_us;
    valid_mask_ |= (1u << 0);
  }
  if (b.pitch_valid) {
    pitch_deg_   = b.pitch_deg;
    pitch_ts_us_ = b.pitch_ts_us;
    valid_mask_ |= (1u << 1);
  }
  if (b.pv_valid) {
    hand_pos_rev_  = b.pos_rev;
    hand_vel_rps_  = b.vel_rps;
    hand_pv_ts_us_ = b.pv_ts_us;
    valid_mask_ |= (1u << 2);
  }
  if (b.iq_valid) {
    hand_iq_a_     = b.iq_a;
    hand_iq_ts_us_ = b.iq_ts_us;
    valid_mask_ |= (1u << 3);
  }
  seq_++;
}

void Proprioception::clearValidity() {
  IRQGuard g;
  seq_++;
  valid_mask_ = 0;
  seq_++;
}

// --------------------------------------------------------------------
// copyOnce() - Single attempt to copy data (for seqlock pattern)
// Returns the sequence number at start of read.
// If odd, write was in progress and data may be inconsistent.
// --------------------------------------------------------------------
uint32_t Proprioception::copyOnce(ProprioceptionData& out) const {
  uint32_t s1 = seq_;
  if (s1 & 1u) return s1;  // Write in progress - don't bother copying

  asm volatile("" ::: "memory");  // Memory barrier

  out.yaw_deg       = yaw_deg_;
  out.pitch_deg     = pitch_deg_;
  out.hand_pos_rev  = hand_pos_rev_;
  out.hand_vel_rps  = hand_vel_rps_;
  out.hand_iq_a     = hand_iq_a_;

  out.yaw_ts_us     = yaw_ts_us_;
  out.pitch_ts_us   = pitch_ts_us_;
  out.hand_pv_ts_us = hand_pv_ts_us_;
  out.hand_iq_ts_us = hand_iq_ts_us_;

  out.valid_mask    = valid_mask_;

  asm volatile("" ::: "memory");  // Memory barrier
  return s1;
}

// --------------------------------------------------------------------
// snapshot() - Get consistent snapshot (retries until consistent)
// --------------------------------------------------------------------
bool Proprioception::snapshot(ProprioceptionData& out) const {
  while (true) {
    uint32_t s1 = copyOnce(out);
    if (s1 & 1u) continue;        // Write was in progress
    uint32_t s2 = seq_;
    if (s1 == s2) return true;    // Data is consistent
    // Sequence changed during read - retry
  }
}

// --------------------------------------------------------------------
// Individual getters with timestamp
// --------------------------------------------------------------------
bool Proprioception::getYaw(float& yaw_deg, uint64_t& ts_us) const {
  ProprioceptionData d;
  snapshot(d);
  if (!d.isYawValid()) return false;
  yaw_deg = d.yaw_deg;
  ts_us   = d.yaw_ts_us;
  return true;
}

bool Proprioception::getPitch(float& pitch_deg, uint64_t& ts_us) const {
  ProprioceptionData d;
  snapshot(d);
  if (!d.isPitchValid()) return false;
  pitch_deg = d.pitch_deg;
  ts_us     = d.pitch_ts_us;
  return true;
}

bool Proprioception::getHandPV(float& pos_rev, float& vel_rps, uint64_t& ts_us) const {
  ProprioceptionData d;
  snapshot(d);
  if (!d.isHandPVValid()) return false;
  pos_rev = d.hand_pos_rev;
  vel_rps = d.hand_vel_rps;
  ts_us   = d.hand_pv_ts_us;
  return true;
}

bool Proprioception::getHandIq(float& iq_a, uint64_t& ts_us) const {
  ProprioceptionData d;
  snapshot(d);
  if (!d.isHandIqValid()) return false;
  iq_a  = d.hand_iq_a;
  ts_us = d.hand_iq_ts_us;
  return true;
}

// --------------------------------------------------------------------
// Simple getters (return NaN if not valid)
// --------------------------------------------------------------------
float Proprioception::getYawDeg() const {
  ProprioceptionData d;
  snapshot(d);
  return d.isYawValid() ? d.yaw_deg : NAN;
}

float Proprioception::getPitchDeg() const {
  ProprioceptionData d;
  snapshot(d);
  return d.isPitchValid() ? d.pitch_deg : NAN;
}

float Proprioception::getHandPosRev() const {
  ProprioceptionData d;
  snapshot(d);
  return d.isHandPVValid() ? d.hand_pos_rev : NAN;
}

float Proprioception::getHandVelRps() const {
  ProprioceptionData d;
  snapshot(d);
  return d.isHandPVValid() ? d.hand_vel_rps : NAN;
}

float Proprioception::getHandIqA() const {
  ProprioceptionData d;
  snapshot(d);
  return d.isHandIqValid() ? d.hand_iq_a : NAN;
}

// --------------------------------------------------------------------
// Global instance
// --------------------------------------------------------------------
Proprioception PRO;
