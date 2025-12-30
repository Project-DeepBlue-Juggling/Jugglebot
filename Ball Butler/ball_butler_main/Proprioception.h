#pragma once
/*
 * Proprioception.h - Central repository for robot state data
 * 
 * Provides a thread-safe (ISR-safe) storage for proprioceptive data from all axes:
 *   - Yaw: position [deg], velocity [rev/s]
 *   - Pitch: position [deg]
 *   - Hand: position [rev], velocity [rev/s], Iq current [A]
 * 
 * DESIGN:
 *   - Uses a seqlock pattern for lock-free reads from main loop while
 *     allowing atomic writes from ISR or CAN callback contexts.
 *   - Debug output uses a ring buffer so debugf() can be called from ISR.
 *   - Global instance 'PRO' is accessible from any module.
 * 
 * MODIFICATION NOTES (State Machine Update):
 *   - This module now receives updates from:
 *       * YawAxis via callback (from ISR) -> setYawDeg()
 *       * CanInterface (from CAN RX) -> setPitchDeg(), setHandPV(), setHandIq()
 *   - The pitch conversion now uses PitchAxis convention: deg = 90 + 360*rev
 *   - Added convenience getters that don't require full snapshot.
 */

#include <stdint.h>
#include <stdarg.h>

// Forward declaration for Arduino Stream
class Stream;

// --------------------------------------------------------------------
// ProprioceptionData - Snapshot structure for reading all state at once
// --------------------------------------------------------------------
struct ProprioceptionData {
  float    yaw_deg;        // Yaw angle [degrees]
  float    pitch_deg;      // Pitch angle [degrees from horizontal]
  float    hand_pos_rev;   // Hand position [motor revolutions]
  float    hand_vel_rps;   // Hand velocity [rev/s]
  float    hand_iq_a;      // Hand motor Iq current [A]

  uint64_t yaw_ts_us;      // Timestamp of yaw update [microseconds]
  uint64_t pitch_ts_us;    // Timestamp of pitch update [microseconds]
  uint64_t hand_pv_ts_us;  // Timestamp of hand pos/vel update [microseconds]
  uint64_t hand_iq_ts_us;  // Timestamp of hand Iq update [microseconds]

  // Validity bitmask: bit0=yaw, bit1=pitch, bit2=hand_pv, bit3=hand_iq
  uint32_t valid_mask;
  
  // Convenience validity checkers
  bool isYawValid()    const { return (valid_mask & (1u << 0)) != 0; }
  bool isPitchValid()  const { return (valid_mask & (1u << 1)) != 0; }
  bool isHandPVValid() const { return (valid_mask & (1u << 2)) != 0; }
  bool isHandIqValid() const { return (valid_mask & (1u << 3)) != 0; }
};

// --------------------------------------------------------------------
// Proprioception - Main class
// --------------------------------------------------------------------
class Proprioception {
public:
  Proprioception();

  // ----------------------------------------------------------------
  // Debug output control
  // ----------------------------------------------------------------
  // Set the output stream and enable/disable debug output
  static void setDebugStream(Stream* s, bool enable = true);
  static void setDebugEnabled(bool on);
  
  // Printf-style debug output - safe to call from ISR (buffered)
  static void debugf(const char* fmt, ...);
  
  // Call from loop() to drain ISR-buffered debug messages to Serial
  static void flushDebug();

  // ----------------------------------------------------------------
  // Writers - Called from ISR or CAN callback contexts
  // ----------------------------------------------------------------
  // Set yaw angle [degrees] with timestamp
  void setYawDeg(float yaw_deg, uint64_t ts_us);
  
  // Set pitch angle [degrees from horizontal] with timestamp
  void setPitchDeg(float pitch_deg, uint64_t ts_us);
  
  // Set hand position [rev] and velocity [rev/s] with timestamp
  void setHandPV(float pos_rev, float vel_rps, uint64_t ts_us);
  
  // Set hand Iq current [A] with timestamp
  void setHandIq(float iq_a, uint64_t ts_us);

  // Bulk update structure for efficient multi-field updates
  struct Bulk {
    bool    yaw_valid   = false;
    bool    pitch_valid = false;
    bool    pv_valid    = false;
    bool    iq_valid    = false;
    float   yaw_deg     = 0;
    float   pitch_deg   = 0;
    float   pos_rev     = 0;
    float   vel_rps     = 0;
    float   iq_a        = 0;
    uint64_t yaw_ts_us   = 0;
    uint64_t pitch_ts_us = 0;
    uint64_t pv_ts_us    = 0;
    uint64_t iq_ts_us    = 0;
  };
  void publishBulk(const Bulk& b);

  // ----------------------------------------------------------------
  // Readers - Safe to call from main loop
  // ----------------------------------------------------------------
  // Get a consistent snapshot of all data (blocks until consistent)
  bool snapshot(ProprioceptionData& out) const;
  
  // Individual getters (return false if data not valid)
  bool getYaw(float& yaw_deg, uint64_t& ts_us) const;
  bool getPitch(float& pitch_deg, uint64_t& ts_us) const;
  bool getHandPV(float& pos_rev, float& vel_rps, uint64_t& ts_us) const;
  bool getHandIq(float& iq_a, uint64_t& ts_us) const;
  
  // Simple getters (return 0 if not valid, no timestamp)
  float getYawDeg() const;
  float getPitchDeg() const;
  float getHandPosRev() const;
  float getHandVelRps() const;
  float getHandIqA() const;

  // Clear all validity flags (e.g., on mode change or error)
  void clearValidity();

private:
  // IRQ guard with previous-state restore
  struct IRQGuard {
  #if defined(TEENSYDUINO)
    uint32_t primask_;
    static inline uint32_t read_primask() {
      uint32_t pm;
      asm volatile("MRS %0, PRIMASK" : "=r"(pm) :: "memory");
      return pm;
    }
    IRQGuard() { primask_ = read_primask(); __disable_irq(); }
    ~IRQGuard() { if ((primask_ & 1u) == 0u) __enable_irq(); }
  #elif defined(__AVR__)
    uint8_t sreg_;
    IRQGuard() { sreg_ = SREG; noInterrupts(); }
    ~IRQGuard() { SREG = sreg_; }
  #else
    bool was_enabled_;
    IRQGuard() { noInterrupts(); was_enabled_ = true; }
    ~IRQGuard() { if (was_enabled_) interrupts(); }
  #endif
  };

  // Seqlock sequence number (odd = write in progress)
  volatile uint32_t seq_;

  // Data storage (all volatile for ISR safety)
  volatile float    yaw_deg_;
  volatile float    pitch_deg_;
  volatile float    hand_pos_rev_;
  volatile float    hand_vel_rps_;
  volatile float    hand_iq_a_;

  volatile uint64_t yaw_ts_us_;
  volatile uint64_t pitch_ts_us_;
  volatile uint64_t hand_pv_ts_us_;
  volatile uint64_t hand_iq_ts_us_;

  volatile uint32_t valid_mask_;

  // Internal: single-attempt copy (returns seq, odd if interrupted)
  uint32_t copyOnce(ProprioceptionData& out) const;

  // Non-copyable
  Proprioception(const Proprioception&) = delete;
  Proprioception& operator=(const Proprioception&) = delete;

  // ----------------------------------------------------------------
  // Debug internals
  // ----------------------------------------------------------------
  static Stream* s_dbg_;
  static volatile bool s_dbg_enabled_;
  
  // Detect if running in ISR context
  static inline bool inISR_();
  
  // Internal formatted output
  static void vdebugf_(const char* fmt, va_list ap);

  // Ring buffer for ISR debug output
  static constexpr size_t DBG_CAP_ = 1024;
  static char rb_[DBG_CAP_];
  static volatile uint16_t rb_head_;
  static volatile uint16_t rb_tail_;
};

// Global proprioception instance - accessible from all modules
extern Proprioception PRO;
