// icm42688.h - small direct-register ICM-42688-P driver (I2C, bank 0 only).
// Transport-agnostic w.r.t. the host link: it only knows a TwoWire&. No blocking-forever paths:
// every call returns an error code (<0) or a count/value (>=0). Designed to be lifted into the
// Platform Teensy firmware unchanged. See ../../README.md for the register-verification status.
#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace icm42688 {

enum Err : int {
  OK = 0,
  ERR_NO_DEVICE = -1,   // nothing ACKed at 0x68/0x69
  ERR_WHO_AM_I = -2,    // something ACKed but WHO_AM_I != 0x47
  ERR_I2C = -3,         // bus error / short read mid-operation
  ERR_BAD_ARG = -4,
  ERR_BAD_PACKET = -5   // FIFO header not the expected 16-byte packet (resync needed)
};
const char* errName(int e);

struct Sample {
  uint16_t ts;      // FIFO timestamp field, raw (wraps at 65536 ticks)
  int16_t a[3];     // raw accel x,y,z (-32768 = invalid)
  int16_t g[3];     // raw gyro x,y,z
  int8_t temp;      // FIFO 8-bit temp: degC = raw/2.07 + 25
};

class Driver {
 public:
  static const int PKT_BYTES = 16;        // header + 6 accel + 6 gyro + 1 temp + 2 timestamp
  static const int FIFO_RECORDS = 128;    // 2 KiB FIFO / 16 B
  // Wire receive buffer on Teensy 4 is BUFFER_LENGTH=136 (WireIMXRT.h) -> 8 packets = 128 B per burst.
  static const int BURST_PKTS = 8;

  explicit Driver(TwoWire& w) : _w(w) {}

  // Scan 0x68/0x69 (or only forceAddr if nonzero), check WHO_AM_I, soft reset, configure
  // (defaults: 1 kHz, +/-16 g, +/-2000 dps, FIFO stream, accel+gyro low-noise). Returns OK or Err.
  int begin(uint8_t forceAddr = 0);
  uint8_t address() const { return _addr; }
  uint8_t whoAmI() const { return _who; }
  uint8_t lastTriedAddr() const { return _lastTried; }

  int setOdr(uint16_t hz);       // 100,200,500,1000,2000 (also 50,4000,8000). Flushes FIFO.
  int setAccelFs(uint8_t g);     // 2,4,8,16
  int setGyroFs(uint16_t dps);   // 2000,1000,500,250,125
  uint16_t odr() const { return _odr; }
  uint8_t accelFs() const { return _accelFs; }
  uint16_t gyroFs() const { return _gyroFs; }
  float lsbPerG() const { return 32768.0f / _accelFs; }
  float lsbPerDps() const { return 32768.0f / _gyroFs; }

  int flushFifo();
  // Reads INT_STATUS+FIFO_COUNT in one transaction. Returns record count (>=0) or Err.
  // *overflow is set if the FIFO-full flag was seen or count is at/near capacity.
  int fifoCount(bool* overflow);
  // Reads up to maxN packets. Returns packets parsed or Err.
  // Empty-marker packets are dropped. On ERR_BAD_PACKET the caller should flushFifo().
  int readFifo(Sample* out, int maxN);

  // Raw register access (bank 0), public for the sketch's diagnostics.
  int readRegs(uint8_t reg, uint8_t* buf, uint8_t n);
  int readReg(uint8_t reg);                 // returns byte or Err
  int writeReg(uint8_t reg, uint8_t val);

 private:
  int applyOdrFs();
  TwoWire& _w;
  uint8_t _addr = 0, _who = 0, _lastTried = 0;
  uint16_t _odr = 1000;
  uint8_t _accelFs = 16;
  uint16_t _gyroFs = 2000;
};

}  // namespace icm42688
