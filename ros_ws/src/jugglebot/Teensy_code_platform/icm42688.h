// icm42688.h - small direct-register ICM-42688-P driver (I2C, bank 0 only), Platform Teensy copy.
// Lifted from tools/probes/imu_bench/firmware/src (the bench copy stays untouched). Differences:
//   * init is a NON-BLOCKING step function (initStart / initStep): each call does at most ONE I2C transaction
//     (plus the 200 us FIFO-flush pause on the last) and never delays; the waits the datasheet requires are
//     elapsed-time checks against the caller's clock. The transaction is bounded only by Wire's own timeout,
//     which P2 must verify on the teensy40 core. The bench's blocking begin() (~80 ms of delay()) is gone.
//   * agreed configuration: accel +/-16 g, gyro +/-1000 dps, ODR 1 kHz, UI filter per UI_FILT_CODE below.
// Every call returns an error code (<0) or a count/value (>=0). See tools/probes/imu_bench/README.md for the
// register-verification table that covers the shared register facts.
#pragma once
#include <Arduino.h>
#include <Wire.h>

#ifndef PLATFORM_IMU_DIAG
#define PLATFORM_IMU_DIAG 0   // 1 = the diagnostic build (Teensy_code_platform.ino, "IMU DIAGNOSTIC BUILD")
#endif

namespace icm42688 {

enum Err : int {
  OK = 0,
  ERR_NO_DEVICE = -1,   // nothing ACKed at 0x68/0x69
  ERR_WHO_AM_I = -2,    // something ACKed but WHO_AM_I != 0x47
  ERR_I2C = -3,         // bus error / short read mid-operation
  ERR_BAD_ARG = -4,
  ERR_BAD_PACKET = -5,  // FIFO header not the expected 16-byte packet (resync needed)
  ERR_NOT_STARTED = -6  // initStep called without initStart
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

  // GYRO_ACCEL_CONFIG0 (0x52) value: accel UI filter BW in [7:4], gyro in [3:0].
  // Decided 2026-11-20 (owner): 0x00, UI filter BW = ODR/2 for both, the bench value. The data is used offline,
  // where any further low-pass is applied in software, so the wider band costs nothing. The capture header's
  // GYRO_ACCEL_CONFIG0 byte records this constant (the code written at init), read 0x00 on the robot 2026-11-20.
  static const uint8_t UI_FILT_CODE = 0x00;
  static const uint8_t INIT_STEP_RESET_POLLS = 20;

  explicit Driver(TwoWire& w) : _w(w) {}

  // Non-blocking init. initStart() arms it (forceAddr: only that address, else scan 0x68/0x69); then call
  // initStep(now_us) every loop() pass. It returns INIT_PENDING while work remains, OK only when configured and
  // streaming, or an Err (init ended). A failed init keeps returning its Err, and a driver on which initStart
  // was never called returns ERR_NOT_STARTED, until initStart is called again (retry from the probe). Call
  // initStart on every (re-)entry to PROBING. now_us is micros(), wrap-safe.
  static const int INIT_PENDING = 1;
  void initStart(uint8_t forceAddr = 0);
  int initStep(uint32_t now_us);
  bool initBusy() const { return _ist != IST_IDLE && _ist != IST_DONE && _ist != IST_FAILED; }
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

  // Raw register values the configuration writes (the capture header records them). No I2C.
  uint8_t regGyroConfig0() const;
  uint8_t regAccelConfig0() const;
  // ONE transaction: OK if WHO_AM_I still reads 0x47 at the configured address, else an Err.
  int checkWhoAmI();

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

#if PLATFORM_IMU_DIAG
  // DIAGNOSTIC BUILD ONLY (-DPLATFORM_IMU_DIAG=1; logbook 2026-11-20-platform-imu-i2c-error-bursts). The last Wire
  // failure inside this driver: which call failed and what Wire returned. The sketch clears it before every Io op,
  // so after a failed op it describes that op (stage 0 = the op failed without a Wire failure, e.g. WHO mismatch).
  enum DiagStage : uint8_t { DS_NONE = 0, DS_ADDR_WRITE = 1, DS_READ_SHORT = 2, DS_WRITE_REG = 3, DS_PROBE_ACK = 4 };
  struct DiagWire {
    uint8_t stage = DS_NONE;
    uint8_t code = 0;        // endTransmission() return (stages 1, 3, 4) or bytes received (stage 2)
    uint8_t requested = 0;   // bytes requested (stage 2)
    uint8_t int_status = 0;  // INT_STATUS of the last successful fifoCount()
    uint8_t count = 0;       // FIFO count of the last successful fifoCount(), clamped to 255
    uint8_t init_failed_at = 0;  // InitState in which the last init ended in failure (IST_* below)
    uint8_t good_hdr = 0;        // the last FIFO header that passed the check (readFifo)
  };
  DiagWire diag;
  void diagClear() { diag.stage = DS_NONE; diag.code = 0; diag.requested = 0; }
  uint8_t initState() const { return (uint8_t)_ist; }
#endif

 private:
  int applyOdrFs();
  int probeNext();
  int fail(int err);
  enum InitState : uint8_t { IST_IDLE, IST_PROBE_ACK, IST_PROBE_BANK, IST_PROBE_WHO, IST_RESET_BANK, IST_RESET,
                             IST_POLL_RESET, IST_POLL_BANK, IST_POLL_WHO, IST_CONFIG, IST_PWR, IST_WAIT_GYRO,
                             IST_FLUSH, IST_DONE, IST_FAILED };
  TwoWire& _w;
  InitState _ist = IST_IDLE;
  uint8_t _force = 0, _cfgIdx = 0, _polls = 0, _cand = 0;
  bool _anyAck = false;
  int _err = OK;
  uint32_t _waitStartUs = 0, _waitUs = 0;
  uint8_t _addr = 0, _who = 0, _lastTried = 0;
  uint16_t _odr = 1000;
  uint8_t _accelFs = 16;
  uint16_t _gyroFs = 1000;
};

}  // namespace icm42688
