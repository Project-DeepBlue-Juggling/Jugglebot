#include "icm42688.h"

namespace icm42688 {

// ---- Register map (bank 0). Every address, bit and code below was checked against DS-000347 rev 1.6
// (pcbsync.com mirror) on 2026-11-18; see README "Register verification" for the table. The one exception is
// UI_FILT_CODE (icm42688.h), which is the bench value and NOT verified for the agreed ODR/4 setting. ----
static const uint8_t REG_DEVICE_CONFIG = 0x11;   // bit0 SOFT_RESET_CONFIG
static const uint8_t REG_INT_STATUS = 0x2D;      // bit4 RESET_DONE, bit1 FIFO_FULL (clear on read)
static const uint8_t REG_FIFO_DATA = 0x30;
static const uint8_t REG_SIGNAL_PATH_RESET = 0x4B;  // bit1 FIFO_FLUSH
static const uint8_t REG_INTF_CONFIG0 = 0x4C;    // b6 FIFO_COUNT_REC, b5 COUNT_ENDIAN, b4 DATA_ENDIAN
static const uint8_t REG_PWR_MGMT0 = 0x4E;
static const uint8_t REG_GYRO_CONFIG0 = 0x4F;    // FS_SEL[7:5], ODR[3:0]
static const uint8_t REG_ACCEL_CONFIG0 = 0x50;
static const uint8_t REG_GYRO_ACCEL_CONFIG0 = 0x52;  // UI filter BW: accel[7:4], gyro[3:0]
static const uint8_t REG_FIFO_CONFIG1 = 0x5F;    // b0 ACCEL, b1 GYRO, b2 TEMP, b3 TMST_FSYNC, b6 RESUME_PARTIAL_RD
static const uint8_t REG_FIFO_CONFIG = 0x16;     // b7:6 mode, 01 = stream
static const uint8_t REG_INT_CONFIG1 = 0x64;     // b4 INT_ASYNC_RESET (must be 0)
static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t REG_BANK_SEL = 0x76;

static const uint8_t WHO_AM_I_VAL = 0x47;

const char* errName(int e) {
  switch (e) {
    case OK: return "ok";
    case ERR_NO_DEVICE: return "no_device";
    case ERR_WHO_AM_I: return "bad_who_am_i";
    case ERR_I2C: return "i2c_error";
    case ERR_BAD_ARG: return "bad_arg";
    case ERR_BAD_PACKET: return "bad_fifo_packet";
    case ERR_NOT_STARTED: return "init_not_started";
    default: return "?";
  }
}

int Driver::readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  _w.beginTransmission(_addr);
  _w.write(reg);
  if (_w.endTransmission(false) != 0) return ERR_I2C;   // repeated start
  uint8_t got = _w.requestFrom((int)_addr, (int)n);
  if (got != n) { while (_w.available()) _w.read(); return ERR_I2C; }
  for (uint8_t i = 0; i < n; i++) buf[i] = _w.read();
  return OK;
}
int Driver::readReg(uint8_t reg) {
  uint8_t v;
  int r = readRegs(reg, &v, 1);
  return r < 0 ? r : v;
}
int Driver::writeReg(uint8_t reg, uint8_t val) {
  _w.beginTransmission(_addr);
  _w.write(reg);
  _w.write(val);
  return _w.endTransmission() == 0 ? OK : ERR_I2C;
}

static int odrCode(uint16_t hz) {
  switch (hz) {
    case 8000: return 3; case 4000: return 4; case 2000: return 5; case 1000: return 6;
    case 200: return 7; case 100: return 8; case 50: return 9; case 500: return 0xF;
    default: return -1;
  }
}
static int accelFsCode(uint8_t g) {   // 16g=0, 8=1, 4=2, 2=3
  switch (g) { case 16: return 0; case 8: return 1; case 4: return 2; case 2: return 3; default: return -1; }
}
static int gyroFsCode(uint16_t dps) { // 2000=0, 1000=1, 500=2, 250=3, 125=4
  switch (dps) { case 2000: return 0; case 1000: return 1; case 500: return 2; case 250: return 3; case 125: return 4; default: return -1; }
}

int Driver::applyOdrFs() {
  int r = writeReg(REG_GYRO_CONFIG0, (uint8_t)((gyroFsCode(_gyroFs) << 5) | odrCode(_odr)));
  if (r < 0) return r;
  return writeReg(REG_ACCEL_CONFIG0, (uint8_t)((accelFsCode(_accelFs) << 5) | odrCode(_odr)));
}

static const uint8_t PROBE_ADDRS[2] = {0x68, 0x69};

// First candidate index >= from that the force filter allows (2 = none). No I2C.
static uint8_t candFrom(uint8_t from, uint8_t force) {
  while (from < 2 && force && PROBE_ADDRS[from] != force) from++;
  return from;
}

void Driver::initStart(uint8_t forceAddr) {
  _force = forceAddr;
  _addr = 0; _who = 0; _cfgIdx = 0; _polls = 0; _cand = candFrom(0, forceAddr); _anyAck = false; _err = OK;
  _odr = 1000; _accelFs = 16; _gyroFs = 1000;
  _ist = IST_PROBE_ACK;
}

// The configuration writes, one per initStep call, in the bench driver's order. All before PWR_MGMT0
// (datasheet 12.9: only ODR/FS/mode may change while the sensors run).
static const uint8_t N_CFG = 7;

// Next probe candidate after a failed one, or the end of the probe. Issues no I2C transaction.
int Driver::probeNext() {
  _cand = candFrom(_cand + 1, _force);
  if (_cand >= 2) return fail(_anyAck ? ERR_WHO_AM_I : ERR_NO_DEVICE);
  _ist = IST_PROBE_ACK;
  return INIT_PENDING;
}

int Driver::fail(int err) {
  _ist = IST_FAILED;
  _err = err;
  return err;
}

// Exactly ONE I2C transaction per call in every state (a state that only waits issues none), so one loop() pass
// can never sit in more than one Wire timeout. Waits are elapsed-time checks (now - start >= wait, unsigned), not
// absolute due times: they stay correct across the micros() wrap and, if init is parked and resumed late, they
// release at once instead of stalling. A caller that parks init (DISABLE, FW_HALTED) calls initStart on re-entry.
int Driver::initStep(uint32_t now_us) {
  int r;
  switch (_ist) {
    case IST_IDLE:
      return ERR_NOT_STARTED;               // never started: not success
    case IST_DONE:
      return OK;
    case IST_FAILED:
      return _err;                          // sticky until initStart
    case IST_PROBE_ACK: {
      if (_cand >= 2) return fail(ERR_NO_DEVICE);   // forced address is not a legal one: nothing to try
      uint8_t a = PROBE_ADDRS[_cand];
      _lastTried = a;
      _w.beginTransmission(a);
      if (_w.endTransmission() != 0) return probeNext();
      _anyAck = true;
      _addr = a;
      _ist = IST_PROBE_BANK;
      return INIT_PENDING;
    }
    case IST_PROBE_BANK:
      writeReg(REG_BANK_SEL, 0);            // result ignored, as in the bench driver
      _ist = IST_PROBE_WHO;
      return INIT_PENDING;
    case IST_PROBE_WHO: {
      int who = readReg(REG_WHO_AM_I);
      if (who == WHO_AM_I_VAL) { _who = (uint8_t)who; _ist = IST_RESET_BANK; return INIT_PENDING; }
      _who = who < 0 ? 0 : (uint8_t)who;
      _addr = 0;
      return probeNext();
    }
    case IST_RESET_BANK:
      if ((r = writeReg(REG_BANK_SEL, 0)) < 0) return fail(r);
      _ist = IST_RESET;
      return INIT_PENDING;
    case IST_RESET:
      if ((r = writeReg(REG_DEVICE_CONFIG, 0x01)) < 0) return fail(r);   // soft reset
      _waitStartUs = now_us; _waitUs = 10000u;   // datasheet minimum 1 ms; the bench waited 10
      _polls = 0;
      _ist = IST_POLL_RESET;
      return INIT_PENDING;
    case IST_POLL_RESET: {
      // RESET_DONE (INT_STATUS bit4) is informational: up to INIT_STEP_RESET_POLLS polls 1 ms apart, then go on.
      if ((uint32_t)(now_us - _waitStartUs) < _waitUs) return INIT_PENDING;
      int s = readReg(REG_INT_STATUS);
      if ((s >= 0 && (s & 0x10)) || ++_polls >= INIT_STEP_RESET_POLLS) {
        _ist = IST_POLL_BANK;
      } else {
        _waitStartUs = now_us; _waitUs = 1000u;
      }
      return INIT_PENDING;
    }
    case IST_POLL_BANK:
      writeReg(REG_BANK_SEL, 0);            // result ignored, as in the bench driver
      _ist = IST_POLL_WHO;
      return INIT_PENDING;
    case IST_POLL_WHO:
      if (readReg(REG_WHO_AM_I) != WHO_AM_I_VAL) return fail(ERR_WHO_AM_I);
      _cfgIdx = 0;
      _ist = IST_CONFIG;
      return INIT_PENDING;
    case IST_CONFIG: {
      switch (_cfgIdx) {
        case 0: r = writeReg(REG_INT_CONFIG1, 0x00); break;      // INT_ASYNC_RESET cleared (DS 12.6)
        case 1: r = writeReg(REG_INTF_CONFIG0, 0x70); break;     // FIFO count in records, big-endian
        case 2: r = writeReg(REG_GYRO_CONFIG0, (uint8_t)((gyroFsCode(_gyroFs) << 5) | odrCode(_odr))); break;
        case 3: r = writeReg(REG_ACCEL_CONFIG0, (uint8_t)((accelFsCode(_accelFs) << 5) | odrCode(_odr))); break;
        case 4: r = writeReg(REG_GYRO_ACCEL_CONFIG0, UI_FILT_CODE); break;
        case 5: r = writeReg(REG_FIFO_CONFIG1, 0x4F); break;     // accel+gyro+temp+timestamp, RESUME_PARTIAL_RD
        default: r = writeReg(REG_FIFO_CONFIG, 0x40); break;     // stream mode
      }
      if (r < 0) return fail(r);
      if (++_cfgIdx >= N_CFG) _ist = IST_PWR;
      return INIT_PENDING;
    }
    case IST_PWR:
      if ((r = writeReg(REG_PWR_MGMT0, 0x0F)) < 0) return fail(r);   // accel+gyro low-noise
      _waitStartUs = now_us; _waitUs = 50000u;   // gyro needs >= 45 ms before use
      _ist = IST_WAIT_GYRO;
      return INIT_PENDING;
    case IST_WAIT_GYRO:
      if ((uint32_t)(now_us - _waitStartUs) < _waitUs) return INIT_PENDING;
      _ist = IST_FLUSH;
      return INIT_PENDING;
    case IST_FLUSH:
      r = flushFifo();
      if (r < 0) return fail(r);
      _ist = IST_DONE;
      return OK;
  }
  return ERR_BAD_ARG;
}

int Driver::setOdr(uint16_t hz) {
  if (odrCode(hz) < 0) return ERR_BAD_ARG;
  uint16_t old = _odr; _odr = hz;
  int r = applyOdrFs();
  if (r < 0) { _odr = old; return r; }
  return flushFifo();
}
int Driver::setAccelFs(uint8_t g) {
  if (accelFsCode(g) < 0) return ERR_BAD_ARG;
  uint8_t old = _accelFs; _accelFs = g;
  int r = applyOdrFs();
  if (r < 0) { _accelFs = old; return r; }
  return flushFifo();
}
int Driver::setGyroFs(uint16_t dps) {
  if (gyroFsCode(dps) < 0) return ERR_BAD_ARG;
  uint16_t old = _gyroFs; _gyroFs = dps;
  int r = applyOdrFs();
  if (r < 0) { _gyroFs = old; return r; }
  return flushFifo();
}

int Driver::flushFifo() {
  int r = writeReg(REG_SIGNAL_PATH_RESET, 0x02);
  delayMicroseconds(200);
  return r;
}

int Driver::fifoCount(bool* overflow) {
  uint8_t b[3];                       // INT_STATUS, FIFO_COUNTH, FIFO_COUNTL are contiguous
  int r = readRegs(REG_INT_STATUS, b, 3);
  if (r < 0) return r;
  int cnt = ((int)b[1] << 8) | b[2];  // big-endian (INTF_CONFIG0 bit5 = 1)
  if (overflow) *overflow = (b[0] & 0x02) || cnt >= FIFO_RECORDS - 1;
  return cnt;
}

int Driver::readFifo(Sample* out, int maxN) {
  uint8_t buf[BURST_PKTS * PKT_BYTES];
  int done = 0, remaining = maxN;
  while (remaining > 0) {
    int n = remaining > BURST_PKTS ? BURST_PKTS : remaining;
    int r = readRegs(REG_FIFO_DATA, buf, (uint8_t)(n * PKT_BYTES));
    if (r < 0) return r;
    remaining -= n;
    for (int p = 0; p < n; p++) {
      const uint8_t* q = buf + p * PKT_BYTES;
      uint8_t h = q[0];
      if (h & 0x80) continue;                         // FIFO-empty marker
      if ((h & 0x70) != 0x60) return ERR_BAD_PACKET;  // 0110_10xx: accel+gyro, no 20-bit ext (DS 6.2)
      Sample& s = out[done];
      s.a[0] = (int16_t)((q[1] << 8) | q[2]);
      s.a[1] = (int16_t)((q[3] << 8) | q[4]);
      s.a[2] = (int16_t)((q[5] << 8) | q[6]);
      s.g[0] = (int16_t)((q[7] << 8) | q[8]);
      s.g[1] = (int16_t)((q[9] << 8) | q[10]);
      s.g[2] = (int16_t)((q[11] << 8) | q[12]);
      s.temp = (int8_t)q[13];
      s.ts = (uint16_t)((q[14] << 8) | q[15]);
      done++;
    }
  }
  return done;
}

}  // namespace icm42688
