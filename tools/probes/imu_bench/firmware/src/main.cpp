// Bench sketch: ICM-42688-P (I2C, Wire SDA=18 SCL=19) -> USB serial text stream. See ../../README.md.
//
// Wire protocol (device -> host), one record per line:
//   #S {json}        status/config (boot, on request, after any change)
//   #I text          info
//   #W text          warning (FIFO overflow, I2C trouble, host drops)
//   #E text          error / init diagnostic
//   D,seq,ts,ax,ay,az,gx,gy,gz,temp    one sample (raw int16, temp = FIFO int8)
// Host -> device (line commands, newline terminated):
//   o <hz>   ODR 50/100/200/500/1000/2000(/4000/8000)    a <g> 2/4/8/16    g <dps> 2000/1000/500/250/125
//   i <hz|khz> I2C clock 400 or 1000 (400000 / 1000000 accepted)    s status    p pause    r resume
#include <Arduino.h>
#include <Wire.h>
#include "icm42688.h"

using icm42688::Driver;
using icm42688::Sample;

static Driver imu(Wire);
static bool ready = false;
static bool paused = false;
static uint32_t i2cHz = 400000;
static uint32_t seq = 0;            // counts every sample delivered by the chip, incl. ones we failed to print
static uint32_t ts32 = 0;           // unwrapped FIFO timestamp
static uint16_t lastTs16 = 0;
static uint32_t overflows = 0, serialDrops = 0, i2cErrs = 0, badPackets = 0;
static uint32_t lastPollUs = 0, lastRetryMs = 0;
static int consecI2cErr = 0;
static char cmdBuf[40];
static uint8_t cmdLen = 0;
static bool cmdOverflow = false;

// FIFO timestamp unit: TMST_RES=0 (reset default) is nominally 1 us, but DS-000347 12.7 says that without an
// external CLKIN the interval in the FIFO reads 937.5 per 1 ms, so true time = ticks * 32/30 us.
static const float TS_TICK_US = 32.0f / 30.0f;

static void printStatus() {
  char b[420];
  int n = snprintf(b, sizeof b,
    "#S {\"fw\":\"imu_bench 1\",\"ready\":%d,\"addr\":%u,\"who\":%u,\"odr\":%u,\"accel_fs_g\":%u,"
    "\"gyro_fs_dps\":%u,\"lsb_per_g\":%.3f,\"lsb_per_dps\":%.4f,\"temp_lsb_per_c\":2.07,\"temp_off_c\":25,"
    "\"ts_tick_us\":%.4f,\"i2c_hz\":%lu,\"overflows\":%lu,\"serial_drops\":%lu,\"i2c_errs\":%lu,"
    "\"bad_packets\":%lu,\"paused\":%d,\"seq\":%lu}\n",
    ready ? 1 : 0, imu.address(), imu.whoAmI(), imu.odr(), imu.accelFs(), imu.gyroFs(),
    imu.lsbPerG(), imu.lsbPerDps(), TS_TICK_US, (unsigned long)i2cHz, (unsigned long)overflows,
    (unsigned long)serialDrops, (unsigned long)i2cErrs, (unsigned long)badPackets, paused ? 1 : 0,
    (unsigned long)seq);
  Serial.write(b, n);
}

static void info(char tag, const char* msg) { Serial.print('#'); Serial.print(tag); Serial.print(' '); Serial.println(msg); }

static void tryInit() {
  Wire.setClock(i2cHz);
  int r = imu.begin();
  if (r == icm42688::OK) {
    ready = true; consecI2cErr = 0; seq = 0; ts32 = 0; lastTs16 = 0;
    info('I', "ICM-42688-P initialised");
    printStatus();
    return;
  }
  ready = false;
  char m[220];
  if (r == icm42688::ERR_NO_DEVICE)
    snprintf(m, sizeof m, "init failed: no ACK at 0x68 or 0x69 on SDA=18/SCL=19. Check cable/Stemma QT, 3V3, "
             "SDA/SCL pull-ups, CS tied high (I2C mode), address strap. Retrying every 1 s.");
  else if (r == icm42688::ERR_WHO_AM_I)
    snprintf(m, sizeof m, "init failed: device ACKed at 0x%02X but WHO_AM_I=0x%02X (expected 0x47). Wrong chip or "
             "bus noise. Retrying every 1 s.", imu.lastTriedAddr(), imu.whoAmI());
  else
    snprintf(m, sizeof m, "init failed: %s (addr tried 0x%02X). Retrying every 1 s.", icm42688::errName(r),
             imu.lastTriedAddr());
  info('E', m);
}

static void handleCommand(char* s) {
  while (*s == ' ') s++;
  char c = *s++;
  long v = strtol(s, nullptr, 10);
  if (v < 0 || v > 65535) v = 0;   // 0 is rejected as a bad argument by every setter (no uint cast wrap-around)
  int r = 0;
  switch (c) {
    case 0: return;
    case 's': case '?': break;
    case 'p': paused = true; break;
    case 'r': paused = false; break;
    case 'o': r = ready ? imu.setOdr((uint16_t)v) : icm42688::ERR_NO_DEVICE; break;
    case 'a': r = ready ? imu.setAccelFs((uint8_t)v) : icm42688::ERR_NO_DEVICE; break;
    case 'g': r = ready ? imu.setGyroFs((uint16_t)v) : icm42688::ERR_NO_DEVICE; break;
    case 'i':
      if (v == 400 || v == 400000) i2cHz = 400000;
      else if (v == 1000 || v == 1000000) i2cHz = 1000000;
      else { r = icm42688::ERR_BAD_ARG; break; }
      Wire.setClock(i2cHz);
      break;
    default: info('W', "unknown command"); return;
  }
  if (r < 0) { char m[60]; snprintf(m, sizeof m, "command '%c %ld' failed: %s", c, v, icm42688::errName(r)); info('W', m); }
  printStatus();
}

static void pollCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdLen] = 0;
      if (cmdOverflow) info('W', "command line too long, ignored");
      else if (cmdLen) handleCommand(cmdBuf);
      cmdLen = 0; cmdOverflow = false;
    }
    else if (cmdLen < sizeof cmdBuf - 1) cmdBuf[cmdLen++] = c;
    else cmdOverflow = true;      // discard the whole line, not its tail as a new command
  }
}

static void drain() {
  static Sample smp[32];
  static char out[32 * 56];
  bool ovf = false;
  int cnt = imu.fifoCount(&ovf);
  if (cnt < 0) {
    i2cErrs++;
    if (++consecI2cErr >= 5) { info('E', "I2C read failing repeatedly; re-initialising"); ready = false; }
    return;
  }
  consecI2cErr = 0;
  if (ovf) {
    overflows++;
    imu.flushFifo();            // resync packet alignment; samples in the FIFO are lost
    info('W', "FIFO overflow: flushed (lower ODR, raise I2C clock, or drain faster)");
    return;
  }
  if (cnt == 0) return;
  if (cnt > 32) cnt = 32;
  int n = imu.readFifo(smp, cnt);
  if (n == icm42688::ERR_BAD_PACKET) { badPackets++; imu.flushFifo(); info('W', "bad FIFO packet header: flushed"); return; }
  if (n < 0) { i2cErrs++; consecI2cErr++; return; }
  size_t len = 0;
  for (int i = 0; i < n; i++) {
    // -32768 = invalid sample (first ODR after power-up / sensor not yet running, DS 12.8): not real data.
    if (smp[i].a[0] == -32768 || smp[i].a[1] == -32768 || smp[i].a[2] == -32768 ||
        smp[i].g[0] == -32768 || smp[i].g[1] == -32768 || smp[i].g[2] == -32768) continue;
    ts32 += (uint16_t)(smp[i].ts - lastTs16);
    lastTs16 = smp[i].ts;
    int w = snprintf(out + len, sizeof out - len, "D,%lu,%lu,%d,%d,%d,%d,%d,%d,%d\n", (unsigned long)seq++,
                     (unsigned long)ts32, smp[i].a[0], smp[i].a[1], smp[i].a[2], smp[i].g[0], smp[i].g[1],
                     smp[i].g[2], smp[i].temp);
    if (w > 0) len += (size_t)w;
  }
  if (paused) return;           // FIFO still drained so it never overflows while paused
  if ((size_t)Serial.availableForWrite() >= len) Serial.write(out, len);
  else serialDrops += (uint32_t)n;   // host not keeping up: drop whole batch, seq gap makes it visible
}

void setup() {
  Serial.begin(115200);         // USB CDC; baud ignored
  Wire.begin();
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  info('I', "imu_bench boot");
  tryInit();
}

void loop() {
  pollCommands();
  if (!ready) {
    if (millis() - lastRetryMs >= 1000) { lastRetryMs = millis(); tryInit(); }
    return;
  }
  uint32_t period = imu.odr() >= 500 ? 2000 : 5000;   // poll every 2-4 packets; FIFO holds 128
  if ((uint32_t)(micros() - lastPollUs) >= period) {
    lastPollUs = micros();
    drain();
  }
}
