/*****************************************************************************************
 *  Teensy 4.0 — Jugglebot “platform” microcontroller
 *  ------------------------------------------------------------------
 *  Functions already present:
 *    • CAN traffic monitor (ID 0x7DF)
 *    • Optional debugging of specific CAN IDs
 *    • Timed analysis of RxSdo messages
 *    • SCL3300 inclinometer → CAN (ID 0x7DE)
 *    • Robot state exchange (ID 0x6E0)
 *    • TIME‑SYNC layer (ID 0x7DD)
 *      – Maintains wall‑time offset (Jetson wall‑time − micros64())
 *      – Prints jitter stats once per second
 *
 *  NOTE on CAN IDs:
 *    ODrives occupy 0x000–0x0DF (7 × 32 IDs). Broadcast ID 0x7E0..0x7FF must
 *    also stay free.  All custom IDs here (0x6E0–0x7DF) are safe.
 *****************************************************************************************/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <SCL3300.h>
#include <vector>
#include "Trajectory.h"

#define DEBUG_TRAFFIC 0    // 0 = silent, 1 = Serial print report CAN traffic frames
#define DEBUG_TRAJ 0       // 0 = silent, 1 = Serial print each hand traj frame as it gets sent out
#define DEBUG_TIME_SYNC 0  // 0 = silent, 1 = Serial print periodic messages showing how tight the clocks are synced

/*----------------------------------------------------------------------------*/
/*                                CAN BUS SET‑UP                              */
/*----------------------------------------------------------------------------*/
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
constexpr uint32_t CAN_BITRATE = 1'000'000;  // 1 Mbps

/*----------------------------------------------------------------------------*/
/*                             INCLINOMETER (SCL3300)                         */
/*----------------------------------------------------------------------------*/
SCL3300 inclinometer;

/*----------------------------------------------------------------------------*/
/*                               CAN  ID MAP                                  */
/*----------------------------------------------------------------------------*/
constexpr uint32_t REPORT_ID = 0x7DF;      // traffic monitor
constexpr uint32_t tiltID = 0x7DE;         // inclinometer request / reply
constexpr uint32_t timeSyncID = 0x7DD;     // NEW — wall‑time sync
constexpr uint32_t stateUpdateID = 0x6E0;  // robot state exchange
constexpr uint32_t CMD_TRAJ_ID = 0x6D0;    // for Jetson commands to generate hand traj
constexpr uint32_t TRAJ_OUT_ID = 0xCC;     // "set input pos" for hand (node id = 6; ie. (6 << 5)|0x0c )

/*----------------------------------------------------------------------------*/
/*                            TRAFFIC MONITOR                                 */
/*----------------------------------------------------------------------------*/
uint32_t receivedCount = 0;
uint32_t lastReportTime = 0;
const uint32_t reportInterval = 500;  // [ms]

/*----------------------------------------------------------------------------*/
/*                      DEBUG / TIMING‑ANALYSIS                               */
/*----------------------------------------------------------------------------*/
const uint32_t node_id = 0;
const uint32_t cmd_id = 0x018;
const uint32_t debugID = (node_id << 5) | cmd_id;

uint32_t timingID = (node_id << 5) | 0x04;
uint16_t targetContentID = 385;
unsigned long lastMessageTime = 0;
const unsigned long analysisPeriod = 500;
unsigned long totalIntervals = 0;
int messageCount = 0;

/*----------------------------------------------------------------------------*/
/*                             ROBOT STATE                                    */
/*----------------------------------------------------------------------------*/
struct RobotState {
  bool is_homed;
  bool levelling_complete;
  float pose_offset_tiltX;
  float pose_offset_tiltY;
};
RobotState state = { false, false, 0.0f, 0.0f };

/*----------------------------------------------------------------------------*/
/*                           PLATFORM TILT                                    */
/*----------------------------------------------------------------------------*/
struct platformTilt {
  float tiltX;
  float tiltY;
};

/*----------------------------------------------------------------------------*/
/*                        HAND ENCODER ESTIMATES                              */
/*----------------------------------------------------------------------------*/
constexpr uint32_t HAND_AXIS_NODE = 6;
constexpr uint32_t HAND_ENC_EST_ID = (HAND_AXIS_NODE << 5) | 0x09;  // 0xC9

/* most-recent hand state (updated at 500 Hz)                           *
 * 32-bit float writes are atomic on Cortex-M7, but mark them           *
 * volatile so the compiler never caches them between ISR & main loop. */
volatile float current_hand_position = 0.0f;  // rev
volatile float current_hand_velocity = 0.0f;  // rev/s
volatile uint64_t last_hand_update_us = 0;    // wall-clock, µs

inline void getHandPosVel(float &pos, float &vel, uint64_t &t_us) {
  uint64_t ts1, ts2;
  do {
    ts1 = last_hand_update_us;
    pos = current_hand_position;
    vel = current_hand_velocity;
    ts2 = last_hand_update_us;
  } while (ts1 != ts2);  // repeat if an update happened
  t_us = ts1;
}

/*----------------------------------------------------------------------------*/
/*                        ──  TIME–SYNC  LAYER ──                             */
/*----------------------------------------------------------------------------*/
namespace TimeSync {

/* 64‑bit free‑running microsecond counter (wraps after 71 min) */
uint64_t micros64() {
  static uint32_t last_lo = ::micros();
  static uint64_t hi = 0;
  uint32_t now = ::micros();
  if (now < last_lo) hi += UINT64_C(1) << 32;
  last_lo = now;
  return hi | now;
}

/* Wall‑time offset: Jetson_wall_us − micros64() */
volatile int64_t wall_offset_us = 0;
volatile bool have_offset = false;
constexpr uint8_t ALPHA_SHIFT = 3;  // I‑filter gain = 1/8

/* Stats for jitter read‑out */
struct Stats {
  int32_t sum = 0;
  uint32_t sum_sq = 0;
  int32_t min = INT32_MAX, max = INT32_MIN;
  uint32_t n = 0;
  void add(int32_t x) {
    sum += x;
    sum_sq += uint32_t(x) * x;
    if (x < min) min = x;
    if (x > max) max = x;
    ++n;
  }
  void clear() {
    *this = {};
  }
  float mean() const {
    return n ? float(sum) / n : 0.f;
  }
  float rms() const {
    return n ? sqrtf(float(sum_sq) / n) : 0.f;
  }
} stats;
uint64_t nextPrint_us = 0;
constexpr uint32_t PRINT_PERIOD_US = 1'000'000;

/* Convert helpers (can be used by main application) */
uint64_t get_wall_time_us() {
  return micros64() + wall_offset_us;
}
uint64_t wall_to_local_us(uint64_t w_us) {
  return w_us - wall_offset_us;
}

/* Process a single 8‑byte sync frame (ID 0x7DD) */
inline void handleSyncFrame(const CAN_message_t &msg) {
  uint32_t sec = msg.buf[0] | (msg.buf[1] << 8) | (msg.buf[2] << 16) | (msg.buf[3] << 24);
  uint32_t usec = msg.buf[4] | (msg.buf[5] << 8) | (msg.buf[6] << 16) | (msg.buf[7] << 24);
  uint64_t jetson_us = uint64_t(sec) * 1'000'000ULL + usec;
  uint64_t local_us = micros64();
  int64_t offset = int64_t(jetson_us) - int64_t(local_us);

  if (!have_offset) {  // first frame → step
    wall_offset_us = offset;
    have_offset = true;
  } else {  // subsequent → slew
    int64_t diff = offset - wall_offset_us;
    wall_offset_us += diff >> ALPHA_SHIFT;
  }

  /* delta from current offset for jitter stats */
  int32_t delta = int32_t(offset - wall_offset_us);
  stats.add(delta);
}

/* Periodic console print */
inline void maybePrintStats() {
  uint64_t now = micros64();
  if (now < nextPrint_us) return;
  nextPrint_us = now + PRINT_PERIOD_US;

  if (stats.n) {
    Serial.printf("Δmean %+0.1f us | rms %.1f us | min %+d us | max %+d us | frames %lu\n",
                  (double)stats.mean(), (double)stats.rms(),
                  stats.min, stats.max, stats.n);
    stats.clear();
  }
}
}  // namespace TimeSync

/* Reconstruct full 64-bit wall_time_ms from low 32-bit field    */
/* Assumes local wall-clock is synced to Jetson within ±24 days   */
static uint64_t reconstructWallMs(uint32_t low32) {
  uint64_t now_ms = TimeSync::get_wall_time_us() / 1000ULL;  // full 64-bit

  uint64_t candidate = (now_ms & 0xFFFFFFFF00000000ULL) | low32;

  /* if candidate is more than +24.9 d in the future, subtract one wrap */
  if (candidate > now_ms + 0x80000000ULL) candidate -= 0x100000000ULL;
  /* if candidate is more than -24.9 d in the past, add one wrap */
  else if (candidate + 0x80000000ULL < now_ms) candidate += 0x100000000ULL;

  return candidate;  // full 64-bit wall_time_ms
}

/* ----------- helper: true (wall-clock) time, 64-bit ----------- */
inline uint64_t now_wall_us() {
  return TimeSync::get_wall_time_us();
}

/*----------------------------------------------------------------------------*/
/*                           Trajectory Scheduler                             */
/*----------------------------------------------------------------------------*/

Trajectory activeTraj;
elapsedMicros trajClock;  // runs only when trajectory active
bool trajActive = false;
size_t nextIdx = 0;
uint32_t nextSendUs = 0;
float vel_scale = 100.0;                    // Scaling factor as set on the ODrive `input_vel_scale`
float tor_scale = 100.0;                    // Scaling factor as set on the ODrive `input_torque_scale`
std::vector<CAN_message_t> packedMsgs;      // Pre-packed CAN frames (to tighten broadcasting timing)
std::vector<uint64_t> sendUs;               // Absolute μs after traj start for each CAN frame
constexpr uint32_t SAFETY_GAP_US = 20'000;  // 20 ms pause (minimum) after smooth-move before the main trajectory begins

/* pack one full trajectory into the global buffers -------------- */
void packTrajectory(const Trajectory &tr, uint64_t abs_t0_us) {
  for (size_t k = 0; k < tr.t.size(); ++k) {

    CAN_message_t fm;
    fm.id = TRAJ_OUT_ID;
    fm.len = 8;

    float pos_f = tr.x[k];
    int16_t vel_i = int16_t(lrintf(tr.v[k] * vel_scale));
    int16_t tor_i = int16_t(lrintf(tr.tor[k] * tor_scale));

    memcpy(&fm.buf[0], &pos_f, 4);
    fm.buf[4] = vel_i & 0xFF;
    fm.buf[5] = vel_i >> 8;
    fm.buf[6] = tor_i & 0xFF;
    fm.buf[7] = tor_i >> 8;

    packedMsgs.push_back(fm);

    int64_t rel_us = lrintf(tr.t[k] * 1'000'000.f);
    uint64_t abs_us = uint64_t(int64_t(abs_t0_us) + rel_us);
    sendUs.push_back(abs_us);
  }
}

/*----------------------------------------------------------------------------*/
/*                          T R A F F I C   R E P O R T                       */
/*----------------------------------------------------------------------------*/
void reportStatus() {
  uint32_t now = millis();
  if (now - lastReportTime < reportInterval) return;

  CAN_message_t m;
  m.id = REPORT_ID;
  m.len = 4;
  m.buf[0] = receivedCount & 0xFF;
  m.buf[1] = receivedCount >> 8;
  m.buf[2] = reportInterval & 0xFF;
  m.buf[3] = reportInterval >> 8;
  can1.write(m);

#if DEBUG_TRAFFIC
  Serial.printf("CAN traffic report: %u msgs / %u ms\n",
                receivedCount, reportInterval);
#endif

  receivedCount = 0;
  lastReportTime = now;
}

/*----------------------------------------------------------------------------*/
/*                     I N C L I N O M E T E R  H E L P E R S                 */
/*----------------------------------------------------------------------------*/
float convertTo180Range(float a) {
  return (a > 180.f) ? a - 360.f : a;
}

platformTilt getInclination() {
  platformTilt t{ PI, PI };  // default error value
  const int maxAttempts = 3;

  for (int k = 0; k < maxAttempts; ++k) {
    if (inclinometer.available()) {
      float ay = convertTo180Range(inclinometer.getCalculatedAngleY());
      float az = convertTo180Range(inclinometer.getCalculatedAngleZ());

      t.tiltX = -(az)*PI / 180.f;
      t.tiltY = -(ay)*PI / 180.f;

      Serial.printf("X:%f Y:%f\n\n", t.tiltX, t.tiltY);
      return t;
    }
    inclinometer.reset();
    delay(100);
  }
  return t;  // returns {π,π} if failed
}

void sendTiltData(platformTilt til) {
  CAN_message_t m;
  m.id = tiltID;
  m.len = 8;
  memcpy(&m.buf[0], &til.tiltX, 4);
  memcpy(&m.buf[4], &til.tiltY, 4);
  can1.write(m);
  Serial.printf("Sent tilt: X=%f Y=%f\n", til.tiltX, til.tiltY);
}

/*----------------------------------------------------------------------------*/
/*            R O B O T  S T A T E  P A C K / U N P A C K                     */
/*----------------------------------------------------------------------------*/
void createStateCANMessage(const RobotState &s, CAN_message_t &m) {
  uint8_t flags = (s.is_homed << 0) | (s.levelling_complete << 1);
  int16_t x = int16_t(s.pose_offset_tiltX * 1000.f);
  int16_t y = int16_t(s.pose_offset_tiltY * 1000.f);

  m.len = 8;
  m.buf[0] = flags;
  m.buf[1] = x & 0xFF;
  m.buf[2] = x >> 8;
  m.buf[3] = y & 0xFF;
  m.buf[4] = y >> 8;
  m.buf[5] = m.buf[6] = m.buf[7] = 0;
}

void decodeStateCANMessage(const CAN_message_t &m, RobotState &s) {
  if (m.len != 8) return;
  uint8_t f = m.buf[0];
  s.is_homed = f & 1;
  s.levelling_complete = (f >> 1) & 1;
  int16_t x = (m.buf[2] << 8) | m.buf[1];
  int16_t y = (m.buf[4] << 8) | m.buf[3];
  s.pose_offset_tiltX = x / 1000.f;
  s.pose_offset_tiltY = y / 1000.f;
}

/*----------------------------------------------------------------------------*/
/*                    D E B U G /   T I M I N G                               */
/*----------------------------------------------------------------------------*/
void debugPrintMessage(const CAN_message_t &msg) {
  if (msg.id != debugID) return;
  Serial.printf("Debug ID 0x%X:", msg.id);
  for (int i = 0; i < msg.len; ++i) Serial.printf(" 0x%02X", msg.buf[i]);
  Serial.println();
}

void analyzeMessageTiming(const CAN_message_t &msg) {
  if (msg.id != timingID) return;

  unsigned long now = millis();
  if (lastMessageTime) {
    totalIntervals += now - lastMessageTime;
    ++messageCount;
  }
  lastMessageTime = now;

  if (now - lastReportTime < analysisPeriod) return;
  if (messageCount) {
    Serial.printf("Avg interval for 0x%X: %lu ms\n",
                  timingID, totalIntervals / messageCount);
  }
  totalIntervals = messageCount = 0;
  lastReportTime = now;
}

/*----------------------------------------------------------------------------*/
/*                 C A N   C A L L B A C K  &  S N I F F E R                  */
/*----------------------------------------------------------------------------*/
void canSniff(const CAN_message_t &msg) {
  receivedCount++;

  /* — Inclinometer trigger — */
  if (msg.id == tiltID) {
    platformTilt t = getInclination();
    sendTiltData(t);
  }

  /* — Robot‑state exchange — */
  if (msg.id == stateUpdateID) {
    if (msg.len == 1 && msg.buf[0] == 0x01) {  // state request
      CAN_message_t reply;
      createStateCANMessage(state, reply);
      reply.id = stateUpdateID;
      can1.write(reply);
      Serial.println("State sent to host PC.");
    } else if (msg.len == 8) {  // state update
      decodeStateCANMessage(msg, state);
      Serial.println("State updated from host PC.");
    }
  }

  /* — time‑sync frame — */
  if (msg.id == timeSyncID && msg.len == 8) {
    TimeSync::handleSyncFrame(msg);
  }

  /* ── hand encoder estimates (axis 6) ───────────────────────────── */
  if (msg.id == HAND_ENC_EST_ID && msg.len == 8) {
    float pos, vel;
    memcpy(&pos, &msg.buf[0], 4);  // Pos_Estimate   [rev]
    memcpy(&vel, &msg.buf[4], 4);  // Vel_Estimate   [rev/s]

    current_hand_position = pos;
    current_hand_velocity = vel;
    last_hand_update_us = TimeSync::get_wall_time_us();  // 64-bit

    return;  // nothing else to do for this frame
  }

  /* ── trajectory command  ID 0x6D0  ───────────────────────────────
  Byte 0 : kind          (0=Throw  1=Catch  2=Full)
  Byte 1‑2 : velocity*100 (uint16, little‑endian, 0.01 m/s units)
  Byte 3‑6 : wall‑time ms (uint32, little‑endian, absolute Unix)
  Byte 7 : reserved (0)
  */
  if (msg.id == CMD_TRAJ_ID && msg.len == 8) {

    /* ---- unpack ---- */
    uint8_t kind = msg.buf[0];

    uint16_t vel_u16 = msg.buf[1] | (msg.buf[2] << 8);
    float vel = vel_u16 * 0.01f;  // back to m/s

    uint32_t event_lo = msg.buf[3] |  // Lowest 32 bits of the timestamp
                        (msg.buf[4] << 8) | (msg.buf[5] << 16) | (msg.buf[6] << 24);
    uint64_t event_ms_full = reconstructWallMs(event_lo);
    uint64_t event_wall_us = event_ms_full * 1000ULL;  // Convert the event wall time into microseconds

    /* ----- debug: show received main-event time ---------------- */
    // Serial.printf("[HAND TRAJ] received event wall-time_ms=0x%08lX  "
    //               "(%llu ms (UTC)))\n",
    //               event_lo, (unsigned long long)event_ms_full);

    // Serial.printf("Current wall-time: %llu ms (UTC)\n",
    //               TimeSync::get_wall_time_us()/1000ULL);

    /* ---- build trajectory ---- */
    HandTrajGenerator gen(vel);
    switch (kind) {
      case 0: activeTraj = gen.makeThrow(); break;
      case 1: activeTraj = gen.makeCatch(); break;
      case 2: activeTraj = gen.makeFull(); break;
      default: Serial.println("! unknown traj kind"); return;
    }

    /* -------------- SMOOTH-MOVE PRELUDE --------------------------- */
    Trajectory smooth = makeSmoothMove(activeTraj.x.front());
    uint64_t now_us = TimeSync::get_wall_time_us();
    uint32_t smoothDur_us = smooth.t.empty() ? 0 : uint32_t(lrintf(smooth.t.back() * 1'000'000.f));

    /* first frame of main trajectory (could be < event time) */
    int64_t firstMainRel_us =
      lrintf(activeTraj.t.front() * 1'000'000.f);
    uint64_t firstMainAbs_us =
      uint64_t(int64_t(event_wall_us) + firstMainRel_us);

    /* ----- time-budget check ------------------------------------- */
    if (now_us + smoothDur_us + SAFETY_GAP_US > firstMainAbs_us) {
      Serial.println("❌  Not enough time for smooth-move; command ignored.");
      return;  // abort
    }

    /* --------------- PACK BOTH TRAJECTORIES ----------------------- */
    packedMsgs.clear();
    sendUs.clear();
    packedMsgs.reserve(smooth.t.size() + activeTraj.t.size());
    sendUs.reserve(smooth.t.size() + activeTraj.t.size());

    if (!smooth.t.empty())
      packTrajectory(smooth, now_us);  // start immediately

    packTrajectory(activeTraj, event_wall_us);  // already wall-aligned

    /* -------------- ARM SCHEDULER ------------------------------- */
    nextIdx = 0;
    trajActive = true;

    /* -------------- DEBUG --------------------------------------- */
    float wait_main_s = float(firstMainAbs_us - now_us) / 1e6f;
    Serial.printf("Smooth-move pts=%u dur=%.2f s | "
                  "main pts=%u starts_in=%.2f s  (kind=%u vel=%.2f)\n",
                  smooth.t.size(), smoothDur_us / 1e6f,
                  activeTraj.t.size(), wait_main_s,
                  kind, vel);

    /* ---------- debugging ------------- */
    float wait_s = float(event_wall_us - now_wall_us()) / 1e6f;
    if (wait_s < 0) wait_s = 0.f;  // shouldn’t be, but guard
    Serial.printf("Trajectory armed: kind=%u  vel=%.2f  pts=%u  starts_in=%.3f s\n",
                  kind, vel, packedMsgs.size(), wait_s);
  }

  // debugPrintMessage(msg);
  analyzeMessageTiming(msg);
}

/*----------------------------------------------------------------------------*/
/*                                   SET‑UP                                   */
/*----------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);

  /* CAN bus */
  can1.begin();
  can1.setBaudRate(CAN_BITRATE);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);

  /* Inclinometer */
  while (!inclinometer.begin()) {
    Serial.println("SCL3300 not found, resetting…");
    inclinometer.reset();
    delay(500);
  }
  Serial.println("SCL3300 initialised.");
  Serial.println("Teensy platform MCU ready.");
}

/*----------------------------------------------------------------------------*/
/*                                   L O O P                                   */
/*----------------------------------------------------------------------------*/
void loop() {
  can1.events();   // dispatch CAN callbacks
  reportStatus();  // traffic monitor

  #if DEBUG_TIME_SYNC
    TimeSync::maybePrintStats();  // console jitter read‑out, if desired.
  #endif

  /* ---- trajectory point streamer ---- */
  if (trajActive && nextIdx < packedMsgs.size()) {
    uint64_t now_us = TimeSync::get_wall_time_us();

    /* send every frame whose absolute timestamp has passed */
    while (nextIdx < packedMsgs.size() && now_us >= sendUs[nextIdx]) {
      can1.write(packedMsgs[nextIdx]);
    
    /* optional debug print — pays ≤5 µs */
    #if DEBUG_TRAJ
          float pos_f;
          memcpy(&pos_f, &packedMsgs[nextIdx].buf[0], 4);
          int16_t v_i = packedMsgs[nextIdx].buf[4] | (packedMsgs[nextIdx].buf[5] << 8);
          int16_t q_i = packedMsgs[nextIdx].buf[6] | (packedMsgs[nextIdx].buf[7] << 8);
          Serial.printf("idx=%u | t=%.3f s | pos=%.3f rev | vel=%.3f rev/s | tor=%.3f N·m\n",
                        nextIdx, sendUs[nextIdx] / 1e6f, pos_f,
                        v_i / vel_scale, q_i / tor_scale);
    #endif
      ++nextIdx;
      now_us = TimeSync::get_wall_time_us();  // refresh for tight bursts
    }
    if (nextIdx >= packedMsgs.size()) trajActive = false;  // done
  }
}
