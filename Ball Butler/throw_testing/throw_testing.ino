/*****************************************************************************************
 *  Teensy 4.0 — Ball Butler  (Serial-commanded trajectories)
 *  ------------------------------------------------------------------
 *  Replaces CAN CMD_TRAJ_ID control with a line-based Serial protocol.
 *  Usual trajectories: THROW, CATCH, FULL
 *  Smooth moves: SMOOTH <end_pos_rev> [delay_ms]
 *
 *  Serial commands (newline-terminated):
 *    THROW <vel_mps> [delay_ms]
 *    CATCH <vel_mps> [delay_ms]
 *    FULL  <vel_mps> [delay_ms]
 *    SMOOTH <end_pos_rev> [delay_ms]
 *    STOP
 *    STATUS
 *    SCALE VEL <scale>
 *    SCALE TOR <scale>
 *
 *  CAN time-sync + encoder ingest unchanged.
 *****************************************************************************************/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <SCL3300.h>
#include <vector>
#include "Trajectory.h"

#define DEBUG_TRAJ 0       // 0 = silent, 1 = Serial print each hand traj frame as it gets sent out
#define DEBUG_TIME_SYNC 0  // 0 = silent, 1 = Serial print periodic messages showing how tight the clocks are synced
#define DEBUG_CAN 0        // 0 = silent, 1 = Serial print CAN messages (current pos, vel)

/*----------------------------------------------------------------------------*/
/*                                CAN BUS SET-UP                              */
/*----------------------------------------------------------------------------*/
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
constexpr uint32_t CAN_BITRATE = 1'000'000;  // 1 Mbps

/*----------------------------------------------------------------------------*/
/*                               CAN  ID MAP                                  */
/*----------------------------------------------------------------------------*/
constexpr uint32_t timeSyncID = 0x7DD;     // wall-time sync

/*----------------------------------------------------------------------------*/
/*                      DEBUG / TIMING-ANALYSIS                               */
/*----------------------------------------------------------------------------*/
const uint32_t node_id = 8;
const uint32_t cmd_id = 0x018;
const uint32_t debugID = (node_id << 5) | cmd_id;

uint32_t timingID = (node_id << 5) | 0x04;
uint16_t targetContentID = 385;
unsigned long lastMessageTime = 0;
const unsigned long analysisPeriod = 500;
unsigned long totalIntervals = 0;
int messageCount = 0;

/*----------------------------------------------------------------------------*/
/*                        HAND ENCODER ESTIMATES                              */
/*----------------------------------------------------------------------------*/
constexpr uint32_t HAND_AXIS_NODE = 8;
constexpr uint32_t HAND_ENC_EST_ID = (HAND_AXIS_NODE << 5) | 0x09;  // 0xC9
constexpr uint32_t TRAJ_OUT_ID = (HAND_AXIS_NODE << 5) | 0x0c;      // "set input pos" for hand

/* most-recent hand state (updated at 500 Hz) */
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
  } while (ts1 != ts2);
  t_us = ts1;
}

/*----------------------------------------------------------------------------*/
/*                        ──  TIME–SYNC LAYER ──                              */
/*----------------------------------------------------------------------------*/
namespace TimeSync {
uint64_t micros64() {
  static uint32_t last_lo = ::micros();
  static uint64_t hi = 0;
  uint32_t now = ::micros();
  if (now < last_lo) hi += UINT64_C(1) << 32;
  last_lo = now;
  return hi | now;
}
volatile int64_t wall_offset_us = 0;
volatile bool have_offset = false;
constexpr uint8_t ALPHA_SHIFT = 3;

struct Stats {
  int32_t sum = 0;
  uint32_t sum_sq = 0;
  int32_t min = INT32_MAX, max = INT32_MIN;
  uint32_t n = 0;
  void add(int32_t x) { sum += x; sum_sq += uint32_t(x) * x; if (x < min) min = x; if (x > max) max = x; ++n; }
  void clear() { *this = {}; }
  float mean() const { return n ? float(sum) / n : 0.f; }
  float rms()  const { return n ? sqrtf(float(sum_sq) / n) : 0.f; }
} stats;
uint64_t nextPrint_us = 0;
constexpr uint32_t PRINT_PERIOD_US = 1'000'000;

uint64_t get_wall_time_us() { return micros64() + wall_offset_us; }
uint64_t wall_to_local_us(uint64_t w_us) { return w_us - wall_offset_us; }

inline void handleSyncFrame(const CAN_message_t &msg) {
  uint32_t sec  = msg.buf[0] | (msg.buf[1] << 8) | (msg.buf[2] << 16) | (msg.buf[3] << 24);
  uint32_t usec = msg.buf[4] | (msg.buf[5] << 8) | (msg.buf[6] << 16) | (msg.buf[7] << 24);
  uint64_t jetson_us = uint64_t(sec) * 1'000'000ULL + usec;
  uint64_t local_us  = micros64();
  int64_t offset = int64_t(jetson_us) - int64_t(local_us);

  if (!have_offset) { wall_offset_us = offset; have_offset = true; }
  else { int64_t diff = offset - wall_offset_us; wall_offset_us += diff >> ALPHA_SHIFT; }

  int32_t delta = int32_t(offset - wall_offset_us);
  stats.add(delta);
}

inline void maybePrintStats() {
#if DEBUG_TIME_SYNC
  uint64_t now = micros64();
  if (now < nextPrint_us) return;
  nextPrint_us = now + PRINT_PERIOD_US;
  if (stats.n) {
    Serial.printf("Δmean %+0.1f us | rms %.1f us | min %+d us | max %+d us | frames %lu\n",
                  (double)stats.mean(), (double)stats.rms(), stats.min, stats.max, stats.n);
    stats.clear();
  }
#endif
}
}  // namespace TimeSync

static uint64_t reconstructWallMs(uint32_t low32) {
  uint64_t now_ms = TimeSync::get_wall_time_us() / 1000ULL;
  uint64_t candidate = (now_ms & 0xFFFFFFFF00000000ULL) | low32;
  if (candidate > now_ms + 0x80000000ULL) candidate -= 0x100000000ULL;
  else if (candidate + 0x80000000ULL < now_ms) candidate += 0x100000000ULL;
  return candidate;
}
inline uint64_t now_wall_us() { return TimeSync::get_wall_time_us(); }

/*----------------------------------------------------------------------------*/
/*                           Trajectory Scheduler                             */
/*----------------------------------------------------------------------------*/
Trajectory activeTraj;
elapsedMicros trajClock;
bool trajActive = false;
size_t nextIdx = 0;
uint32_t nextSendUs = 0;
float vel_scale = 100.0f;                 // ODrive input_vel_scale
float tor_scale = 100.0f;                 // ODrive input_torque_scale
std::vector<CAN_message_t> packedMsgs;
std::vector<uint64_t> sendUs;
constexpr uint32_t SAFETY_GAP_US = 20'000;  // 20 ms pause after smooth-move before main traj

void packTrajectory(const Trajectory &tr, uint64_t abs_t0_us) {
  // Reserve to avoid reallocation during the loop
  packedMsgs.reserve(packedMsgs.size() + tr.t.size());
  sendUs.reserve(sendUs.size() + tr.t.size());

  for (size_t k = 0; k < tr.t.size(); ++k) {
    CAN_message_t fm;
    fm.id = TRAJ_OUT_ID;
    fm.len = 8;

    float pos_f = tr.x[k];
    int16_t vel_i = int16_t(lrintf(tr.v[k] * vel_scale));
    int16_t tor_i = int16_t(lrintf(tr.tor[k] * tor_scale));

    memcpy(&fm.buf[0], &pos_f, 4);
    fm.buf[4] = vel_i & 0xFF; fm.buf[5] = vel_i >> 8;
    fm.buf[6] = tor_i & 0xFF; fm.buf[7] = tor_i >> 8;

    packedMsgs.push_back(fm);

    int64_t rel_us = lrintf(tr.t[k] * 1'000'000.f);
    uint64_t abs_us = uint64_t(int64_t(abs_t0_us) + rel_us);
    sendUs.push_back(abs_us);
  }
}

void debugPrintMessage(const CAN_message_t &msg) {
  if (msg.id != debugID) return;
  Serial.printf("Debug ID 0x%X:", msg.id);
  for (int i = 0; i < msg.len; ++i) Serial.printf(" 0x%02X", msg.buf[i]);
  Serial.println();
}


/*----------------------------------------------------------------------------*/
/*                 C A N   C A L L B A C K  &  S N I F F E R                  */
/*----------------------------------------------------------------------------*/
void canSniff(const CAN_message_t &msg) {
  if (msg.id == timeSyncID && msg.len == 8) {
    TimeSync::handleSyncFrame(msg);
  }

  if (msg.id == HAND_ENC_EST_ID && msg.len == 8) {
    float pos, vel;
    memcpy(&pos, &msg.buf[0], 4);
    memcpy(&vel, &msg.buf[4], 4);
    current_hand_position = pos;
    current_hand_velocity = vel;
    last_hand_update_us = TimeSync::get_wall_time_us();

    #if DEBUG_CAN
      Serial.printf("pos=%.3f rev | vel=%.3f rev/s \n",
                    current_hand_position, current_hand_velocity);
    #endif

    return;
  }
}

/*----------------------------------------------------------------------------*/
/*                      SERIAL COMMAND PARSER / EXECUTOR                      */
/*----------------------------------------------------------------------------*/
String rxLine;

enum class MainKind { None, ThrowK, CatchK, FullK };

bool armAndSchedule(const Trajectory& mainTraj,
                    bool withSmoothPrelude,
                    float smoothEndPos,
                    uint32_t delay_ms)
{
  packedMsgs.clear();
  sendUs.clear();

  float start_p, start_v; uint64_t t_us;
  getHandPosVel(start_p, start_v, t_us);

  if (t_us == 0) {                      // no encoder yet
    Serial.println("! No encoder reading yet; cannot generate smooth move.");
    return false;
  }

  const uint64_t now_us = TimeSync::get_wall_time_us();
  const uint64_t event_wall_us = now_us + uint64_t(delay_ms) * 1000ULL;

  // If requested, build the smooth move to align before main
  uint32_t smoothDur_us = 0;
  Trajectory smooth;
  if (withSmoothPrelude) {
    smooth = makeSmoothMove(start_p, smoothEndPos);
    if (!smooth.t.empty())
      smoothDur_us = uint32_t(lrintf(smooth.t.back() * 1'000'000.f));

    // First frame of main trajectory in absolute wall time
    int64_t firstMainRel_us = lrintf(mainTraj.t.front() * 1'000'000.f);
    uint64_t firstMainAbs_us = uint64_t(int64_t(event_wall_us) + firstMainRel_us);

    // Time-budget check: smooth + safety gap must fit before main starts
    if (now_us + smoothDur_us + SAFETY_GAP_US > firstMainAbs_us) {
      Serial.println("❌ Not enough time for smooth-move before main; increase delay_ms.");
      return false;
    }

    // Pack smooth move to start immediately
    if (!smooth.t.empty())
      packTrajectory(smooth, now_us);
  }

  // Pack main trajectory aligned to event_wall_us
  packTrajectory(mainTraj, event_wall_us);

  nextIdx = 0;
  trajActive = true;

  float wait_main_s = float(event_wall_us - now_us) / 1e6f;
  Serial.printf("Armed: main pts=%u starts_in=%.3f s | smooth=%s (pts=%u, dur=%.2f s)\n",
                (unsigned)mainTraj.t.size(), wait_main_s,
                withSmoothPrelude ? "yes" : "no",
                (unsigned)smooth.t.size(), smoothDur_us / 1e6f);
  return true;
}

// Smooth-only scheduling
bool scheduleSmoothOnly(float endPosRev, uint32_t delay_ms) {
  packedMsgs.clear();
  sendUs.clear();

  float start_p, start_v; uint64_t t_us;
  getHandPosVel(start_p, start_v, t_us);

  if (t_us == 0) { // no encoder yet
    Serial.println("! No encoder reading yet; cannot generate smooth move.");
    return false;
  }

  Trajectory smooth = makeSmoothMove(start_p, endPosRev);
  if (smooth.t.empty()) {
    Serial.println("! Smooth move is empty; nothing to do.");
    return false;
  }
  uint64_t t0 = TimeSync::get_wall_time_us() + uint64_t(delay_ms) * 1000ULL;
  packTrajectory(smooth, t0);
  nextIdx = 0;
  trajActive = true;

  float dur_s = smooth.t.back();
  Serial.printf("Armed: SMOOTH only, pts=%u, starts_in=%.3f s, dur=%.2f s (end=%.3f rev)\n",
                (unsigned)smooth.t.size(),
                delay_ms / 1000.0f, dur_s, endPosRev);
  return true;
}

void handleSerialLine(const String& line) {
  // Basic tokenizer on spaces
  const int maxTok = 6;
  String tok[maxTok];
  int nt = 0;
  size_t start = 0;
  while (nt < maxTok) {
    int sp = line.indexOf(' ', start);
    if (sp < 0) { tok[nt++] = line.substring(start); break; }
    tok[nt++] = line.substring(start, sp);
    start = sp + 1;
    // skip multiple spaces
    while (start < line.length() && line[start] == ' ') start++;
  }
  for (int i = 0; i < nt; ++i) tok[i].trim();
  if (nt == 0 || tok[0].length() == 0) return;

  auto asFloat = [](const String& s, float& out)->bool {
    char* endp = nullptr;
    out = strtof(s.c_str(), &endp);
    return (endp && *endp == '\0');
  };
  auto asU32 = [](const String& s, uint32_t& out)->bool {
    char* endp = nullptr;
    unsigned long v = strtoul(s.c_str(), &endp, 10);
    if (!(endp && *endp == '\0')) return false;
    out = (uint32_t)v; return true;
  };

  String cmd = tok[0];
  cmd.toUpperCase();

  if (cmd == "STOP") {
    trajActive = false;
    packedMsgs.clear();
    sendUs.clear();
    Serial.println("OK: streaming cancelled.");
    return;
  }

  if (cmd == "STATUS") {
    float p, v; uint64_t t;
    getHandPosVel(p, v, t);
    Serial.printf("pos=%.4f rev  vel=%.4f rev/s  t_us=%llu  queued_pts=%u  active=%d\n",
                  p, v, (unsigned long long)t, (unsigned)packedMsgs.size(), (int)trajActive);
    return;
  }

  if (cmd == "SCALE" && nt >= 3) {
    String which = tok[1]; which.toUpperCase();
    float val;
    if (!asFloat(tok[2], val)) { Serial.println("! SCALE value parse error."); return; }
    if (which == "VEL") { vel_scale = val; Serial.printf("OK: vel_scale=%.3f\n", vel_scale); return; }
    if (which == "TOR") { tor_scale = val; Serial.printf("OK: tor_scale=%.3f\n", tor_scale); return; }
    Serial.println("! SCALE which must be VEL or TOR");
    return;
  }

  if (cmd == "SMOOTH") {
    if (nt < 2) { Serial.println("Usage: SMOOTH <end_pos_rev> [delay_ms]"); return; }
    float endPos;
    if (!asFloat(tok[1], endPos)) { Serial.println("! end_pos_rev parse error"); return; }
    uint32_t delay_ms = 0;
    if (nt >= 3 && !asU32(tok[2], delay_ms)) { Serial.println("! delay_ms parse error"); return; }
    scheduleSmoothOnly(endPos, delay_ms);
    return;
  }

  // Usual trajectories (THROW / CATCH / FULL)
  if (cmd == "THROW" || cmd == "CATCH" || cmd == "FULL") {
    if (nt < 2) { Serial.printf("Usage: %s <vel_mps> [delay_ms]\n", cmd.c_str()); return; }
    float vel_mps;
    if (!asFloat(tok[1], vel_mps)) { Serial.println("! vel_mps parse error"); return; }
    uint32_t delay_ms = 0;
    if (nt >= 3 && !asU32(tok[2], delay_ms)) { Serial.println("! delay_ms parse error"); return; }

    HandTrajGenerator gen(vel_mps);
    Trajectory mainTr;
    if (cmd == "THROW")      mainTr = gen.makeThrow();
    else if (cmd == "CATCH") mainTr = gen.makeCatch();
    else                     mainTr = gen.makeFull();

    if (mainTr.t.empty()) { Serial.println("! main trajectory is empty"); return; }

    // Use a smooth prelude to move to main's first position
    const float smoothEnd = mainTr.x.front();
    armAndSchedule(mainTr, /*withSmoothPrelude=*/true, smoothEnd, delay_ms);
    return;
  }

  Serial.println("! Unknown command.");
}

/*----------------------------------------------------------------------------*/
/*                                   SET-UP                                   */
/*----------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);

  /* CAN bus */
  can2.begin();
  can2.setBaudRate(CAN_BITRATE);
  can2.setMaxMB(16);
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(canSniff);

  Serial.println("Teensy Ready (Serial-commanded trajectories).");
}

/*----------------------------------------------------------------------------*/
/*                                   L O O P                                  */
/*----------------------------------------------------------------------------*/
void loop() {
  can2.events();   // dispatch CAN callbacks
  TimeSync::maybePrintStats();

  /* ---- stream scheduled trajectory frames ---- */
  if (trajActive && nextIdx < packedMsgs.size()) {
    uint64_t now_us = TimeSync::get_wall_time_us();

    while (nextIdx < packedMsgs.size() && now_us >= sendUs[nextIdx]) {
      can2.write(packedMsgs[nextIdx]);

      #if DEBUG_TRAJ
        float pos_f; memcpy(&pos_f, &packedMsgs[nextIdx].buf[0], 4);
        int16_t v_i = packedMsgs[nextIdx].buf[4] | (packedMsgs[nextIdx].buf[5] << 8);
        int16_t q_i = packedMsgs[nextIdx].buf[6] | (packedMsgs[nextIdx].buf[7] << 8);
        Serial.printf("idx=%u | t=%.3f s | pos=%.3f rev | vel=%.3f rev/s | tor=%.3f\n",
                      (unsigned)nextIdx, sendUs[nextIdx] / 1e6f, pos_f,
                      v_i / vel_scale, q_i / tor_scale);
      #endif
      ++nextIdx;
      now_us = TimeSync::get_wall_time_us();  // refresh for bursts
    }
    if (nextIdx >= packedMsgs.size()) trajActive = false;
  }

  /* ---- read line-oriented serial commands ---- */
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rxLine.trim();
      if (rxLine.length()) handleSerialLine(rxLine);
      rxLine = "";
    } else {
      rxLine += c;
      if (rxLine.length() > 200) rxLine.remove(0, rxLine.length() - 200); // guard
    }
  }
}
