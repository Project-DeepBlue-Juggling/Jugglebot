// ball_butler_main.ino
#include <vector>
#include "YawAxis.h"
#include "PitchAxis.h"
#include "HandPathPlanner.h"
#include "CanInterface.h"
#include "HandTrajectoryStreamer.h"
#include "TrajFrame.h"    // shared frame type for planner/streamer/CSV
#include "Trajectory.h"   // for HAND_MAX_SMOOTH_MOVE_POS bound

// ============================== Policy / Guard Rails =========================
static constexpr float   THROW_VEL_MIN_MPS     = 0.05f;   // must be > 0
static constexpr float   THROW_VEL_MAX_MPS     = 6.0f;    // hard limit
static constexpr float   SCHEDULE_MARGIN_S     = 0.1f;    // Extra margin to wait at the bottom [sec]
static constexpr uint64_t PV_MAX_AGE_US        = 20'000;  // 20 ms freshness for pos/vel encoder data
using Err = CanInterface::ODriveErrors;

// ============================== Globals ======================================
CanInterface           canif;
HandPathPlanner        planner;                 // 500 Hz, 0.5 s pause default
HandTrajectoryStreamer streamer(canif);

// CAN and Node IDs
static constexpr uint32_t HOST_THROW_CAN_ID = 0x7D0;
static constexpr uint8_t PITCH_NODE_ID = 7;
static constexpr uint8_t HAND_NODE_ID = 8;

PitchAxis pitch(canif, PITCH_NODE_ID);

// Persistent buffer to keep frames alive while streaming
static std::vector<TrajFrame> g_traj_buffer;

// -------------------------- YAW axis hardware pins ---------------------------------------
YawAxis yawAxis(/*CS*/10, /*INA1*/16, /*INA2*/15, /*PWM*/14, /*PWM_MIN*/60, /*PWM_HZ*/20000);

// -------------------------- YAW telemetry controls ---------------------------------------
static bool     yaw_stream_on         = false;
static const    uint16_t YAW_TELEM_MS = 500;
static uint32_t yaw_last_stream_ms    = 0;

// -------------------------- Forward decls -----------------------------------------------
void onHostThrow(const CanInterface::HostThrowCmd& c, void* user);
void yawPrintHelp();
void yawHandleLine(const String& yawLine);
void routeCommand(const String& rawLine);

static bool waitForClear(uint32_t node_id, uint32_t mask,
                         uint32_t timeout_ms, uint16_t poll_ms = 5);
static bool homeHandWithAutoClearRetry(uint32_t node_id,
                         uint8_t max_attempts = 3,
                         uint32_t pre_wait_ms = 1500,
                         uint32_t post_fail_wait_ms = 1500);

void  printTopHelp();
bool  handleThrowCmd(const String& line);    // "throw <vel_mps> <time_from_now_s>"
bool  handleSmoothCmd(const String& line);   // "smooth <target_pos_rev>"
static bool getFreshPV(float& pos_rev, float& vel_rps, uint64_t& t_wall_us);

void pitchHandleLine(const String& line);

// CSV emitter: called for every frame when printing a plan for debugging
static void emitCSV(const TrajFrame& f) {
  Serial.printf("%.6f,%.6f,%.6f,%.6f\n", f.t_s, f.pos_cmd, f.vel_ff, f.tor_ff);
}

// ====================================== SETUP ============================================
void setup(){
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  canif.begin(1'000'000); // CAN @ 1 Mbps
  canif.setDebugStream(&Serial);
  canif.setDebugFlags(/*timeSync*/false, /*canTraffic*/ false);

  canif.setHostThrowCmdId(HOST_THROW_CAN_ID);
  canif.setHostThrowCallback(&onHostThrow, nullptr);

  canif.setAutoClearBrakeResistor(true, /*min_interval_ms=*/500);

  canif.requireHomeOnlyFor(HAND_NODE_ID);

  // Robust homing: wait for the auto-clear to clean the bit, then home (with retries)
  const bool hand_homed = homeHandWithAutoClearRetry(HAND_NODE_ID, /*max_attempts=*/3,
                                                /*pre_wait_ms=*/1500, /*post_fail_wait_ms=*/1500);
  Serial.printf("Home result: %s\n", hand_homed ? "OK" : "FAIL");

  yawAxis.begin();
  pitch.begin();
  printTopHelp();
}

// ======================================= LOOP ============================================
void loop(){
  canif.loop();   // Service CAN and time-sync layer
  streamer.tick();

  // Serial line reader
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      buf.trim();
      if (buf.length()) routeCommand(buf);
      buf = "";
    } else {
      buf += c;
    }
  }

  // ---- YAW telemetry stream (printing only; YawAxis library does not print) ----
  if (yaw_stream_on && (millis() - yaw_last_stream_ms >= YAW_TELEM_MS)) {
    yaw_last_stream_ms = millis();
    auto t = yawAxis.readTelemetry();
    Serial.printf(
      "YAW | EN=%d ESTOP=%d pos=%.3fdeg vel=%.3frps cmd=%.3fdeg err=%.3fdeg u=%.1f u_slew=%.1f pwm=%d A=%.0f D=%.0f\n",
      (int)t.enabled, (int)t.estop,
      t.pos_deg, t.vel_rps, t.cmd_deg, t.err_deg,
      t.u, t.u_slew, (int)t.pwm, t.accelPPS, t.decelPPS
    );
  }
}

// ============================== Processing Jetson Commands ===============================
void onHostThrow(const CanInterface::HostThrowCmd& c, void* /*user*/) {
  // 1) Point yaw & pitch
  const float yaw_deg   = c.yaw_rad   * (180.0f / (float)M_PI);
  const float pitch_deg = c.pitch_rad * (180.0f / (float)M_PI);

  Serial.println("CAN frame received!");
  Serial.println(yaw_deg);
  Serial.println(pitch_deg);
  Serial.println(c.speed_mps);
  Serial.println(c.in_s);
  yawAxis.setTargetDeg(yaw_deg);
  pitch.setTargetDeg(pitch_deg);

  // 2) Hand must be homed
  if (!canif.isAxisHomed(HAND_NODE_ID)) {
    Serial.println("[HostCmd] Ignored throw: hand not homed.");
    return;
  }

  // 3) Fresh PV snapshot
  float pos_rev=0.f, vel_rps=0.f; uint64_t t_wall_us=0;
  if (!canif.getAxisPV(HAND_NODE_ID, pos_rev, vel_rps, t_wall_us)) {
    Serial.println("[HostCmd] PV unavailable; ignoring throw.");
    return;
  }
  if (canif.wallTimeUs() - t_wall_us > PV_MAX_AGE_US) {
    Serial.println("[HostCmd] PV stale; ignoring throw.");
    return;
  }

  // 4) Plan & check lead time
  auto plan = planner.planThrowDecelZero(c.speed_mps, pos_rev, vel_rps, (uint32_t)(t_wall_us & 0xFFFFFFFFu));
  if (plan.trajectory.empty()) {
    Serial.println("[HostCmd] Planner returned empty trajectory.");
    return;
  }
  float t_min = 0.f;
  for (auto& f : plan.trajectory) if (f.t_s < t_min) t_min = f.t_s;
  const float need_lead = -t_min + SCHEDULE_MARGIN_S;
  if (c.in_s < need_lead) {
    Serial.printf("[HostCmd] Not enough lead time (in=%.3f s need≥%.3f s). Ignoring.\n",
                  (double)c.in_s, (double)need_lead);
    return;
  }

  // 5) Arm streamer
  g_traj_buffer = std::move(plan.trajectory);
  const float start_wall_s = canif.wallTimeUs() * 1e-6f + c.in_s;
  const bool ok = streamer.arm(HAND_NODE_ID, g_traj_buffer.data(), g_traj_buffer.size(), start_wall_s);

  Serial.printf("[HostCmd] yaw=%.2f° pitch=%.2f° speed=%.3f m/s in=%.3f s -> %s (frames=%u, ready=%.3fs)\n",
                (double)yaw_deg, (double)pitch_deg, (double)c.speed_mps, (double)c.in_s,
                ok ? "ARMed" : "ARM FAIL", (unsigned)g_traj_buffer.size(), planner.lastTimeToReadyS());
}

// ============================== COMMAND ROUTING (TOP-LEVEL) ===============================
void routeCommand(const String& rawLine){
  String line = rawLine;
  line.trim();

  // Lowercase copy for prefix check (do not mutate original params past the prefix)
  String lc = line;
  lc.toLowerCase();

  // Top-level commands
  if (lc.startsWith("throw "))  { if (!handleThrowCmd(line))  Serial.println(F("Usage: throw <vel_mps> <in_s>")); return; }
  if (lc.startsWith("smooth ")) { if (!handleSmoothCmd(line)) Serial.println(F("Usage: smooth <target_pos_rev>")); return; }

  if (lc == "help" || lc == "h") { printTopHelp(); return; }

  // Pitch command family
  if (lc.startsWith("p "))      { pitchHandleLine(line.substring(2)); return; }     // short prefix
  if (lc.startsWith("pitch "))  { pitchHandleLine(line.substring(6)); return; }     // long prefix


  // Route lines to the YAW command handler when prefixed with "y " or "yaw "
  if (lc.startsWith("y "))   { yawHandleLine(line.substring(2));  return; }
  if (lc.startsWith("yaw ")) { yawHandleLine(line.substring(4));  return; }

  Serial.println(F("Unknown command. Type 'help' for top-level, or use 'y ' / 'yaw ' for YAW axis."));
}

// ================================== TOP-LEVEL HELP =======================================
void printTopHelp() {
  Serial.println(F(
    "Top-level commands:\n"
    "  throw <vel_mps> <in_s>   : Plan hand throw at <vel_mps>; schedule so decel starts in <in_s>\n"
    "  smooth <target_pos_rev>  : Smooth-move hand to absolute position (rev), start now\n"
    "\n"
    "Pitch commands (prefix 'p ' or 'pitch '): type 'p help'\n"
    "YAW   commands (prefix 'y ' or 'yaw '):  type 'y help'\n"
  ));
}

// ============================== PV SNAPSHOT (freshness) ================================
static bool getFreshPV(float& pos_rev, float& vel_rps, uint64_t& t_wall_us) {
  if (!canif.getAxisPV(HAND_NODE_ID, pos_rev, vel_rps, t_wall_us)) return false;
  const uint64_t now_us = canif.wallTimeUs();
  return (now_us - t_wall_us) <= PV_MAX_AGE_US;
}

// ================================== THROW HANDLER =======================================
bool handleThrowCmd(const String& line) {
  float vel=0.0f, in_s=0.0f;
  const char* p = line.c_str();
  if (strncmp(p, "throw", 5) == 0) p += 5;
  int n = sscanf(p, "%f %f", &vel, &in_s);
  if (n != 2) return false;

  // ---- Guard rail: velocity range ----
  if (!(vel > THROW_VEL_MIN_MPS && vel <= THROW_VEL_MAX_MPS)) {
    Serial.printf("THROW: REJECTED — velocity %.3f m/s out of range (%.2f..%.2f).\n",
                  vel, THROW_VEL_MIN_MPS, THROW_VEL_MAX_MPS);
    return true;
  }

  // ---- Guard rail: non-negative schedule ----
  if (in_s < 0.0f) {
    Serial.printf("THROW: REJECTED — negative schedule (in=%.3f s).\n", in_s);
    return true;
  }

  // ---- PV freshness ----
  float pos_rev=0, vel_rps=0; uint64_t t_wall_us=0;
  if (!getFreshPV(pos_rev, vel_rps, t_wall_us)) {
    Serial.println("THROW: REJECTED — PV unavailable or stale.");
    return true;
  }

  // ---- Plan (t_s = 0 at decel) using PV snapshot ----
  auto plan = planner.planThrowDecelZero(vel, pos_rev, vel_rps, (uint32_t)(t_wall_us & 0xFFFFFFFFu));
  if (plan.trajectory.empty()) {
    Serial.println("THROW: REJECTED — planner returned empty trajectory.");
    return true;
  }

  // ---- Guard rail: sufficient lead time to cover negative t_s frames ----
  float min_ts = 0.0f;
  for (const auto& f : plan.trajectory) if (f.t_s < min_ts) min_ts = f.t_s;
  const float required_lead_s = -min_ts;  // may be 0 if all frames ≥ 0
  if (in_s + SCHEDULE_MARGIN_S < required_lead_s) {
    const float suggested = required_lead_s + SCHEDULE_MARGIN_S;
    Serial.printf("THROW: REJECTED — in=%.3f s is too soon; need ≥ %.3f s (lead %.3f + margin %.3f).\n",
                  in_s, suggested, required_lead_s, SCHEDULE_MARGIN_S);
    return true;
  }

  // ---- Arm streamer (safe - all checks passed) ----
  g_traj_buffer = std::move(plan.trajectory);
  const float now_s        = canif.wallTimeUs() * 1e-6f;
  const float throw_wall_s = now_s + in_s;

  bool ok = streamer.arm(HAND_NODE_ID, g_traj_buffer.data(), g_traj_buffer.size(), throw_wall_s);
  Serial.printf("THROW: vel=%.3f, in=%.3f, frames=%u, ready=%.3fs, %s\n",
                vel, in_s, (unsigned)g_traj_buffer.size(), planner.lastTimeToReadyS(), ok?"ARMed":"ARM FAIL");
  return true;
}

// ================================ SMOOTH MOVER =========================================
// Parse and execute: smooth <target_pos_rev>
bool handleSmoothCmd(const String& line) {
  float target_rev=0.0f;
  const char* p = line.c_str();
  if (strncmp(p, "smooth", 6) == 0) p += 6;
  int n = sscanf(p, "%f", &target_rev);
  if (n != 1) return false;

  // ---- Guard rail: target in allowed range ----
  if (!(target_rev >= 0.0f && target_rev <= HAND_MAX_SMOOTH_MOVE_POS)) {
    Serial.printf("SMOOTH: REJECTED — target %.4f rev out of range [0, %.4f].\n",
                  target_rev, HAND_MAX_SMOOTH_MOVE_POS);
    return true;
  }

  // ---- PV freshness ----
  float pos_rev=0, vel_rps=0; uint64_t t_wall_us=0;
  if (!getFreshPV(pos_rev, vel_rps, t_wall_us)) {
    Serial.println("SMOOTH: REJECTED — PV unavailable or stale.");
    return true;
  }

  // ---- Plan smooth using PV snapshot ----
  auto plan = planner.planSmoothTo(target_rev, pos_rev, vel_rps, (uint32_t)(t_wall_us & 0xFFFFFFFFu));
  if (plan.trajectory.empty()) {
    Serial.println("SMOOTH: no movement needed (already at target).");
    return true;
  }

  // ---- Arm streamer to start now (frames have t_s starting at 0) ----
  g_traj_buffer = std::move(plan.trajectory);
  const float time_offset_s = canif.wallTimeUs() * 1e-6f; // start now (relative times)
  bool ok = streamer.arm(HAND_NODE_ID, g_traj_buffer.data(), g_traj_buffer.size(), time_offset_s);
  Serial.printf("SMOOTH: target=%.4f rev, frames=%u, dur=%.3fs, %s\n",
                target_rev, (unsigned)g_traj_buffer.size(), planner.lastTimeToReadyS(), ok?"ARMed":"ARM FAIL");
  return true;
}

// ================================== YAW HELP & PARSER ====================================
void yawPrintHelp(){
  Serial.println(F(
    "YAW axis commands (prefix with 'y ' or 'yaw '):\n"
    "  p <deg>          : YAW set absolute target (degrees)\n"
    "  P <rev>          : YAW set absolute target (revolutions)\n"
    "  m <deg>          : YAW move relative (degrees)\n"
    "  M <rev>          : YAW move relative (revolutions)\n"
    "  z                : YAW zero here (make current = 0 rev)\n"
    "  d                : YAW E-STOP (latched, coast)\n"
    "  c                : YAW clear E-STOP (auto-enable resumes)\n"
    "  g <kp> <ki> <kd> : YAW set PID gains\n"
    "  a <acc> [dec]    : YAW set accel/decels (PWM counts/sec)\n"
    "  f <ff_pwm>       : YAW set friction feedforward (PWM)\n"
    "  o <deg>          : YAW set tolerance band (degrees)\n"
    "  s                : YAW print one telemetry snapshot\n"
    "  t                : YAW toggle telemetry stream\n"
  ));
}

void yawHandleLine(const String& yawLine){
  if (!yawLine.length()) return;
  char cmd = yawLine.charAt(0);

  if (yawLine == "h" || yawLine == "help") { yawPrintHelp(); return; }

  if (cmd == 'p') { float deg = yawLine.substring(1).toFloat();
    yawAxis.setTargetDeg(deg); Serial.printf("YAW: target set: %.3f deg\n", deg); return; }
  if (cmd == 'P') { float rev = yawLine.substring(1).toFloat();
    yawAxis.setTargetRev(rev); Serial.printf("YAW: target set: %.4f rev\n", rev); return; }
  if (cmd == 'm') { float ddeg = yawLine.substring(1).toFloat();
    yawAxis.moveRelDeg(ddeg);  Serial.printf("YAW: move rel: %+-.3f deg\n", ddeg); return; }
  if (cmd == 'M') { float drev = yawLine.substring(1).toFloat();
    yawAxis.moveRelRev(drev);  Serial.printf("YAW: move rel: %+-.4f rev\n", drev); return; }

  if (yawLine == "z") { yawAxis.zeroHere(); Serial.println("YAW: zeroed here."); return; }
  if (yawLine == "d") { yawAxis.estop();    Serial.println("YAW: E-STOP LATCHED (type 'c' to clear)"); return; }
  if (yawLine == "c") { yawAxis.clearEstop(); Serial.println("YAW: E-STOP CLEARED (auto-enable active)"); return; }

  if (cmd == 'g') {
    float kp, ki, kd; int n = sscanf(yawLine.c_str()+1, "%f %f %f", &kp, &ki, &kd);
    if (n == 3) { yawAxis.setGains(kp, ki, kd); Serial.printf("YAW: gains -> Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd); }
    else        { Serial.println("YAW: usage: g <kp> <ki> <kd>"); }
    return;
  }

  if (cmd == 'a') {
    float acc, dec; int n = sscanf(yawLine.c_str()+1, "%f %f", &acc, &dec);
    if (n >= 1) {
      yawAxis.setAccel(acc, (n == 2 ? dec : acc));
      Serial.printf("YAW: accel limiter -> ACCEL=%.1f PPS  DECEL=%.1f PPS\n", acc, (n==2?dec:acc));
    } else {
      Serial.println("YAW: usage: a <accel> [decel]");
    }
    return;
  }

  if (cmd == 'f') { float ff; int n = sscanf(yawLine.c_str()+1, "%f", &ff);
    if (n == 1) { yawAxis.setFF(ff); Serial.printf("YAW: FF set: %.1f PWM\n", ff); }
    else        { Serial.println("YAW: usage: f <ff_pwm>"); }
    return;
  }

  if (cmd == 'o') { float deg; int n = sscanf(yawLine.c_str()+1, "%f", &deg);
    if (n == 1) { yawAxis.setToleranceRev(deg * (1.0f/360.0f)); Serial.printf("YAW: tolerance: %.3f deg\n", deg); }
    else        { Serial.println("YAW: usage: o <deg>"); }
    return;
  }

  if (yawLine == "s") {
    auto t = yawAxis.readTelemetry();
    Serial.printf(
      "YAW | EN=%d ESTOP=%d pos=%.3fdeg vel=%.3frps cmd=%.3fdeg err=%.3fdeg u=%.1f u_slew=%.1f pwm=%d A=%.0f D=%.0f\n",
      (int)t.enabled, (int)t.estop,
      t.pos_deg, t.vel_rps, t.cmd_deg, t.err_deg,
      t.u, t.u_slew, (int)t.pwm, t.accelPPS, t.decelPPS
    );
    return;
  }

  if (yawLine == "t") {
    yaw_stream_on = !yaw_stream_on;
    Serial.printf("YAW: telemetry %s\n", yaw_stream_on ? "ON" : "OFF");
    return;
  }

  Serial.println("YAW: unknown command. Type 'y help' or 'yaw help'.");
}

// ======================= PITCH PARSER =============================
void pitchHandleLine(const String& line){
  String s = line; s.trim();
  if (s.length() == 0) { Serial.println("PITCH: type 'p help'"); return; }

  // Simple tokens
  if (s == "help") {
    Serial.println(F(
      "PITCH commands (prefix with 'p ' or 'pitch '):\n"
      "  <deg>               : Move to absolute angle (degrees from horizontal)\n"
      "  status              : Print one status snapshot\n"
      "  traj <vel> <acc> <dec>  : Set trap limits (rev/s, rev/s^2, rev/s^2)\n"
      "  gains <kp> <kv> <ki>    : Set ODrive pos/vel gains\n"
      "  range               : Print allowed user range\n"
    ));
    return;
  }

  if (s == "status") { pitch.printStatusOnce(); return; }
  if (s == "range") {
    Serial.printf("PITCH range: %.2f .. %.2f deg  (%.4f .. %.4f rev)\n",
                  PitchAxis::DEG_MIN, PitchAxis::DEG_MAX,
                  PitchAxis::REV_MIN, PitchAxis::REV_MAX);
    return;
  }

  if (s.startsWith("traj ")) {
    float v,a,d; int n = sscanf(s.c_str()+5, "%f %f %f", &v, &a, &d);
    if (n != 3) { Serial.println("PITCH: usage: traj <vel_rps> <acc_rps2> <dec_rps2>"); return; }
    pitch.setTrajLimits(v,a,d);
    return;
  }

  if (s.startsWith("gains ")) {
    float kp,kv,ki; int n = sscanf(s.c_str()+6, "%f %f %f", &kp, &kv, &ki);
    if (n != 3) { Serial.println("PITCH: usage: gains <kp_pos> <kv_vel> <ki_vel>"); return; }
    pitch.setGains(kp,kv,ki);
    return;
  }

  // If it wasn't a keyword, treat the token as a DEGREE command
  // Accept either "<deg>" or "deg <value>"
  float deg = NAN;
  if (s.startsWith("deg ")) {
    int n = sscanf(s.c_str()+4, "%f", &deg);
    if (n != 1) { Serial.println("PITCH: usage: deg <angle_deg>"); return; }
  } else {
    deg = s.toFloat(); // tolerant: "45", "45.0"
  }

  if (!isfinite(deg)) {
    Serial.println("PITCH: could not parse angle. Try:  p 45   or   p deg 45");
    return;
  }

  pitch.setTargetDeg(deg);
}

// ================================== Wait for no errors before homing  ====================================
// Wait for 'mask' error bits to be clear (uses cached heartbeat). Returns true if cleared before timeout.
static bool waitForClear(uint32_t node_id, uint32_t mask, uint32_t timeout_ms, uint16_t poll_ms) {
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    canif.loop(); // keep heartbeats flowing
    if (canif.waitForAxisErrorClear(node_id, mask, 0 /*immediate*/)) return true; // uses cached snapshot
    delay(poll_ms);
  }
  return false;
}

// Homing wrapper that gives the auto-clearer time before/after attempts.
static bool homeHandWithAutoClearRetry(uint32_t node_id,
                                       uint8_t max_attempts,
                                       uint32_t pre_wait_ms,
                                       uint32_t post_fail_wait_ms) {
  // Pre-wait so the auto-clearer can clean the spurious bit before first attempt.
  (void)waitForClear(node_id, Err::BRAKE_RESISTOR_DISARMED, pre_wait_ms);

  for (uint8_t attempt = 1; attempt <= max_attempts; ++attempt) {
    Serial.printf("[Home] Attempt %u...\n", attempt);
    if (canif.homeHandStandard(node_id)) {
      Serial.println("[Home] Success.");
      return true;
    }
    Serial.println("[Home] Failed. Waiting for BRAKE_RESISTOR_DISARMED to clear before retry...");
    (void)waitForClear(node_id, Err::BRAKE_RESISTOR_DISARMED, post_fail_wait_ms);
  }
  return false;
}