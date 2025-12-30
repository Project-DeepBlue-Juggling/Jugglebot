/*
 * ball_butler_main.ino - Main sketch for Ball Butler robot
 * 
 * ARCHITECTURE:
 *   - StateMachine: Orchestrates robot behavior (BOOT -> IDLE -> THROWING -> RELOADING)
 *   - Proprioception: Central repository for axis state data
 *   - CanInterface: CAN communication with ODrives
 *   - YawAxis: Brushed DC motor control
 *   - PitchAxis: ODrive position control
 *   - HandPathPlanner + HandTrajectoryStreamer: Throw trajectory execution
 */

#include <vector>
#include "YawAxis.h"
#include "PitchAxis.h"
#include "HandPathPlanner.h"
#include "CanInterface.h"
#include "HandTrajectoryStreamer.h"
#include "TrajFrame.h"
#include "Trajectory.h"
#include "Proprioception.h"
#include "StateMachine.h"
#include <stdarg.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
static constexpr float    THROW_VEL_MIN_MPS  = 0.05f;
static constexpr float    THROW_VEL_MAX_MPS  = 6.0f;
static constexpr float    SCHEDULE_MARGIN_S  = 0.1f;
static constexpr uint64_t PV_MAX_AGE_US      = 20000;

static constexpr uint32_t HOST_THROW_CAN_ID  = 0x7D0;
static constexpr uint8_t  PITCH_NODE_ID      = 7;
static constexpr uint8_t  HAND_NODE_ID       = 8;

using Err = CanInterface::ODriveErrors;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
CanInterface           canif;
HandPathPlanner        planner;
HandTrajectoryStreamer streamer(canif);
PitchAxis              pitch(canif, PITCH_NODE_ID);

YawAxis yawAxis(10, 16, 15, 14, 60, 20000);

StateMachine stateMachine(canif, yawAxis, pitch, streamer, planner);

std::vector<TrajFrame> g_traj_buffer;

static bool     yaw_stream_on      = false;
static uint32_t yaw_last_stream_ms = 0;
static const uint16_t YAW_TELEM_MS = 500;

// ============================================================================
// YAW PROPRIOCEPTION CALLBACK (called from ISR)
// ============================================================================
void yawProprioceptionCallback(float pos_deg, float vel_rps, uint64_t ts_us) {
  PRO.setYawDeg(pos_deg, ts_us);
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
static bool getFreshPV(float& pos_rev, float& vel_rps, uint64_t& t_wall_us) {
  if (!canif.getAxisPV(HAND_NODE_ID, pos_rev, vel_rps, t_wall_us)) return false;
  return (canif.wallTimeUs() - t_wall_us) <= PV_MAX_AGE_US;
}

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void onHostThrow(const CanInterface::HostThrowCmd& c, void* user);
void yawPrintHelp();
void yawHandleLine(const String& yawLine);
void pitchHandleLine(const String& line);
void routeCommand(const String& rawLine);
void printTopHelp();
void printStatus();
bool handleThrowCmd(const String& line);
bool handleSmoothCmd(const String& line);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Wait for Serial, but with a shorter timeout and non-blocking approach
  // This prevents hanging if USB is connected but no terminal is open
  uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 2000)) {
    // Yield to USB stack
    delay(10);
  }
  
  // Additional check: if Serial says it's ready but buffer is full, 
  // we might still block. Give it a moment to stabilize.
  delay(100);
  
  // Only print startup banner if Serial is actually ready
  if (Serial && Serial.availableForWrite() > 64) {
    Serial.println(F("\n========================================"));
    Serial.println(F("       Ball Butler - Starting Up"));
    Serial.println(F("========================================\n"));
  }

  Proprioception::setDebugStream(&Serial, false);

  canif.begin(1000000);
  canif.setDebugStream(&Serial);
  canif.setDebugFlags(false, false);
  canif.setHandAxisNode(HAND_NODE_ID);
  canif.setPitchAxisNode(PITCH_NODE_ID);
  canif.setHostThrowCmdId(HOST_THROW_CAN_ID);
  canif.setHostThrowCallback(&onHostThrow, nullptr);
  canif.setAutoClearBrakeResistor(true, 500);
  canif.requireHomeOnlyFor(HAND_NODE_ID);

  yawAxis.begin();
  // Configure yaw axis with safe initial settings before StateMachine takes over
  // This prevents any erroneous movement during the boot phase
  // The offset places user 0° at encoder 5°, giving headroom for overshoot
  // Limits are set wide initially; StateMachine will tighten them after homing
  yawAxis.setZeroOffset(5.0f);  // encoder 5° = user 0°
  yawAxis.setSoftLimitsDeg(-5.0f, 190.0f);  // Wide limits during boot
  yawAxis.setHardLimitOvershoot(10.0f);  // Allow 10° overshoot before fault
  yawAxis.setProprioceptionCallback(yawProprioceptionCallback);

  pitch.begin();

  StateMachine::Config smConfig;
  smConfig.hand_node_id  = HAND_NODE_ID;
  smConfig.pitch_node_id = PITCH_NODE_ID;
  smConfig.homing_timeout_ms   = 10000;
  smConfig.reload_timeout_ms   = 10000;
  smConfig.post_throw_delay_ms = 1000;
  stateMachine.setConfig(smConfig);
  stateMachine.setDebugStream(&Serial);
  stateMachine.setDebugEnabled(true);
  stateMachine.begin();

  Serial.println(F("\nType 'help' for commands.\n"));
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  canif.loop();
  streamer.tick();
  Proprioception::flushDebug();
  stateMachine.update();

  static String serialBuf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuf.trim();
      if (serialBuf.length()) routeCommand(serialBuf);
      serialBuf = "";
    } else {
      serialBuf += c;
    }
  }

  if (yaw_stream_on && (millis() - yaw_last_stream_ms >= YAW_TELEM_MS)) {
    yaw_last_stream_ms = millis();
    auto t = yawAxis.readTelemetry();
    Serial.printf("YAW | pos=%.2f cmd=%.2f err=%.2f pwm=%d en=%d\n",
                  t.pos_deg, t.cmd_deg, t.err_deg, (int)t.pwm, t.enabled);
  }
}

// ============================================================================
// CAN THROW HANDLER
// ============================================================================
void onHostThrow(const CanInterface::HostThrowCmd& c, void*) {
  const float yaw_deg   = c.yaw_rad   * (180.0f / (float)M_PI);
  const float pitch_deg = c.pitch_rad * (180.0f / (float)M_PI);

  Serial.println(F("--- CAN Throw Command ---"));
  Serial.printf("  Yaw: %.2f  Pitch: %.2f  Speed: %.3f  In: %.3f\n",
                yaw_deg, pitch_deg, c.speed_mps, c.in_s);

  if (c.speed_mps == 0.0f) {
    yawAxis.setTargetDeg(yaw_deg);
    pitch.setTargetDeg(pitch_deg);
    return;
  }

  if (stateMachine.requestThrow(yaw_deg, pitch_deg, c.speed_mps, c.in_s)) {
    Serial.println(F("  -> Accepted"));
  } else {
    Serial.printf("  -> REJECTED (state: %s)\n", robotStateToString(stateMachine.getState()));
  }
}

// ============================================================================
// COMMAND ROUTING
// ============================================================================
void routeCommand(const String& rawLine) {
  String line = rawLine;
  line.trim();
  String lc = line;
  lc.toLowerCase();

  if (lc == "status") { printStatus(); return; }
  if (lc == "reset")  { stateMachine.reset(); return; }
  if (lc == "ball")  { Serial.printf("Ball in hand: %s\n", stateMachine.isBallInHand() ? "YES" : "NO"); return; }
  if (lc == "reload") { stateMachine.requestReload(); return;}
  if (lc.startsWith("throw ")) { handleThrowCmd(line); return; }
  if (lc.startsWith("smooth ")) { handleSmoothCmd(line); return; }
  if (lc == "help" || lc == "h") { printTopHelp(); return; }
  if (lc.startsWith("p "))     { pitchHandleLine(line.substring(2)); return; }
  if (lc.startsWith("pitch ")) { pitchHandleLine(line.substring(6)); return; }
  if (lc.startsWith("y "))     { yawHandleLine(line.substring(2)); return; }
  if (lc.startsWith("yaw "))   { yawHandleLine(line.substring(4)); return; }

  Serial.println(F("Unknown command. Type 'help'."));
}

void printTopHelp() {
  Serial.println(F(
    "\n=== Ball Butler Commands ===\n"
    "  status              : Show state and proprioception\n"
    "  reset               : Reset from ERROR state\n"
    "  throw <vel> <in_s>  : Throw at velocity\n"
    "  reload              : Trigger a reload\n"
    "  smooth <pos_rev>    : Smooth-move hand\n"
    "  p <deg>             : Pitch to angle\n"
    "  y help              : Show yaw commands\n"
  ));
}

void printStatus() {
  Serial.println(F("\n=== Status ==="));
  Serial.printf("State: %s", robotStateToString(stateMachine.getState()));
  if (stateMachine.isError()) Serial.printf(" - %s", stateMachine.getErrorMessage());
  Serial.println();
  Serial.printf("Ball: %s\n", stateMachine.isBallInHand() ? "YES" : "NO");
  
  ProprioceptionData d;
  PRO.snapshot(d);
  Serial.printf("Yaw: %.1f  Pitch: %.1f  Hand: %.3f rev\n",
                d.yaw_deg, d.pitch_deg, d.hand_pos_rev);
  Serial.printf("Hand homed: %s  Streamer: %s\n",
                canif.isAxisHomed(HAND_NODE_ID) ? "YES" : "NO",
                streamer.isActive() ? "ACTIVE" : "idle");
}

bool handleThrowCmd(const String& line) {
  float vel = 0, in_s = 0;
  if (sscanf(line.c_str() + 6, "%f %f", &vel, &in_s) != 2) {
    Serial.println(F("Usage: throw <vel> <in_s>"));
    return false;
  }
  if (vel <= THROW_VEL_MIN_MPS || vel > THROW_VEL_MAX_MPS) {
    Serial.printf("THROW: velocity out of range\n");
    return true;
  }
  float yaw_deg = PRO.getYawDeg();
  float pitch_deg = PRO.getPitchDeg();
  if (stateMachine.requestThrow(yaw_deg, pitch_deg, vel, in_s)) {
    Serial.printf("THROW: Queued %.3f m/s in %.3f s\n", vel, in_s);
  } else {
    Serial.printf("THROW: REJECTED (state: %s)\n", robotStateToString(stateMachine.getState()));
  }
  return true;
}

bool handleReloadCmd(const String& line) {
  if (stateMachine.requestReload()) { Serial.println("Reload requested!");
  } else {
    Serial.printf("Reload REJECTED (state: %s)\n", robotStateToString(stateMachine.getState()));
  }
  return true;
}

bool handleSmoothCmd(const String& line) {
  float target = 0;
  if (sscanf(line.c_str() + 7, "%f", &target) != 1) {
    Serial.println(F("Usage: smooth <pos_rev>"));
    return false;
  }
  if (target < 0 || target > HAND_MAX_SMOOTH_MOVE_POS) {
    Serial.printf("SMOOTH: out of range [0, %.2f]\n", HAND_MAX_SMOOTH_MOVE_POS);
    return true;
  }
  float pos = 0, vel = 0; uint64_t t = 0;
  if (!getFreshPV(pos, vel, t)) {
    Serial.println(F("SMOOTH: PV unavailable"));
    return true;
  }
  auto plan = planner.planSmoothTo(target, pos, vel, (uint32_t)(t & 0xFFFFFFFF));
  if (plan.trajectory.empty()) {
    Serial.println(F("SMOOTH: Already at target"));
    return true;
  }
  g_traj_buffer = std::move(plan.trajectory);
  bool ok = streamer.arm(HAND_NODE_ID, g_traj_buffer.data(), g_traj_buffer.size(), 
                         canif.wallTimeUs() * 1e-6f);
  Serial.printf("SMOOTH to %.2f: %s\n", target, ok ? "Armed" : "FAIL");
  return true;
}

// ============================================================================
// YAW COMMAND HANDLER
// ============================================================================
void yawPrintHelp() {
  Serial.println(F(
    "\n=== YAW Commands ===\n"
    "Motion:\n"
    "  p <deg>           : Go to position (degrees)\n"
    "  m <deg>           : Move relative (degrees)\n"
    "  d                 : E-stop (disable)\n"
    "  c                 : Clear e-stop\n"
    "\n"
    "Zero Position:\n"
    "  zero              : Set current position as 0 deg\n"
    "  offset <deg>      : Set zero offset directly\n"
    "\n"
    "Limits:\n"
    "  lim <min> <max>   : Set soft limits (degrees)\n"
    "\n"
    "Tuning:\n"
    "  gains <kp> <ki> <kd> : Set PID gains\n"
    "  accel <a> <d>     : Set accel/decel (PWM/s)\n"
    "  ff <pwm>          : Set friction feedforward\n"
    "\n"
    "Status:\n"
    "  s                 : Show status\n"
    "  t                 : Toggle telemetry stream\n"
    "  full              : Show full config\n"
  ));
}

void yawHandleLine(const String& yawLine) {
  if (!yawLine.length()) return;
  
  String line = yawLine;
  line.trim();
  String lc = line;
  lc.toLowerCase();
  
  // Help
  if (lc == "help" || lc == "h") { 
    yawPrintHelp(); 
    return; 
  }
  
  // Position command: p <deg>
  if (lc.startsWith("p ") || lc.startsWith("p")) {
    if (lc.length() > 1 && lc.charAt(1) != ' ') {
      // Single letter followed by number: p45
      float deg = line.substring(1).toFloat();
      if (yawAxis.setTargetDeg(deg)) {
        Serial.printf("YAW: Target -> %.2f deg\n", deg);
      } else {
        Serial.printf("YAW: Target %.2f deg out of limits\n", deg);
      }
    } else if (lc.startsWith("p ")) {
      float deg = line.substring(2).toFloat();
      if (yawAxis.setTargetDeg(deg)) {
        Serial.printf("YAW: Target -> %.2f deg\n", deg);
      } else {
        Serial.printf("YAW: Target %.2f deg out of limits\n", deg);
      }
    }
    return;
  }
  
  // Relative move: m <deg>
  if (lc.startsWith("m ") || (lc.length() > 1 && lc.charAt(0) == 'm' && (isdigit(lc.charAt(1)) || lc.charAt(1) == '-'))) {
    float deg = line.substring(1).toFloat();
    if (yawAxis.moveRelDeg(deg)) {
      Serial.printf("YAW: Move relative -> %.2f deg\n", deg);
    } else {
      Serial.printf("YAW: Relative move %.2f deg out of limits\n", deg);
    }
    return;
  }
  
  // E-stop
  if (lc == "d") { 
    yawAxis.estop(); 
    Serial.println(F("YAW: E-STOP engaged")); 
    return; 
  }
  
  // Clear e-stop
  if (lc == "c") { 
    yawAxis.clearEstop(); 
    Serial.println(F("YAW: E-stop cleared")); 
    return; 
  }
  
  // Set zero here
  if (lc == "zero") {
    yawAxis.setZeroHere();
    Serial.printf("YAW: Zero set at current position (offset=%.2f)\n", yawAxis.getZeroOffset());
    return;
  }
  
  // Set zero offset directly: offset <deg>
  if (lc.startsWith("offset ")) {
    float offset = line.substring(7).toFloat();
    yawAxis.setZeroOffset(offset);
    Serial.printf("YAW: Zero offset -> %.2f deg\n", offset);
    return;
  }
  
  // Set soft limits: lim <min> <max>
  if (lc.startsWith("lim ")) {
    float minDeg = 0, maxDeg = 0;
    if (sscanf(line.c_str() + 4, "%f %f", &minDeg, &maxDeg) == 2) {
      yawAxis.setSoftLimitsDeg(minDeg, maxDeg);
      float actualMin, actualMax;
      yawAxis.getSoftLimitsDeg(actualMin, actualMax);
      Serial.printf("YAW: Limits -> [%.1f, %.1f] deg\n", actualMin, actualMax);
    } else {
      Serial.println(F("YAW: Usage: lim <min> <max>"));
    }
    return;
  }
  
  // Set PID gains: gains <kp> <ki> <kd>
  if (lc.startsWith("gains ")) {
    float kp = 0, ki = 0, kd = 0;
    if (sscanf(line.c_str() + 6, "%f %f %f", &kp, &ki, &kd) == 3) {
      yawAxis.setGains(kp, ki, kd);
      Serial.printf("YAW: Gains -> Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
    } else {
      Serial.println(F("YAW: Usage: gains <kp> <ki> <kd>"));
    }
    return;
  }
  
  // Set acceleration: accel <accel> <decel>
  if (lc.startsWith("accel ")) {
    float accel = 0, decel = 0;
    int n = sscanf(line.c_str() + 6, "%f %f", &accel, &decel);
    if (n >= 1) {
      if (n == 1) decel = accel;  // If only one value, use for both
      yawAxis.setAccel(accel, decel);
      Serial.printf("YAW: Accel -> %.1f / Decel -> %.1f PWM/s\n", accel, decel);
    } else {
      Serial.println(F("YAW: Usage: accel <accel> [<decel>]"));
    }
    return;
  }
  
  // Set friction feedforward: ff <pwm>
  if (lc.startsWith("ff ")) {
    float ff = line.substring(3).toFloat();
    yawAxis.setFF(ff);
    Serial.printf("YAW: Friction FF -> %.1f PWM\n", ff);
    return;
  }
  
  // Status (brief)
  if (lc == "s") {
    auto t = yawAxis.readTelemetry();
    Serial.printf("YAW | pos=%.2f cmd=%.2f err=%.2f pwm=%d en=%d estop=%d\n",
                  t.pos_deg, t.cmd_deg, t.err_deg, (int)t.pwm, t.enabled, t.estop);
    return;
  }
  
  // Toggle telemetry stream
  if (lc == "t") { 
    yaw_stream_on = !yaw_stream_on; 
    Serial.printf("YAW: Telemetry stream %s\n", yaw_stream_on ? "ON" : "OFF");
    return; 
  }
  
  // Full configuration dump
  if (lc == "full") {
    auto t = yawAxis.readTelemetry();
    Serial.println(F("\n=== YAW Full Configuration ==="));
    Serial.printf("Position:     %.2f deg (cmd: %.2f, err: %.2f)\n", t.pos_deg, t.cmd_deg, t.err_deg);
    Serial.printf("Velocity:     %.3f rev/s\n", t.vel_rps);
    Serial.printf("PWM:          %d (u=%.1f, u_slew=%.1f)\n", t.pwm, t.u, t.u_slew);
    Serial.printf("Enabled:      %s\n", t.enabled ? "YES" : "NO");
    Serial.printf("E-Stop:       %s\n", t.estop ? "YES" : "NO");
    Serial.printf("Zero Offset:  %.2f deg\n", t.zero_offset_deg);
    Serial.printf("Limits:       [%.1f, %.1f] deg\n", t.lim_min_deg, t.lim_max_deg);
    Serial.printf("PID Gains:    Kp=%.2f Ki=%.2f Kd=%.2f\n", t.kp, t.ki, t.kd);
    Serial.printf("Accel/Decel:  %.1f / %.1f PWM/s\n", t.accelPPS, t.decelPPS);
    Serial.printf("Encoder:      raw=%u\n", t.raw);
    Serial.println();
    return;
  }
  
  Serial.println(F("YAW: Unknown command. Type 'y help'"));
}

void pitchHandleLine(const String& line) {
  String s = line; s.trim();
  if (s.length() == 0) { Serial.println(F("PITCH: type 'p help'")); return; }
  if (s == "help") {
    Serial.println(F("PITCH: <deg> | status | range"));
    return;
  }
  if (s == "status") { pitch.printStatusOnce(); return; }
  if (s == "range") {
    Serial.printf("PITCH: %.1f .. %.1f deg\n", PitchAxis::DEG_MIN, PitchAxis::DEG_MAX);
    return;
  }
  float deg = s.toFloat();
  if (isfinite(deg)) pitch.setTargetDeg(deg);
}
