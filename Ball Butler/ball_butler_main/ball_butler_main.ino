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

#include "YawAxis.h"
#include "PitchAxis.h"
#include "HandPathPlanner.h"
#include "BallButlerConfig.h"
#include "CanInterface.h"
#include "HandTrajectoryStreamer.h"
#include "Trajectory.h"
#include "Proprioception.h"
#include "StateMachine.h"
#include <stdarg.h>

using Err = CanInterface::ODriveErrors;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
CanInterface           canif;
HandPathPlanner        planner;
HandTrajectoryStreamer streamer(canif);
PitchAxis              pitch(canif, NodeId::PITCH);

YawAxis yawAxis(Pins::YAW_PWM_A, Pins::YAW_PWM_B, Pins::YAW_ENC_A, Pins::YAW_ENC_B,
                YawDefaults::ENC_CPR, YawDefaults::PWM_FREQ_HZ);

StateMachine stateMachine(canif, yawAxis, pitch, streamer, planner);

static bool     yaw_stream_on      = false;
static uint32_t yaw_last_stream_ms = 0;

// ============================================================================
// YAW PROPRIOCEPTION CALLBACK (called from ISR)
// ============================================================================
void yawProprioceptionCallback(float pos_deg, float vel_rps, uint64_t ts_us) {
  PRO.setYawDeg(pos_deg, ts_us);
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
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
  Serial.begin(OpCfg::SERIAL_BAUD);
  
  // Wait for Serial, but with a shorter timeout and non-blocking approach
  // This prevents hanging if USB is connected but no terminal is open
  uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < OpCfg::SERIAL_WAIT_MS)) {
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

  canif.begin(CanCfg::BAUD_RATE);
  canif.setDebugStream(&Serial);
  canif.setDebugFlags(false, false); // Enable CAN message debug, disable axis state debug to reduce spam
  canif.setHandAxisNode(NodeId::HAND);
  canif.setPitchAxisNode(NodeId::PITCH);
  canif.setAutoClearBrakeResistor(true, CanCfg::AUTO_CLEAR_BRAKE_MS);
  canif.requireHomeOnlyFor(NodeId::HAND);

  // Add a short delay to give ODrives time to boot and respond to pings
  uint32_t now = millis();
  while (millis() - now < OpCfg::ODRIVE_BOOT_MS) {
    canif.loop();
    delay(20);
  }

  yawAxis.begin();
  // Configure yaw axis with safe initial settings before StateMachine takes over
  // This prevents any erroneous movement during the boot phase
  // The offset places user 0° at encoder 5°, giving headroom for overshoot
  // Limits are set wide initially; StateMachine will tighten them after homing
  yawAxis.setZeroOffset(YawDefaults::BOOT_ZERO_OFFSET_DEG);
  yawAxis.setSoftLimitsDeg(YawDefaults::BOOT_SOFT_LIM_MIN_DEG, YawDefaults::BOOT_SOFT_LIM_MAX_DEG);
  yawAxis.setHardLimitOvershoot(YawDefaults::BOOT_HARD_LIMIT_OVERSHOOT);
  yawAxis.setProprioceptionCallback(yawProprioceptionCallback);

  pitch.begin();

  // StateMachine::Config defaults come from BallButlerConfig.h (SMDefaults::)
  // Only override here if this specific robot needs different values.
  stateMachine.setConfig(StateMachine::Config{});
  stateMachine.setDebugStream(&Serial);
  stateMachine.setDebugEnabled(true);
  stateMachine.begin();

  // Connect state machine to CAN interface for heartbeat publishing
  canif.setStateMachine(&stateMachine);

  Serial.println(F("\nType 'help' for commands.\n"));
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  canif.loop();
  pitch.loop();
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

  if (yaw_stream_on && (millis() - yaw_last_stream_ms >= OpCfg::YAW_TELEM_MS)) {
    yaw_last_stream_ms = millis();
    auto t = yawAxis.readTelemetry();
    Serial.printf("YAW | pos=%.2f cmd=%.2f err=%.2f pwm=%d en=%d\n",
                  t.pos_deg, t.cmd_deg, t.err_deg, (int)t.pwm, t.enabled);
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
  if (lc == "ball")  { Serial.printf("Ball in hand: %s\n",  canif.isBallInHand() ? "YES" : "NO"); return; }
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
  Serial.printf("Ball: %s\n", canif.isBallInHand() ? "YES" : "NO");
  
  ProprioceptionData d;
  PRO.snapshot(d);
  Serial.printf("Yaw: %.1f  Pitch: %.1f  Hand: %.3f rev\n",
                d.yaw_deg, d.pitch_deg, d.hand_pos_rev);
  Serial.printf("Hand homed: %s  Streamer: %s\n",
                canif.isAxisHomed(NodeId::HAND) ? "YES" : "NO",
                streamer.isActive() ? "ACTIVE" : "idle");
}

bool handleThrowCmd(const String& line) {
  float vel = 0, in_s = 0;
  if (sscanf(line.c_str() + 6, "%f %f", &vel, &in_s) != 2) {
    Serial.println(F("Usage: throw <vel> <in_s>"));
    return false;
  }
  if (vel <= OpCfg::THROW_VEL_MIN_MPS || vel > OpCfg::THROW_VEL_MAX_MPS) {
    Serial.printf("THROW: velocity out of range\n");
    return true;
  }
  // Convert relative time to absolute wall-clock time
  const uint64_t throw_wall_us = canif.wallTimeUs() + (uint64_t)(in_s * 1e6f);
  float yaw_deg = PRO.getYawDeg();
  float pitch_deg = PRO.getPitchDeg();
  if (stateMachine.requestThrow(yaw_deg, pitch_deg, vel, throw_wall_us)) {
    Serial.printf("THROW: Queued %.3f m/s in %.3f s\n", vel, in_s);
  } else {
    Serial.printf("THROW: REJECTED (state: %s)\n", robotStateToString(stateMachine.getState()));
  }
  return true;
}

bool handleSmoothCmd(const String& line) {
  float target = 0;
  if (sscanf(line.c_str() + 7, "%f", &target) != 1) {
    Serial.println(F("Usage: smooth <pos_rev>"));
    return false;
  }
  if (target < 0 || target > TrajCfg::HAND_MAX_SMOOTH_POS) {
    Serial.printf("SMOOTH: out of range [0, %.2f]\n", TrajCfg::HAND_MAX_SMOOTH_POS);
    return true;
  }
  bool ok = stateMachine.requestSmoothMove(target);
  Serial.printf("SMOOTH to %.2f: %s\n", target, ok ? "OK" : "FAIL");
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
      // Single letter followed by number: p 45
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
  // Serial.printf("PITCH: Target -> %.2f deg\n", deg);
  if (isfinite(deg)) pitch.setTargetDeg(deg);
}
