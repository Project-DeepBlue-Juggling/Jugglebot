// sensored_hand_testing.ino
// Command the hand along a smooth trajectory to a target position a set number of times
// (back and forth), while sampling pogo-pin contact continuity at 10 kHz and logging
// drop counts per cycle to SD card.
//
// OPTIMIZATION: Trajectories between TARGET_POS_REV_1 and TARGET_POS_REV_2 are
// pre-computed and cached. Only the initial move from the starting position
// requires on-the-fly calculation.

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <IntervalTimer.h>
#include "CanInterface.h"
#include "HandPathPlanner.h"
#include "HandTrajectoryStreamer.h"
#include "CachedHandTrajectory.h"

// ============================================================================
// Configuration
// ============================================================================

// Motor control
constexpr uint32_t CAN_BITRATE            = 1000000;    // 1 Mbps
constexpr uint32_t HAND_NODE_ID           = 0;          // ODrive node ID
constexpr float    TARGET_POS_REV_1       = 8.5f;       // First target position (rev)
constexpr float    TARGET_POS_REV_2       = 1.0f;       // Second target position (rev)
constexpr float    START_TARGET_POS_REV   = TARGET_POS_REV_1;  // Initial target = target1
constexpr int      NUM_CYCLES             = 3;          // Number of back-and-forth cycles
constexpr uint16_t ERROR_LOG_INTERVAL_MS  = 500;        // Log errors at most this often
constexpr float    POSITION_TOLERANCE_REV = 0.01f;      // Tolerance to consider target reached

// Pogo-pin sampling
constexpr uint8_t  SENSOR_PIN             = 21;         // Digital input for hand sensor circuit
constexpr uint32_t SAMPLE_RATE_HZ       = 10000;      // 10 kHz sampling rate
constexpr uint32_t SAMPLE_PERIOD_US     = 1000000 / SAMPLE_RATE_HZ; // 100 µs

// SD card (Teensy 4.1 built-in SD slot uses SDIO)
constexpr int      SD_CHIP_SELECT       = BUILTIN_SDCARD;

// ============================================================================
// Global State
// ============================================================================

// Motor control objects
CanInterface can;
HandPathPlanner hand_planner;
HandTrajectoryStreamer streamer(can);
CachedHandTrajectory traj_cache(TARGET_POS_REV_1, TARGET_POS_REV_2);
std::vector<TrajFrame> g_traj_buffer;  // Only needed for initial move now
CanInterface::AxisHeartbeat heartbeat;

// Motion state
uint64_t last_error_log_time = 0;
float target_position = 0.0f;
static int cycle_count = 0;           // Counts half-cycles (up OR down movements)
static bool moving_to_first = true;
static bool smooth_running = false;
static int requested_cycles = 0;      // Number of cycles to run (0 = indefinite)
static bool run_indefinitely = false; // True when running until 'stop' command

// Pogo-pin sampling state (volatile for ISR access)
IntervalTimer sampleTimer;
volatile bool sampling_active = false;
volatile uint32_t drop_count = 0;       // Counts samples where circuit was broken
volatile uint32_t total_samples = 0;    // Total samples in current half-cycle

// Logging state
File logFile;
bool sd_initialized = false;
int session_number = 0;
int logged_cycle = 0;                   // Full cycles logged (1 cycle = up + down)

// Buffers for transferring data from ISR to main loop
// We store drop counts for each half-cycle, then combine into full cycles
volatile uint32_t half_cycle_drops[2] = {0, 0};   // [0] = first half, [1] = second half
volatile uint32_t half_cycle_samples[2] = {0, 0};
volatile int half_cycle_index = 0;
volatile bool half_cycle_complete = false;

// ============================================================================
// ISR: High-speed pogo-pin sampling (10 kHz)
// ============================================================================
void samplePogoISR() {
  if (!sampling_active) return;
  
  // Read the pogo pin - HIGH means circuit is broken (contact lost)
  if (digitalReadFast(SENSOR_PIN) == HIGH) {
    drop_count++;
  }
  total_samples++;
}

// ============================================================================
// SD Card Functions
// ============================================================================

bool initSD() {
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("ERROR: SD card initialization failed!");
    return false;
  }
  Serial.println("SD card initialized successfully.");
  sd_initialized = true;
  return true;
}

// Find the next available session number
int findNextSessionNumber() {
  int num = 1;
  char filename[32];
  while (true) {
    snprintf(filename, sizeof(filename), "pogo_%04d.csv", num);
    if (!SD.exists(filename)) {
      return num;
    }
    num++;
    if (num > 9999) {
      Serial.println("ERROR: Too many session files!");
      return -1;
    }
  }
}

bool startNewSession() {
  if (!sd_initialized) {
    if (!initSD()) return false;
  }
  
  // Close any existing file
  if (logFile) {
    logFile.close();
  }
  
  // Find next available session number
  session_number = findNextSessionNumber();
  if (session_number < 0) return false;
  
  // Create new file
  char filename[32];
  snprintf(filename, sizeof(filename), "pogo_%04d.csv", session_number);
  
  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    Serial.printf("ERROR: Could not create file: %s\n", filename);
    return false;
  }
  
  // Write header with metadata
  logFile.println("# Pogo-Pin Contact Reliability Test Log");
  logFile.println("# Date: " __DATE__ " Time: " __TIME__);
  logFile.printf("# Session: %d\n", session_number);
  logFile.printf("# Sampling Rate: %lu Hz\n", SAMPLE_RATE_HZ);
  logFile.printf("# Target Position 1: %.3f rev\n", (double)TARGET_POS_REV_1);
  logFile.printf("# Target Position 2: %.3f rev\n", (double)TARGET_POS_REV_2);
  if (run_indefinitely) {
    logFile.println("# Planned Cycles: INDEFINITE");
  } else {
    logFile.printf("# Planned Cycles: %d\n", requested_cycles);
  }
  logFile.printf("# Pogo Pin: %d (INPUT_PULLUP, HIGH = break)\n", SENSOR_PIN);
  logFile.printf("# Max Accel: %.2f rev/s²\n", (double)getMaxSmoothMoveHandAccel());
  logFile.println("#");
  logFile.println("cycle,drop_count,total_samples,drop_rate_pct");
  logFile.flush();
  
  Serial.printf("Started new session: %s\n", filename);
  logged_cycle = 0;
  return true;
}

void logCycleData(int cycle, uint32_t drops, uint32_t samples) {
  if (!logFile) return;
  
  float drop_rate = (samples > 0) ? (100.0f * drops / samples) : 0.0f;
  logFile.printf("%d,%lu,%lu,%.4f\n", cycle, drops, samples, (double)drop_rate);
  logFile.flush();  // Ensure data is written in case of power loss
  
  Serial.printf("Logged cycle %d: %lu drops / %lu samples (%.4f%%)\n", 
                cycle, drops, samples, (double)drop_rate);
}

void closeSession() {
  if (logFile) {
    logFile.close();
    Serial.printf("Session %d closed.\n", session_number);
  }
}

// ============================================================================
// Sampling Control
// ============================================================================

void startSampling() {
  noInterrupts();
  drop_count = 0;
  total_samples = 0;
  sampling_active = true;
  interrupts();
}

void stopSampling() {
  noInterrupts();
  sampling_active = false;
  // Store results for current half-cycle
  half_cycle_drops[half_cycle_index] = drop_count;
  half_cycle_samples[half_cycle_index] = total_samples;
  half_cycle_index++;
  
  // Check if we've completed a full cycle (up + down)
  if (half_cycle_index >= 2) {
    half_cycle_complete = true;
    half_cycle_index = 0;
  }
  interrupts();
}

void resetHalfCycleTracking() {
  noInterrupts();
  half_cycle_index = 0;
  half_cycle_complete = false;
  half_cycle_drops[0] = half_cycle_drops[1] = 0;
  half_cycle_samples[0] = half_cycle_samples[1] = 0;
  interrupts();
}

// ============================================================================
// Trajectory Cache Helper
// ============================================================================

/**
 * Rebuild trajectory cache if acceleration has changed.
 * Should be called before starting a new session.
 */
void ensureTrajectoryCacheValid() {
  if (!traj_cache.isBuilt()) {
    Serial.println("Building trajectory cache...");
    traj_cache.build();
    Serial.printf("  Cache built: %zu + %zu frames (%.1f KB)\n",
                  traj_cache.getTrajectory1to2Count(),
                  traj_cache.getTrajectory2to1Count(),
                  traj_cache.getMemoryUsage() / 1024.0);
    Serial.printf("  Durations: %.3fs (1→2), %.3fs (2→1)\n",
                  (double)traj_cache.getDuration1to2(),
                  (double)traj_cache.getDuration2to1());
  } else if (traj_cache.rebuildIfAccelChanged()) {
    Serial.println("Acceleration changed - trajectory cache rebuilt.");
    Serial.printf("  New durations: %.3fs (1→2), %.3fs (2→1)\n",
                  (double)traj_cache.getDuration1to2(),
                  (double)traj_cache.getDuration2to1());
  }
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) ; // Wait for Serial (with timeout)
  
  Serial.println("\n========================================");
  Serial.println("Pogo-Pin Contact Reliability Test Rig");
  Serial.println("(with cached trajectories)");
  Serial.println("========================================\n");

  // Initialize pogo pin
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  Serial.printf("Pogo pin %d configured as INPUT_PULLUP\n", SENSOR_PIN);
  
  // Initialize SD card
  if (initSD()) {
    Serial.println("SD card ready. Type 'start' to begin a test session.");
  } else {
    Serial.println("WARNING: SD card not available. Data will not be logged.");
  }

  // Initialize CAN interface
  Serial.println("Starting CanInterface...");
  can.begin(CAN_BITRATE);
  can.setDebugStream(&Serial);
  can.setHandAxisNode(HAND_NODE_ID);

  // Configure motor control mode
  can.setControllerMode(HAND_NODE_ID, 3, 1); // Position / Passthrough

  // Home the hand
  Serial.println("Homing hand...");
  can.homeHandStandard(HAND_NODE_ID);

  // Wait for fresh position feedback after homing resets the position
  Serial.println("Waiting for fresh position feedback...");
  float pos = 0, vel = 0; uint64_t t = 0;
  uint32_t timeout_start = millis();
  while (!can.getAxisPV(HAND_NODE_ID, pos, vel, t)) {
    can.loop();
    if (millis() - timeout_start > 2000) {
      Serial.println("WARNING: Timed out waiting for fresh position feedback");
      break;
    }
    delay(10);
  }
  Serial.printf("Homing complete. Current position: %.3f rev\n", (double)pos);

  // Pre-build the trajectory cache
  Serial.println("\nPre-computing trajectory cache...");
  traj_cache.build();
  Serial.printf("  Trajectories cached: %zu + %zu frames\n",
                traj_cache.getTrajectory1to2Count(),
                traj_cache.getTrajectory2to1Count());
  Serial.printf("  Memory usage: %.1f KB\n", traj_cache.getMemoryUsage() / 1024.0);
  Serial.printf("  Move durations: %.3fs (1→2), %.3fs (2→1)\n",
                (double)traj_cache.getDuration1to2(),
                (double)traj_cache.getDuration2to1());

  // Start the sampling timer (runs continuously, but sampling_active controls actual sampling)
  if (!sampleTimer.begin(samplePogoISR, SAMPLE_PERIOD_US)) {
    Serial.println("ERROR: Failed to start sample timer!");
  } else {
    Serial.printf("Sample timer started at %lu Hz (%lu µs period)\n", 
                  SAMPLE_RATE_HZ, SAMPLE_PERIOD_US);
  }

  Serial.println("\nSetup complete.");
  Serial.println("Commands: start [cycles] | stop | reset | home | status | accel [value]");
  Serial.println("  start       - Run for default cycles (NUM_CYCLES)");
  Serial.println("  start 0     - Run indefinitely until 'stop'");
  Serial.println("  start N     - Run for N cycles");
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  // Check for motor errors
  if (can.getAxisHeartbeat(HAND_NODE_ID, heartbeat)) {
    if (heartbeat.axis_error != 0) {
      uint64_t now = millis();
      if (now - last_error_log_time >= ERROR_LOG_INTERVAL_MS) {
        last_error_log_time = now;
        Serial.printf("ERROR: Hand axis error code: 0x%08lX\n", (unsigned long)heartbeat.axis_error);
        smooth_running = false;
        stopSampling();
      }
    }
  }

  handleSerial();
  can.loop();

  // Check if a full cycle was completed and needs logging
  if (half_cycle_complete) {
    noInterrupts();
    uint32_t total_drops = half_cycle_drops[0] + half_cycle_drops[1];
    uint32_t total_samp = half_cycle_samples[0] + half_cycle_samples[1];
    half_cycle_complete = false;
    interrupts();
    
    logged_cycle++;
    logCycleData(logged_cycle, total_drops, total_samp);
  }

  // Motion control state machine
  if (smooth_running && (run_indefinitely || cycle_count < requested_cycles * 2)) {
    // Get latest feedback
    float pos = 0, vel = 0; uint64_t t = 0;
    if (!can.getAxisPV(HAND_NODE_ID, pos, vel, t)) {
      Serial.println(F("SMOOTH: PV unavailable"));
      return;
    }

    // Determine which cached trajectory to use
    const TrajFrame* frames = nullptr;
    size_t frame_count = 0;

    if (moving_to_first) {
      // Moving to TARGET_POS_REV_1 (from TARGET_POS_REV_2)
      frames = traj_cache.getTrajectory2to1();
      frame_count = traj_cache.getTrajectory2to1Count();
    } else {
      // Moving to TARGET_POS_REV_2 (from TARGET_POS_REV_1)
      frames = traj_cache.getTrajectory1to2();
      frame_count = traj_cache.getTrajectory1to2Count();
    }

    if (frames == nullptr || frame_count == 0) {
      Serial.println(F("SMOOTH: Cached trajectory unavailable"));
      return;
    }

    // Arm trajectory streamer with cached trajectory
    bool ok = streamer.arm(HAND_NODE_ID, frames, frame_count,
                           can.wallTimeUs() / 1e6f);

    // Start sampling for this movement
    startSampling();

    // Wait until we've reached the target
    while (abs(pos - target_position) > POSITION_TOLERANCE_REV && smooth_running) {
      can.loop();
      streamer.tick();
      handleSerial();
      can.getAxisPV(HAND_NODE_ID, pos, vel, t);
      
      // Brief yield to prevent watchdog issues
      yield();
    }

    // Stop sampling for this half-cycle
    stopSampling();

    // Update for next move
    moving_to_first = !moving_to_first;
    target_position = moving_to_first ? TARGET_POS_REV_1 : TARGET_POS_REV_2;
    cycle_count++;
  }

  // Session complete (only when not running indefinitely)
  if (smooth_running && !run_indefinitely && cycle_count >= requested_cycles * 2) {
    // Log the final cycle if pending
    if (half_cycle_complete) {
      noInterrupts();
      uint32_t total_drops = half_cycle_drops[0] + half_cycle_drops[1];
      uint32_t total_samp = half_cycle_samples[0] + half_cycle_samples[1];
      half_cycle_complete = false;
      interrupts();
      
      logged_cycle++;
      logCycleData(logged_cycle, total_drops, total_samp);
    }
    
    can.setRequestedState(HAND_NODE_ID, 1); // Idle
    Serial.println("\n========================================");
    Serial.println("Completed all cycles. Hand set to idle.");
    Serial.printf("Total full cycles: %d\n", requested_cycles);
    Serial.println("========================================\n");
    closeSession();
    smooth_running = false;
  }
}

// ============================================================================
// Serial Command Handler
// ============================================================================

void handleSerial() {
  static String cmd_buffer;
  
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmd_buffer.length() > 0) {
        String cmd = cmd_buffer;
        cmd.trim();
        cmd.toLowerCase();
        cmd_buffer = "";

        if (cmd == "start" || cmd.startsWith("start ")) {
          // Parse cycle count from command (default to NUM_CYCLES constant)
          int cycles_to_run = NUM_CYCLES;
          if (cmd.startsWith("start ")) {
            cycles_to_run = cmd.substring(6).toInt();
            if (cycles_to_run < 0) cycles_to_run = 0;
          }
          
          // Set up cycle control
          requested_cycles = cycles_to_run;
          run_indefinitely = (cycles_to_run == 0);
          
          // Only start if the hand has been homed
          if (!can.isAxisHomed(HAND_NODE_ID)) {
            Serial.println("ERROR: Hand axis is not homed. Cannot start test session.");
            return;
          }

          // Ensure trajectory cache is valid (rebuild if accel changed)
          ensureTrajectoryCacheValid();

          // Start a new test session
          if (!startNewSession()) {
            Serial.println("ERROR: Could not start new session (SD card issue?)");
            Serial.println("Continuing without logging...");
          }
          
          cycle_count = 0;
          moving_to_first = true;
          resetHalfCycleTracking();
          
          can.setRequestedState(HAND_NODE_ID, 8); // Closed-loop control
          
          Serial.printf("\nSTART: Beginning test session %d\n", session_number);
          if (run_indefinitely) {
            Serial.println("  Cycles planned: INDEFINITE (use 'stop' to end)");
          } else {
            Serial.printf("  Cycles planned: %d (back-and-forth)\n", requested_cycles);
          }
          Serial.printf("  Sampling at: %lu Hz\n", SAMPLE_RATE_HZ);
          Serial.printf("  Using cached trajectories (%zu + %zu frames)\n",
                        traj_cache.getTrajectory1to2Count(),
                        traj_cache.getTrajectory2to1Count());
          Serial.println("");
          
          // Move to starting position before cycling (this is the only on-the-fly calculation)
          Serial.println("Moving to starting position (on-the-fly trajectory)...");
          float pos = 0, vel = 0; uint64_t t = 0;
          can.getAxisPV(HAND_NODE_ID, pos, vel, t);
          
          // Set smooth_running before the initial move so 'stop' command works
          smooth_running = true;
          
          // Use the cache's planInitialMove for on-the-fly computation to target1
          auto plan = traj_cache.planInitialMove(pos, vel, (uint32_t)(t & 0xFFFFFFFF), hand_planner);
          if (!plan.trajectory.empty()) {
            g_traj_buffer = std::move(plan.trajectory);
            streamer.arm(HAND_NODE_ID, g_traj_buffer.data(), g_traj_buffer.size(),
                         can.wallTimeUs() / 1e6f);
            
            // Wait until we reach the starting position
            while (abs(pos - START_TARGET_POS_REV) > POSITION_TOLERANCE_REV && smooth_running) {
              can.loop();
              streamer.tick();
              handleSerial();  // Allow 'stop' command during initial move
              can.getAxisPV(HAND_NODE_ID, pos, vel, t);
              yield();
            }
          }
          
          if (smooth_running) {
            Serial.printf("Starting position reached (%.3f rev). Beginning cached cycles...\n", (double)pos);
            // Set up for first cycle: move to TARGET_POS_REV_2
            moving_to_first = false;
            target_position = TARGET_POS_REV_2;
          }
          
        } else if (cmd == "reset") {
          // Clear ODrive errors
          const bool ok = can.clearErrors(HAND_NODE_ID);
          Serial.printf("RESET: clearErrors sent to node %lu -> %s\n",
                        (unsigned long)HAND_NODE_ID, ok ? "OK" : "FAIL");
                        
        } else if (cmd == "stop") {
          // Stop the current session
          smooth_running = false;
          run_indefinitely = false;
          stopSampling();
          can.setRequestedState(HAND_NODE_ID, 1); // Idle
          closeSession();
          Serial.printf("STOP: Session stopped after %d full cycles. Hand set to idle.\n", logged_cycle);
          
        } else if (cmd == "home") {
          // Home the hand
          Serial.println("Homing hand...");
          can.homeHandStandard(HAND_NODE_ID);
          
          // Wait for fresh position feedback after homing resets the position
          Serial.println("Waiting for fresh position feedback...");
          float pos = 0, vel = 0; uint64_t t = 0;
          uint32_t timeout_start = millis();
          while (!can.getAxisPV(HAND_NODE_ID, pos, vel, t)) {
            can.loop();
            if (millis() - timeout_start > 2000) {
              Serial.println("WARNING: Timed out waiting for fresh position feedback");
              break;
            }
            delay(10);
          }
          Serial.printf("Homing complete. Current position: %.3f rev\n", (double)pos);
          
        } else if (cmd.startsWith("accel ")) {
          // Set max smooth move acceleration
          float new_accel = cmd.substring(6).toFloat();
          if (new_accel > 0.0f) {
            if (setMaxSmoothMoveHandAccel(new_accel)) {
              Serial.printf("ACCEL: Max smooth move acceleration set to %.2f rev/s²\n", (double)new_accel);
              // Note: Cache will be rebuilt automatically on next 'start'
              Serial.println("  (Trajectory cache will rebuild on next start)");
            } else {
              Serial.println("ERROR: Failed to set acceleration (invalid value)");
            }
          } else {
            Serial.println("ERROR: Acceleration must be a positive number");
            Serial.println("Usage: accel <value>  (e.g., accel 200)");
          }
          
        } else if (cmd == "accel") {
          // Print current max acceleration
          Serial.printf("ACCEL: Current max smooth move acceleration = %.2f rev/s²\n", 
                        (double)getMaxSmoothMoveHandAccel());
          Serial.printf("  Cached acceleration = %.2f rev/s²\n",
                        (double)traj_cache.getCachedAccel());
          if (fabsf(getMaxSmoothMoveHandAccel() - traj_cache.getCachedAccel()) > 1e-6f) {
            Serial.println("  (Cache will rebuild on next start)");
          }
          
        } else if (cmd == "status") {
          // Print current status
          Serial.println("\n--- Status ---");
          Serial.printf("Session: %d\n", session_number);
          Serial.printf("Running: %s\n", smooth_running ? "YES" : "NO");
          if (run_indefinitely) {
            Serial.printf("Half-cycles completed: %d (indefinite mode)\n", cycle_count);
          } else {
            Serial.printf("Half-cycles completed: %d / %d\n", cycle_count, requested_cycles * 2);
          }
          Serial.printf("Full cycles logged: %d\n", logged_cycle);
          Serial.printf("SD card: %s\n", sd_initialized ? "OK" : "NOT AVAILABLE");
          
          // Cache status
          Serial.printf("Trajectory cache: %s\n", traj_cache.isBuilt() ? "BUILT" : "NOT BUILT");
          Serial.printf("  Frames: %zu (1→2) + %zu (2→1)\n",
                        traj_cache.getTrajectory1to2Count(),
                        traj_cache.getTrajectory2to1Count());
          Serial.printf("  Memory: %.1f KB\n", traj_cache.getMemoryUsage() / 1024.0);
          Serial.printf("  Cached accel: %.2f rev/s²\n", (double)traj_cache.getCachedAccel());
          
          // Read current pogo state
          int pogo_state = digitalRead(SENSOR_PIN);
          Serial.printf("Pogo pin state: %s (pin %d = %d)\n", 
                        pogo_state == LOW ? "CONTACT OK" : "BREAK DETECTED",
                        SENSOR_PIN, pogo_state);
          
          // Current position
          float pos = 0, vel = 0; uint64_t t = 0;
          if (can.getAxisPV(HAND_NODE_ID, pos, vel, t)) {
            Serial.printf("Position: %.3f rev, Velocity: %.3f rev/s\n", (double)pos, (double)vel);
          }
          Serial.println("--------------\n");
          
        } else if (cmd == "cache") {
          // Force rebuild of trajectory cache
          Serial.println("Rebuilding trajectory cache...");
          traj_cache.forceRebuild();
          Serial.printf("  Frames: %zu (1→2) + %zu (2→1)\n",
                        traj_cache.getTrajectory1to2Count(),
                        traj_cache.getTrajectory2to1Count());
          Serial.printf("  Durations: %.3fs (1→2), %.3fs (2→1)\n",
                        (double)traj_cache.getDuration1to2(),
                        (double)traj_cache.getDuration2to1());
          Serial.printf("  Memory: %.1f KB\n", traj_cache.getMemoryUsage() / 1024.0);
          
        } else if (cmd.length() > 0) {
          Serial.printf("Unknown command: '%s'\n", cmd.c_str());
          Serial.println("Commands: start [cycles] | stop | reset | home | status | accel [value] | cache");
        }
      }
    } else {
      cmd_buffer += c;
      if (cmd_buffer.length() > 64) cmd_buffer.remove(0);
    }
  }
}
