/*****************************************************************************************
 *  Teensy 4.0 — Jugglebot “platform” microcontroller
 *  ------------------------------------------------------------------
 *  Functions already present:
 *    • CAN traffic monitor (ID 0x7DF)
 *    • Optional debugging of specific CAN IDs
 *    • Timed analysis of RxSdo messages
 *    • SCL3300 inclinometer → CAN (ID 0x7DE), served from loop() since FW 11
 *      (loop_service.h), with an explicit invalid reply when it cannot answer
 *    • Robot state exchange (ID 0x6E0) — also reports this firmware's
 *      FW_VERSION in bytes 5-6; see the FIRMWARE IDENTITY block below and
 *      ros_ws/docs/platform_fw_version.md
 *    • TIME‑SYNC layer (ID 0x7DD)
 *      – Maintains wall‑time offset (Jetson wall‑time − micros64())
 *      – Prints jitter stats once per second
 *    • Hand cooling fan (see hand_fan.h): reads the Hand ODrive's broadcast
 *      Get_Temperature frame and drives the fan MOSFET's gate
 *
 *  INVARIANT I-FW-15 (since FW 7, skill-stack R1).  The can-bridge is the SOLE
 *  axis-6 writer: this board neither transmits to CAN node 6 nor subscribes to
 *  its encoder (FW 8 listens to node 6's temperature broadcast for the Hand fan,
 *  which is receive-only and carries no motion).  This board's TX set is the inclinometer frame (0x7DE), the
 *  RobotState reply (0x6E0), the firmware-update reply (0x6F1), the traffic
 *  report (0x7DF), since FW 9 the Hand fan status (0x6E2) and, since FW 11, the IMU reply,
 *  status and data frames (0x6E4-0x6E6, ros_ws/docs/platform_imu_protocol.md) — nothing else.  The stroke engine (Trajectory.h, the 0x6D0
 *  TRAJ_CMD decode, the 0x0C9 hand-encoder cache) was deleted with it.
 *
 *  NOTE on CAN IDs:
 *    ODrives occupy 0x000–0x0DF (7 × 32 IDs). Broadcast ID 0x7E0..0x7FF must
 *    also stay free.  All custom IDs here (0x6E0–0x7DF) are safe.
 *****************************************************************************************/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <SCL3300.h>
#include "hardware_config.h"  // Auto-generated from config/hardware_config.yaml
#include "protocol_config.h"  // Auto-generated from config/protocol_config.yaml
#include "hand_fan.h"
#include "loop_service.h"  // FW 11: tilt service, inclinometer probe, CAN dispatch bound

/*  IMU KILL SWITCH (FW 11, IDEA-20 P2b).  1 = the ICM-42688 capture (imu_service.h); 0 = every
 *  IMU code path compiled out: no Wire, no ring, no 0x6E3 handling, no 0x6E4-0x6E6 frames, so a
 *  known-good no-IMU image builds from this same tree (pio run -e teensy40 with
 *  build_flags -DPLATFORM_IMU_ENABLED=0).  At run time the IMU_CMD DISABLE does the same. */
#ifndef PLATFORM_IMU_ENABLED
#define PLATFORM_IMU_ENABLED 1
#endif
#if PLATFORM_IMU_ENABLED
#include <Wire.h>
#include "icm42688.h"
#include "imu_service.h"
#endif

#define DEBUG_TRAFFIC 0    // 0 = silent, 1 = Serial print report CAN traffic frames
#define DEBUG_TIME_SYNC 0  // 0 = silent, 1 = Serial print periodic messages showing how tight the clocks are synced

/*----------------------------------------------------------------------------*/
/*                        F I R M W A R E   I D E N T I T Y                   */
/*----------------------------------------------------------------------------*/
/*  Mirrors the can-bridge's identity block (Teensy_code_canbridge/
 *  canbridge_config.h, `CanBridge::FW_NAME` / `FW_VERSION`): a hand-authored
 *  marker with its bump history inline, printed at boot AND — unlike the
 *  can-bridge, which has a USB serial console the Jetson can read — reported to
 *  the host over CAN, because nothing on this board is reachable from the Jetson
 *  except through the can-bridge conduit.
 *
 *  WHY THIS EXISTS.  Until 2026-07-27 this board carried no version of any kind,
 *  so an UN-FLASHED Platform Teensy was indistinguishable from a flashed one from
 *  the Jetson: no log line, no field, no warning.  Every other deployment step in
 *  the stack fails loudly when skipped (a stale colcon install throws; a stale
 *  jugglebot_interfaces build kills trajectory_node ~200 ms after launch) — the
 *  firmware was the only one that failed SILENTLY, because a pre-fix board simply
 *  behaves like a pre-fix board.
 *
 *  WHERE IT IS REPORTED: bytes 5-6 of the 0x6E0 RobotState reply
 *  (`createStateCANMessage`), which were previously hard-zeroed reserved bytes.
 *  That choice is load-bearing, not incidental — see the frame-layout comment on
 *  `createStateCANMessage` for why the pre-versioning board's zeros ARE the
 *  sentinel.
 *
 *  VERSION SEMANTICS (identical to the can-bridge's): this is a human-facing
 *  identity marker.  It has NO runtime effect on this board and gates nothing;
 *  the host WARNS on a skew and never refuses a hand command
 *  (ros_ws/docs/platform_fw_version.md § Warn, never refuse).
 *
 *    0 = "pre-versioning" — no numbered release; the board predates this block.
 *        NEVER assign 0 to a release.  It is reserved as the sentinel a
 *        pre-2026-07-27 board transmits for free (its `createStateCANMessage`
 *        zero-fills bytes 5-7 unconditionally).
 *    1 = 2026-07-27.  First numbered release.  Carries the velocity-continuous
 *        `makeSmoothMove` (`Trajectory.h`, commit 5369fc2): the quintic is seeded
 *        from the live `current_hand_velocity` instead of v = 0, the empty-return
 *        branch is now conjunct on at_rest as well as |delta| < 1e-6, and the
 *        duration bound is the positive root of the corrected quadratic (capped by
 *        `smoothMoveMaxDuration()`).  Plan: plans/archived/hand-command-continuity.md
 *        Phase 4.  Also carries this identity block itself.
 *    2 = 2026-07-28.  POST-RELEASE DECELERATION FEEDFORWARD.  `Trajectory.h`'s
 *        `buildThrow` now sizes the torque feedforward of the decel segment
 *        (x2 -> x3, after the ball has left) from the axis's TOTAL reflected
 *        inertia — `throwDecelToTorque`, 9.5e-6 kg m^2 — instead of from
 *        `accelToTorque`'s hand-mass-on-a-spool model, which implied 7.3695e-6
 *        and so under-torqued the brake by ~30 %.  The commanded POSITION and
 *        VELOCITY streams are BIT-IDENTICAL to v1 on every kind; the accel and
 *        velocity-hold torques, all of kind 1, and `makeSmoothMove` are
 *        untouched.  Fixes the light end-stop contact measured at ~1.2 m throws
 *        on 2026-07-27.  Contract: C-HAND-2, retired with this code at FW 7;
 *        its surviving measurements are ros_ws/docs/hand_throw_envelope.md
 *        § Surviving measurements.  Plan: plans/archived/hand-command-continuity.md Phase 7.
 *        A v1 board is not unsafe, it simply still coasts — but every
 *        § CHECK HAND-7 row is meaningless on one, so read FW-1 first.
 *
 *  Bump on any behavioural change worth telling a bench operator about, and add a
 *  line above saying WHAT changed and WHEN (the can-bridge's comment style).
 *
 *    4 = 2026-09-08.  FW 18 BUNDLE hand-clip re-measurement — BEHAVIOURAL.
 *        Geometry::HAND_MOTOR_HARD_STOP_REVS 10.8 → 10.701 rev (operator
 *        re-measurement, config/hardware_config.yaml). Trajectory.h consumes it
 *        directly (no compensating margin widening this time, unlike 2→3):
 *        SMOOTH_MOVE_POS_CEIL_REV moves 10.60 → 10.501 rev, and
 *        smoothMoveMaxDuration() 0.78964 → 0.78602 s, tightening which branch
 *        makeSmoothMove takes for a prelude whose honoured duration lands in
 *        (0.78602, 0.78964] s, i.e. |v0| in (20.03, 20.14] rev/s. Conservative:
 *        that (now slightly wider) band takes the rest-to-rest fallback. This
 *        version bump is a HOST-side ripple of the can-bridge's FW 18 (which
 *        added `hand_clip_margin_rev` for its OWN, differently-shaped clip
 *        guard) — this board carries no clip-margin logic of its own; only the
 *        shared `hand_motor_hard_stop_revs` moved. Plan:
 *        plans/archived/unified-7dof-planner.md § FW 18 bundle. Logbook:
 *        logbook/2026-09-08-fw18-bundle-hand-clip-homing-counters-rename.md
 *    5 = 2026-09-09.  FIRST IMAGE THAT CAN BE UPDATED OVER CAN.  Adds the
 *        0x6F0/0x6F1 firmware-update endpoint (§ FIRMWARE UPDATE OVER CAN
 *        below): BEGIN/DATA/VERIFY/COMMIT, flash staging above the running
 *        image, CRC-32 + identity gate, then a FlasherX-style self-copy and
 *        reboot.  Motion behaviour is BIT-IDENTICAL to FW 4 — no trajectory,
 *        tilt, time-sync or RobotState path was touched — with one deliberate
 *        exception: while an update session is open this board REFUSES to arm
 *        new 0x6D0 trajectories (a sector flush stalls the streamer for tens
 *        of ms, which is not a thing to do mid-throw).
 *        ⚠ THIS IMAGE MUST BE FLASHED OVER USB ONCE, from the Arduino IDE.  A
 *        board running FW ≤ 4 has no 0x6F0 listener, so it cannot be given its
 *        own successor over CAN; FW 5 is the bootstrap that opens the path.
 *        Why the path exists at all: this board's micro-USB port is physically
 *        damaged and the Teensy bootloader chip speaks ONLY USB, so after this
 *        one flash, CAN is the only remaining route in.
 *    6 = 2026-09-09.  FIRST IMAGE BUILT ON THE JETSON (`pio run -e teensy40
 *        -t upload`) and flashed over CAN.  No code change beyond the number:
 *        with the USB console gone, the STATE_READ version (5 -> 6) is the
 *        only proof the copy landed and the new image runs.
 *    7 = 2026-09-11.  STROKE ENGINE DELETED — skill-stack R1, one hand master.
 *        Gone: `Trajectory.h` (makeThrow/makeCatch/makeFull/makeSmoothMove/
 *        throwDecelToTorque/accelToTorque), the 0x6D0 TRAJ_CMD decode and its
 *        smooth-move prelude, the packed-frame scheduler that streamed
 *        set_input_pos to CAN node 6, and the 0x0C9 hand-encoder cache.  After
 *        this image the can-bridge is the SOLE axis-6 writer (I-FW-15) and this
 *        board never transmits to node 6 at all.  UNCHANGED: the SCL3300
 *        inclinometer (0x7DE), the TimeSync slave (0x7DD), the RobotState
 *        exchange incl. the FW_VERSION reply bytes and the cold-start state,
 *        the traffic monitor, and the 0x6F0/0x6F1 firmware-update endpoint —
 *        whose BEGIN/COMMIT `engineIdle()` guard went with the streamer it
 *        protected (there is no longer a throw to reboot into the middle of;
 *        `ST_BUSY` is retired but its number stays reserved).  Plan:
 *        plans/active/two-ball-skill-stack.md § 4 R1.
 *    8 = 2026-11-07.  HAND COOLING FAN.  Pin TeensyOp::HAND_FAN_PIN drives the gate of
 *        an N-channel low-side MOSFET (HIGH = on; the gate pull-down holds it off
 *        whenever this board is not driving the pin).  Runs from the Hand ODrive's
 *        motor thermistor broadcast alone, so it works with no ROS2 and no Jetson;
 *        see hand_fan.h for the rule.  Receive-only on CAN: the TX set and every
 *        other path are unchanged.  Never flashed; superseded by 9 the same day.
 *    9 = 2026-11-07.  HAND FAN RUNTIME CONTROL, for A/B tests.  0x6E1 sets the mode
 *        (AUTO / FORCE_ON / FORCE_OFF) and thresholds in RAM, so every reboot returns
 *        to AUTO at the defaults.  0x6E2 is a new status frame at 1 Hz, on every
 *        command and on every fan change.  Needs can-bridge FW 41 for both; on an
 *        older bridge AUTO still works.  No motion path touched.
 *   11 = 2026-11-19.  ROBUSTNESS for the Platform IMU (IDEA-20 P2a): this board must
 *        keep calling can1.events(), because firmware over CAN is its only way in.
 *        (a) The 0x7DE tilt request no longer runs the inclinometer inside the CAN
 *        callback: the callback sets a flag and loop() serves it (loop_service.h),
 *        reading in the same pass, at most one sensor operation per pass, retries
 *        10 ms apart.  A failing SCL3300 used to hold the callback >= 600 ms.
 *        (b) When the inclinometer cannot answer, the reply is an explicit INVALID
 *        frame, two quiet NaNs with a reason (protocol_config.yaml platform_tilt),
 *        instead of the old (pi, pi).  A FW <= 10 host decodes it as (nan, nan) and
 *        rejects it with its |tilt| bound, as it rejected (pi, pi).
 *        (c) setup() no longer waits for ever for the SCL3300: three begin() calls
 *        at most (about 310 ms), then the board runs with tilt NOT_READY and loop()
 *        re-probes (1 s, backing off to 60 s; never during a firmware update, never
 *        brought forward by a tilt request).
 *        With the sensor present, setup() makes the same single begin() as before.
 *        (d) loop() dispatches up to 32 received CAN frames per pass, not one, each
 *        only while 5 TX mailboxes are free and FlexCAN's software TX queue is empty;
 *        during a firmware update, exactly one per pass as before.
 *        No motion path, no CAN id, no TX frame other than the tilt reply changed.
 *        (e) IDEA-20 P2b: the Platform IMU (ICM-42688-P on Wire, pins 18/19, 1 MHz):
 *        a 32 s capture ring in RAM2, frozen and uploaded on request over 0x6E3-0x6E6
 *        (ros_ws/docs/platform_imu_protocol.md), status at 1 Hz.  STRICTLY OPTIONAL:
 *        absent, unplugged or stuck, it costs at most one bounded I2C transaction per
 *        loop() pass and backs off to one probe a minute; a firmware-update BEGIN stops
 *        all of it.  Nothing IMU runs for the first 10 s after boot, and a BEGIN in
 *        those 10 s keeps it off until the next reboot (the recovery window if IMU code
 *        ever faults).  Kill switch: PLATFORM_IMU_ENABLED (below the includes).
 */
constexpr char     FW_NAME[]  = "jugglebot-platform";
constexpr uint16_t FW_VERSION = 12;  // 1→2: 2026-07-29 throwDecelToTorque (post-release decel
                                     //      feedforward, C-HAND-2).
                                     // 2→3: 2026-08-18 hand END-STOP correction — BEHAVIOURAL.
                                     //      Geometry::HAND_MOTOR_HARD_STOP_REVS 11.1 → 10.8 rev
                                     //      (operator-measured metal contact). Trajectory.h
                                     //      consumes it: SMOOTH_MOVE_POS_CEIL_REV is UNCHANGED at
                                     //      10.60 rev (the margin moved 0.5 → 0.2 to hold it), but
                                     //      smoothMoveMaxDuration() 0.80054 → 0.78964 s, which
                                     //      changes which branch makeSmoothMove takes for a
                                     //      prelude whose honoured duration lands in
                                     //      (0.78964, 0.80054] s, i.e. |v0| in (20.04, 20.32] rev/s.
                                     //      Conservative: that band now takes the rest-to-rest
                                     //      fallback. See logbook/2026-08-18-hand-end-stop-corrected.md
                                     // 3→4: 2026-09-08 FW 18 BUNDLE hand-clip re-measurement —
                                     //      BEHAVIOURAL. HAND_MOTOR_HARD_STOP_REVS 10.8 → 10.701
                                     //      rev; SMOOTH_MOVE_POS_CEIL_REV 10.60 → 10.501 rev;
                                     //      smoothMoveMaxDuration() 0.78964 → 0.78602 s. See the
                                     //      identity-block entry above for the full derivation.
                                     // 4→5: 2026-09-09 FIRMWARE UPDATE OVER CAN (0x6F0/0x6F1).
                                     //      Motion paths bit-identical to FW 4; the only
                                     //      behavioural change outside the new endpoint is that
                                     //      0x6D0 trajectory arming is refused while an update
                                     //      session is open. FW 5 must be USB-flashed once — a
                                     //      FW ≤ 4 board cannot receive its own successor.
                                     // 5→6: 2026-09-09 the first image BUILT ON THE JETSON
                                     //      (`pio run -e teensy40 -t upload`) and flashed over
                                     //      CAN. No code change beyond this number: the bump IS
                                     //      the receipt — with the USB console gone, the
                                     //      STATE_READ version (5 → 6) is the only proof the
                                     //      copy landed and the new image runs.
                                     // 6→7: 2026-09-11 skill-stack R1 — stroke engine deleted:
                                     //      0x6D0 decode, hand encoder cache, Trajectory.h.
                                     //      Inclinometer / time-sync / RobotState / firmware
                                     //      update unchanged. The can-bridge is now the sole
                                     //      axis-6 writer (I-FW-15).
                                     // 7→8: 2026-11-07 Hand cooling fan (hand_fan.h), driven
                                     //      from the Hand ODrive's temperature broadcast.
                                     // 8→9: 2026-11-07 Hand fan runtime control and status
                                     //      (0x6E1 / 0x6E2), RAM only.
                                     // 9→10: 2026-11-10 BUG-7 the levelling offset is held
                                     //      as the raw int16 wire counts and echoed exactly:
                                     //      no float round trip, no truncating cast.
                                     // 10→11: 2026-11-19 IDEA-20 P2a robustness: tilt served
                                     //      from loop() with an explicit invalid reply,
                                     //      bounded inclinometer probe in setup() plus
                                     //      re-probes from loop(), up to 32 CAN frames
                                     //      dispatched per loop() pass. P2b: the Platform
                                     //      IMU capture, status and upload (0x6E3-0x6E6).
                                     // 11→12: 2026-11-20 Platform IMU I2C robustness: a bus
                                     //      recovery before every re-probe, and the RUNNING
                                     //      retry policy (a failed FIFO count/burst/flush is
                                     //      recovered and retried, BACKOFF only on the 3rd
                                     //      in a row; imu_service.h drain_step).

/*----------------------------------------------------------------------------*/
/*                                CAN BUS SET‑UP                              */
/*----------------------------------------------------------------------------*/
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
constexpr uint32_t CAN_BITRATE = CanBus::BAUD_RATE;

/*----------------------------------------------------------------------------*/
/*                             INCLINOMETER (SCL3300)                         */
/*----------------------------------------------------------------------------*/
SCL3300 inclinometer;

/*----------------------------------------------------------------------------*/
/*                               CAN  ID MAP                                  */
/*----------------------------------------------------------------------------*/
// CAN IDs from protocol_config.h
constexpr uint32_t REPORT_ID      = PlatformCanId::TRAFFIC_REPORT;
constexpr uint32_t tiltID          = PlatformCanId::TILT_READING;
constexpr uint32_t timeSyncID      = SharedCanId::TIME_SYNC;
constexpr uint32_t stateUpdateID   = PlatformCanId::STATE_UPDATE;
constexpr uint32_t FW_UPD_CMD_ID   = PlatformCanId::FW_UPDATE_CMD;    // host → this board
constexpr uint32_t FW_UPD_REPLY_ID = PlatformCanId::FW_UPDATE_REPLY;  // this board → host
constexpr uint32_t HAND_TEMPS_ID = (uint32_t(NodeId::JUGGLEBOT_HAND) << 5) | ODriveCmd::get_temps;
constexpr uint32_t HAND_FAN_CMD_ID    = PlatformCanId::HAND_FAN_CMD;     // host → this board
constexpr uint32_t HAND_FAN_STATUS_ID = PlatformCanId::HAND_FAN_STATUS;  // this board → host

/*----------------------------------------------------------------------------*/
/*                            TRAFFIC MONITOR                                 */
/*----------------------------------------------------------------------------*/
uint32_t receivedCount = 0;
uint32_t lastReportTime = 0;
constexpr uint32_t reportInterval = TeensyOp::REPORT_INTERVAL_MS;

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
  // Levelling offset in 0x6E0 wire counts. This board only stores and echoes
  // it, so it stays an integer and needs no knowledge of the scale (BUG-7).
  int16_t pose_offset_tiltX_counts;
  int16_t pose_offset_tiltY_counts;
};
RobotState state = { false, false, 0, 0 };

/*----------------------------------------------------------------------------*/
/*                           PLATFORM TILT                                    */
/*----------------------------------------------------------------------------*/
struct platformTilt {
  float tiltX;
  float tiltY;
};

/*----------------------------------------------------------------------------*/
/*                             HAND COOLING FAN                               */
/*----------------------------------------------------------------------------*/
/*  N-channel low-side MOSFET with a gate pull-down: HIGH = fan on, and the fan is
 *  off whenever the pin is undriven (board off, reset, boot).  During a firmware
 *  COMMIT the main loop has stopped, so the pin holds its last state until the
 *  reset releases it. */
/*  This section sits below the structs above on purpose: the .ino preprocessor puts
 *  every function prototype ahead of the first function it finds, and
 *  createStateCANMessage needs RobotState declared by then (as sendTiltData needed
 *  platformTilt before FW 11). */
constexpr uint8_t HAND_FAN_PIN = uint8_t(TeensyOp::HAND_FAN_PIN);
static_assert(HAND_FAN_PIN != 10 && HAND_FAN_PIN != 11 && HAND_FAN_PIN != 12 && HAND_FAN_PIN != 13
                  && HAND_FAN_PIN != 18 && HAND_FAN_PIN != 19 && HAND_FAN_PIN != 22 && HAND_FAN_PIN != 23,
              "hand_fan_pin collides with the SCL3300's SPI, the IMU's I2C (18/19) or this board's CAN");
static_assert(TeensyOp::HAND_FAN_OFF_BELOW_C < TeensyOp::HAND_FAN_ON_ABOVE_C,
              "the Hand fan thresholds need a gap, or the fan chatters at the threshold");

HandFan::Controller handFan(TeensyOp::HAND_FAN_ON_ABOVE_C, TeensyOp::HAND_FAN_OFF_BELOW_C,
                            TeensyOp::HAND_FAN_TEMP_TIMEOUT_MS,
                            TeensyOp::HAND_FAN_THRESHOLD_MIN_C, TeensyOp::HAND_FAN_THRESHOLD_MAX_C);
bool handFanRunning = false;
uint32_t lastHandFanStatusMs = 0;

void sendHandFanStatus() {
  CAN_message_t m;
  m.id = HAND_FAN_STATUS_ID;
  m.len = HandFan::STATUS_FRAME_LEN;
  handFan.encode_status(m.buf);
  can1.write(m);
  lastHandFanStatusMs = millis();
}

void updateHandFan() {
  const uint32_t now = millis();
  const bool want = handFan.update(now);
  const bool changed = (want != handFanRunning);
  if (changed) {
    digitalWrite(HAND_FAN_PIN, want ? HIGH : LOW);
    handFanRunning = want;
  }
  if (changed || (now - lastHandFanStatusMs) >= TeensyOp::HAND_FAN_STATUS_INTERVAL_MS) {
    sendHandFanStatus();
  }
}

/*----------------------------------------------------------------------------*/
/*                       P L A T F O R M   I M U  (IDEA-20)                    */
/*----------------------------------------------------------------------------*/
#if PLATFORM_IMU_ENABLED
/*  1 MHz (Fast-mode Plus).  400000 is the fallback: set it here if the board's pull-ups or
 *  wiring do not hold 1 MHz; the I2C_400K flags then say so in STATUS and the header, and a
 *  burst of 8 packets takes ~3.0 ms instead of ~1.3 ms. */
#ifndef PLATFORM_IMU_I2C_HZ
#define PLATFORM_IMU_I2C_HZ 1000000   // build-time override: build_flags -DPLATFORM_IMU_I2C_HZ=400000
#endif
constexpr uint32_t IMU_I2C_HZ = PLATFORM_IMU_I2C_HZ;
constexpr uint8_t IMU_SDA_PIN = 18, IMU_SCL_PIN = 19;   // Wire (LPI2C1) on the Teensy 4.0

/*  IMU DIAGNOSTIC BUILD (logbook 2026-11-20-platform-imu-i2c-error-bursts).  Default 0: the
 *  normal image carries none of this.  platformio.ini env teensy40_imudiag sets
 *  -DPLATFORM_IMU_DIAG=1; env teensy40_imudiag_mask also sets -DPLATFORM_IMU_DIAG_MASK_CAN=1,
 *  which masks the CAN1 interrupt for the duration of every IMU Wire operation (A/B for "the
 *  CAN RX interrupt breaks an I2C transfer"; DIAGNOSTIC ONLY, robot unpowered: a Wire timeout
 *  then holds CAN RX off for up to ~70 ms and the 6-deep RX FIFO overflows).
 *
 *  In a diag build IMU_STATUS keeps state, flags, fifo_overflows, i2c_errors and boot, and
 *  repurposes three bytes: temp_c (byte 2) := PAGE 0..7 (one page per STATUS, so a full cycle
 *  is 8 s), sync_peak (byte 5) := A, fill_s (byte 6) := B.
 *
 *    page  A                                        B
 *    0     last error KIND (below; 0 = none yet)    driver STAGE of that error's Wire failure
 *    1     Wire CODE (stage 1/3/4: endTransmission  bytes REQUESTED (stage 2); for kind 6:
 *          return 2 NACK, 4 busy/timeout/ALF/PLTF,  (burst packets << 4) | index of the bad one
 *          5 FIFO error; stage 2: bytes received;
 *          kind 6: the bad header byte)
 *    2     LPI2C MSR flags right after the failure: duration of the failed op, 100 us units
 *          b0 NDF b1 ALF b2 FEF b3 PLTF b4 SDF      (255 = >= 25.5 ms: a Wire timeout path)
 *          b5 MBF b6 BBF
 *    3     # kind 1 (FIFO_COUNT fails)              # kind 2 (burst fails)
 *    4     # kind 3 + 5 (flush, resume fails)       # kind 4 (probe/init fails)
 *    5     # kind 6 (bad FIFO headers; NOT in       # bus recoveries (bus clear + controller
 *          i2c_errors in any build)                 reset before a re-probe or, FW 12, a
 *                                                   RUNNING retry)
 *    6     sync_peak of the last second (10 us)     longest gap between two loop() passes
 *                                                   since the last page 6, ms (255 = >= 255)
 *    7     init state the last failed probe ended   last FIFO overflow: count (0..127) | 0x80 if
 *          in (1 PROBE_ACK 2 PROBE_BANK 3 PROBE_WHO INT_STATUS.FIFO_FULL was set
 *          4 RESET_BANK 5 RESET 6 POLL_RESET
 *          7 POLL_BANK 8 POLL_WHO 9 CONFIG 10 PWR
 *          11 WAIT_GYRO 12 FLUSH)
 *
 *  KIND: 1 FIFO_COUNT read, 2 FIFO_DATA burst (Wire level), 3 drain flush, 4 probe/init step,
 *  5 RESUME flush/WHO, 6 bad FIFO header (ImuService::DiagKind).  STAGE: 0 none (the op failed
 *  without a Wire failure, e.g. WHO mismatch), 1 register-address write (endTransmission(false))
 *  of a read, 2 requestFrom returned short, 3 register write, 4 probe address ACK.  Counts
 *  saturate at 255. */
#ifndef PLATFORM_IMU_DIAG
#define PLATFORM_IMU_DIAG 0
#endif
#ifndef PLATFORM_IMU_DIAG_MASK_CAN
#define PLATFORM_IMU_DIAG_MASK_CAN 0
#endif
#if PLATFORM_IMU_DIAG_MASK_CAN
static_assert(PLATFORM_IMU_DIAG, "the CAN-mask variant is a diagnostic build only");
static_assert(PLATFORM_IMU_I2C_HZ >= 1000000, "CAN masked over a 400 kHz burst (3.2 ms) overflows the RX FIFO");
#endif
static_assert(ImuService::INIT_PENDING == icm42688::Driver::INIT_PENDING, "imu_service.h mirrors the driver");
static_assert(ImuService::ERR_BAD_PACKET == icm42688::ERR_BAD_PACKET, "imu_service.h mirrors the driver");
static_assert(ImuService::BURST_PKTS <= icm42688::Driver::BURST_PKTS, "one burst is one Wire read");

/*  The capture ring (32000 x 12 B = 384000 B) and anchor table in RAM2 (DMAMEM, .bss.dma),
 *  linker-placed below the heap and so below the CrashReport area at 0x2027FF80.  DMAMEM is
 *  NOT zeroed at boot: ImuCapture only ever reads slots it has written (count_, a_count_), so
 *  nothing depends on the initial contents. */
DMAMEM int16_t imuRingBuf[PlatformImu::RING_SAMPLES * 6] __attribute__((aligned(32)));
DMAMEM ImuCapture::Anchor imuAnchors[PlatformImu::ANCHOR_RING];
ImuCapture::Capture imuCapture;
ImuService::Service imuService;
icm42688::Driver imuChip(Wire);
/*  Mixed with the cycle counter by every CAN frame received during setup() (canSniff runs in
 *  FlexCAN's ISR then): the bus's timing against this board's boot is what makes boot_id
 *  differ from one boot to the next.  Integer only. */
volatile uint32_t imuBootEntropy = 0;
DMAMEM uint32_t imuBootSeq;  // uninitialised on purpose: a boot counter that survives warm resets
volatile uint32_t imuTxQueuedAnyway = 0;  // write() queued an IMU frame (never expected; see ImuTx)
#endif

/*----------------------------------------------------------------------------*/
/*                        ──  TIME–SYNC  LAYER ──                             */
/*----------------------------------------------------------------------------*/
namespace TimeSync {

/* 64‑bit free‑running microsecond counter (wraps after 71 min) */
/* ISR-SAFE: wrap detection runs with interrupts masked (same idiom as
 * BallButler ball_butler_main/Micros64.h). The unguarded version is racy
 * whenever an ISR and the main loop both call it — an in-call preemption that
 * advances last_lo reads as a false 32-bit wrap (+2^32 µs on the clock). That
 * bit the catching cone (piezo ISR) on 2026-06-10; this copy had no ISR
 * caller but is fixed for parity per the keep-in-sync rule — see
 * logbook/2026-06-10-cone-micros64-false-wrap.md. NOT yet flashed to the
 * platform Teensy (deploy on its next natural firmware update). */
uint64_t micros64() {
  uint32_t primask;
  asm volatile("MRS %0, PRIMASK" : "=r"(primask) :: "memory");
  __disable_irq();

  static uint32_t last_lo = 0;
  static uint64_t hi = 0;
  uint32_t now = ::micros();
  if (now < last_lo) hi += UINT64_C(1) << 32;
  last_lo = now;
  uint64_t result = hi | now;

  if ((primask & 1u) == 0u) __enable_irq();  // re-enable only if enabled before
  return result;
}

/* Wall‑time offset: Jetson_wall_us − micros64() */
volatile int64_t wall_offset_us = 0;
volatile bool have_offset = false;
constexpr uint8_t ALPHA_SHIFT = TeensyOp::TIME_SYNC_ALPHA_SHIFT;  // I‑filter gain = 1/2^n

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
#if PLATFORM_IMU_ENABLED
  /* IMU_STATUS sync_peak: wall time received minus get_wall_time_us() before the IIR update,
   * i.e. offset - wall_offset_us.  Integer only: this runs in FlexCAN's ISR during setup(). */
  imuService.sync().note(!have_offset, offset - wall_offset_us);
#endif

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

/*----------------------------------------------------------------------------*/
/*              F I R M W A R E   U P D A T E   O V E R   C A N               */
/*----------------------------------------------------------------------------*/
/*  WHY THIS EXISTS.  This board's micro-USB port is physically damaged, and the
 *  Teensy bootloader chip speaks ONLY USB — there is no serial, no HID, no
 *  half-speed fallback once the connector is gone.  So every image after FW 5
 *  has to arrive over CAN, through the one conduit this board still has: the
 *  can-bridge already relays typed RPCs to us and uplinks our reply frames
 *  verbatim, so the update path is
 *
 *      host tool ── UDP ──▶ can-bridge ── CAN 0x6F0 ──▶ here
 *                                       ◀── CAN 0x6F1 ──
 *
 *  FW 5 ITSELF MUST BE FLASHED OVER USB ONCE (Arduino IDE, from this .ino).  A
 *  board running FW ≤ 4 has no 0x6F0 listener at all, so it cannot be handed its
 *  own successor.  Miss that one flash and the CAN path never opens.
 *
 *  MECHANISM: mirrors FlasherX (joepasquariello/FlasherX) without vendoring it —
 *  the same three moves, written inline in ~200 lines.  The new image is staged
 *  into the unused flash ABOVE the running image, CRC-32 and identity checked
 *  there, and only then copied down over the program region with interrupts
 *  masked, followed by a reboot.  On a Teensy 4.x every function runs from ITCM
 *  RAM unless it is FLASHMEM/PROGMEM, and .rodata is copied to DTCM at boot, so
 *  nothing on the commit path — not this code, not memcpy, not the core's flash
 *  primitives (cores/teensy4/eeprom.c carries no FLASHMEM at all) — needs to
 *  read the flash it is busy rewriting.
 *
 *  ⚠ THE COMMIT COPY IS THE ONE UNPROTECTED MOMENT.  It erases sector 0, which
 *  holds the FlexSPI configuration block and the IVT.  A power cut between that
 *  first erase and the end of the copy leaves an unbootable board that ONLY a
 *  USB reflash can recover — which is exactly what this board no longer has.
 *  Do not COMMIT on a flaky supply.  Everything BEFORE commit is free: the
 *  staging region is scratch, and an abandoned session costs nothing.
 *
 *  WIRE CONTRACT (little-endian throughout; mirrored by the can-bridge relay and
 *  the host tool — this comment, the bridge's and the host's are one document):
 *
 *    Command, id 0x6F0, byte 0 = opcode
 *      0x01 BEGIN   [op][image_len u32 @1..4][0,0,0]
 *      0x02 DATA    [op][seq u16 @1..2][payload @3..7], n = dlc-3, 1..5 bytes
 *      0x03 VERIFY  [op][crc32 u32 @1..4][0,0,0]
 *      0x04 COMMIT  [op][0...]
 *
 *    Reply, id 0x6F1, dlc 8
 *      [opcode echoed @0][status @1][seq u16 @2..3][detail u32 @4..7]
 *      seq    = last accepted DATA seq, or the EXPECTED seq on BAD_SEQ
 *      detail = staged byte count (BEGIN/DATA), computed crc (VERIFY), else 0
 *
 *  SEQUENCING.  DATA is strictly in order.  `seq` counts DATA frames from 0 and
 *  wraps mod 65536; the receiver tracks the absolute byte offset separately, so
 *  the wrap is invisible to the image.  A frame whose seq != expected is NAKed
 *  (BAD_SEQ, seq field = the expected seq), dropped, and the receiver keeps
 *  waiting for that same seq — the host rewinds to it.  ACK (OK) is sent on
 *  every 16th accepted frame (seq % 16 == 15) and on the frame that completes
 *  image_len; every other accepted frame is silent.  65536 % 16 == 0, so the ACK
 *  cadence is continuous across the seq wrap.
 *
 *  WHY THE HOST WILL SEE THE ODD BURST OF NAKs.  Staging a 4 KB sector means a
 *  sector erase (typ. 45 ms, up to 400 ms on a W25Q16) plus sixteen page writes,
 *  and this handler runs in the main loop, so `can1.events()` is not pumped for
 *  that whole window.  FlexCAN_T4's 256-frame software RX queue absorbs a
 *  typical erase; a worst-case one can overrun it.  That is DESIGNED-FOR, not a
 *  fault: the next frame to arrive NAKs with the expected seq and the host
 *  rewinds.  Do not "fix" it by widening the queue.
 */

extern "C" {
/*  cores/teensy4/eeprom.c — "To be called from LittleFS_Program, any other use
 *  at your own risk!".  Both mask interrupts internally and RE-ENABLE them on
 *  the way out (via that file's flash_wait()), which is why the commit loop
 *  re-masks after every single call rather than once at the top. */
void eepromemu_flash_write(void *addr, const void *data, uint32_t len);
void eepromemu_flash_erase_sector(void *addr);
}

/*  Linker symbol from cores/teensy4/imxrt1062.ld:
 *      _flashimagelen = __text_csf_end - ORIGIN(FLASH);
 *  i.e. the byte length of the RUNNING image.  Its ADDRESS is the value. */
extern unsigned long _flashimagelen;

namespace FwUpdate {

constexpr uint32_t FLASH_BASE   = 0x60000000u;
constexpr uint32_t SECTOR_SIZE  = 4096u;
constexpr uint32_t WRITE_CHUNK  = 256u;  // one QSPI page — LittleFS_Program's prog_size,
                                         // so a proven length for eepromemu_flash_write
/*  Top of usable program flash = base of the core's EEPROM-emulation reserve.
 *  Two independent sources agree on this number for a Teensy 4.0: eeprom.c's
 *  FLASH_BASEADDR under ARDUINO_TEENSY40, and ORIGIN(FLASH) + LENGTH(FLASH)
 *  = 0x60000000 + 1984K.  Nothing may be staged at or above it. */
constexpr uint32_t EEPROM_RESERVE = 0x601F0000u;

constexpr uint32_t COMMIT_DRAIN_MS   = 50;      // let the COMMIT reply reach the bridge
constexpr uint32_t SESSION_IDLE_MS   = 60000;   // abandoned-session escape hatch

/*  Opcodes (host → platform, byte 0 of a 0x6F0 frame). */
constexpr uint8_t OP_BEGIN = 0x01, OP_DATA = 0x02, OP_VERIFY = 0x03, OP_COMMIT = 0x04;

/*  Status codes (byte 1 of the 0x6F1 reply).  THE CAN-BRIDGE RELAY AND THE HOST
 *  TOOL MIRROR THIS TABLE VERBATIM — never renumber a code, only append. */
constexpr uint8_t ST_OK           = 0;
constexpr uint8_t ST_BUSY         = 1;  // RETIRED at FW 7 — never sent since the stroke
                                        // engine was deleted (I-FW-15). The number stays
                                        // reserved so the host status table keeps its
                                        // meaning for pre-FW-7 replies.
constexpr uint8_t ST_BAD_STATE    = 2;  // no session open, wrong phase, or malformed dlc
constexpr uint8_t ST_BAD_SEQ      = 3;  // out-of-order DATA; seq field = the expected seq
constexpr uint8_t ST_TOO_BIG      = 4;  // image_len > staging capacity, or DATA past image_len
constexpr uint8_t ST_BAD_CRC      = 5;
constexpr uint8_t ST_BAD_IDENTITY = 6;  // staged image is not a jugglebot-platform build
constexpr uint8_t ST_FLASH_ERR    = 7;  // staging write failed its read-back

/*  Session state.  IDLE → (BEGIN) → RECEIVING → (VERIFY ok) → VERIFIED → (COMMIT).
 *  A failed VERIFY drops back to RECEIVING; a failed staging write drops all the
 *  way to IDLE; BEGIN resets from anywhere. */
enum Phase : uint8_t { P_IDLE = 0, P_RECEIVING = 1, P_VERIFIED = 2 };

Phase    phase       = P_IDLE;
uint32_t image_len   = 0;   // declared at BEGIN
uint32_t stage_base  = 0;   // computed at BEGIN
uint32_t stage_cap   = 0;   // EEPROM_RESERVE - stage_base
uint32_t flushed     = 0;   // image bytes already committed to the staging flash
uint32_t buf_fill    = 0;   // image bytes held in sector_buf
uint16_t expect_seq  = 0;   // next DATA seq we will accept
uint16_t last_seq    = 0;   // last DATA seq accepted (reported in replies)
uint32_t last_cmd_ms = 0;   // millis() of the last 0x6F0 frame, for the idle timeout

/*  One 4 KB sector, reused for staging AND for the commit copy (the commit reads
 *  a staged sector into here before erasing its destination — the flash cannot
 *  be read while it is being programmed). */
alignas(4) uint8_t sector_buf[SECTOR_SIZE];

inline uint32_t staged() { return flushed + buf_fill; }
inline bool sessionOpen() { return phase != P_IDLE; }

/*  Staging starts at the first sector boundary ABOVE the running image, so the
 *  staged copy never overlaps the code that is doing the staging. */
inline uint32_t stagingBase() {
  const uint32_t img_end = FLASH_BASE + (uint32_t)&_flashimagelen;
  return (img_end + (SECTOR_SIZE - 1u)) & ~(SECTOR_SIZE - 1u);
}

void reply(uint8_t op, uint8_t status, uint16_t seq, uint32_t detail) {
  CAN_message_t m;
  m.id  = FW_UPD_REPLY_ID;
  m.len = 8;
  m.buf[0] = op;
  m.buf[1] = status;
  m.buf[2] = uint8_t(seq & 0xFF);
  m.buf[3] = uint8_t(seq >> 8);
  m.buf[4] = uint8_t(detail & 0xFF);
  m.buf[5] = uint8_t((detail >> 8) & 0xFF);
  m.buf[6] = uint8_t((detail >> 16) & 0xFF);
  m.buf[7] = uint8_t((detail >> 24) & 0xFF);
  can1.write(m);
}

/*  Erase + write the buffered bytes into the staging flash, then READ THEM BACK.
 *  The read-back is the only way a failed erase or write can be noticed at all:
 *  the core's primitives return void and the QSPI status register is consumed
 *  inside them.  Erasing lazily — here, rather than blanking the whole region at
 *  BEGIN — keeps BEGIN's reply immediate and spreads the stall over the
 *  transfer instead of concentrating a multi-second freeze at its start. */
bool flushBuffer() {
  if (buf_fill == 0) return true;

  const uint32_t addr = stage_base + flushed;
  const uint32_t n    = (buf_fill + 3u) & ~3u;          // pad up to a 4-byte write
  for (uint32_t i = buf_fill; i < n; ++i) sector_buf[i] = 0xFF;

  eepromemu_flash_erase_sector((void *)addr);
  for (uint32_t off = 0; off < n; off += WRITE_CHUNK) {
    uint32_t len = n - off;
    if (len > WRITE_CHUNK) len = WRITE_CHUNK;
    eepromemu_flash_write((void *)(addr + off), sector_buf + off, len);
  }

  arm_dcache_delete((void *)addr, n);                   // read from flash, not from cache
  if (memcmp((const void *)addr, sector_buf, n) != 0) {
    Serial.printf("[fwupd] FLASH_ERR: read-back mismatch at 0x%08lX\n", (unsigned long)addr);
    return false;
  }

  flushed  += buf_fill;   // padding is NOT image content
  buf_fill  = 0;
  return true;
}

/*  CRC-32/IEEE 802.3 — the zlib / binascii.crc32 flavour the host computes:
 *  reflected polynomial 0xEDB88320, init 0xFFFFFFFF, final xor 0xFFFFFFFF.
 *  Bitwise, so it needs no 1 KB table in RAM; ~20 ms for a 150 KB image. */
uint32_t crc32(const uint8_t *p, uint32_t n) {
  uint32_t crc = 0xFFFFFFFFu;
  while (n--) {
    crc ^= *p++;
    for (uint8_t k = 0; k < 8; ++k)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
  }
  return ~crc;
}

/*  Identity gate: the staged image must contain this board's own FW_NAME marker.
 *  That single byte string is what separates a jugglebot-platform build from a
 *  can-bridge image (Teensy 4.1, "jugglebot-canbridge" — which does NOT contain
 *  "jugglebot-platform" as a substring), a bench-sysid image, or any foreign hex
 *  that happened to CRC correctly because the host hashed what it sent.  The
 *  CRC proves the transfer; only this proves the INTENT. */
bool identityOk() {
  const uint8_t *img  = (const uint8_t *)stage_base;
  const uint32_t nlen = sizeof(FW_NAME) - 1u;   // 18 bytes, NUL excluded
  if (image_len < nlen) return false;
  for (uint32_t i = 0; i + nlen <= image_len; ++i) {
    if (img[i] == (uint8_t)FW_NAME[0] && memcmp(img + i, FW_NAME, nlen) == 0) return true;
  }
  return false;
}

/*  Copy staged → program flash, sector by sector, low to high, then reboot.
 *  Never returns.
 *
 *  THE OVERLAP INVARIANT.  Destination sector s spans [FLASH_BASE + s*4096,
 *  +4096); its source is [stage_base + s*4096, +4096).  stage_base is at least
 *  one sector above FLASH_BASE, so dst_s + 4096 <= src_s for every s: erasing a
 *  destination sector can only destroy source bytes that were read one iteration
 *  EARLIER, never bytes still to come.  A new image LONGER than the staging
 *  offset therefore overwrites its own tail's source region safely — those bytes
 *  are already in the destination by the time the writer reaches them.  This is
 *  the whole reason the copy must run low to high and must never be reordered.
 *
 *  Each sector is read into RAM before its destination is touched, because the
 *  QSPI cannot serve an AHB read while an erase or page-program is in flight. */
void commitAndReboot() {
  const uint32_t sectors = (image_len + SECTOR_SIZE - 1u) / SECTOR_SIZE;

  __disable_irq();
  for (uint32_t s = 0; s < sectors; ++s) {
    const uint32_t src = stage_base + s * SECTOR_SIZE;
    const uint32_t dst = FLASH_BASE  + s * SECTOR_SIZE;

    arm_dcache_delete((void *)src, SECTOR_SIZE);
    memcpy(sector_buf, (const void *)src, SECTOR_SIZE);

    __disable_irq();                                  // the primitives re-enable on exit
    eepromemu_flash_erase_sector((void *)dst);
    __disable_irq();
    for (uint32_t off = 0; off < SECTOR_SIZE; off += WRITE_CHUNK) {
      eepromemu_flash_write((void *)(dst + off), sector_buf + off, WRITE_CHUNK);
      __disable_irq();
    }
  }

  SCB_AIRCR = 0x05FA0004;   // system reset
  while (1) {}
}

/*  Handle one 0x6F0 command frame.  Runs in main-loop context: FlexCAN_T4 queues
 *  frames in the ISR and dispatches them from can1.events(), so blocking here
 *  costs latency, not received frames. */
void handleCommand(const CAN_message_t &msg) {
  if (msg.len < 1) return;
  const uint8_t op = msg.buf[0];
  last_cmd_ms = millis();

  switch (op) {

    case OP_BEGIN: {
      if (msg.len < 5) { reply(op, ST_BAD_STATE, 0, 0); return; }
      const uint32_t len = uint32_t(msg.buf[1]) | (uint32_t(msg.buf[2]) << 8)
                         | (uint32_t(msg.buf[3]) << 16) | (uint32_t(msg.buf[4]) << 24);

      stage_base = stagingBase();
      stage_cap  = EEPROM_RESERVE - stage_base;
      if (len == 0 || len > stage_cap) {
        phase = P_IDLE;
        /* detail carries the CAPACITY here, not the staged count — a bare 0
         * would tell the operator nothing, and the 0x6F1 reply is this board's
         * only voice (the USB console is gone with the connector). */
        reply(op, ST_TOO_BIG, 0, stage_cap);
        return;
      }

      image_len  = len;
      flushed    = 0;
      buf_fill   = 0;
      expect_seq = 0;
      last_seq   = 0;
      phase      = P_RECEIVING;
      reply(op, ST_OK, 0, 0);
      Serial.printf("[fwupd] BEGIN %lu B  staging 0x%08lX..0x%08lX (%lu KB free)\n",
                    (unsigned long)image_len, (unsigned long)stage_base,
                    (unsigned long)EEPROM_RESERVE, (unsigned long)(stage_cap / 1024u));
      return;
    }

    case OP_DATA: {
      if (phase != P_RECEIVING)         { reply(op, ST_BAD_STATE, expect_seq, staged()); return; }
      if (msg.len < 4 || msg.len > 8)   { reply(op, ST_BAD_STATE, expect_seq, staged()); return; }

      const uint16_t seq = uint16_t(msg.buf[1]) | (uint16_t(msg.buf[2]) << 8);
      const uint32_t n   = uint32_t(msg.len) - 3u;
      if (seq != expect_seq)            { reply(op, ST_BAD_SEQ, expect_seq, staged()); return; }
      if (staged() + n > image_len)     { reply(op, ST_TOO_BIG, seq, staged()); return; }

      /* A payload can straddle a sector boundary (n is 1..5 and need not divide
       * 4096), so take it in two bites when it does. */
      uint32_t off = 0;
      while (off < n) {
        uint32_t take = SECTOR_SIZE - buf_fill;
        if (take > n - off) take = n - off;
        memcpy(sector_buf + buf_fill, &msg.buf[3 + off], take);
        buf_fill += take;
        off      += take;
        if (buf_fill == SECTOR_SIZE && !flushBuffer()) {
          phase = P_IDLE;
          reply(op, ST_FLASH_ERR, seq, staged());
          return;
        }
      }

      last_seq   = seq;
      expect_seq = uint16_t(seq + 1u);

      const bool complete = (staged() == image_len);
      if ((seq % 16u) == 15u || complete) reply(op, ST_OK, seq, staged());
      return;
    }

    case OP_VERIFY: {
      if (phase == P_IDLE || msg.len < 5) { reply(op, ST_BAD_STATE, last_seq, 0); return; }

      if (!flushBuffer()) {               // push the tail out before hashing
        phase = P_IDLE;
        reply(op, ST_FLASH_ERR, last_seq, 0);
        return;
      }
      if (staged() != image_len) {        // short image — nothing to verify yet
        phase = P_RECEIVING;
        reply(op, ST_BAD_STATE, expect_seq, staged());
        return;
      }

      const uint32_t want = uint32_t(msg.buf[1]) | (uint32_t(msg.buf[2]) << 8)
                          | (uint32_t(msg.buf[3]) << 16) | (uint32_t(msg.buf[4]) << 24);
      arm_dcache_delete((void *)stage_base, image_len);
      const uint32_t got = crc32((const uint8_t *)stage_base, image_len);

      if (got != want) {
        phase = P_RECEIVING;
        Serial.printf("[fwupd] BAD_CRC want=0x%08lX got=0x%08lX\n",
                      (unsigned long)want, (unsigned long)got);
        reply(op, ST_BAD_CRC, last_seq, got);
        return;
      }
      if (!identityOk()) {
        phase = P_RECEIVING;
        Serial.println("[fwupd] BAD_IDENTITY: staged image carries no FW_NAME marker");
        reply(op, ST_BAD_IDENTITY, last_seq, got);
        return;
      }

      phase = P_VERIFIED;
      Serial.printf("[fwupd] VERIFY ok, crc=0x%08lX — COMMIT armed\n", (unsigned long)got);
      reply(op, ST_OK, last_seq, got);
      return;
    }

    case OP_COMMIT: {
      if (phase != P_VERIFIED) { reply(op, ST_BAD_STATE, last_seq, 0); return; }
      /* Reply BEFORE the copy and let it drain: once the erase starts, this
       * board answers nothing until it comes back up on the new image. */
      reply(op, ST_OK, last_seq, 0);
      Serial.println("[fwupd] COMMIT — copying staged image over program flash");
      delay(COMMIT_DRAIN_MS);
      commitAndReboot();       // never returns
      return;
    }

    default:
      reply(op, ST_BAD_STATE, last_seq, 0);
      return;
  }
}

/*  Called from loop().  An abandoned session (host crashed, cable pulled) would
 *  otherwise hold flash-staging state for ever — and this board has no console
 *  left to explain why — so time it out.
 *  60 s is long enough that no live transfer, and no operator pausing between
 *  VERIFY and COMMIT, can trip it. */
void tick() {
  if (phase == P_IDLE) return;
  if (millis() - last_cmd_ms > SESSION_IDLE_MS) {
    phase    = P_IDLE;
    buf_fill = 0;
    Serial.println("[fwupd] session idle — aborted");
  }
}

}  // namespace FwUpdate

/*----------------------------------------------------------------------------*/
/*                     C A N   T X   M A I L B O X E S                        */
/*----------------------------------------------------------------------------*/
/*  Free TX mailboxes, read from the FlexCAN registers (FW 11).  The decision is
 *  LoopService::Can::free_tx_mailboxes() (pure, tested natively: same formula as FlexCAN_T4's
 *  private mailboxOffset(), with setMaxMB(16) and the default RFFN: MB 8-15); this only reads
 *  the C/S words of the TX mailboxes, never the RX FIFO's (reading an RX C/S word locks it).
 *  Used by loop()'s dispatch gate and by the IMU's TX rule.  Hardware-only: never run on host. */
static_assert(LoopService::Can::CODE_TX_INACTIVE == FLEXCAN_MB_CODE_TX_INACTIVE, "loop_service.h mirrors FlexCAN_T4");
uint32_t canFreeTxMailboxes() {
  const uint32_t b = uint32_t(CAN1);
  const uint32_t maxmb = FLEXCANb_MAXMB_SIZE(b) > 64u ? 64u : FLEXCANb_MAXMB_SIZE(b);
  const bool fen = (FLEXCANb_MCR(b) & FLEXCAN_MCR_FEN) != 0;
  const uint32_t rffn = (FLEXCANb_CTRL2(b) >> FLEXCAN_CTRL2_RFFN_BIT_NO) & 0xFu;
  uint8_t codes[64];
  for (uint32_t i = LoopService::Can::mailbox_offset(maxmb, fen, rffn); i < maxmb; ++i)
    codes[i] = uint8_t(FLEXCAN_get_code(FLEXCANb_MBn_CS(b, i)));
  return LoopService::Can::free_tx_mailboxes(maxmb, fen, rffn, codes);
}

#if PLATFORM_IMU_ENABLED
/*----------------------------------------------------------------------------*/
/*                 P L A T F O R M   I M U   A D A P T E R S                  */
/*----------------------------------------------------------------------------*/
/*  ImuService (imu_service.h) decides; these are its hands.  Every Io method except stamp()
 *  and header_info() is exactly ONE Wire transaction, bounded by the core's Wire timeouts
 *  (WireIMXRT.cpp: wait_idle 16 ms then a 9-pulse force_clock(); 50 ms per transfer loop):
 *  ~0.1 ms when nothing answers, ~17 ms with SDA held low, <= ~68 ms worst case per Wire call.
 *  Those loops call yield(), which on this core runs only serialEvent/HardwareSerial/
 *  EventResponder hooks, none of which this sketch defines: never can1.events(), never canSniff. */
bool imuWireStarted = false;
struct ImuIo {
  /*  Wire is configured here, at the first probe (>= BOOT_SETTLE_MS after boot), not in
   *  setup(): a fault in Wire's set-up is then inside the region the boot settle protects. */
  void init_start() {
    if (!imuWireStarted) {
      Wire.begin();
      Wire.setClock(IMU_I2C_HZ);
      imuWireStarted = true;
    }
    imuChip.initStart();
  }
#if PLATFORM_IMU_DIAG
  /*  Diagnostic build: every Wire op runs through op(), which clears the driver's failure
   *  record, times the op, optionally masks CAN1 over it, and snapshots the LPI2C1 status
   *  flags if it failed (Wire leaves NDF/ALF/FEF/PLTF set until its next wait_idle()). */
  uint32_t d_dur_us = 0;
  uint8_t d_msr = 0;
  template <class F> int op(F f) {
    imuChip.diagClear();
    const uint32_t t0 = micros();
#if PLATFORM_IMU_DIAG_MASK_CAN
    NVIC_DISABLE_IRQ(IRQ_CAN1);
#endif
    const int r = f();
#if PLATFORM_IMU_DIAG_MASK_CAN
    NVIC_ENABLE_IRQ(IRQ_CAN1);
#endif
    d_dur_us = micros() - t0;
    if (r < 0) {
      const uint32_t m = LPI2C1_MSR;
      d_msr = uint8_t(((m & LPI2C_MSR_NDF) ? 0x01 : 0) | ((m & LPI2C_MSR_ALF) ? 0x02 : 0) |
                      ((m & LPI2C_MSR_FEF) ? 0x04 : 0) | ((m & LPI2C_MSR_PLTF) ? 0x08 : 0) |
                      ((m & LPI2C_MSR_SDF) ? 0x10 : 0) | ((m & LPI2C_MSR_MBF) ? 0x20 : 0) |
                      ((m & LPI2C_MSR_BBF) ? 0x40 : 0));
    }
    return r;
  }
  void diag_op(ImuService::DiagOp *d) {
    d->stage = imuChip.diag.stage;
    d->code = imuChip.diag.code;
    d->requested = imuChip.diag.requested;
    d->msr_flags = d_msr;
    d->dur_us = d_dur_us;
    d->init_failed_at = imuChip.diag.init_failed_at;
    d->int_status = imuChip.diag.int_status;
    d->count = imuChip.diag.count;
  }
#define IMU_OP(expr) op([&]() -> int { return (expr); })
#else
#undef IMU_OP   // PlatformIO's .ino converter preprocesses both branches; silence its redefinition warning
#define IMU_OP(expr) (expr)
#endif
  int init_step(uint32_t now_us) { return IMU_OP(imuChip.initStep(now_us)); }
  int fifo_count(bool *overflow) { return IMU_OP(imuChip.fifoCount(overflow)); }
  int read_burst(ImuService::Pkt *out, int n) {
    icm42688::Sample smp[icm42688::Driver::BURST_PKTS];
    if (n > icm42688::Driver::BURST_PKTS) n = icm42688::Driver::BURST_PKTS;
    const int got = IMU_OP(imuChip.readFifo(smp, n));   // n <= 8: one 16n-byte Wire read
    for (int i = 0; i < got; ++i) {
      for (int j = 0; j < 3; ++j) { out[i].s[j] = smp[i].a[j]; out[i].s[3 + j] = smp[i].g[j]; }
      out[i].temp = smp[i].temp;
    }
    return got;
  }
  int flush() { return IMU_OP(imuChip.flushFifo()); }
  int check_who() { return IMU_OP(imuChip.checkWhoAmI()); }
#undef IMU_OP
  /*  I2C bus clear (UM10204 3.1.16) then an LPI2C1 reset, before the next transfer after every
   *  counted error (ImuService::probe_step before a re-probe, drain_step before a RUNNING retry).  Nine SCL pulses with SDA released let a device that was
   *  left driving a data bit finish its byte and see a NACK; the STOP resets its interface;
   *  Wire.begin() writes LPI2C_MCR_RST and re-muxes both pins (the core never resets the
   *  controller otherwise, and its own 9-pulse force_clock() runs only when BBF stays set
   *  16 ms).  Bounded: ~0.11 ms of 5 us half-periods, no device transaction, no wait on the bus. */
  void bus_recover() {
    digitalWriteFast(IMU_SDA_PIN, HIGH);   // output latch high before the pins become outputs
    digitalWriteFast(IMU_SCL_PIN, HIGH);
    pinMode(IMU_SDA_PIN, OUTPUT_OPENDRAIN);
    pinMode(IMU_SCL_PIN, OUTPUT_OPENDRAIN);
    delayMicroseconds(5);
    for (int i = 0; i < 9; ++i) {
      digitalWriteFast(IMU_SCL_PIN, LOW);
      delayMicroseconds(5);
      digitalWriteFast(IMU_SCL_PIN, HIGH);
      delayMicroseconds(5);
    }
    digitalWriteFast(IMU_SCL_PIN, LOW);    // STOP: SDA low -> SCL high -> SDA high
    delayMicroseconds(5);
    digitalWriteFast(IMU_SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWriteFast(IMU_SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWriteFast(IMU_SDA_PIN, HIGH);
    delayMicroseconds(5);
    if (imuWireStarted) {                  // first probe of a boot: init_start() does the begin()
      Wire.begin();
      Wire.setClock(IMU_I2C_HZ);
    }
  }
  /*  micros64() and the wall clock at one instant.  wall_offset_us is 64-bit and written by
   *  handleSyncFrame, which after loop()'s first can1.events() runs only inside events(), in
   *  loop() context; this runs in loop() too, so the two never interleave (no torn read). */
  void stamp(ImuService::Stamp *s) {
    const uint64_t l = TimeSync::micros64();
    s->local_us = uint32_t(l);
    s->wall_us = uint64_t(int64_t(l) + TimeSync::wall_offset_us);
    s->wall_valid = TimeSync::have_offset;
  }
  void header_info(ImuCapture::HeaderInfo *h) {
    h->odr_hz = imuChip.odr();
    h->accel_lsb_per_g = uint16_t(32768u / imuChip.accelFs());                                // 2048 at 16 g
    h->gyro_lsb_per_dps_x10 = uint16_t((327680u + imuChip.gyroFs() / 2) / imuChip.gyroFs());  // 328 at 1000 dps
    h->reg_gyro_accel_config0 = icm42688::Driver::UI_FILT_CODE;
    h->reg_accel_config0 = imuChip.regAccelConfig0();
    h->reg_gyro_config0 = imuChip.regGyroConfig0();
    h->i2c_clock_100khz = uint8_t(IMU_I2C_HZ / 100000u);
    h->fw_version = FW_VERSION;
    if (IMU_I2C_HZ < 1000000u) h->extra_flags |= PlatformImuCaptureFlag::I2C_400K;
  }
};

/*  The spec's TX rule (section 4): an IMU frame goes to write() only when write() is certain
 *  to land it in a mailbox (ImuService::tx_may_place: SW queue empty, more than the reserve
 *  free), so it can never enter FlexCAN_T4's queued-frame path, whose events() writes one
 *  queued frame into every free mailbox.  The CAN interrupt is masked from the count to the
 *  write so the TX-complete ISR cannot take the mailbox in between; a few microseconds, and
 *  the 6-deep hardware RX FIFO holds what arrives meanwhile. */
struct ImuTx {
  bool place(uint32_t id, const uint8_t *d) {
    NVIC_DISABLE_IRQ(IRQ_CAN1);
    const bool ok = ImuService::tx_may_place(canFreeTxMailboxes(), can1.getTXQueueCount());
    if (ok) {
      CAN_message_t m;
      m.id = id;
      m.len = 8;
      memcpy(m.buf, d, 8);
      if (can1.write(m) != 1) imuTxQueuedAnyway = imuTxQueuedAnyway + 1;
    }
    NVIC_ENABLE_IRQ(IRQ_CAN1);
    return ok;
  }
};

ImuIo imuIo;
ImuTx imuTx;
#endif  // PLATFORM_IMU_ENABLED

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

/*  The tilt service's hands (LoopService::Tilt::Service calls these from loop(), at
 *  most one read() or probe() per pass).  Nothing here runs in the CAN callback: since
 *  FW 11 canSniff only sets tiltRequested.  No Serial output on this path either: the
 *  USB console is gone, and a write to a port that enumerated with no reader can block
 *  for up to 120 ms. */
struct TiltIo {
  platformTilt last{ 0.f, 0.f };

  /*  One read, about 1.3 ms.  Same angles and signs as FW <= 10. */
  bool read() {
    if (!inclinometer.available()) return false;
    const float ay = convertTo180Range(inclinometer.getCalculatedAngleY());
    const float az = convertTo180Range(inclinometer.getCalculatedAngleZ());
    last.tiltX = -(az)*PI / 180.f;
    last.tiltY = -(ay)*PI / 180.f;
    return true;
  }

  /*  One (re)initialisation, about 103 ms of the library's busy waits; the same
   *  begin() setup() has always used (mode 4, angle outputs enabled, WHOAMI 0xC1). */
  bool probe() { return inclinometer.begin(); }

  void reply(LoopService::Tilt::Reply r) {
    CAN_message_t m;
    m.id = tiltID;
    m.len = 8;
    if (r == LoopService::Tilt::Reply::VALID) {
      LoopService::Tilt::encode_valid(last.tiltX, last.tiltY, m.buf);
    } else {
      LoopService::Tilt::encode_invalid(r, m.buf);
    }
    can1.write(m);
  }
};

TiltIo tiltIo;
LoopService::Tilt::Service tiltService;

/*  Set by canSniff, read and cleared by loop() after the pass's dispatch.  volatile
 *  because until loop()'s first can1.events() FlexCAN_T4 runs canSniff in its ISR
 *  (struct2queueRx, isEventsUsed == 0), i.e. during the whole of setup(); after that
 *  canSniff only runs inside events(), in loop() context, so the read-and-clear in
 *  loop() has no concurrent writer. */
volatile bool tiltRequested = false;

/*----------------------------------------------------------------------------*/
/*            R O B O T  S T A T E  P A C K / U N P A C K                     */
/*----------------------------------------------------------------------------*/
/*  0x6E0 RobotState frame — dlc 8, BOTH directions.
 *
 *    byte 0     flags: bit0 is_homed, bit1 levelling_complete
 *    bytes 1-2  int16 LE  pose offset tilt X, counts (PlatformState::TILT_COUNTS_PER_RAD)
 *    bytes 3-4  int16 LE  pose offset tilt Y, counts (PlatformState::TILT_COUNTS_PER_RAD)
 *    bytes 5-6  uint16 LE FW_VERSION      ← Teensy→host ONLY (see below)
 *    byte 7     reserved, 0
 *
 *  BYTES 5-6 ARE MEANINGFUL ONLY IN THE TEENSY→HOST DIRECTION.  The can-bridge's
 *  STATE_WRITE (platform_relay.cpp `state_write`) zero-fills 5-7 and
 *  `decodeStateCANMessage` below deliberately never reads them, so a host write
 *  can never overwrite this board's own version — and FW_VERSION is a compile-time
 *  constant that lives in no mutable state, so a self-received reply (FlexCAN
 *  loopback) cannot clobber it either.
 *
 *  WHY BYTES 5-6, AND WHY THIS IS THE WHOLE DETECTION MECHANISM.  Every firmware
 *  built before 2026-07-27 executed `m.buf[5] = m.buf[6] = m.buf[7] = 0;` here,
 *  unconditionally, at the same dlc 8.  So a PRE-VERSIONING BOARD ANSWERS — with
 *  0 — rather than staying silent.  That is the property that makes the check
 *  work: the case that matters most (an un-flashed board) produces a DEFINITE wire
 *  value, not a timeout that would be indistinguishable from a CAN3 hiccup, an
 *  unpowered Platform Teensy, or a bridge that does not forward a new id.  A
 *  brand-new query frame or a dedicated RPC would have had exactly the opposite
 *  property, and would additionally have needed a CAN-BRIDGE flash to detect a
 *  stale PLATFORM flash — a second silent deployment to solve a silent deployment.
 *
 *  The dlc stays 8 in both directions, so the can-bridge's (can_id, dlc) reply
 *  correlator (udp_protocol.h PlatformFrame) is untouched, no new CAN id exists,
 *  and NOT ONE EXTRA FRAME is added to CAN3.
 */
void createStateCANMessage(const RobotState &s, CAN_message_t &m) {
  uint8_t flags = (s.is_homed << 0) | (s.levelling_complete << 1);
  int16_t x = s.pose_offset_tiltX_counts;
  int16_t y = s.pose_offset_tiltY_counts;

  m.len = 8;
  m.buf[0] = flags;
  m.buf[1] = x & 0xFF;
  m.buf[2] = x >> 8;
  m.buf[3] = y & 0xFF;
  m.buf[4] = y >> 8;
  m.buf[5] = uint8_t(FW_VERSION & 0xFF);   // firmware identity, LE low byte
  m.buf[6] = uint8_t(FW_VERSION >> 8);     // firmware identity, LE high byte
  m.buf[7] = 0;                            // reserved
}

/*  Host→Teensy state write.  Reads bytes 0-4 ONLY: bytes 5-6 are this board's
 *  own FW_VERSION on the reply path and must never be written back into `state`
 *  (see createStateCANMessage above). */
void decodeStateCANMessage(const CAN_message_t &m, RobotState &s) {
  if (m.len != 8) return;
  uint8_t f = m.buf[0];
  s.is_homed = f & 1;
  s.levelling_complete = (f >> 1) & 1;
  int16_t x = (m.buf[2] << 8) | m.buf[1];
  int16_t y = (m.buf[4] << 8) | m.buf[3];
  s.pose_offset_tiltX_counts = x;
  s.pose_offset_tiltY_counts = y;
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

  /* — Inclinometer trigger: served from loop() (FW 11), never here — */
  if (msg.id == tiltID) {
    tiltRequested = true;
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

  /* — Hand fan: the Hand ODrive's temperature broadcast — */
  if (msg.id == HAND_TEMPS_ID) {
    handFan.on_temps_frame(msg.buf, msg.len, msg.flags.remote, millis());
  }

  /* — Hand fan runtime control: always answered, so the host sees a refusal — */
  if (msg.id == HAND_FAN_CMD_ID) {
    handFan.on_command_frame(msg.buf, msg.len, msg.flags.remote);
    updateHandFan();
    sendHandFanStatus();
  }

#if PLATFORM_IMU_ENABLED
  /* — Platform IMU command: copied into a pending slot, nothing else (FW 11).  No I2C and
   * no engine work here: until loop()'s first events() this runs in FlexCAN's ISR. — */
  imuBootEntropy = (imuBootEntropy * 31u) ^ ARM_DWT_CYCCNT ^ msg.id;
  if (msg.id == PlatformCanId::IMU_CMD && !msg.flags.remote) {
    imuService.cmds().push(msg.buf, msg.len);
  }
#endif

  /* ── firmware update over CAN  ID 0x6F0 ──────────────────────────
   * See § FIRMWARE UPDATE OVER CAN.  Returns early: the id has no other
   * meaning on this board, and a BEGIN/DATA burst has no business walking
   * the timing analyser below. */
  if (msg.id == FW_UPD_CMD_ID) {
    FwUpdate::handleCommand(msg);
    return;
  }

  // debugPrintMessage(msg);
  analyzeMessageTiming(msg);
}

/*----------------------------------------------------------------------------*/
/*                                   SET‑UP                                   */
/*----------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);

  /* Identity banner — same shape as the can-bridge's `[boot] %s v%u …`
   * (Teensy_code_canbridge.ino).  Serial-only, so it is NOT the primary
   * report path (there is no serial console during a launch); the wire report
   * in createStateCANMessage is.  This line exists for the one moment a serial
   * monitor IS attached: the flash itself, where it turns the runbook's
   * "the board rebooted" evidence into "the board rebooted INTO v<N>". */
  Serial.printf("[boot] %s v%u\n", FW_NAME, (unsigned)FW_VERSION);

  /* Hand fan off first, before CAN can deliver a fan command. */
  pinMode(HAND_FAN_PIN, OUTPUT);
  digitalWrite(HAND_FAN_PIN, LOW);

  /* CAN bus */
  can1.begin();
  can1.setBaudRate(CAN_BITRATE);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);

  /* Inclinometer: OPTIONAL, so bounded (FW 11).  Everything above came up without
   * it.  At most BOOT_PROBE_ATTEMPTS begin() calls, about 103 ms each; a present
   * sensor answers the first, exactly as before.  An absent one leaves tilt
   * NOT_READY and loop() re-probes.  CAN frames arriving meanwhile are handled by
   * canSniff in FlexCAN's ISR (loop() has not called events() yet); the tilt branch
   * there only sets a flag, so no SPI runs in the ISR while begin() uses it here. */
  bool sclOk = false;
  for (uint8_t i = 0; i < LoopService::Tilt::BOOT_PROBE_ATTEMPTS && !sclOk; ++i) {
    sclOk = inclinometer.begin();
  }
  tiltService.boot(sclOk, millis());
  Serial.println(sclOk ? "SCL3300 initialised." : "SCL3300 not answering: tilt NOT_READY, re-probing from loop().");
#if PLATFORM_IMU_ENABLED
  /* Platform IMU: OPTIONAL, after CAN.  No Wire and no device traffic in setup(): loop()'s
   * imuService.pass() is inert for BOOT_SETTLE_MS (10 s), then probes one bounded
   * transaction per pass and starts Wire at the first one (ImuIo::init_start).
   * boot_id: imuBootSeq steps on every boot (RAM2 is not cleared by startup.c and survives
   * the SCB_AIRCR resets of a fault and of a firmware COMMIT) and the RTC's 32 kHz count
   * moves on between boots, so a quiet bus no longer gives the same id twice. */
  imuBootSeq = imuBootSeq * 1664525u + 1013904223u;
  uint32_t bootId = imuBootEntropy ^ (ARM_DWT_CYCCNT * 2654435761u) ^ (receivedCount << 16)
                  ^ imuBootSeq ^ (uint32_t(SNVS_HPRTCLR) * 0x9E3779B9u);
  bootId ^= bootId >> 15;
  bootId *= 0x2C1B3C6Du;
  bootId ^= bootId >> 12;
  if (bootId == 0) bootId = 1;
  imuCapture.init(imuRingBuf, PlatformImu::RING_SAMPLES, imuAnchors, PlatformImu::ANCHOR_RING, bootId);
  imuService.set_i2c_400k(IMU_I2C_HZ < 1000000u);
  imuService.init(&imuCapture, bootId, millis());
#endif
  Serial.println("Teensy platform MCU ready.");
}

/*----------------------------------------------------------------------------*/
/*                                   L O O P                                   */
/*----------------------------------------------------------------------------*/
void loop() {
  /* Dispatch CAN callbacks (loop_service.h, LoopService::Can).  During a firmware update:
   * exactly one events() per pass, as FW 10.  Otherwise up to CAN_FRAMES_PER_PASS, each only
   * while FlexCAN's software TX queue is empty and enough TX mailboxes are free for every
   * reply the frame and the rest of this pass can write. */
  uint32_t calls = 0;
  uint64_t ev = 0;
  while (LoopService::Can::may_dispatch(calls, ev, FwUpdate::sessionOpen(),
                                        canFreeTxMailboxes(), can1.getTXQueueCount())) {
    ev = can1.events();
    ++calls;
  }

  /* Tilt: at most one inclinometer read (~1.3 ms) or probe (~103 ms, rationed) per pass. */
  const bool tiltReq = tiltRequested;
  tiltRequested = false;
  tiltService.service(millis(), tiltReq, FwUpdate::sessionOpen(), tiltIo);

  reportStatus();  // traffic monitor
  FwUpdate::tick();  // time out an abandoned firmware-update session
#if PLATFORM_IMU_ENABLED
  /* Platform IMU: at most ONE bounded step (one I2C transaction, one CRC slice or one
   * IMU_DATA frame), after the dispatch and the firmware-update tick, so a BEGIN or an
   * abandoned session is seen before any I2C this pass (imu_service.h). */
  imuService.pass(micros(), millis(), FwUpdate::sessionOpen(), imuIo, imuTx);
#endif
  updateHandFan();   // apply the fan decision (fed from canSniff)

  #if DEBUG_TIME_SYNC
    TimeSync::maybePrintStats();  // console jitter read‑out, if desired.
  #endif
}
