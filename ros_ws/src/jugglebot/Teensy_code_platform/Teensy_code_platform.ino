/*****************************************************************************************
 *  Teensy 4.0 — Jugglebot “platform” microcontroller
 *  ------------------------------------------------------------------
 *  Functions already present:
 *    • CAN traffic monitor (ID 0x7DF)
 *    • Optional debugging of specific CAN IDs
 *    • Timed analysis of RxSdo messages
 *    • SCL3300 inclinometer → CAN (ID 0x7DE)
 *    • Robot state exchange (ID 0x6E0) — also reports this firmware's
 *      FW_VERSION in bytes 5-6; see the FIRMWARE IDENTITY block below and
 *      ros_ws/docs/platform_fw_version.md
 *    • TIME‑SYNC layer (ID 0x7DD)
 *      – Maintains wall‑time offset (Jetson wall‑time − micros64())
 *      – Prints jitter stats once per second
 *
 *  INVARIANT I-FW-15 (since FW 7, skill-stack R1).  The can-bridge is the SOLE
 *  axis-6 writer: this board neither transmits to CAN node 6 nor subscribes to
 *  its encoder.  This board's TX set is the inclinometer frame (0x7DE), the
 *  RobotState reply (0x6E0), the firmware-update reply (0x6F1) and the traffic
 *  report (0x7DF) — nothing else.  The stroke engine (Trajectory.h, the 0x6D0
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
 */
constexpr char     FW_NAME[]  = "jugglebot-platform";
constexpr uint16_t FW_VERSION = 7;   // 1→2: 2026-07-29 throwDecelToTorque (post-release decel
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
/*  0x6E0 RobotState frame — dlc 8, BOTH directions.
 *
 *    byte 0     flags: bit0 is_homed, bit1 levelling_complete
 *    bytes 1-2  int16 LE  pose_offset_tiltX * 1000  (rad)
 *    bytes 3-4  int16 LE  pose_offset_tiltY * 1000  (rad)
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
  int16_t x = int16_t(s.pose_offset_tiltX * 1000.f);
  int16_t y = int16_t(s.pose_offset_tiltY * 1000.f);

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
  FwUpdate::tick();  // time out an abandoned firmware-update session

  #if DEBUG_TIME_SYNC
    TimeSync::maybePrintStats();  // console jitter read‑out, if desired.
  #endif
}
