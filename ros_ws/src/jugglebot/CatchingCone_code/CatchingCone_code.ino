/*****************************************************************************************
 *  Teensy 4.x — Jugglebot "catching cone" microcontroller
 *  ------------------------------------------------------------------
 *  Instrumented catching cone: a piezo contact sensor on the cup detects a
 *  ball landing and the cone broadcasts a precisely-timed catch event on the
 *  shared CAN bus, synced to the Jugglebot global clock.
 *
 *  Functions:
 *    • TIME-SYNC layer (ID 0x7DD)  — reused verbatim from the platform Teensy.
 *      Maintains wall-time offset (Jetson wall-time − micros64()).
 *    • Piezo impact detection — a clamped piezo signal on an interrupt-capable
 *      GPIO; the ISR latches micros64() on the rising edge.
 *    • CATCH_EVENT broadcast (ID 0x7E0) — one per impact, wall-clock timestamp.
 *    • CONE_HEARTBEAT broadcast (ID 0x7E1) — liveness + time-sync status.
 *
 *  Piezo wiring:
 *
 *      PIEZO(+) ─┬─[ R_series 1 kΩ ]─┬── PIEZO_PIN (interrupt-cap. GPIO)
 *                │                   │
 *           [R_load 57 kΩ]    [Zener 3.0-3.3 V]
 *                │             cathode → pin
 *                │              anode  → GND
 *                │                   │
 *      PIEZO(−) ─┴───────────────────┴── GND
 *
 *    One GND-referenced Zener clamps both polarities and works even with the
 *    Teensy unpowered. The 57 kΩ load shunts mains-hum pickup well below the
 *    GPIO threshold. The Teensy 4.x is NOT 5V tolerant.
 *
 *  Build: PlatformIO — `pio run -t upload` (see platformio.ini). Also builds
 *  under Arduino IDE / Teensyduino, board "Teensy 4.0" (or 4.1).
 *  protocol_config.h / hardware_config.h are auto-generated — run
 *  `python config/generate_config.py` and re-flash after any config change.
 *****************************************************************************************/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "protocol_config.h"   // Auto-generated from config/protocol_config.yaml
#include "hardware_config.h"   // Auto-generated from config/hardware_config.yaml

#define DEBUG_TIME_SYNC 0   // 1 = print periodic time-sync jitter stats
#define DEBUG_CATCH     1   // 1 = print a line on each catch event

/*----------------------------------------------------------------------------*/
/*                                CAN BUS SET-UP                              */
/*----------------------------------------------------------------------------*/
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
constexpr uint32_t CAN_BITRATE = CanBus::BAUD_RATE;

/* CAN IDs from protocol_config.h */
constexpr uint32_t timeSyncID   = SharedCanId::TIME_SYNC;          // 0x7DD (in)
constexpr uint32_t catchEventID = CatchingConeCanId::CATCH_EVENT;  // 0x7E0 (out)
constexpr uint32_t heartbeatID  = CatchingConeCanId::HEARTBEAT;    // 0x7E1 (out)

/*----------------------------------------------------------------------------*/
/*                              CONE CONFIG                                   */
/*----------------------------------------------------------------------------*/
/* From hardware_config.h (namespace CatchingCone) */
constexpr uint8_t  PIEZO_PIN           = CatchingCone::PIEZO_PIN;
constexpr uint32_t DEAD_TIME_US        = CatchingCone::DEAD_TIME_US;        // ISR refractory window
constexpr uint32_t REPORT_DELAY_US     = CatchingCone::REPORT_DELAY_US;     // CAN-send deferral after an impact
constexpr uint32_t HEARTBEAT_PERIOD_MS = CatchingCone::HEARTBEAT_PERIOD_MS;

/* flags byte bit positions — must match jugglebot/can/catching_cone.py.
 * FLAG_RETRIGGER_SUPPRESSED and FLAG_ANY_CATCH share bit value 0x02 but live
 * in different frames (CATCH_EVENT byte 5 vs HEARTBEAT byte 6) so there is
 * no clash; the names disambiguate per-frame intent. */
constexpr uint8_t FLAG_TIME_SYNCED          = 0x01;  // CATCH_EVENT byte 5 / HEARTBEAT byte 6
constexpr uint8_t FLAG_RETRIGGER_SUPPRESSED = 0x02;  // CATCH_EVENT byte 5
constexpr uint8_t FLAG_ANY_CATCH            = 0x02;  // HEARTBEAT byte 6 — set once a catch has been sent

/*----------------------------------------------------------------------------*/
/*                        ──  TIME-SYNC  LAYER ──                             */
/*  Reused from Teensy_code.ino — keep the two in sync if either changes.      */
/*----------------------------------------------------------------------------*/
namespace TimeSync {

/* 64-bit free-running microsecond counter (32-bit micros() wraps after 71 min).
 *
 * ISR-SAFE: called from BOTH piezoISR and the main loop, so the wrap detection
 * runs with interrupts masked (same idiom as BallButler ball_butler_main/
 * Micros64.h). Unguarded, the piezo ISR could advance last_lo past a main-loop
 * call's already-loaded `now`, reading as a false 32-bit wrap: hi jumped
 * +2^32 µs and the time-sync IIR then slewed wall time back over ~1 s, so any
 * catch sent inside that window carried a future-biased timestamp (bench
 * 2026-06-10: +40.1 s at 350 ms post-wrap = 2^32 µs × (7/8)^35 exactly; see
 * logbook/2026-06-10-cone-micros64-false-wrap.md). */
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

/* Wall-time offset: Jetson_wall_us − micros64() */
volatile int64_t wall_offset_us = 0;
volatile bool have_offset = false;
volatile uint64_t last_sync_us = 0;   // micros64() at last 0x7DD frame (freshness)
constexpr uint8_t ALPHA_SHIFT = TeensyOp::TIME_SYNC_ALPHA_SHIFT;  // I-filter gain = 1/2^n

// Step (don't slew) when the broadcast jumps more than this — the master stepped
// after a re-acquisition (logbook/2026-06-12-temporal-warmup-drift.md). Slewing a
// big jump at 1/8 @ 100 Hz takes ~0.5 s; a step re-locks in one frame. Normal
// inter-frame drift is < 1 µs.
constexpr int64_t TIME_STEP_THRESHOLD_US = 20'000;   // 20 ms
// Report NOT synced if no 0x7DD for this long: the master self-gates its broadcast
// when its own Jetson anchor goes stale, so silence ⇒ untrusted (drifting) clock.
constexpr uint64_t SYNC_STALE_US = 1'000'000;        // 1 s (broadcast is 100 Hz)

/* Stats for jitter read-out */
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
  void clear() { *this = {}; }
  float mean() const { return n ? float(sum) / n : 0.f; }
  float rms() const { return n ? sqrtf(float(sum_sq) / n) : 0.f; }
} stats;
uint64_t nextStats_us = 0;
constexpr uint32_t STATS_PERIOD_US = 1'000'000;
uint8_t latched_rms_us = 0;  // most recent RMS jitter, clamped to a byte (for heartbeat)

/* Convert helpers */
uint64_t get_wall_time_us() { return micros64() + wall_offset_us; }
uint64_t local_to_wall_us(uint64_t local_us) { return local_us + wall_offset_us; }

// True only if we have an offset AND a recent broadcast. Goes false when the
// master stops broadcasting (its own Jetson anchor went stale), so catches/
// heartbeats honestly flag time_synced=0 instead of carrying a drifting stamp.
bool is_time_synced() { return have_offset && (micros64() - last_sync_us < SYNC_STALE_US); }

/* Process a single 8-byte sync frame (ID 0x7DD) */
inline void handleSyncFrame(const CAN_message_t &msg) {
  uint32_t sec  = msg.buf[0] | (msg.buf[1] << 8) | (msg.buf[2] << 16) | (msg.buf[3] << 24);
  uint32_t usec = msg.buf[4] | (msg.buf[5] << 8) | (msg.buf[6] << 16) | (msg.buf[7] << 24);
  uint64_t jetson_us = uint64_t(sec) * 1'000'000ULL + usec;
  uint64_t local_us = micros64();
  int64_t offset = int64_t(jetson_us) - int64_t(local_us);

  if (!have_offset) {  // first frame → step
    wall_offset_us = offset;
    have_offset = true;
  } else {
    int64_t diff = offset - wall_offset_us;
    if (diff > TIME_STEP_THRESHOLD_US || diff < -TIME_STEP_THRESHOLD_US) {
      wall_offset_us = offset;             // master stepped (re-acquisition) → step, re-lock in 1 frame
    } else {
      wall_offset_us += diff >> ALPHA_SHIFT;   // small drift → slew with the I-filter
    }
  }
  last_sync_us = local_us;                 // freshness stamp

  int32_t delta = int32_t(offset - wall_offset_us);
  stats.add(delta);
}

/* Latch the RMS jitter once per second (and optionally print it). */
inline void updateStats() {
  uint64_t now = micros64();
  if (now < nextStats_us) return;
  nextStats_us = now + STATS_PERIOD_US;
  if (stats.n) {
    float r = stats.rms();
    latched_rms_us = (r > 255.f) ? 255 : uint8_t(lrintf(r));
#if DEBUG_TIME_SYNC
    Serial.printf("Δmean %+0.1f us | rms %.1f us | min %+d us | max %+d us | frames %lu\n",
                  (double)stats.mean(), (double)stats.rms(),
                  stats.min, stats.max, stats.n);
#endif
    stats.clear();
  }
}
}  // namespace TimeSync

/*----------------------------------------------------------------------------*/
/*                         PIEZO IMPACT DETECTION                             */
/*----------------------------------------------------------------------------*/
/* ISR-shared state. 64-bit values are read under noInterrupts() in the main
 * loop because 64-bit reads are not atomic on the Cortex-M7. */
volatile uint64_t catch_local_us      = 0;     // micros64() latched at the impact edge
volatile uint64_t last_accept_local_us = 0;    // local time of the last ACCEPTED impact
volatile bool     catch_pending       = false; // an impact awaits processing
volatile bool     retrigger_seen      = false; // extra edges during the dead-time window

/* Rising-edge ISR. The FIRST edge of an impact wins and latches the timestamp;
 * later edges within DEAD_TIME_US (piezo ring / ball bounce) are flagged as
 * retriggers, not new catches. micros64() is ISR-safe (only reads ::micros()). */
void piezoISR() {
  uint64_t now = TimeSync::micros64();
  if (now - last_accept_local_us < DEAD_TIME_US) {
    retrigger_seen = true;   // ring / bounce — same impact
    return;
  }
  last_accept_local_us = now;
  catch_local_us = now;
  retrigger_seen = false;
  catch_pending = true;
}

/*----------------------------------------------------------------------------*/
/*                          CONE STATE / HEARTBEAT                            */
/*----------------------------------------------------------------------------*/
uint8_t  cone_state      = CatchingConeState::BOOT;
uint8_t  sequence        = 0;        // monotonic per-event counter (wraps at 256)
uint8_t  last_catch_seq  = 0;
uint32_t last_catch_ms   = 0;        // millis() of the last sent catch
bool     have_any_catch  = false;
uint32_t nextHeartbeat_ms = 0;

/* Send one CATCH_EVENT frame (8 bytes, layout '<IBBH'). */
void sendCatchEvent(uint64_t local_us, bool retrigger) {
  uint8_t flags = 0;
  uint32_t low32;
  if (TimeSync::is_time_synced()) {
    flags |= FLAG_TIME_SYNCED;
    low32 = uint32_t(TimeSync::local_to_wall_us(local_us));  // low 32 bits of wall µs
  } else {
    // No time-sync yet: report the raw local timestamp. time_synced=0 tells the
    // host this is NOT comparable to wall time and must be discarded for timing.
    low32 = uint32_t(local_us);
  }
  if (retrigger) flags |= FLAG_RETRIGGER_SUPPRESSED;

  ++sequence;
  last_catch_seq = sequence;
  last_catch_ms = millis();
  have_any_catch = true;

  CAN_message_t m;
  m.id = catchEventID;
  m.len = 8;
  m.buf[0] = low32 & 0xFF;
  m.buf[1] = (low32 >> 8) & 0xFF;
  m.buf[2] = (low32 >> 16) & 0xFF;
  m.buf[3] = (low32 >> 24) & 0xFF;
  m.buf[4] = sequence;
  m.buf[5] = flags;
  m.buf[6] = 0;
  m.buf[7] = 0;
  can1.write(m);

#if DEBUG_CATCH
  Serial.printf("CATCH seq=%u  low32=0x%08lX  synced=%d  retrigger=%d\n",
                sequence, (unsigned long)low32,
                (flags & FLAG_TIME_SYNCED) ? 1 : 0, retrigger ? 1 : 0);
#endif
}

/* Send one CONE_HEARTBEAT frame (8 bytes). */
void sendHeartbeat() {
  // No catches yet → emit 0; the explicit FLAG_ANY_CATCH bit disambiguates
  // "never seen one" from a genuine "long time since the last one" reading.
  uint32_t ms_since = have_any_catch ? (millis() - last_catch_ms) : 0;
  if (ms_since > 0xFFFFFF) ms_since = 0xFFFFFF;   // clamp to the uint24 field

  uint8_t flags = 0;
  if (TimeSync::is_time_synced()) flags |= FLAG_TIME_SYNCED;
  if (have_any_catch)             flags |= FLAG_ANY_CATCH;

  CAN_message_t m;
  m.id = heartbeatID;
  m.len = 8;
  m.buf[0] = cone_state;
  m.buf[1] = TimeSync::latched_rms_us;            // sync-jitter RMS {µs}
  m.buf[2] = last_catch_seq;
  m.buf[3] = ms_since & 0xFF;
  m.buf[4] = (ms_since >> 8) & 0xFF;
  m.buf[5] = (ms_since >> 16) & 0xFF;
  m.buf[6] = flags;
  m.buf[7] = 0;
  can1.write(m);
}

/*----------------------------------------------------------------------------*/
/*                       C A N   C A L L B A C K                              */
/*----------------------------------------------------------------------------*/
void canSniff(const CAN_message_t &msg) {
  /* The cone only listens for the global-clock time-sync broadcast. */
  if (msg.id == timeSyncID && msg.len == 8) {
    TimeSync::handleSyncFrame(msg);
  }
}

/*----------------------------------------------------------------------------*/
/*                                   SET-UP                                   */
/*----------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);

  /* Piezo input. The clamp circuit holds the pin in-range; no pull-up — a
   * pull-up would fight the 57 kΩ load resistor (see wiring sketch above). */
  pinMode(PIEZO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIEZO_PIN), piezoISR, RISING);

  /* CAN bus */
  can1.begin();
  can1.setBaudRate(CAN_BITRATE);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);

  /* Running, but no global-clock lock yet. */
  cone_state = CatchingConeState::UNSYNCED;
  nextHeartbeat_ms = millis();
  Serial.println("Catching cone MCU ready (UNSYNCED — awaiting time-sync).");
}

/*----------------------------------------------------------------------------*/
/*                                   L O O P                                   */
/*----------------------------------------------------------------------------*/
void loop() {
  can1.events();          // dispatch CAN RX callbacks (canSniff)
  TimeSync::updateStats();  // latch jitter RMS once per second

  /* State: UNSYNCED → READY once the first time-sync frame has been processed. */
  if (cone_state == CatchingConeState::UNSYNCED && TimeSync::have_offset) {
    cone_state = CatchingConeState::READY;
    Serial.println("Time-sync acquired — cone READY.");
  }

  /* Catch-event processing. Two timers are at play:
   *   - REPORT_DELAY_US (~30 ms): defer the CAN send so retrigger_seen
   *     captures the immediate piezo ring within this window — sets the
   *     retrigger_suppressed flag on the outgoing frame.
   *   - DEAD_TIME_US    (~500 ms): ISR refractory; suppresses double-counts
   *     from later mechanical bounces (those do NOT set retrigger_suppressed
   *     because the frame has already gone out).
   * The timestamp is the ISR-latched first edge, so deferring the CAN frame
   * does not affect timing precision — only delivery latency. */
  if (catch_pending) {
    uint64_t local_snapshot;
    noInterrupts();
    local_snapshot = catch_local_us;
    interrupts();

    if (TimeSync::micros64() - local_snapshot >= REPORT_DELAY_US) {
      bool retrig;
      noInterrupts();
      retrig = retrigger_seen;
      catch_pending = false;
      interrupts();
      sendCatchEvent(local_snapshot, retrig);
    }
  }

  /* Heartbeat broadcast. */
  uint32_t now_ms = millis();
  if (int32_t(now_ms - nextHeartbeat_ms) >= 0) {
    nextHeartbeat_ms += HEARTBEAT_PERIOD_MS;
    sendHeartbeat();
  }
}
