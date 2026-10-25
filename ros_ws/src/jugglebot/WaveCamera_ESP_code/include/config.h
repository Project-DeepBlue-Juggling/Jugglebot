#pragma once

// Wave GPIO explorer: hardware configuration and compiled-in guards.
// Pin table mirrors wave_camera_gpio_explorer.md section 1. Update both together.
// Design docs (ros_ws/docs/): wave_camera_gpio_explorer.md, wave_camera_can_adapter.md,
// wave_camera_findings.md; the short names below refer to those files.
//
// This header is Arduino-free so the native unit tests can include it.

#include <cstddef>
#include <cstdint>

// --- Camera connector pins ---------------------------------------------------
// Wiring changed 2026-10-23 (owner-approved): GPI2 moved GPIO25 -> GPIO26,
// Pin 3 moved GPIO26 -> GPIO32, Wave Pin 5 (GPI1) is no longer wired and
// GPIO25 is unused. Pin 4 stays on GPIO27.
//
// Wave pin 6 GPI2 / RS. Direct wire. The ONLY pad we ever drive, and only
// open-drain: the camera's own pull-up to VIN supplies the high level.
constexpr uint8_t PIN_GPI2 = 26;
// Wave pin 3 (GPO1, camera UART out: "Created new clip.") and pin 4 (GPO2,
// 30 Hz tick). Each via 1 kOhm series + 10 kOhm external pull-up to 3V3.
// Inputs only, always.
constexpr uint8_t PIN_CAM_P3 = 32;
constexpr uint8_t PIN_CAM_P4 = 27;
// Wave pin 5 (GPI1) has no GPIO: it was found inert (wave_camera_findings.md) and dropped.
constexpr uint8_t PIN_NONE = 0xFF;

// Every pin with an edge ISR, in console/report order.
constexpr uint8_t CAPTURE_PINS[] = {PIN_GPI2, PIN_CAM_P3, PIN_CAM_P4};
constexpr size_t  NUM_CAPTURE_PINS = sizeof(CAPTURE_PINS) / sizeof(CAPTURE_PINS[0]);

// Wave pin number for each capture pin (same order as CAPTURE_PINS); used for
// the `wave=` field in log output.
constexpr uint8_t CAPTURE_WAVE_PINS[NUM_CAPTURE_PINS] = {6, 3, 4};

// UART2 RX may be routed to Pin 3 or Pin 4. GPI2 is deliberately absent.
constexpr uint8_t UART_RX_CANDIDATE_PINS[] = {PIN_CAM_P3, PIN_CAM_P4};
constexpr uint8_t UART_DEFAULT_RX_PIN = PIN_CAM_P3;
// UART2 TX went to Wave Pin 5, which is no longer wired. The transmit code
// stays compiled but every entry point refuses while this is false.
constexpr bool    UART_TX_WIRED = false;
constexpr uint8_t UART_DEFAULT_TX_PIN = PIN_NONE;

constexpr bool is_capture_pin(uint8_t gpio) {
    return gpio == PIN_GPI2 || gpio == PIN_CAM_P3 || gpio == PIN_CAM_P4;
}
constexpr bool is_uart_rx_candidate_pin(uint8_t gpio) { return gpio == PIN_CAM_P3 || gpio == PIN_CAM_P4; }
// No pad may carry UART TX: Pin 5 is not connected, and Pins 3/4 are camera
// outputs that must never be driven.
constexpr bool is_uart_tx_candidate_pin(uint8_t gpio) {
    return UART_TX_WIRED && gpio != PIN_NONE && gpio == UART_DEFAULT_TX_PIN;
}
constexpr int capture_index(uint8_t gpio) {
    return gpio == PIN_GPI2 ? 0 : gpio == PIN_CAM_P3 ? 1 : gpio == PIN_CAM_P4 ? 2 : -1;
}
// Wave connector pin -> GPIO (-1 for pins without one, e.g. Pin 5) and back.
constexpr int gpio_for_wave_pin(uint8_t wave_pin) {
    return wave_pin == 6 ? PIN_GPI2 : wave_pin == 3 ? PIN_CAM_P3 : wave_pin == 4 ? PIN_CAM_P4 : -1;
}
constexpr int wave_pin_for_gpio(uint8_t gpio) {
    return capture_index(gpio) >= 0 ? CAPTURE_WAVE_PINS[capture_index(gpio)] : -1;
}
// The only pads the firmware may ever make an (open-drain) output.
constexpr bool is_output_allowed_pin(uint8_t gpio) { return gpio == PIN_GPI2 || is_uart_tx_candidate_pin(gpio); }

// ESP32 (WROOM-32D) sanity: no strapping pins (0, 2, 5, 12, 15), flash pins
// (6-11), UART0 (1, 3) or nonexistent pads.
constexpr bool is_strap_flash_or_uart0(uint8_t g) {
    return g == 0 || g == 1 || g == 2 || g == 3 || g == 5 || g == 12 || g == 15 || (g >= 6 && g <= 11);
}
// Pads that exist on the ESP32: 0-19, 21-23, 25-27, 32-39.
constexpr bool is_esp32_gpio(uint8_t g) {
    return g <= 19 || (g >= 21 && g <= 23) || (g >= 25 && g <= 27) || (g >= 32 && g <= 39);
}
// 34-39 are input-only and have no internal pull-up.
constexpr bool is_output_capable_gpio(uint8_t g) { return is_esp32_gpio(g) && g < 34; }
// Camera pins: ordinary bidirectional GPIOs with an internal pull-up.
constexpr bool is_safe_esp32_gpio(uint8_t g) { return is_output_capable_gpio(g) && !is_strap_flash_or_uart0(g); }

constexpr bool capture_pins_distinct() {
    for (size_t i = 0; i < NUM_CAPTURE_PINS; ++i)
        for (size_t j = i + 1; j < NUM_CAPTURE_PINS; ++j)
            if (CAPTURE_PINS[i] == CAPTURE_PINS[j]) return false;
    return true;
}
constexpr bool capture_pins_safe() {
    for (uint8_t g : CAPTURE_PINS)
        if (!is_safe_esp32_gpio(g)) return false;
    return true;
}
static_assert(capture_pins_distinct(), "camera pins must be distinct");
static_assert(capture_pins_safe(), "camera pins must be ordinary GPIOs (no strapping/flash/UART0/input-only)");
static_assert(is_output_capable_gpio(PIN_GPI2), "GPI2 must be on an output-capable GPIO");
static_assert(!is_uart_rx_candidate_pin(PIN_GPI2) && !is_uart_tx_candidate_pin(PIN_GPI2),
              "GPI2 must never be a UART pin");
static_assert(is_uart_rx_candidate_pin(UART_DEFAULT_RX_PIN), "UART RX default must be a candidate pin");
static_assert(UART_TX_WIRED || !is_uart_tx_candidate_pin(UART_DEFAULT_TX_PIN), "no TX pad while TX is unwired");
static_assert(!is_output_allowed_pin(PIN_CAM_P3) && !is_output_allowed_pin(PIN_CAM_P4),
              "camera output pins (Wave 3, 4) must stay inputs");
static_assert(gpio_for_wave_pin(6) == PIN_GPI2 && gpio_for_wave_pin(3) == PIN_CAM_P3 &&
              gpio_for_wave_pin(4) == PIN_CAM_P4 && gpio_for_wave_pin(5) == -1,
              "wave pin map out of step with the pin constants");

// --- Serial links ------------------------------------------------------------
constexpr uint32_t CONSOLE_BAUD        = 115200;  // UART0 via CP210x (COM5)
constexpr size_t   CONSOLE_TX_BUF_BYTES = 8192;
constexpr size_t   CONSOLE_LINE_MAX_BYTES = 300;
constexpr uint32_t CAMERA_UART_BAUD    = 115200;  // Wave UART, 8-N-1
constexpr uint32_t CAMERA_UART_BIT_NS  = 1000000000UL / CAMERA_UART_BAUD;  // ~8681 ns
constexpr size_t   CAMERA_UART_RX_BUF_BYTES = 4096;

// UART line assembly
constexpr size_t   UART_LINE_MAX_BYTES     = 256;   // longer -> flushed as partial
constexpr uint32_t UART_LINE_IDLE_FLUSH_MS = 200;   // no terminator for this long -> partial
// A falling edge on the RX pin after at least this much idle is a start bit
// that begins a new burst; used to stamp t_first_byte from the edge ISR.
constexpr uint32_t UART_IDLE_GAP_US        = 1000;
constexpr uint32_t UART_EDGE_STAMP_MAX_AGE_MS = 50;  // older burst starts are not trusted

// --- Edge capture --------------------------------------------------------------
constexpr size_t EDGE_RING_CAPACITY = 2048;  // events; power of two
static_assert((EDGE_RING_CAPACITY & (EDGE_RING_CAPACITY - 1)) == 0, "ring must be a power of two");
static_assert(EDGE_RING_CAPACITY >= 1024, "ring must hold at least 1024 events");
constexpr size_t EDGE_DRAIN_MAX_PER_LOOP = 256;  // bound loop() latency per pass

// --- GPI2 drive timing (wave_camera_gpio_explorer.md s5) -----------------------------------------
// Ember documents edge mode min 10 ms low / 500 ms high, level mode min 500 ms
// each way. The Wave limits are unpublished, so we stay on the safe side.
constexpr uint32_t GPI2_EDGE_LOW_MS       = 20;
constexpr uint32_t GPI2_EDGE_MIN_HIGH_MS  = 500;
constexpr uint32_t GPI2_LEVEL_MIN_STATE_MS = 500;
static_assert(GPI2_EDGE_LOW_MS >= 10, "edge-mode low pulse below Ember minimum");
static_assert(GPI2_EDGE_LOW_MS <= 100, "edge-mode low pulse unreasonably long");
static_assert(GPI2_EDGE_MIN_HIGH_MS >= 500 && GPI2_LEVEL_MIN_STATE_MS >= 500,
              "minimum GPI2 state durations below documented limits");
// An ISR edge on GPI2 is attributed to our own drive if it lands within this
// window after the command (pad RC + ISR latency are microseconds).
constexpr uint32_t GPI2_EDGE_MATCH_WINDOW_MS = 5;

// --- TX hold (bench measurement of the Rx pull-down depth) ---------------------
// `tx hold low` drives the UART TX pad low (open-drain) continuously; loop()
// releases it after this long no matter what.
constexpr uint32_t TX_HOLD_MAX_MS = 10000;

// --- Latency test ------------------------------------------------------------
constexpr uint32_t LAT_DEFAULT_RUNS   = 10;
constexpr float    LAT_DEFAULT_REC_S  = 2.0f;
constexpr float    LAT_DEFAULT_GAP_S  = 3.0f;
constexpr uint32_t LAT_MAX_RUNS       = 200;
constexpr float    LAT_MIN_REC_S      = 0.5f;
constexpr float    LAT_MAX_REC_S      = 600.0f;
constexpr float    LAT_MIN_GAP_S      = 0.5f;
constexpr float    LAT_MAX_GAP_S      = 600.0f;
constexpr uint32_t LAT_RUN_TIMEOUT_MS = 5000;
constexpr const char* LAT_MATCH_TEXT  = "Created new clip.";

// --- Probe / classifier (wave_camera_gpio_explorer.md s4) ------------------------------------------
constexpr uint32_t PROBE_DEFAULT_S        = 5;
constexpr uint32_t PROBE_MAX_S            = 600;
constexpr uint32_t PULSE_MIN_EDGES        = 20;
constexpr float    PULSE_MAX_PERIOD_CV    = 0.05f;
constexpr uint32_t PULSE_PERIOD_MIN_US    = 30000;
constexpr uint32_t PULSE_PERIOD_MAX_US    = 36000;
constexpr uint32_t UART_MIN_PULSE_LO_US   = 6;
constexpr uint32_t UART_MIN_PULSE_HI_US   = 12;
constexpr uint32_t UART_BURST_GAP_US      = 1000;
constexpr size_t   PULSE_HISTORY_EDGES    = 128;  // per pin, for `pulse`

// --- QX10001 remote protocol --------------------------------------------------
constexpr size_t   QX_PACKET_BYTES     = 24;
constexpr uint32_t QX_PRESS_PACKETS    = 3;
constexpr uint32_t QX_PRESS_GAP_MS     = 10;
constexpr uint32_t QX_STREAM_MAX_HZ    = 100;

// --- Transmit guard (wave_camera_gpio_explorer.md s5). No runtime override exists. ---------------
constexpr const char* BLOCKED_WORDS[] = {
    "format", "erase", "delete", "update", "reset", "factory",
    "wipe",   "rm",    "mkfs",   "flash",  "upgrade",
    "reboot", "poweroff", "shutdown", "halt", "dd", "mv", "kill",
};
// Characters never sent by `>` (shell metacharacters and quoting). Bytes
// outside printable ASCII 0x20..0x7E are refused as well.
constexpr const char BLOCKED_CHARS[] = "\\\"'$`;|&<>";
constexpr size_t NUM_BLOCKED_WORDS = sizeof(BLOCKED_WORDS) / sizeof(BLOCKED_WORDS[0]);

// =============================================================================
// CAN phase (wave_camera_can_adapter.md s3-s5). Every timeout and rate in the design lives
// here with its unit in the name.
// =============================================================================

// --- Identity ------------------------------------------------------------------
constexpr uint16_t FW_VERSION   = 1;  // WAVE_INFO page 0; bump on every flashed change
constexpr uint8_t  WIRE_VERSION = 1;  // wave_camera_can_adapter.md s4 frame layout

// --- CAN pins (SN65HVD230). Not camera pins, so is_safe_esp32_gpio() does not
// apply: RX may be an input-only pad because the transceiver drives it.
constexpr uint8_t PIN_CAN_TX = 13;
constexpr uint8_t PIN_CAN_RX = 34;
constexpr bool is_ok_can_pin(uint8_t g, bool input_only_ok) {
    return is_esp32_gpio(g) && !is_strap_flash_or_uart0(g) && !is_capture_pin(g) && (input_only_ok || g < 34);
}
static_assert(is_ok_can_pin(PIN_CAN_TX, false), "CAN TX must be an ordinary output-capable GPIO");
static_assert(is_ok_can_pin(PIN_CAN_RX, true), "CAN RX must not be a strapping/flash/UART0/camera pin");
static_assert(PIN_CAN_TX != PIN_CAN_RX, "CAN TX and RX must differ");

// --- Bus -------------------------------------------------------------------------
constexpr uint32_t CAN_BITRATE_BPS     = 1000000;  // classic CAN, 11-bit IDs, DLC 8
constexpr uint8_t  CAN_DLC             = 8;
constexpr uint32_t CAN_RX_QUEUE_LEN    = 32;   // TWAI driver RX queue (frames)
constexpr uint32_t CAN_TX_QUEUE_LEN    = 4;    // TWAI driver TX queue; we keep one in flight
constexpr uint32_t CAN_RX_FWD_QUEUE_LEN = 64;  // stamped frames handed from the RX task to loop()
constexpr uint32_t CAN_TX_REQ_QUEUE_LEN = 16;  // frames queued by loop() for the TX task
constexpr uint32_t CAN_LOG_QUEUE_LEN   = 64;   // `events on` records (dropped and counted when full)
constexpr uint32_t CAN_RX_BLOCK_MS     = 100;  // RX task twai_receive timeout
constexpr uint32_t CAN_TX_DONE_TIMEOUT_MS = 20;  // wait for TX_SUCCESS / TX_FAILED per attempt
constexpr uint32_t CAN_BUS_OFF_RETRY_MS = 1000;  // re-initiate recovery if still bus-off
constexpr uint32_t CAN_STATUS_POLL_MS  = 10;   // TX task idle wake-up for alerts / bus state
constexpr uint8_t  CAN_TASK_CORE       = 0;    // loop() runs on core 1
constexpr uint32_t CAN_RX_TASK_PRIORITY = 3;   // above TX: the RX stamp must not wait out a TX time slice
constexpr uint32_t CAN_TX_TASK_PRIORITY = 2;   // above loop() (1)
static_assert(CAN_RX_TASK_PRIORITY > CAN_TX_TASK_PRIORITY, "RX stamping must preempt the TX task");
constexpr uint32_t CAN_TASK_STACK_BYTES = 4096;
constexpr uint8_t  REC_EVENT_TX_ATTEMPTS = 3;  // WAVE_REC_EVENT attempts (heartbeat / INFO: 1)
constexpr uint8_t  REC_EVENT_TX_RETRIES = REC_EVENT_TX_ATTEMPTS - 1;
constexpr uint32_t CAN_TX_RETRY_DELAY_MS = 5;  // between REC_EVENT attempts; the age is recomputed each time
constexpr size_t   CAN_RX_DRAIN_MAX_PER_LOOP = 32;

// --- Rates -------------------------------------------------------------------------
constexpr uint32_t HEARTBEAT_RATE_HZ   = 10;   // WAVE_HEARTBEAT, from boot, unconditionally
constexpr uint32_t HEARTBEAT_PERIOD_US = 1000000UL / HEARTBEAT_RATE_HZ;
// WAVE_INFO: one page per slot, 4 pages, so each page repeats at 1 Hz
// (wave_camera_can_adapter.md s4.4 table and bring-up row 1).
constexpr uint32_t INFO_NUM_PAGES      = 4;
constexpr uint32_t INFO_PAGE_PERIOD_MS = 1000 / INFO_NUM_PAGES;
constexpr uint32_t SCHEDULE_RESYNC_PERIODS = 5;  // a periodic sender this far behind re-bases instead of bursting
constexpr uint32_t INTENT_RATE_HZ      = 5;    // the node's WAVE_CMD stream; also `sim intent`
constexpr uint32_t INTENT_PERIOD_MS    = 1000 / INTENT_RATE_HZ;
constexpr uint32_t RATE_WINDOW_MS      = 1000; // per-second counters in WAVE_INFO

// --- Record controller (wave_camera_can_adapter.md s5.2, s5.3) ---------------------------------
constexpr uint32_t INTENT_TIMEOUT_MS        = 3000;  // RECORDING/STARTING with no WAVE_CMD -> AUTO_STOP
constexpr uint32_t CAMERA_ON_TICK_WINDOW_MS = 200;   // CAMERA_ON = a tick within this window
constexpr uint32_t TICK_LOST_MS             = 200;   // RECORDING with no tick -> AUTO_STOP TICK_LOST
constexpr uint32_t TICK_AFTER_EDGE_MS       = 50;    // no tick this long after a GPI2 edge -> stamp the edge
constexpr uint32_t CLIP_WAIT_MS             = 150;   // "Created new clip." deadline after the start tick
constexpr uint32_t CLIP_WAIT_AFTER_TICK_MS  = CLIP_WAIT_MS;  // wave_camera_can_adapter.md s5.1 name
constexpr uint32_t CLIP_LATENCY_MIN_US      = 10000; // start tick -> first clip byte plausibility window
constexpr uint32_t CLIP_LATENCY_MAX_US      = 25000;
constexpr uint32_t START_EDGE_TIMEOUT_MS    = 1000;  // 500 ms stretch + margin: no falling edge -> GPI2_FAULT
constexpr uint32_t STOP_EDGE_TIMEOUT_MS     = 1000;  // no rising edge after release -> FAULT
constexpr uint32_t TICK_MIN_GAP_US          = 1000;  // a Pin 4 (GPIO27) edge this long after the previous one is a tick
constexpr uint32_t TICK_NOMINAL_PERIOD_US   = 33334; // wave_camera_findings.md: 30.000 Hz
constexpr size_t   TICK_STATS_PERIODS       = 30;    // WAVE_INFO page 1 window
static_assert(CLIP_WAIT_MS > CLIP_LATENCY_MAX_US / 1000, "clip wait must cover the latency window");
static_assert(TICK_AFTER_EDGE_MS * 1000 > TICK_NOMINAL_PERIOD_US, "tick wait must exceed one tick period");
static_assert(START_EDGE_TIMEOUT_MS > GPI2_LEVEL_MIN_STATE_MS, "start timeout must cover the GPI2 stretch");
static_assert(STOP_EDGE_TIMEOUT_MS > GPI2_LEVEL_MIN_STATE_MS, "stop timeout must cover the GPI2 stretch");

// --- Time slave (wave_camera_can_adapter.md s5.5) --------------------------------------------
constexpr uint32_t TIME_SYNC_CAN_ID       = 0x7DD;
constexpr uint32_t SYNC_MIN_UNIX_S        = 1767225600UL;  // 2026-01-01T00:00:00Z
constexpr uint32_t SYNC_WINDOW_MS         = 1000;  // max-offset window (~100 beacons)
constexpr uint32_t SYNC_ANCHOR_FRAMES     = 10;    // first anchor = max of the first 10 valid frames
constexpr size_t   SYNC_HISTORY_WINDOWS   = 8;     // least-squares line through the last 8 window maxima
constexpr uint32_t SYNC_STEP_US           = 2000;  // window max this far off the prediction ...
constexpr uint32_t SYNC_STEP_WINDOWS      = 2;     // ... this many windows in a row -> re-anchor
constexpr uint32_t SYNC_EARLY_REJECT_US   = 1000;  // a frame this far ahead of the prediction is rejected
constexpr uint32_t SYNC_LATE_WINDOW_US    = 500;   // a window max this far below prediction is not tracked
constexpr uint32_t SYNC_LATE_REANCHOR_WINDOWS = 10;  // ... unless it persists this many windows
constexpr uint32_t SYNC_STALE_MS          = 5000;  // TIME_SYNCED = anchored and last accepted beacon younger
constexpr double   SYNC_MAX_SLOPE_PPM     = 200.0; // clamp on the fitted skew
