#include "pin_io.h"

#include "console.h"

#include <cstdarg>
#include <cstdlib>
#include <cstring>

#include <esp_system.h>

#include "can_link.h"
#include "edge_capture.h"
#include "gpi2_drive.h"
#include "heartbeat.h"
#include "latency_test.h"
#include "pin_probe.h"
#include "pulse_stats.h"
#include "qx_protocol.h"
#include "record_ctrl.h"
#include "time_slave.h"
#include "uart_link.h"
#include "word_filter.h"

namespace {

bool   s_armed = false;  // disarmed at boot, always
bool   s_edges = false;
char   s_line[CONSOLE_LINE_MAX_BYTES + 1];
size_t s_len = 0;
bool   s_overlong = false;
uint32_t s_dropped_lines = 0;  // console output dropped instead of blocking loop()

// Headroom on top of the line itself: the IDF TX ring buffer stores writes as
// items with headers, so "free" bytes are not all usable in one write.
constexpr size_t CONSOLE_TX_HEADROOM_BYTES = 64;

// Per-pin bookkeeping for EDGE lines and status.
static_assert(NUM_CAPTURE_PINS == 3, "update the s_last_edge_us initialiser");
int64_t  s_last_edge_us[NUM_CAPTURE_PINS] = {-1, -1, -1};
uint32_t s_edge_count[NUM_CAPTURE_PINS] = {};

struct Command {
    const char* usage;
    const char* help;
};

const Command COMMANDS[] = {
    {"help", "list commands"},
    {"status", "pin levels, roles, UART assignment, rec state, ring/overflow counters"},
    {"probe [seconds]", "edge stats on all pins for a window (default 5 s), then classify each pin"},
    {"edges on|off", "stream every edge event live (EDGE lines)"},
    {"uart rx [32|27]", "route UART2 RX to Wave pin 3 (32) or 4 (27) (default: probe's UART pin, else 32)"},
    {"uart tx", "refused: no tx pin wired (Wave pin 5 not connected)"},
    {"uart hex on|off", "also print raw received bytes (UARTHEX lines)"},
    {"uart off", "detach UART2; all camera pins back to inputs"},
    {"> <text>", "send text + CRLF to the camera (refused: no tx pin wired)"},
    {">>", "send an empty line (CRLF) to the camera (refused: no tx pin wired)"},
    {"pulse", "30 Hz pulse stats over the last edges (period, jitter, width, polarity)"},
    {"can", "CAN status: rec state, bus state, rx/tx/error counters, 0x7DD age and offset stats, last intent"},
    {"ctl console|can", "hand GPI2 to the console (only from IDLE; rec/lat then work) or back to CAN"},
    {"events on|off", "print every CAN frame sent/received as hex (CAN lines; t_us = rx stamp / tx attempt)"},
    {"sim intent on|off", "BENCH ONLY: inject a local 5 Hz desired=1 intent (needs arm); off = intent stops"},
    {"rec mode edge|level", "declare the camera's GPI mode (level at boot); rec/lat need `ctl console` first"},
    {"rec toggle", "edge mode: 20 ms low pulse on GPI2; >= 500 ms high enforced (stretched)"},
    {"rec start|stop", "level mode: hold GPI2 low / release; >= 500 ms per state (stretched)"},
    {"rec assume idle|recording", "resync the firmware's guess of the camera's record state"},
    {"lat [n] [rec_s] [gap_s]", "latency test: n runs GPI2 -> 'Created new clip.' (default 10, 2 s, 3 s)"},
    {"lat abort", "stop a running latency test (rec state becomes unknown)"},
    {"qx idle|rec|sel|jog|stream|raw", "QX10001 remote packets (refused: no tx pin wired; `qx stream off` always allowed)"},
    {"tx hold low|release", "bench uart tx break (refused: no tx pin wired)"},
    {"arm", "enable transmitting to the camera (> and qx; currently unwired) and `sim intent`; disarmed at boot"},
    {"disarm", "disable transmitting; stops any qx stream and sim intent"},
};

void vout(int64_t t_us, const char* tag, const char* fmt, va_list ap) {
    static char buf[1400];  // fits an escaped 256-byte UART line plus fields
    int n = snprintf(buf, sizeof(buf), "%lld %s ", (long long)t_us, tag);
    if (n < 0) return;
    int m = vsnprintf(buf + n, sizeof(buf) - n - 1, fmt, ap);
    if (m < 0) m = 0;
    size_t len = static_cast<size_t>(n) + static_cast<size_t>(m);
    if (len > sizeof(buf) - 2) len = sizeof(buf) - 2;
    buf[len++] = '\n';
    // Never block loop(): a stalled console must not stretch a GPI2 pulse or
    // delay UART stamping. Drop and count instead.
    if (static_cast<size_t>(Serial.availableForWrite()) < len + CONSOLE_TX_HEADROOM_BYTES) {
        ++s_dropped_lines;
        return;
    }
    Serial.write(reinterpret_cast<const uint8_t*>(buf), len);
}

bool eq(const char* a, const char* b) { return a && b && strcmp(a, b) == 0; }

bool parse_u32(const char* s, uint32_t& out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    const unsigned long v = strtoul(s, &end, 10);
    if (*end) return false;
    out = static_cast<uint32_t>(v);
    return true;
}

bool parse_float(const char* s, float& out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    out = strtof(s, &end);
    return *end == '\0';
}

bool parse_pin(const char* s, uint8_t& out) {
    uint32_t v;
    if (!parse_u32(s, v) || v > 39) return false;
    out = static_cast<uint8_t>(v);
    return true;
}

// Wave pin 5 (the only pad that ever carried UART TX) is not wired, so every
// transmit command stops here before anything else. The paths below stay
// compiled for the day a TX pad is wired again (config.h UART_TX_WIRED).
bool require_tx_wired() {
    if (!UART_TX_WIRED) console::out("ERR", "no tx pin wired (Pin 5 not connected)");
    return UART_TX_WIRED;
}

bool require_armed() {
    if (!s_armed) console::out("ERR", "not armed");
    return s_armed;
}

bool require_tx() {
    if (uart_link::tx_holding()) {
        console::out("ERR", "tx hold active; nothing sent");
        return false;
    }
    if (uart_link::tx_pin() < 0) {
        console::out("ERR", "no uart tx pin; run `uart tx <pin>` first");
        return false;
    }
    return true;
}

void cmd_help() {
    console::out("INFO", "commands (%u):", (unsigned)(sizeof(COMMANDS) / sizeof(COMMANDS[0])));
    for (const Command& c : COMMANDS) console::out("INFO", "  %-26s %s", c.usage, c.help);
}

const char* reset_reason_name(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON: return "poweron";
        case ESP_RST_EXT: return "external";
        case ESP_RST_SW: return "software";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "int_wdt";
        case ESP_RST_TASK_WDT: return "task_wdt";
        case ESP_RST_WDT: return "other_wdt";
        case ESP_RST_DEEPSLEEP: return "deepsleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "sdio";
        default: return "unknown";
    }
}

void cmd_status() {
    const int64_t now = esp_timer_get_time();
    console::out("INFO",
                 "status uptime_s=%.3f armed=%d edges_stream=%d rec_mode=%s%s rec_state=%s gpi2_held_low=%d "
                 "gpi2_busy=%d lat=%s probe=%s qx_stream_hz=%lu tx_hold=%d",
                 now / 1e6, s_armed, s_edges, rec_mode_name(gpi2_drive::mode()),
                 gpi2_drive::mode_confirmed() ? "" : "(default,unset)", rec_state_name(gpi2_drive::state()),
                 gpi2_drive::pad_held_low(), gpi2_drive::busy(), latency_test::running() ? "running" : "idle",
                 pin_probe::running() ? "running" : "idle", (unsigned long)qx_sender::stream_hz(), uart_link::tx_holding());
    for (size_t i = 0; i < NUM_CAPTURE_PINS; ++i) {
        const uint8_t g = CAPTURE_PINS[i];
        const char*   use = g == PIN_GPI2                     ? "gpi2_open_drain"
                            : g == uart_link::tx_pin()        ? "uart_tx_open_drain"
                            : g == uart_link::rx_pin()        ? "uart_rx_input"
                                                              : "input";
        console::out("INFO", "status pin=%u wave=%u level=%d role=%s use=%s open_drain=%d out_signal=%d edges=%lu", g,
                     CAPTURE_WAVE_PINS[i], pin_io::read_level(g), pin_role_name(pin_probe::role(g)), use,
                     pin_io::is_open_drain(g), pin_io::out_signal(g), (unsigned long)s_edge_count[i]);
    }
    console::out("INFO", "status pin=- wave=5 use=not_wired (GPI1 dropped 2026-10-23; no uart tx pad)");
    console::out("INFO", "status uart rx=%d tx=%d tx_wired=%d hex=%d rx_bytes=%lu rx_edges_while_bridged=%lu lines_edge_stamped=%lu "
                         "lines_read_stamped=%lu",
                 uart_link::rx_pin(), uart_link::tx_pin(), UART_TX_WIRED, uart_link::hex(), (unsigned long)uart_link::rx_bytes(),
                 (unsigned long)uart_link::rx_edges_while_bridged(), (unsigned long)uart_link::lines_edge_stamped(),
                 (unsigned long)uart_link::lines_read_stamped());
    console::out("INFO", "status ring capacity=%u pending=%lu high_water=%lu overflows=%lu", (unsigned)EDGE_RING_CAPACITY,
                 (unsigned long)edge_capture::pending(), (unsigned long)edge_capture::high_water(),
                 (unsigned long)edge_capture::overflows());
    {
        const can_link::Stats cs = can_link::stats();
        const int64_t         age = time_slave::sync_age_us(now);
        console::out("INFO",
                     "status can rec_state=%s owner=%s bus=%s rx=%lu tx_ok=%lu tx_failed=%lu bus_off=%lu synced=%d "
                     "sync_age_ms=%lld offset_us=%lld",
                     wave::state_name(record_ctrl::heartbeat(now).state),
                     record_ctrl::console_owns() ? "console" : "can", can_link::bus_state_name(cs.bus_state),
                     (unsigned long)cs.rx_frames, (unsigned long)cs.tx_ok, (unsigned long)cs.tx_failed,
                     (unsigned long)cs.bus_off_events, time_slave::synced(now),
                     (long long)(age < 0 ? -1 : age / 1000), (long long)time_slave::offset_us(now));
    }
    console::out("INFO", "status console_dropped_lines=%lu", (unsigned long)s_dropped_lines);
    console::out("INFO", "status heap_free=%lu heap_min=%lu reset_reason=%s", (unsigned long)ESP.getFreeHeap(),
                 (unsigned long)ESP.getMinFreeHeap(), reset_reason_name(esp_reset_reason()));
}

void cmd_send_text(const char* text) {
    if (!require_tx_wired() || !require_armed() || !require_tx()) return;
    if (const char* c = word_filter::find_blocked_char(text)) {
        console::out("ERR", "blocked char 0x%02X at offset %u; nothing sent", static_cast<unsigned char>(*c),
                     (unsigned)(c - text));
        return;
    }
    if (const char* w = word_filter::find_blocked_word(text)) {
        console::out("ERR", "blocked word \"%s\"; nothing sent", w);
        return;
    }
    char esc[CONSOLE_LINE_MAX_BYTES * 4 + 1];
    LineAssembler::escape(text, strlen(text), esc, sizeof(esc));
    console::out("INFO", "tx \"%s\\r\\n\" (%u bytes)", esc, (unsigned)(strlen(text) + 2));
    const bool ok = uart_link::write_bytes(reinterpret_cast<const uint8_t*>(text), strlen(text)) &&
                    uart_link::write_bytes(reinterpret_cast<const uint8_t*>("\r\n"), 2);
    if (!ok) console::out("ERR", "uart write failed");
}

void cmd_uart(char** t, int n) {
    if (n >= 2 && eq(t[1], "rx")) {
        uint8_t pin;
        if (n >= 3) {
            if (!parse_pin(t[2], pin)) { console::out("ERR", "usage: uart rx [32|27]"); return; }
        } else {
            const int up = pin_probe::uart_pin();
            pin = up >= 0 ? static_cast<uint8_t>(up) : UART_DEFAULT_RX_PIN;
        }
        uart_link::set_rx(pin);
    } else if (n >= 2 && eq(t[1], "tx")) {
        if (!require_tx_wired()) return;
        uint8_t pin = UART_DEFAULT_TX_PIN;
        if (n >= 3 && !parse_pin(t[2], pin)) { console::out("ERR", "usage: uart tx [pin]"); return; }
        uart_link::set_tx(pin);
    } else if (n >= 3 && eq(t[1], "hex") && (eq(t[2], "on") || eq(t[2], "off"))) {
        uart_link::set_hex(eq(t[2], "on"));
        console::out("INFO", "uart hex=%d", uart_link::hex());
    } else if (n >= 2 && eq(t[1], "off")) {
        if (qx_sender::stream_hz() || qx_sender::busy()) qx_sender::stop_all();
        uart_link::off();
    } else {
        console::out("ERR", "usage: uart rx [32|27] | uart hex on|off | uart off   (uart tx: no tx pin wired)");
    }
}

void cmd_tx(char** t, int n) {
    if (!require_tx_wired()) return;
    if (n >= 3 && eq(t[1], "hold") && eq(t[2], "release")) {
        if (uart_link::tx_holding()) {
            uart_link::tx_hold_release();
            console::out("INFO", "tx hold released");
        } else {
            console::out("INFO", "tx hold not active");
        }
    } else if (n >= 3 && eq(t[1], "hold") && eq(t[2], "low")) {
        if (!s_armed) { console::out("ERR", "not armed"); return; }
        if (uart_link::tx_pin() < 0) { console::out("ERR", "no tx pin"); return; }
        if (qx_sender::stream_hz() || qx_sender::busy()) qx_sender::stop_all();
        uart_link::tx_hold_low();
    } else {
        console::out("ERR", "usage: tx hold low|release");
    }
}

// CAN owns GPI2 unless `ctl console` was issued (wave_camera_can_adapter.md s5.4).
bool require_console_owns() {
    if (record_ctrl::console_owns()) return true;
    console::out("ERR", "can owns gpi2; use ctl console");
    return false;
}

void cmd_rec(char** t, int n) {
    if (!record_ctrl::console_owns()) {
        // The one rec command allowed while CAN owns the pad: leaving FAULT.
        if (n >= 3 && eq(t[1], "assume") && eq(t[2], "idle") && record_ctrl::clear_fault_state()) {
            console::out("REC", "CAN FAULT cleared by console; state IDLE");
            return;
        }
        require_console_owns();
        return;
    }
    if (n >= 3 && eq(t[1], "mode") && (eq(t[2], "edge") || eq(t[2], "level"))) {
        gpi2_drive::set_mode(eq(t[2], "edge") ? RecMode::Edge : RecMode::Level);
    } else if (n >= 2 && eq(t[1], "toggle")) {
        if (latency_test::running()) { console::out("ERR", "rec refused: lat running"); return; }
        gpi2_drive::request_toggle();
    } else if (n >= 2 && (eq(t[1], "start") || eq(t[1], "stop"))) {
        if (latency_test::running()) { console::out("ERR", "rec refused: lat running"); return; }
        eq(t[1], "start") ? gpi2_drive::request_start() : gpi2_drive::request_stop();
    } else if (n >= 3 && eq(t[1], "assume") && (eq(t[2], "idle") || eq(t[2], "recording"))) {
        gpi2_drive::assume(eq(t[2], "idle") ? RecState::Idle : RecState::Recording);
    } else {
        console::out("ERR", "usage: rec mode edge|level | rec toggle | rec start|stop | rec assume idle|recording");
    }
}

void cmd_lat(char** t, int n) {
    if (!require_console_owns()) return;
    if (n >= 2 && eq(t[1], "abort")) {
        if (latency_test::running()) latency_test::abort("aborted by user");
        else console::out("INFO", "lat not running");
        return;
    }
    uint32_t runs = LAT_DEFAULT_RUNS;
    float    rec_s = LAT_DEFAULT_REC_S;
    float    gap_s = LAT_DEFAULT_GAP_S;
    if ((n >= 2 && !parse_u32(t[1], runs)) || (n >= 3 && !parse_float(t[2], rec_s)) ||
        (n >= 4 && !parse_float(t[3], gap_s))) {
        console::out("ERR", "usage: lat [n] [rec_s] [gap_s] | lat abort");
        return;
    }
    latency_test::start(runs, rec_s, gap_s);
}

void cmd_qx(char** t, int n, const char* raw_line) {
    if (n >= 2 && eq(t[1], "stream") && n >= 3 && eq(t[2], "off")) {
        qx_sender::set_stream_hz(0);  // stopping is always allowed
        return;
    }
    if (!require_tx_wired()) return;
    if (!require_armed()) return;
    if (!require_tx()) return;
    if (n >= 2 && eq(t[1], "idle")) {
        uint8_t pkt[qx::PACKET_BYTES];
        qx::build(qx::REC_IDLE, 0, 0, pkt);
        qx_sender::send_now(pkt, "idle");
    } else if (n >= 2 && eq(t[1], "rec")) {
        qx_sender::queue_press(qx::REC_PRESSED, 0, 0, "rec");
    } else if (n >= 2 && eq(t[1], "sel")) {
        uint32_t v = 1;
        if (n >= 3 && (!parse_u32(t[2], v) || v > 255)) { console::out("ERR", "usage: qx sel [0..255]"); return; }
        qx_sender::queue_press(qx::REC_IDLE, 0, static_cast<uint8_t>(v), "sel");
    } else if (n >= 3 && eq(t[1], "jog")) {
        char*      end = nullptr;
        const long v = strtol(t[2], &end, 10);
        if (*end || v < -128 || v > 127) { console::out("ERR", "usage: qx jog <-128..127>"); return; }
        qx_sender::queue_press(qx::REC_IDLE, static_cast<uint8_t>(static_cast<int8_t>(v)), 0, "jog");
    } else if (n >= 3 && eq(t[1], "stream")) {
        uint32_t hz;
        if (!parse_u32(t[2], hz) || hz == 0) { console::out("ERR", "usage: qx stream <1..100>|off"); return; }
        qx_sender::set_stream_hz(hz);
    } else if (n >= 3 && eq(t[1], "raw")) {
        const char* p = strstr(raw_line, "raw");
        p += 3;
        uint8_t pkt[qx::PACKET_BYTES];
        if (!qx::parse_hex(p, pkt)) { console::out("ERR", "qx raw needs exactly 24 hex bytes"); return; }
        if (!qx::header_ok(pkt)) {
            console::out("ERR", "qx raw header must be 51 58 14 91 4E 62 00");
            return;
        }
        if (!qx::checksum_ok(pkt)) {
            console::out("ERR", "qx raw checksum wrong: expected %02X, got %02X; nothing sent", qx::checksum(pkt),
                         pkt[qx::CHECKSUM_INDEX]);
            return;
        }
        qx_sender::send_now(pkt, "raw");
    } else {
        console::out("ERR", "usage: qx idle | qx rec | qx sel [v] | qx jog <n> | qx stream <hz>|off | qx raw <hex>");
    }
}

void cmd_can() {
    const int64_t now = esp_timer_get_time();
    record_ctrl::print_status(now);
    const can_link::Stats c = can_link::stats();
    console::out("INFO",
                 "can bus driver_up=%d state=%s tec=%lu rec=%lu rx=%lu rx_fwd_dropped=%lu rx_queue_full=%lu "
                 "tx_requests=%lu tx_queue_full=%lu tx_attempts=%lu tx_ok=%lu tx_failed=%lu tx_bus_down=%lu "
                 "tx_retries=%lu tx_timeouts=%lu tx_aborts=%lu arb_lost=%lu bus_errors=%lu err_passive=%lu bus_off=%lu "
                 "recoveries=%lu rx_pad=%d",
                 c.driver_up, can_link::bus_state_name(c.bus_state), (unsigned long)c.tx_err_counter,
                 (unsigned long)c.rx_err_counter, (unsigned long)c.rx_frames, (unsigned long)c.rx_fwd_dropped,
                 (unsigned long)c.rx_queue_full, (unsigned long)c.tx_requests, (unsigned long)c.tx_queue_full,
                 (unsigned long)c.tx_attempts, (unsigned long)c.tx_ok, (unsigned long)c.tx_failed,
                 (unsigned long)c.tx_not_running, (unsigned long)c.tx_retries, (unsigned long)c.tx_timeouts,
                 (unsigned long)c.tx_aborts, (unsigned long)c.arb_lost,
                 (unsigned long)c.bus_errors, (unsigned long)c.err_passive_events, (unsigned long)c.bus_off_events,
                 (unsigned long)c.recoveries, c.rx_pad_level);
    console::out("INFO",
                 "can heartbeat queued=%lu refused=%lu info_queued=%lu schedule_slips=%lu events=%d log_dropped=%lu",
                 (unsigned long)heartbeat::heartbeats_queued(), (unsigned long)heartbeat::heartbeats_refused(),
                 (unsigned long)heartbeat::info_queued(), (unsigned long)heartbeat::slips(), can_link::events(),
                 (unsigned long)c.log_dropped);
    const int64_t age = time_slave::sync_age_us(now);
    char          age_txt[24];
    if (age < 0) snprintf(age_txt, sizeof(age_txt), "never");
    else snprintf(age_txt, sizeof(age_txt), "%lld", (long long)(age / 1000));
    console::out("INFO",
                 "can sync anchored=%d synced=%d last_0x7dd_age_ms=%s offset_us=%lld spread_us=%lld track_err_us=%lld "
                 "accepted=%lu rejected=%lu accepted_per_s=%u rejected_per_s=%u steps=%lu",
                 time_slave::anchored(), time_slave::synced(now), age_txt, (long long)time_slave::offset_us(now),
                 (long long)time_slave::offset_spread_us(), (long long)time_slave::track_err_us(),
                 (unsigned long)time_slave::accepted(), (unsigned long)time_slave::rejected(),
                 heartbeat::accepted_per_s(), heartbeat::rejected_per_s(), (unsigned long)time_slave::steps());
    uint8_t d[8];
    wave::encode_heartbeat(record_ctrl::heartbeat(now), d);
    console::out("INFO", "can heartbeat_now %02X %02X %02X %02X %02X %02X %02X %02X", d[0], d[1], d[2], d[3], d[4],
                 d[5], d[6], d[7]);
}

void cmd_ctl(char** t, int n) {
    if (n >= 2 && eq(t[1], "console")) {
        if (latency_test::running()) {
            console::out("ERR", "ctl refused: lat running");
            return;
        }
        record_ctrl::ctl_console();
    } else if (n >= 2 && eq(t[1], "can")) {
        record_ctrl::ctl_can();
    } else {
        console::out("ERR", "usage: ctl console|can");
    }
}

void cmd_sim(char** t, int n) {
    if (n >= 3 && eq(t[1], "intent") && eq(t[2], "off")) {
        record_ctrl::sim_intent(false);  // stopping is always allowed
        return;
    }
    if (n >= 3 && eq(t[1], "intent") && eq(t[2], "on")) {
        if (!s_armed) {
            console::out("ERR", "sim intent refused: not armed (bench only; `arm` first)");
            return;
        }
        record_ctrl::sim_intent(true);
        return;
    }
    console::out("ERR", "usage: sim intent on|off   (bench only)");
}

void dispatch(char* line) {
    // Trim leading/trailing whitespace.
    while (*line == ' ' || *line == '\t') ++line;
    size_t len = strlen(line);
    while (len && (line[len - 1] == ' ' || line[len - 1] == '\t')) line[--len] = '\0';
    if (!len) return;

    char esc[CONSOLE_LINE_MAX_BYTES * 4 + 1];
    LineAssembler::escape(line, len, esc, sizeof(esc));
    console::out("INFO", "cmd: %s", esc);

    if (line[0] == '>') {
        if (eq(line, ">>")) { cmd_send_text(""); return; }
        const char* text = line + 1;
        if (*text == ' ') ++text;
        if (!*text) { console::out("ERR", "usage: > <text>   (use >> for an empty line)"); return; }
        cmd_send_text(text);
        return;
    }

    char  copy[CONSOLE_LINE_MAX_BYTES + 1];
    char* tok[8];
    int   n = 0;
    strncpy(copy, line, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';
    for (char* p = strtok(copy, " \t"); p && n < 8; p = strtok(nullptr, " \t")) tok[n++] = p;
    if (!n) return;

    if (eq(tok[0], "help")) cmd_help();
    else if (eq(tok[0], "status")) cmd_status();
    else if (eq(tok[0], "probe")) {
        uint32_t s = PROBE_DEFAULT_S;
        if ((n >= 2 && !parse_u32(tok[1], s)) || s < 1 || s > PROBE_MAX_S) {
            console::out("ERR", "usage: probe [1..%lu]", (unsigned long)PROBE_MAX_S);
        } else if (!pin_probe::start(s)) {
            console::out("ERR", "probe already running");
        }
    } else if (eq(tok[0], "edges") && n >= 2 && (eq(tok[1], "on") || eq(tok[1], "off"))) {
        s_edges = eq(tok[1], "on");
        console::out("INFO", "edges stream=%d", s_edges);
    } else if (eq(tok[0], "events") && n >= 2 && (eq(tok[1], "on") || eq(tok[1], "off"))) {
        can_link::set_events(eq(tok[1], "on"));
        console::out("INFO", "can events=%d", can_link::events());
    } else if (eq(tok[0], "can")) cmd_can();
    else if (eq(tok[0], "ctl")) cmd_ctl(tok, n);
    else if (eq(tok[0], "sim")) cmd_sim(tok, n);
    else if (eq(tok[0], "uart")) cmd_uart(tok, n);
    else if (eq(tok[0], "pulse")) pulse_stats::report();
    else if (eq(tok[0], "rec")) cmd_rec(tok, n);
    else if (eq(tok[0], "lat")) cmd_lat(tok, n);
    else if (eq(tok[0], "qx")) cmd_qx(tok, n, line);
    else if (eq(tok[0], "tx")) cmd_tx(tok, n);
    else if (eq(tok[0], "arm")) {
        s_armed = true;
        console::out("INFO", UART_TX_WIRED ? "armed=1 (> and qx may transmit to the camera)"
                                           : "armed=1 (sim intent allowed; no tx pin wired, > and qx stay refused)");
    } else if (eq(tok[0], "disarm")) {
        s_armed = false;
        qx_sender::stop_all();
        record_ctrl::sim_intent(false);
        if (uart_link::tx_holding()) {
            uart_link::tx_hold_release();
            console::out("INFO", "tx hold released");
        }
        console::out("INFO", "armed=0");
    } else {
        console::out("ERR", "unknown command \"%s\"; try help", tok[0]);
    }
}

}  // namespace

namespace console {

void begin() {
    Serial.setTxBufferSize(CONSOLE_TX_BUF_BYTES);
    Serial.begin(CONSOLE_BAUD);
}

void poll() {
    int budget = 256;
    while (budget-- > 0 && Serial.available() > 0) {
        const int c = Serial.read();
        if (c < 0) break;
        if (c == '\n' || c == '\r') {
            if (s_overlong) {
                console::out("ERR", "command longer than %u bytes ignored", (unsigned)CONSOLE_LINE_MAX_BYTES);
            } else if (s_len) {
                s_line[s_len] = '\0';
                dispatch(s_line);
            }
            s_len = 0;
            s_overlong = false;
        } else if (s_len < CONSOLE_LINE_MAX_BYTES) {
            s_line[s_len++] = static_cast<char>(c);
        } else {
            s_overlong = true;
        }
    }
}

void out(const char* tag, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vout(esp_timer_get_time(), tag, fmt, ap);
    va_end(ap);
}

void out_at(int64_t t_us, const char* tag, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vout(t_us, tag, fmt, ap);
    va_end(ap);
}

bool armed() { return s_armed; }
bool edges_streaming() { return s_edges; }

void on_edge(const EdgeEvent& ev) {
    const int i = capture_index(ev.gpio);
    if (i < 0) return;
    ++s_edge_count[i];
    const int64_t prev = s_last_edge_us[i];
    s_last_edge_us[i] = ev.t_us;
    if (!s_edges) return;
    if (prev >= 0)
        out_at(ev.t_us, "EDGE", "pin=%u wave=%u level=%u dt_us=%lld", ev.gpio, CAPTURE_WAVE_PINS[i], ev.level,
               (long long)(ev.t_us - prev));
    else
        out_at(ev.t_us, "EDGE", "pin=%u wave=%u level=%u dt_us=-", ev.gpio, CAPTURE_WAVE_PINS[i], ev.level);
}

}  // namespace console
