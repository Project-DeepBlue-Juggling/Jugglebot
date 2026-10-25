#include "uart_link.h"

#include <soc/gpio_sig_map.h>

#include "console.h"
#include "pin_probe.h"

namespace {

int                 s_rx = -1;
int                 s_tx = -1;
bool                s_active = false;
bool                s_hex = false;
uart_link::LineHook s_hook = nullptr;

LineAssembler s_asm;
UartLine      s_line;  // static: ~1.3 KB, keep it off the loop stack

int64_t  s_last_rx_edge_us = -1;
int64_t  s_burst_start_us = -1;
int64_t  s_prev_line_end_us = -1;
int64_t  s_line_first_us = 0;
bool     s_line_first_edge = false;
uint32_t s_rx_bytes = 0;
uint32_t s_rx_edges_bridged = 0;
uint32_t s_lines_edge = 0;
uint32_t s_lines_read = 0;

constexpr size_t HEX_CHUNK_BYTES = 32;
uint8_t          s_hex_buf[HEX_CHUNK_BYTES];
size_t           s_hex_len = 0;
int64_t          s_hex_t_us = 0;

void flush_hex() {
    if (!s_hex_len) return;
    static const char HEX_DIGITS[] = "0123456789ABCDEF";
    char   buf[HEX_CHUNK_BYTES * 3 + 1];
    size_t w = 0;
    for (size_t i = 0; i < s_hex_len; ++i) {
        buf[w++] = HEX_DIGITS[s_hex_buf[i] >> 4];
        buf[w++] = HEX_DIGITS[s_hex_buf[i] & 0xF];
        buf[w++] = ' ';
    }
    buf[w ? w - 1 : 0] = '\0';
    console::out("UARTHEX", "t_first_us=%lld n=%u %s", (long long)s_hex_t_us, (unsigned)s_hex_len, buf);
    s_hex_len = 0;
}

void emit_line(const UartLine& line) {
    const bool partial = line.end != LineEnd::Eol;
    char       eol[24];
    if (partial) snprintf(eol, sizeof(eol), "-");
    else snprintf(eol, sizeof(eol), "%lld", (long long)line.t_last_us);
    const int64_t t_first = line.raw_len ? s_line_first_us : line.t_last_us;
    const bool    from_edge = line.raw_len && s_line_first_edge;
    if (line.raw_len) (from_edge ? s_lines_edge : s_lines_read)++;
    console::out("UART", "t_first_byte_us=%lld src=%s t_eol_us=%s end=%s bytes=%u | %s", (long long)t_first,
                 from_edge ? "edge" : "read", eol, line_end_name(line.end), (unsigned)line.raw_len, line.text);
    s_prev_line_end_us = line.t_last_us;
    if (s_hook) s_hook(line, t_first, line.t_last_us);
}

// Pick the first-byte stamp for a line whose first byte was read at t_read:
// the start-bit falling edge of the current burst if it is recent and newer
// than the previous line, else the read time.
void stamp_line_start(int64_t t_read) {
    const int64_t b = s_burst_start_us;
    const bool fresh = b >= 0 && b <= t_read && b > s_prev_line_end_us &&
                       (t_read - b) <= static_cast<int64_t>(UART_EDGE_STAMP_MAX_AGE_MS) * 1000;
    s_line_first_edge = fresh;
    s_line_first_us = fresh ? b : t_read;
    if (fresh) s_burst_start_us = -1;  // one burst start per line
}

bool                 s_hold = false;
volatile bool        s_hold_timed_out = false;  // set by the timer callback, consumed in poll()
volatile int         s_hold_pin = -1;
esp_timer_handle_t   s_hold_timer = nullptr;

// Runs in the esp_timer task: register-level pad release only, no console.
void hold_timer_cb(void*) {
    const int p = s_hold_pin;
    if (p >= 0) pin_io::uart_tx_hold_end(static_cast<uint8_t>(p));
    s_hold_timed_out = true;
}

void hold_timer_stop() {
    if (s_hold_timer) esp_timer_stop(s_hold_timer);  // ESP_ERR_INVALID_STATE if idle: fine
}

bool restart(int new_rx, int new_tx) {
    hold_timer_stop();  // the callback must not fire after a reconfigure
    s_hold_timed_out = false;
    if (s_hold) console::out("INFO", "tx hold ended by uart reconfigure");
    s_hold = false;  // teardown below resets the pad; never leave a stale hold
    // Tear down the previous bridge, leaving every pin a pulled-up input.
    if (s_tx >= 0) pin_io::release_uart_tx(static_cast<uint8_t>(s_tx));
    if (s_active) {
        Serial2.end();
        s_active = false;
    }
    if (s_tx >= 0) pin_io::configure(static_cast<uint8_t>(s_tx), PinDrive::InputPullup);
    if (s_rx >= 0) pin_io::configure(static_cast<uint8_t>(s_rx), PinDrive::InputPullup);
    s_rx = s_tx = -1;
    if (new_rx < 0 && new_tx < 0) return true;

    if (new_tx >= 0 && !pin_io::prepare_uart_tx(static_cast<uint8_t>(new_tx))) {
        console::out("ERR", "uart tx pin %d could not be made open-drain; bridge left off", new_tx);
        pin_io::configure(static_cast<uint8_t>(new_tx), PinDrive::InputPullup);
        return false;
    }
    Serial2.setRxBufferSize(CAMERA_UART_RX_BUF_BYTES);
    // rxfifo_full_thrhd = 1: the driver hands over every byte as it lands, so
    // read-time stamps are tight (the RX timeout covers the idle case anyway).
    Serial2.begin(CAMERA_UART_BAUD, SERIAL_8N1, static_cast<int8_t>(new_rx), static_cast<int8_t>(new_tx), false,
                  20000UL, 1);
    s_active = true;
    s_rx = new_rx;
    s_tx = new_tx;
    if (s_rx >= 0) pin_io::finish_uart_rx(static_cast<uint8_t>(s_rx));
    if (s_tx >= 0 && !pin_io::finish_uart_tx(static_cast<uint8_t>(s_tx))) {
        console::out("ERR", "uart tx pin %d failed open-drain verification (od=%d sig=%d); tx detached", s_tx,
                     pin_io::is_open_drain(static_cast<uint8_t>(s_tx)),
                     pin_io::out_signal(static_cast<uint8_t>(s_tx)));
        const int rx = s_rx;
        restart(-1, -1);
        if (rx >= 0) restart(rx, -1);
        return false;
    }
    s_burst_start_us = -1;
    s_last_rx_edge_us = -1;
    return true;
}

void report_assignment() {
    if (s_tx >= 0) {
        const uint8_t t = static_cast<uint8_t>(s_tx);
        console::out("INFO", "uart rx=%d tx=%d tx_open_drain=%d tx_signal=%d(U2TXD=%d) tx_idle_level=%d", s_rx, s_tx,
                     pin_io::is_open_drain(t), pin_io::out_signal(t), U2TXD_OUT_IDX, pin_io::read_level(t));
    } else if (s_rx >= 0) {
        console::out("INFO", "uart rx=%d tx=- (receive only)", s_rx);
    } else {
        console::out("INFO", "uart off");
    }
}

}  // namespace

namespace uart_link {

void set_line_hook(LineHook hook) { s_hook = hook; }

bool set_rx(uint8_t gpio) {
    if (gpio == PIN_GPI2) {
        console::out("ERR", "uart rx refused on %u: it is the GPI2 output (Wave pin 6); use 32|27", gpio);
        return false;
    }
    if (!is_uart_rx_candidate_pin(gpio)) {
        console::out("ERR", "uart rx must be one of 32|27 (Wave pin 3|4)");
        return false;
    }
    if (s_tx == gpio) console::out("WARN", "rx == tx pin %u: loopback, our own TX will be received", gpio);
    const bool ok = restart(gpio, s_tx);
    report_assignment();
    return ok;
}

bool set_tx(uint8_t gpio) {
    if (!UART_TX_WIRED) {
        console::out("ERR", "no tx pin wired (Pin 5 not connected)");
        return false;
    }
    if (!is_uart_tx_candidate_pin(gpio)) {
        console::out("ERR", "uart tx pin %u is not a wired tx pad", gpio);
        return false;
    }
    const PinRole r = pin_probe::role(gpio);
    if (role_is_camera_output(r)) {
        console::out("ERR", "uart tx refused on pin %u: probe classified it as %s (camera output)", gpio,
                     pin_role_name(r));
        return false;
    }
    if (s_rx == gpio) console::out("WARN", "rx == tx pin %u: loopback, our own TX will be received", gpio);
    const bool ok = restart(s_rx, gpio);
    report_assignment();
    return ok;
}

bool tx_hold_low() {
    if (!s_active || s_tx < 0) return false;
    if (s_hold) return true;
    s_hold_timed_out = false;
    if (!pin_io::uart_tx_hold_low(static_cast<uint8_t>(s_tx))) {
        pin_io::uart_tx_hold_end(static_cast<uint8_t>(s_tx));  // back to a known idle state
        console::out("ERR", "tx hold failed on pin %d; restored", s_tx);
        return false;
    }
    s_hold = true;
    s_hold_timed_out = false;
    s_hold_pin = s_tx;
    if (!s_hold_timer) {
        esp_timer_create_args_t a = {};
        a.callback = hold_timer_cb;
        a.name = "tx_hold";
        if (esp_timer_create(&a, &s_hold_timer) != ESP_OK) s_hold_timer = nullptr;
    }
    if (!s_hold_timer || esp_timer_start_once(s_hold_timer, static_cast<uint64_t>(TX_HOLD_MAX_MS) * 1000) != ESP_OK) {
        s_hold = false;
        pin_io::uart_tx_hold_end(static_cast<uint8_t>(s_tx));
        console::out("ERR", "tx hold timer failed; restored");
        return false;
    }
    console::out("INFO", "tx hold low pin=%d auto_release_s=%lu", s_tx, (unsigned long)(TX_HOLD_MAX_MS / 1000));
    return true;
}

void tx_hold_release() {
    if (!s_hold) return;
    hold_timer_stop();
    s_hold_timed_out = false;
    s_hold = false;
    if (s_tx < 0) return;
    if (!pin_io::uart_tx_hold_end(static_cast<uint8_t>(s_tx))) {
        console::out("ERR", "uart tx pin %d failed open-drain verification after hold; tx detached", s_tx);
        const int rx = s_rx;
        restart(-1, -1);
        if (rx >= 0) restart(rx, -1);
    }
    report_assignment();
}

bool tx_holding() { return s_hold; }

void off() {
    flush_hex();
    restart(-1, -1);
    report_assignment();
}

int  rx_pin() { return s_rx; }
int  tx_pin() { return s_tx; }
void set_hex(bool on) {
    flush_hex();
    s_hex = on;
}
bool hex() { return s_hex; }

void on_edge(const EdgeEvent& ev) {
    if (s_rx < 0 || ev.gpio != s_rx) return;
    if (s_active) ++s_rx_edges_bridged;
    if (ev.level == 0 &&
        (s_last_rx_edge_us < 0 || ev.t_us - s_last_rx_edge_us > static_cast<int64_t>(UART_IDLE_GAP_US)))
        s_burst_start_us = ev.t_us;
    s_last_rx_edge_us = ev.t_us;
}

void poll(int64_t now_us) {
    if (s_hold_timed_out) {
        s_hold_timed_out = false;
        if (s_hold) tx_hold_release();
        console::out("INFO", "tx hold released (timeout)");
    }
    if (!s_active || s_rx < 0) return;
    int budget = 512;  // bound one pass; the driver buffers the rest
    while (budget-- > 0 && Serial2.available() > 0) {
        const int c = Serial2.read();
        if (c < 0) break;
        const int64_t t = esp_timer_get_time();
        ++s_rx_bytes;
        if (s_hex) {
            if (s_hex_len == 0) s_hex_t_us = t;
            s_hex_buf[s_hex_len++] = static_cast<uint8_t>(c);
            if (s_hex_len == HEX_CHUNK_BYTES) flush_hex();
        }
        if (s_asm.empty() && c != '\r' && c != '\n') stamp_line_start(t);
        if (s_asm.feed(static_cast<uint8_t>(c), t, s_line)) emit_line(s_line);
    }
    flush_hex();
    if (s_asm.poll_timeout(now_us, s_line)) emit_line(s_line);
}

bool write_bytes(const uint8_t* data, size_t len) {
    if (!s_active || s_tx < 0 || s_hold) return false;
    return Serial2.write(data, len) == len;
}

uint32_t rx_bytes() { return s_rx_bytes; }
uint32_t rx_edges_while_bridged() { return s_rx_edges_bridged; }
uint32_t lines_edge_stamped() { return s_lines_edge; }
uint32_t lines_read_stamped() { return s_lines_read; }

}  // namespace uart_link
