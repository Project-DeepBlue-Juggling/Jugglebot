#include "qx_protocol.h"

#include "console.h"
#include "uart_link.h"

namespace {

constexpr size_t QUEUE_MAX = 2 * QX_PRESS_PACKETS;

uint8_t     s_queue[QUEUE_MAX][qx::PACKET_BYTES];
size_t      s_q_len = 0;
size_t      s_q_pos = 0;
int64_t     s_next_us = 0;
const char* s_label = "";

uint32_t s_stream_hz = 0;
int64_t  s_stream_next_us = 0;
uint32_t s_stream_sent = 0;
int64_t  s_stream_report_us = 0;
constexpr int64_t STREAM_REPORT_US = 5000000;

bool write_packet(const uint8_t* pkt) { return uart_link::write_bytes(pkt, qx::PACKET_BYTES); }

}  // namespace

namespace qx_sender {

bool send_now(const uint8_t pkt[qx::PACKET_BYTES], const char* label) {
    char hex[qx::PACKET_BYTES * 3];
    console::out("QX", "tx %s %s", label, qx::to_hex(pkt, hex, sizeof(hex)));
    if (!write_packet(pkt)) {
        console::out("ERR", "qx write failed (no uart tx?)");
        return false;
    }
    return true;
}

bool queue_press(uint8_t rec, uint8_t jog, uint8_t sel, const char* label) {
    if (s_q_pos < s_q_len) {
        console::out("ERR", "qx press already in progress");
        return false;
    }
    for (size_t i = 0; i < QX_PRESS_PACKETS; ++i) qx::build(rec, jog, sel, s_queue[i]);
    for (size_t i = QX_PRESS_PACKETS; i < 2 * QX_PRESS_PACKETS; ++i) qx::build(qx::REC_IDLE, 0, 0, s_queue[i]);
    s_q_len = 2 * QX_PRESS_PACKETS;
    s_q_pos = 0;
    s_label = label;
    s_next_us = esp_timer_get_time();
    return true;
}

void set_stream_hz(uint32_t hz) {
    if (hz > QX_STREAM_MAX_HZ) {
        console::out("WARN", "qx stream capped at %lu Hz", (unsigned long)QX_STREAM_MAX_HZ);
        hz = QX_STREAM_MAX_HZ;
    }
    s_stream_hz = hz;
    s_stream_sent = 0;
    s_stream_next_us = esp_timer_get_time();
    s_stream_report_us = s_stream_next_us + STREAM_REPORT_US;
    if (hz) {
        uint8_t pkt[qx::PACKET_BYTES];
        char    hex[qx::PACKET_BYTES * 3];
        qx::build(qx::REC_IDLE, 0, 0, pkt);
        console::out("QX", "stream on hz=%lu packet %s", (unsigned long)hz, qx::to_hex(pkt, hex, sizeof(hex)));
    } else {
        console::out("QX", "stream off");
    }
}

uint32_t stream_hz() { return s_stream_hz; }

void stop_all() {
    s_q_len = s_q_pos = 0;
    if (s_stream_hz) set_stream_hz(0);
}

void poll(int64_t now) {
    // A queued press takes priority; the stream resumes after it.
    if (s_q_pos < s_q_len) {
        if (now >= s_next_us) {
            const bool pressed = s_q_pos < QX_PRESS_PACKETS;
            char       label[32];
            snprintf(label, sizeof(label), "%s %s %u/%u", s_label, pressed ? "press" : "release",
                     (unsigned)(s_q_pos + 1), (unsigned)s_q_len);
            if (!send_now(s_queue[s_q_pos], label)) {
                s_q_len = s_q_pos = 0;
                return;
            }
            ++s_q_pos;
            s_next_us = now + QX_PRESS_GAP_MS * 1000;
        }
        return;
    }
    if (s_stream_hz && now >= s_stream_next_us) {
        uint8_t pkt[qx::PACKET_BYTES];
        qx::build(qx::REC_IDLE, 0, 0, pkt);
        if (!write_packet(pkt)) {
            console::out("ERR", "qx stream write failed; stream off");
            s_stream_hz = 0;
            return;
        }
        ++s_stream_sent;
        s_stream_next_us += 1000000 / s_stream_hz;
        if (s_stream_next_us < now) s_stream_next_us = now;  // don't burst to catch up
        if (now >= s_stream_report_us) {
            console::out("QX", "stream hz=%lu sent=%lu", (unsigned long)s_stream_hz, (unsigned long)s_stream_sent);
            s_stream_report_us = now + STREAM_REPORT_US;
        }
    }
}

bool busy() { return s_q_pos < s_q_len; }

}  // namespace qx_sender
