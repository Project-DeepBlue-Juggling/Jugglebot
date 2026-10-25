// Wave GPIO explorer + CAN adapter: setup() / loop().
//
// loop() never blocks: it drains the edge ring, services the console, drains
// the CAN RX queue (frames stamped by the RX task), and polls each
// non-blocking state machine (GPI2 drive, UART lines, record controller,
// heartbeat/INFO, latency test, probe window, QX sender). WiFi and Bluetooth
// are never started.

#include "pin_io.h"

#include "config.h"
#include "can_link.h"
#include "console.h"
#include "edge_capture.h"
#include "gpi2_drive.h"
#include "latency_test.h"
#include "pin_probe.h"
#include "pulse_stats.h"
#include "heartbeat.h"
#include "qx_protocol.h"
#include "record_ctrl.h"
#include "time_slave.h"
#include "uart_link.h"
#include "wave_frames.h"

namespace {

void handle_edge(const EdgeEvent& ev) {
    console::on_edge(ev);
    pin_probe::on_edge(ev);
    pulse_stats::on_edge(ev);
    uart_link::on_edge(ev);
    if (ev.gpio == PIN_GPI2) gpi2_drive::on_edge(ev);
    record_ctrl::on_edge(ev);
}

void handle_uart_line(const UartLine& line, int64_t t_first_us, int64_t t_eol_us) {
    if (line.end != LineEnd::Overflow && strstr(line.raw, LAT_MATCH_TEXT)) {
        if (line.end == LineEnd::Eol) gpi2_drive::note_clip_created();
        record_ctrl::on_clip_line(t_first_us);
    }
    latency_test::on_uart_line(line, t_first_us, t_eol_us);
}

void handle_can_frames() {
    can_link::RxFrame f;
    size_t n = 0;
    while (n++ < CAN_RX_DRAIN_MAX_PER_LOOP && can_link::pop_rx(f)) {
        if (f.id == wave::ID_TIME_SYNC) time_slave::on_frame(f);
        else if (f.id == wave::ID_CMD) record_ctrl::on_cmd(f);
    }
}

}  // namespace

void setup() {
    // Pins first: GPI2 released open-drain, everything else a pulled-up input.
    const bool pins_ok = edge_capture::begin();
    gpi2_drive::begin();
    uart_link::set_line_hook(handle_uart_line);
    console::begin();
    console::out("INFO", "wave-gpio-explorer boot: fw %u wire %u, esp32 rev %d, %lu MHz, ring=%u events, console %lu baud",
                 FW_VERSION, WIRE_VERSION, ESP.getChipRevision(), (unsigned long)ESP.getCpuFreqMHz(),
                 (unsigned)EDGE_RING_CAPACITY, (unsigned long)CONSOLE_BAUD);
    if (!pins_ok) console::out("ERR", "edge capture setup failed; check pin configuration");
    // CAN mode (wave_camera_can_adapter.md s5.1): level mode, camera UART bridged, CAN owns GPI2.
    record_ctrl::begin();
    uart_link::set_rx(PIN_CAM_P3);
    const bool can_ok = can_link::begin();
    if (!can_ok) console::out("ERR", "TWAI driver install failed; CAN disabled (console still works)");
    else console::out("INFO", "can up: %lu bit/s, TX=GPIO%u RX=GPIO%u, heartbeat %lu Hz, info page every %lu ms",
                      (unsigned long)CAN_BITRATE_BPS, PIN_CAN_TX, PIN_CAN_RX, (unsigned long)HEARTBEAT_RATE_HZ,
                      (unsigned long)INFO_PAGE_PERIOD_MS);
    heartbeat::begin(esp_timer_get_time());
    console::out("INFO", "disarmed; rec mode level; uart rx bridged on %u; CAN owns gpi2. Type help.", PIN_CAM_P3);
}

void loop() {
    edge_capture::drain(handle_edge, EDGE_DRAIN_MAX_PER_LOOP);
    console::poll();
    handle_can_frames();
    const int64_t now = esp_timer_get_time();
    gpi2_drive::poll(now);
    uart_link::poll(now);
    record_ctrl::poll(esp_timer_get_time());  // after uart: a clip line read this pass is already in
    heartbeat::poll(esp_timer_get_time());
    can_link::poll_log();
    latency_test::poll(now);
    pin_probe::poll(now);
    qx_sender::poll(now);
}
