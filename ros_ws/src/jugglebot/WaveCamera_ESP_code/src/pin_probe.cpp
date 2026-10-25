#include "pin_probe.h"

#include "console.h"

namespace {

bool             s_running = false;
int64_t          s_start_us = 0;
int64_t          s_end_us = 0;
ProbeAccumulator s_acc[NUM_CAPTURE_PINS];
PinRole          s_role[NUM_CAPTURE_PINS] = {};

void finish(int64_t now_us) {
    s_running = false;
    const double window_s = (now_us - s_start_us) / 1e6;
    for (size_t i = 0; i < NUM_CAPTURE_PINS; ++i) {
        const uint8_t gpio = CAPTURE_PINS[i];
        const ProbeAccumulator& a = s_acc[i];
        const int level = pin_io::read_level(gpio);
        const PinRole r = classify_pin(a, static_cast<uint8_t>(level));
        s_role[i] = r;
        const RunningStats& p = a.best_period();
        const char* note = "";
        if (r == PinRole::StaticHigh && gpio != PIN_GPI2) note = " note=\"idle high, no traffic yet\"";
        if (r == PinRole::Uart) note = " note=\"bursty, ~1 bit min width\"";
        console::out("PROBE",
                     "pin=%u wave=%u role=%s level=%d edges=%lu bursts=%lu missed=%lu min_low_us=%lld "
                     "min_high_us=%lld period_n=%lu period_mean_us=%.1f period_cv=%.4f rate_hz=%.3f%s",
                     gpio, CAPTURE_WAVE_PINS[i], pin_role_name(r), level, (unsigned long)a.edges,
                     (unsigned long)a.bursts, (unsigned long)a.repeated_levels, (long long)a.min_low_us,
                     (long long)a.min_high_us, (unsigned long)p.n, p.n ? p.mean : 0.0, p.n > 1 ? p.cv() : 0.0,
                     p.n && p.mean > 0 ? 1e6 / p.mean : 0.0, note);
    }
    console::out("PROBE", "done window_s=%.3f pulse_pin=%d uart_pin=%d", window_s, pin_probe::pulse_pin(),
                 pin_probe::uart_pin());
}

}  // namespace

namespace pin_probe {

bool start(uint32_t seconds) {
    if (s_running) return false;
    for (auto& a : s_acc) a.reset();
    s_start_us = esp_timer_get_time();
    s_end_us = s_start_us + static_cast<int64_t>(seconds) * 1000000;
    s_running = true;
    console::out("PROBE", "start seconds=%lu", (unsigned long)seconds);
    return true;
}

void on_edge(const EdgeEvent& ev) {
    if (!s_running || ev.t_us < s_start_us || ev.t_us > s_end_us) return;
    const int i = capture_index(ev.gpio);
    if (i >= 0) s_acc[i].on_edge(ev.t_us, ev.level);
}

void poll(int64_t now_us) {
    if (s_running && now_us >= s_end_us) finish(now_us);
}

bool running() { return s_running; }

PinRole role(uint8_t gpio) {
    const int i = capture_index(gpio);
    return i >= 0 ? s_role[i] : PinRole::Unknown;
}

int pulse_pin() {
    for (size_t i = 0; i < NUM_CAPTURE_PINS; ++i)
        if (s_role[i] == PinRole::Pulse) return CAPTURE_PINS[i];
    return -1;
}

int uart_pin() {
    for (size_t i = 0; i < NUM_CAPTURE_PINS; ++i)
        if (s_role[i] == PinRole::Uart) return CAPTURE_PINS[i];
    return -1;
}

}  // namespace pin_probe
