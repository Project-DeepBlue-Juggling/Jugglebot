#include "pulse_stats.h"

#include "console.h"
#include "pin_probe.h"

namespace {

struct History {
    EdgeRecord rec[PULSE_HISTORY_EDGES];
    size_t     head = 0;   // next write slot
    size_t     count = 0;

    void add(int64_t t_us, uint8_t level) {
        rec[head] = EdgeRecord{t_us, level};
        head = (head + 1) % PULSE_HISTORY_EDGES;
        if (count < PULSE_HISTORY_EDGES) ++count;
    }
    // Chronological copy into out; returns the count.
    size_t copy(EdgeRecord* out) const {
        const size_t start = (head + PULSE_HISTORY_EDGES - count) % PULSE_HISTORY_EDGES;
        for (size_t i = 0; i < count; ++i) out[i] = rec[(start + i) % PULSE_HISTORY_EDGES];
        return count;
    }
};

History    s_hist[NUM_CAPTURE_PINS];
EdgeRecord s_scratch[PULSE_HISTORY_EDGES];

void report_pin(uint8_t gpio) {
    const int i = capture_index(gpio);
    const size_t n = s_hist[i].copy(s_scratch);
    const PulseReport r = analyse_pulse(s_scratch, n);
    const RunningStats& p = r.period_us;
    console::out("PULSE",
                 "pin=%u wave=%u edges=%u periods=%lu period_mean_us=%.1f period_stdev_us=%.2f period_min_us=%.0f "
                 "period_max_us=%.0f jitter_pp_us=%.0f rate_hz=%.4f polarity=%s width_mean_us=%.1f "
                 "width_min_us=%.0f width_max_us=%.0f missed=%lu last_leading_us=%lld",
                 gpio, CAPTURE_WAVE_PINS[i], (unsigned)n, (unsigned long)p.n, p.mean, p.stdev(), p.min, p.max,
                 p.max - p.min, p.n && p.mean > 0 ? 1e6 / p.mean : 0.0, polarity_name(r.polarity), r.width_us.mean,
                 r.width_us.min, r.width_us.max, (unsigned long)r.repeated_levels, (long long)r.last_leading_us);
}

}  // namespace

namespace pulse_stats {

void on_edge(const EdgeEvent& ev) {
    const int i = capture_index(ev.gpio);
    if (i >= 0) s_hist[i].add(ev.t_us, ev.level);
}

void report() {
    const int pp = pin_probe::pulse_pin();
    if (pp >= 0) {
        report_pin(static_cast<uint8_t>(pp));
        return;
    }
    bool any = false;
    for (size_t i = 0; i < NUM_CAPTURE_PINS; ++i) {
        if (s_hist[i].count >= 3) {
            report_pin(CAPTURE_PINS[i]);
            any = true;
        }
    }
    if (!any) console::out("PULSE", "no PULSE pin known and no pin has >= 3 edges; run probe first");
}

int64_t last_leading_edge_us(uint8_t gpio, int64_t t_us) {
    const int i = capture_index(gpio);
    if (i < 0) return -1;
    const size_t n = s_hist[i].copy(s_scratch);
    const PulseReport r = analyse_pulse(s_scratch, n);
    const uint8_t leading = (r.polarity == PulsePolarity::Positive) ? 1 : 0;  // default: falling
    for (size_t k = n; k-- > 0;) {
        if (s_scratch[k].t_us <= t_us && s_scratch[k].level == leading) return s_scratch[k].t_us;
    }
    return -1;
}

}  // namespace pulse_stats
