#pragma once

// Probe accumulator + pin classifier (wave_camera_gpio_explorer.md s4). Pure, header-only.
//
//   PULSE        >= PULSE_MIN_EDGES edges, period CV < 5 %, period 30-36 ms
//   UART         min pulse width 6-12 us (one bit at 115200 is 8.68 us),
//                edges arriving in bursts separated by idle gaps
//   STATIC_HIGH / STATIC_LOW   no edges in the window
//   ACTIVE       edges that match neither pattern

#include <cstdint>

#include "config.h"
#include "stats_math.h"

enum class PinRole : uint8_t { Unknown, StaticHigh, StaticLow, Pulse, Uart, Active };

inline const char* pin_role_name(PinRole r) {
    switch (r) {
        case PinRole::StaticHigh: return "STATIC_HIGH";
        case PinRole::StaticLow:  return "STATIC_LOW";
        case PinRole::Pulse:      return "PULSE";
        case PinRole::Uart:       return "UART";
        case PinRole::Active:     return "ACTIVE";
        default:                  return "UNKNOWN";
    }
}

// True for roles that mean "the camera drives this pin": never transmit on it.
inline bool role_is_camera_output(PinRole r) { return r == PinRole::Pulse || r == PinRole::Uart; }

struct ProbeAccumulator {
    uint32_t     edges = 0;
    uint32_t     bursts = 0;
    uint32_t     repeated_levels = 0;
    int64_t      last_t_us = -1;
    int64_t      last_rise_us = -1;
    int64_t      last_fall_us = -1;
    uint8_t      last_level = 0xFF;
    int64_t      min_low_us = -1;
    int64_t      min_high_us = -1;
    RunningStats rise_period_us;
    RunningStats fall_period_us;

    void reset() { *this = ProbeAccumulator{}; }

    void on_edge(int64_t t_us, uint8_t level) {
        ++edges;
        if (last_t_us < 0 || t_us - last_t_us > static_cast<int64_t>(UART_BURST_GAP_US)) ++bursts;
        if (last_level == level) {
            ++repeated_levels;  // an edge was missed; don't trust this interval
        } else if (last_t_us >= 0) {
            const int64_t w = t_us - last_t_us;
            int64_t& slot = (last_level == 0) ? min_low_us : min_high_us;
            if (slot < 0 || w < slot) slot = w;
        }
        if (level) {
            if (last_rise_us >= 0) rise_period_us.add(static_cast<double>(t_us - last_rise_us));
            last_rise_us = t_us;
        } else {
            if (last_fall_us >= 0) fall_period_us.add(static_cast<double>(t_us - last_fall_us));
            last_fall_us = t_us;
        }
        last_t_us = t_us;
        last_level = level;
    }

    const RunningStats& best_period() const {
        return fall_period_us.n >= rise_period_us.n ? fall_period_us : rise_period_us;
    }
    // Shortest observed state, or -1 if none.
    int64_t min_width_us() const {
        if (min_low_us < 0) return min_high_us;
        if (min_high_us < 0) return min_low_us;
        return min_low_us < min_high_us ? min_low_us : min_high_us;
    }
};

inline PinRole classify_pin(const ProbeAccumulator& a, uint8_t level_now) {
    if (a.edges == 0) return level_now ? PinRole::StaticHigh : PinRole::StaticLow;
    const RunningStats& p = a.best_period();
    if (a.edges >= PULSE_MIN_EDGES && p.n >= 2 && p.cv() < PULSE_MAX_PERIOD_CV &&
        p.mean >= PULSE_PERIOD_MIN_US && p.mean <= PULSE_PERIOD_MAX_US)
        return PinRole::Pulse;
    const int64_t w = a.min_width_us();
    // Bursty: on average several edges per burst (one byte is 2-10 edges).
    const bool bursty = a.bursts > 0 && a.edges >= 2 * a.bursts;
    if (w >= static_cast<int64_t>(UART_MIN_PULSE_LO_US) && w <= static_cast<int64_t>(UART_MIN_PULSE_HI_US) && bursty)
        return PinRole::Uart;
    return PinRole::Active;
}
