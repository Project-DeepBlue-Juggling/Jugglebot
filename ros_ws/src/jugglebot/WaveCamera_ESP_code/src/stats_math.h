#pragma once

// Pure statistics helpers (header-only, Arduino-free, native-tested).
//   RunningStats  - Welford mean / stdev / min / max over doubles.
//   analyse_pulse - period, jitter, width and polarity of a pulse train
//                   from a chronological list of edges.

#include <cmath>
#include <cstddef>
#include <cstdint>

struct RunningStats {
    uint32_t n = 0;
    double   mean = 0.0;
    double   m2 = 0.0;
    double   min = 0.0;
    double   max = 0.0;

    void reset() { *this = RunningStats{}; }

    void add(double x) {
        ++n;
        if (n == 1) { min = max = x; }
        else { if (x < min) min = x; if (x > max) max = x; }
        const double d = x - mean;
        mean += d / n;
        m2 += d * (x - mean);
    }
    // Sample standard deviation (n-1); 0 for fewer than two samples.
    double stdev() const { return n > 1 ? std::sqrt(m2 / (n - 1)) : 0.0; }
    // Coefficient of variation; large sentinel when undefined.
    double cv() const { return (n > 1 && mean > 0.0) ? stdev() / mean : 1e9; }
};

// One captured transition. level is the pad level AFTER the edge.
struct EdgeRecord {
    int64_t t_us;
    uint8_t level;
};

enum class PulsePolarity : uint8_t { Unknown, Negative, Positive };

inline const char* polarity_name(PulsePolarity p) {
    return p == PulsePolarity::Negative ? "negative" : p == PulsePolarity::Positive ? "positive" : "unknown";
}

struct PulseReport {
    RunningStats  period_us;   // leading edge to leading edge
    RunningStats  width_us;    // duration of the pulse (shorter) state
    RunningStats  low_us;
    RunningStats  high_us;
    PulsePolarity polarity = PulsePolarity::Unknown;
    uint32_t      repeated_levels = 0;  // consecutive same-level records (missed edge)
    int64_t       last_leading_us = -1;
};

// Analyse a chronological edge list. Widths are taken only between records of
// alternating level; a repeated level means an edge was missed and the
// interval is skipped. Polarity: the state with the shorter mean duration is
// the pulse. Periods are measured between successive leading edges (falling
// for a negative pulse), skipping intervals that contain a repeated level.
inline PulseReport analyse_pulse(const EdgeRecord* e, size_t n) {
    PulseReport r;
    for (size_t i = 1; i < n; ++i) {
        if (e[i].level == e[i - 1].level) { ++r.repeated_levels; continue; }
        const double dt = static_cast<double>(e[i].t_us - e[i - 1].t_us);
        // e[i-1].level is the state that just ended.
        if (e[i - 1].level == 0) r.low_us.add(dt); else r.high_us.add(dt);
    }
    if (r.low_us.n == 0 || r.high_us.n == 0) return r;
    r.polarity = (r.low_us.mean <= r.high_us.mean) ? PulsePolarity::Negative : PulsePolarity::Positive;
    r.width_us = (r.polarity == PulsePolarity::Negative) ? r.low_us : r.high_us;
    const uint8_t leading_level = (r.polarity == PulsePolarity::Negative) ? 0 : 1;
    int64_t prev = -1;
    bool clean = true;  // no repeated level since prev
    for (size_t i = 0; i < n; ++i) {
        const bool repeated = i > 0 && e[i].level == e[i - 1].level;
        if (repeated) clean = false;
        if (e[i].level != leading_level) continue;
        // A repeated leading level still restarts the reference, but the
        // interval that contains the missed edge is not a trusted period.
        if (prev >= 0 && clean) r.period_us.add(static_cast<double>(e[i].t_us - prev));
        prev = e[i].t_us;
        clean = true;  // measure from this record onwards
        r.last_leading_us = e[i].t_us;
    }
    return r;
}

