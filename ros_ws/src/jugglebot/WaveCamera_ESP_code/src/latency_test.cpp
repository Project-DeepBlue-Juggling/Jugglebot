#include "latency_test.h"

#include <cstring>

#include "console.h"
#include "gpi2_drive.h"
#include "pin_probe.h"
#include "pulse_stats.h"
#include "stats_math.h"
#include "uart_link.h"

namespace {

enum class Phase : uint8_t { Idle, Starting, WaitLine, Hold, Stopping, Gap };

Phase    s_phase = Phase::Idle;
uint32_t s_runs = 0;
uint32_t s_run = 0;  // 1-based index of the current run
int64_t  s_rec_us = 0;
int64_t  s_gap_us = 0;

int64_t s_t_issue = 0;
int64_t s_edge_before = -1;  // last trigger-edge stamp before this run's request
int64_t s_t_edge = -1;
int64_t s_phase_us = -1;
bool    s_have_line = false;
int64_t s_t_first = 0;
int64_t s_t_eol = 0;
int64_t s_deadline = 0;
int64_t s_until = 0;

RunningStats s_first;
RunningStats s_eol;

constexpr int64_t MS = 1000;

int64_t trigger_edge_stamp() {
    return gpi2_drive::mode() == RecMode::Edge ? gpi2_drive::last_rise_us() : gpi2_drive::last_fall_us();
}
const char* trigger_edge_name() { return gpi2_drive::mode() == RecMode::Edge ? "rising" : "falling"; }

void print_summary() {
    if (s_first.n == 0) {
        console::out("LAT", "SUMMARY n=0");
        return;
    }
    console::out("LAT",
                 "SUMMARY n=%lu first_min_us=%.0f first_mean_us=%.1f first_max_us=%.0f first_stdev_us=%.1f "
                 "eol_min_us=%.0f eol_mean_us=%.1f eol_max_us=%.0f eol_stdev_us=%.1f",
                 (unsigned long)s_first.n, s_first.min, s_first.mean, s_first.max, s_first.stdev(), s_eol.min,
                 s_eol.mean, s_eol.max, s_eol.stdev());
}

// Returns false (after aborting) if the start trigger was refused.
bool begin_run(int64_t now) {
    s_t_issue = now;
    s_edge_before = trigger_edge_stamp();
    s_t_edge = -1;
    s_phase_us = -1;
    s_have_line = false;
    const bool ok = gpi2_drive::mode() == RecMode::Edge ? gpi2_drive::request_toggle() : gpi2_drive::request_start();
    if (!ok) {
        latency_test::abort("start trigger refused");
        return false;
    }
    s_phase = Phase::Starting;
    // Allow for a stretched trigger on top of the run timeout.
    s_deadline = now + (LAT_RUN_TIMEOUT_MS + GPI2_LEVEL_MIN_STATE_MS) * MS;
    return true;
}

void finish_run_line() {
    const int64_t lat_first = s_t_first - s_t_edge;
    const int64_t lat_eol = s_t_eol - s_t_edge;
    s_first.add(static_cast<double>(lat_first));
    s_eol.add(static_cast<double>(lat_eol));
    char phase[24];
    if (s_phase_us >= 0) snprintf(phase, sizeof(phase), "%lld", (long long)s_phase_us);
    else snprintf(phase, sizeof(phase), "-");
    console::out("LAT",
                 "run=%lu t_edge_%s_us=%lld t_first_byte_us=%lld t_eol_us=%lld latency_first_us=%lld "
                 "latency_eol_us=%lld phase_to_last_pulse_us=%s",
                 (unsigned long)s_run, trigger_edge_name(), (long long)s_t_edge, (long long)s_t_first,
                 (long long)s_t_eol, (long long)lat_first, (long long)lat_eol, phase);
}

}  // namespace

namespace latency_test {

bool start(uint32_t runs, float rec_s, float gap_s) {
    if (s_phase != Phase::Idle) {
        console::out("ERR", "lat already running");
        return false;
    }
    if (runs < 1 || runs > LAT_MAX_RUNS) {
        console::out("ERR", "lat runs must be 1..%lu", (unsigned long)LAT_MAX_RUNS);
        return false;
    }
    if (!(rec_s >= LAT_MIN_REC_S && rec_s <= LAT_MAX_REC_S) || !(gap_s >= LAT_MIN_GAP_S && gap_s <= LAT_MAX_GAP_S)) {
        console::out("ERR", "lat rec_s must be %.1f..%.0f and gap_s %.1f..%.0f", LAT_MIN_REC_S, LAT_MAX_REC_S,
                     LAT_MIN_GAP_S, LAT_MAX_GAP_S);
        return false;
    }
    if (!gpi2_drive::mode_confirmed()) {
        console::out("ERR", "lat refused: rec mode not set; run `rec mode edge|level` first");
        return false;
    }
    if (uart_link::rx_pin() < 0) {
        console::out("ERR", "lat refused: no uart rx pin; run `uart rx 32` first");
        return false;
    }
    if (gpi2_drive::state() != RecState::Idle) {
        console::out("ERR", "lat refused: rec state is %s; confirm the camera is idle and run `rec assume idle`",
                     rec_state_name(gpi2_drive::state()));
        return false;
    }
    if (gpi2_drive::busy() || gpi2_drive::pad_held_low()) {
        console::out("ERR", "lat refused: GPI2 busy or held low");
        return false;
    }
    s_runs = runs;
    s_run = 1;
    s_rec_us = static_cast<int64_t>(rec_s * 1e6f);
    s_gap_us = static_cast<int64_t>(gap_s * 1e6f);
    s_first.reset();
    s_eol.reset();
    const int pp = pin_probe::pulse_pin();
    console::out("LAT", "start runs=%lu rec_s=%.3f gap_s=%.3f mode=%s uart_rx=%d pulse_pin=%d match=\"%s\"",
                 (unsigned long)runs, rec_s, gap_s, rec_mode_name(gpi2_drive::mode()), uart_link::rx_pin(), pp,
                 LAT_MATCH_TEXT);
    if (!begin_run(esp_timer_get_time())) {
        console::out("ERR", "lat refused: first start trigger was refused");
        return false;
    }
    return true;
}

void abort(const char* why) {
    if (s_phase == Phase::Idle) return;
    console::out("ERR", "LAT aborted at run %lu: %s; rec state set to unknown", (unsigned long)s_run, why);
    s_phase = Phase::Idle;
    // A trigger still waiting out its stretch must never fire after the abort.
    if (gpi2_drive::cancel_pending()) console::out("WARN", "cancelled pending GPI2 operation");
    if (gpi2_drive::mode() == RecMode::Level && gpi2_drive::pad_held_low() && !gpi2_drive::busy()) {
        console::out("WARN", "releasing GPI2 (level mode) after abort");
        gpi2_drive::request_stop();
    }
    gpi2_drive::assume(RecState::Unknown);
    print_summary();
}

void poll(int64_t now) {
    switch (s_phase) {
        case Phase::Idle:
            return;
        case Phase::Starting: {
            const int64_t e = trigger_edge_stamp();
            if (e >= 0 && e != s_edge_before && e >= s_t_issue) {
                s_t_edge = e;
                const int pp = pin_probe::pulse_pin();
                if (pp >= 0) {
                    const int64_t lead = pulse_stats::last_leading_edge_us(static_cast<uint8_t>(pp), e);
                    if (lead >= 0) s_phase_us = e - lead;
                }
                s_phase = Phase::WaitLine;
                s_deadline = e + LAT_RUN_TIMEOUT_MS * MS;
            } else if (now > s_deadline) {
                abort("no GPI2 trigger edge seen by the ISR");
                return;
            }
            break;
        }
        case Phase::WaitLine:
            if (s_have_line && s_t_first < s_t_edge) {
                // Started before our edge: not caused by it. Keep waiting.
                console::out("WARN", "LAT run %lu: matching line began %lld us before the trigger edge; ignored",
                             (unsigned long)s_run, (long long)(s_t_edge - s_t_first));
                s_have_line = false;
            }
            if (s_have_line) {
                finish_run_line();
                s_phase = Phase::Hold;
                s_until = s_t_edge + s_rec_us;
            } else if (now > s_deadline) {
                abort("timeout waiting for camera line");
                return;
            }
            break;
        case Phase::Hold:
            if (now >= s_until && !gpi2_drive::busy()) {
                const bool ok =
                    gpi2_drive::mode() == RecMode::Edge ? gpi2_drive::request_toggle() : gpi2_drive::request_stop();
                if (!ok) {
                    abort("stop trigger refused");
                    return;
                }
                s_phase = Phase::Stopping;
            }
            break;
        case Phase::Stopping:
            if (!gpi2_drive::busy()) {
                s_phase = Phase::Gap;
                s_until = now + s_gap_us;
            }
            break;
        case Phase::Gap:
            if (now >= s_until) {
                if (s_run >= s_runs) {
                    s_phase = Phase::Idle;
                    print_summary();
                } else {
                    ++s_run;
                    begin_run(now);  // aborts itself on refusal
                }
            }
            break;
    }
}

void on_uart_line(const UartLine& line, int64_t t_first_us, int64_t t_eol_us) {
    if (s_phase != Phase::Starting && s_phase != Phase::WaitLine) return;
    if (s_have_line) return;
    // A terminated line, or one flushed by the idle timeout (camera omitted
    // the terminator); an overflow fragment is not trusted.
    if (line.end == LineEnd::Overflow) return;
    if (t_first_us < s_t_issue) return;  // began before this run's trigger request
    if (!strstr(line.raw, LAT_MATCH_TEXT)) return;
    s_have_line = true;
    s_t_first = t_first_us;
    s_t_eol = t_eol_us;
}

bool running() { return s_phase != Phase::Idle; }

}  // namespace latency_test
