#include "gpi2_drive.h"

#include "console.h"

namespace {

enum class Op : uint8_t { None, ToggleWait, ToggleLow, StartWait, StopWait };

RecMode  s_mode = RecMode::Edge;  // default edge, but must be confirmed by `rec mode`
bool     s_mode_confirmed = false;
RecState s_state = RecState::Unknown;
Op       s_op = Op::None;
int64_t  s_due_us = 0;

bool    s_held_low = false;
int64_t s_high_since_us = 0;  // pad released (ISR rise refines the command time)
int64_t s_low_since_us = 0;
int64_t s_low_cmd_us = -1;
int64_t s_release_cmd_us = -1;
int64_t s_last_rise_us = -1;
int64_t s_last_fall_us = -1;

constexpr int64_t MS = 1000;

bool check_ready(RecMode needed, const char* what) {
    if (!s_mode_confirmed) {
        console::out("ERR", "%s refused: rec mode not set; run `rec mode edge|level` first", what);
        return false;
    }
    if (s_mode != needed) {
        console::out("ERR", "%s refused: rec mode is %s", what, rec_mode_name(s_mode));
        return false;
    }
    if (s_op != Op::None) {
        console::out("ERR", "%s refused: GPI2 operation in progress", what);
        return false;
    }
    return true;
}

// Schedule at the later of now and earliest; log a stretch if needed.
void schedule(Op op, int64_t now, int64_t earliest, const char* what, uint32_t min_ms, const char* rule) {
    s_op = op;
    if (now < earliest) {
        s_due_us = earliest;
        console::out("WARN", "%s stretched by %lld ms (min %lu ms %s); due_us=%lld", what,
                     (long long)((earliest - now + MS - 1) / MS), (unsigned long)min_ms, rule, (long long)earliest);
    } else {
        s_due_us = now;
    }
}

void drive_low(int64_t now) {
    if (!pin_io::od_drive_low(PIN_GPI2)) {
        console::out("ERR", "GPI2 drive refused by pin_io (pin not open-drain)");
        s_op = Op::None;
        return;
    }
    s_held_low = true;
    s_low_cmd_us = now;
    s_low_since_us = now;
    console::out("REC", "drive=low t_cmd_us=%lld mode=%s", (long long)now, rec_mode_name(s_mode));
}

void release(int64_t now) {
    pin_io::od_release(PIN_GPI2);
    s_held_low = false;
    s_release_cmd_us = now;
    s_high_since_us = now;
    console::out("REC", "drive=release t_cmd_us=%lld low_ms=%.3f mode=%s", (long long)now,
                 (now - s_low_cmd_us) / 1000.0, rec_mode_name(s_mode));
}

}  // namespace

namespace gpi2_drive {

void begin() {
    pin_io::od_release(PIN_GPI2);
    s_high_since_us = esp_timer_get_time();  // treat boot as the start of "high"
}

void set_mode(RecMode m) {
    if (s_op != Op::None || s_held_low) {
        console::out("ERR", "rec mode change refused while GPI2 is busy or held low");
        return;
    }
    s_mode = m;
    s_mode_confirmed = true;
    console::out("REC", "mode=%s (must match the camera's GPI setting)", rec_mode_name(m));
}

RecMode mode() { return s_mode; }
bool    mode_confirmed() { return s_mode_confirmed; }

bool request_toggle() {
    if (!check_ready(RecMode::Edge, "rec toggle")) return false;
    const int64_t now = esp_timer_get_time();
    schedule(Op::ToggleWait, now, s_high_since_us + GPI2_EDGE_MIN_HIGH_MS * MS, "rec toggle", GPI2_EDGE_MIN_HIGH_MS,
             "high since previous rising edge");
    return true;
}

bool request_start() {
    if (!check_ready(RecMode::Level, "rec start")) return false;
    if (s_held_low) {
        console::out("ERR", "rec start refused: GPI2 already held low");
        return false;
    }
    const int64_t now = esp_timer_get_time();
    schedule(Op::StartWait, now, s_high_since_us + GPI2_LEVEL_MIN_STATE_MS * MS, "rec start",
             GPI2_LEVEL_MIN_STATE_MS, "high");
    return true;
}

bool request_stop() {
    if (!check_ready(RecMode::Level, "rec stop")) return false;
    if (!s_held_low) {
        console::out("ERR", "rec stop refused: GPI2 is not held low");
        return false;
    }
    const int64_t now = esp_timer_get_time();
    schedule(Op::StopWait, now, s_low_since_us + GPI2_LEVEL_MIN_STATE_MS * MS, "rec stop", GPI2_LEVEL_MIN_STATE_MS,
             "low");
    return true;
}

void poll(int64_t now) {
    switch (s_op) {
        case Op::ToggleWait:
            if (now >= s_due_us) {
                drive_low(now);
                if (s_op != Op::None) {
                    s_op = Op::ToggleLow;
                    s_due_us = now + GPI2_EDGE_LOW_MS * MS;
                }
            }
            break;
        case Op::ToggleLow:
            if (now >= s_due_us) {
                release(now);
                s_op = Op::None;
                // The camera toggles on the rising edge.
                if (s_state == RecState::Idle) s_state = RecState::Recording;
                else if (s_state == RecState::Recording) s_state = RecState::Idle;
                console::out("REC", "state=%s", rec_state_name(s_state));
            }
            break;
        case Op::StartWait:
            if (now >= s_due_us) {
                drive_low(now);
                s_op = Op::None;
                if (s_held_low) s_state = RecState::Recording;
                console::out("REC", "state=%s", rec_state_name(s_state));
            }
            break;
        case Op::StopWait:
            if (now >= s_due_us) {
                release(now);
                s_op = Op::None;
                s_state = RecState::Idle;
                console::out("REC", "state=%s", rec_state_name(s_state));
            }
            break;
        case Op::None:
            break;
    }
}

void on_edge(const EdgeEvent& ev) {
    if (ev.gpio != PIN_GPI2) return;
    const bool    rising = ev.level != 0;
    const int64_t cmd = rising ? s_release_cmd_us : s_low_cmd_us;
    const bool    ours = cmd >= 0 && ev.t_us >= cmd && ev.t_us - cmd <= static_cast<int64_t>(GPI2_EDGE_MATCH_WINDOW_MS) * MS;
    if (rising) {
        s_last_rise_us = ev.t_us;
        if (!s_held_low) s_high_since_us = ev.t_us;
    } else {
        s_last_fall_us = ev.t_us;
    }
    if (ours) {
        console::out("REC", "edge=%s t_edge_us=%lld t_cmd_us=%lld cmd_to_edge_us=%lld", rising ? "rising" : "falling",
                     (long long)ev.t_us, (long long)cmd, (long long)(ev.t_us - cmd));
    } else {
        console::out("WARN", "GPI2 edge=%s t_edge_us=%lld not caused by our drive (external or glitch)",
                     rising ? "rising" : "falling", (long long)ev.t_us);
    }
}

bool busy() { return s_op != Op::None; }

bool cancel_pending() {
    if (s_op != Op::ToggleWait && s_op != Op::StartWait && s_op != Op::StopWait) return false;
    s_op = Op::None;
    return true;
}
RecState state() { return s_state; }
void     assume(RecState s) {
    s_state = s;
    console::out("REC", "state=%s (assumed)", rec_state_name(s));
}
void note_clip_created() {
    if (s_state != RecState::Recording) {
        s_state = RecState::Recording;
        console::out("REC", "state=recording (camera reported a new clip)");
    }
}

int64_t last_rise_us() { return s_last_rise_us; }
int64_t last_fall_us() { return s_last_fall_us; }
bool    pad_held_low() { return s_held_low; }

}  // namespace gpi2_drive
