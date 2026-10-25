#pragma once

// Recording state machine (wave_camera_can_adapter.md s5.2-s5.4): pure, header-only.
//
// Every input carries its own timestamp (ISR stamps for edges and ticks, the
// UART first-byte stamp for the clip line, `now` for polls), so the native
// tests drive it with synthetic time. The only side effects go through
// RecordIo (drive / release GPI2) and the event queue the glue drains.
//
// States: BOOT -> IDLE -> STARTING -> RECORDING -> STOPPING -> IDLE, and
// FAULT for a GPI2 drive the pad did not follow.
//
// Start stamp: the first GPIO27 tick after the GPIO26 falling-edge stamp. If
// no tick arrives within TICK_AFTER_EDGE_MS, the falling edge itself is used
// and TICK_STAMPED is clear. Stop is stamped the same way from the rising
// edge. The clip line ("Created new clip.") must start within CLIP_WAIT_MS of
// the start tick; if it does not, START is reported with reason CLIP_MISSING
// and GPI2 stays low (owner decision).
//
// Ack rule (s4.5): every new cmd_seq (or new session_id) produces exactly one
// event carrying it; repeats of the same seq are keepalives, except that a
// change of `desired` is always acted on (a lost stop frame is repaired by
// the repeats). A new seq that arrives while an earlier one is still in
// flight (STARTING / STOPPING) is acked at once with REFUSED NO_CHANGE, and
// the earlier seq keeps its START / STOP.

#include <cstddef>
#include <cstdint>

#include "config.h"
#include "wave_frames.h"

// --- Tick detection ------------------------------------------------------------
// The camera tick is a ~8 us positive pulse on GPIO27, about the same as the
// ISR latency, so the level the ISR reads is not trustworthy for telling the
// leading edge from the trailing one. The first edge after a quiet gap of
// TICK_MIN_GAP_US is the leading edge whatever level was read.
class TickDetector {
public:
    bool is_tick(int64_t t_us) {
        const bool tick = last_edge_us_ < 0 || t_us - last_edge_us_ >= static_cast<int64_t>(TICK_MIN_GAP_US);
        last_edge_us_ = t_us;
        return tick;
    }

private:
    int64_t last_edge_us_ = -1;
};

// --- Tick statistics for WAVE_INFO page 1 --------------------------------------------
class TickTracker {
public:
    void on_tick(int64_t t_us) {
        if (last_us_ >= 0) {
            const int64_t dt = t_us - last_us_;
            const int64_t p = TICK_NOMINAL_PERIOD_US;
            if (dt > p / 2 && dt < p + p / 2) {
                dt_[head_] = dt;
                head_ = (head_ + 1) % TICK_STATS_PERIODS;
                if (n_ < TICK_STATS_PERIODS) ++n_;
            } else if (dt >= p + p / 2) {
                missed_ += static_cast<uint32_t>((dt + p / 2) / p - 1);
            }
        }
        last_us_ = t_us;
    }
    // Mean period over the last TICK_STATS_PERIODS periods, ns (0 if none).
    uint32_t period_ns() const {
        if (!n_) return 0;
        int64_t sum = 0;
        for (size_t i = 0; i < n_; ++i) sum += dt_[i];
        return static_cast<uint32_t>(sum * 1000 / static_cast<int64_t>(n_));
    }
    uint32_t jitter_pp_us() const {
        if (!n_) return 0;
        int64_t lo = dt_[0], hi = dt_[0];
        for (size_t i = 1; i < n_; ++i) {
            if (dt_[i] < lo) lo = dt_[i];
            if (dt_[i] > hi) hi = dt_[i];
        }
        return static_cast<uint32_t>(hi - lo);
    }
    uint32_t missed_total() const { return missed_; }

private:
    int64_t  dt_[TICK_STATS_PERIODS] = {};
    size_t   head_ = 0;
    size_t   n_ = 0;
    int64_t  last_us_ = -1;
    uint32_t missed_ = 0;
};

// --- GPI2 side effects ------------------------------------------------------------------
enum class ReleaseResult : uint8_t {
    Pending,    // release requested; a rising edge will follow
    NotDriven,  // the pad was never driven low (a pending start was cancelled)
    Refused,    // the drive layer refused; pad state unknown
};

class RecordIo {
public:
    virtual bool          drive_low() = 0;  // false = refused
    virtual ReleaseResult release() = 0;

protected:
    ~RecordIo() = default;
};

// One emitted event, in local time. The glue converts it to WAVE_REC_EVENT.
struct FsmEvent {
    wave::EventKind kind;
    wave::Reason    reason;
    bool            tick_stamped;
    int64_t         t_local_us;
    uint8_t         cmd_seq;
    uint8_t         recording_seq;
};

class RecordFsm {
public:
    static constexpr size_t EVENT_QUEUE = 8;
    static constexpr uint8_t FSM_FAULT_MASK =
        wave::F_GPI2_REFUSED | wave::F_TICK_LOST_MID_REC | wave::F_CLIP_MISSING | wave::F_INTENT_TIMEOUT;

    explicit RecordFsm(RecordIo& io) : io_(io) {}

    // ---- inputs ----
    void poll(int64_t now) {
        switch (state_) {
            case wave::State::Boot:
                state_ = wave::State::Idle;
                break;
            case wave::State::Idle:
            case wave::State::Fault:
                break;
            case wave::State::Starting:
                poll_starting(now);
                break;
            case wave::State::Recording:
                poll_recording(now);
                break;
            case wave::State::Stopping:
                poll_stopping(now);
                break;
        }
    }

    // A decoded WAVE_CMD (desired already validated as 0/1).
    void on_intent(uint8_t desired, uint8_t seq, uint8_t flags, uint32_t session_id, int64_t now) {
        if (flags & wave::CMD_FLAG_CLEAR_FAULTS) {
            faults_ = 0;
            if (state_ == wave::State::Fault) state_ = wave::State::Idle;
        }
        // A new session (ROS relaunch) counts as a new seq even if the byte matches.
        const bool new_session = !have_session_ || session_id != session_id_;
        if (new_session) {
            recordings_session_ = 0;
            session_id_ = session_id;
            have_session_ = true;
        }
        const bool want = desired != 0;
        const bool is_new = !have_intent_ || seq != last_seq_ || new_session;
        // A change of desired is actionable even with an unchanged seq: if the
        // node's new-seq stop frame is lost, the repeats that follow still stop.
        const bool edge = want != (have_intent_ && desired_);
        have_intent_ = true;
        last_intent_us_ = now;
        desired_ = want;
        last_seq_ = seq;
        ++intents_;
        if (!is_new && !edge) return;  // keepalive
        if (is_new) acted_seq_ = seq;

        switch (state_) {
            case wave::State::Boot:
            case wave::State::Idle:
                if (desired_) try_start(seq, now);
                else if (is_new) refuse(console_owns_ ? wave::Reason::ConsoleOwns : wave::Reason::NoChange, seq, now);
                break;
            case wave::State::Starting:
                // A second start keeps the original seq (it gets START); the
                // newer one is acked at once. desired=0 stops once RECORDING
                // is entered, carrying last_seq_.
                if (desired_ && is_new) refuse(wave::Reason::NoChange, seq, now);
                break;
            case wave::State::Recording:
                if (desired_) {
                    if (is_new) refuse(wave::Reason::NoChange, seq, now);
                } else {
                    begin_stop(wave::EventKind::Stop, wave::Reason::None, seq, now);
                }
                break;
            case wave::State::Stopping:
                if (desired_) {
                    want_start_ = true;  // started (and acked with this seq) once IDLE
                    want_start_seq_ = seq;
                } else {
                    want_start_ = false;
                    if (is_new) refuse(wave::Reason::NoChange, seq, now);  // stop_seq_ keeps its STOP
                }
                break;
            case wave::State::Fault:
                if (is_new) refuse(desired_ ? wave::Reason::Gpi2Fault : wave::Reason::NoChange, seq, now);
                break;
        }
    }

    // GPIO26 edge as stamped by its ISR (level read in the ISR).
    void on_gpi2_edge(uint8_t level, int64_t t) {
        pad_low_ = level == 0;
        if (level == 0) {
            if (state_ == wave::State::Starting && t_fall_ < 0 && t >= t_req_) t_fall_ = t;
        } else {
            if (state_ == wave::State::Stopping && t_rise_ < 0 && t >= t_release_req_) t_rise_ = t;
        }
    }

    // GPIO27 leading edge (see TickDetector).
    void on_tick(int64_t t) {
        last_tick_us_ = t;
        tick_hist_[tick_head_] = t;
        tick_head_ = (tick_head_ + 1) % TICK_HIST;
        if (tick_n_ < TICK_HIST) ++tick_n_;
        const int64_t win = static_cast<int64_t>(TICK_AFTER_EDGE_MS) * 1000;
        if (state_ == wave::State::Starting && t_fall_ >= 0 && t_start_ < 0 && t > t_fall_ && t - t_fall_ <= win) {
            t_start_ = t;
            start_tick_stamped_ = true;
        }
        if (state_ == wave::State::Stopping && t_rise_ >= 0 && t_stop_ < 0 && t > t_rise_ && t - t_rise_ <= win) {
            t_stop_ = t;
            stop_tick_stamped_ = true;
        }
    }

    // A "Created new clip." line; t_first is its first-byte stamp.
    void on_clip_line(int64_t t_first) {
        uart_seen_ = true;
        if (state_ == wave::State::Starting && t_fall_ >= 0 && t_first > t_fall_ && t_clip_ < 0) t_clip_ = t_first;
    }

    // Console ownership of GPI2. Taking it is refused unless IDLE; handing it
    // back is always allowed (the glue checks the pad is released).
    bool set_console_owns(bool on) {
        if (on && state_ != wave::State::Idle && state_ != wave::State::Boot) return false;
        console_owns_ = on;
        return true;
    }

    // Console `rec assume idle` while in FAULT.
    bool clear_fault_state() {
        if (state_ != wave::State::Fault) return false;
        state_ = wave::State::Idle;
        return true;
    }
    void clear_faults() { faults_ = 0; }

    // ---- outputs ----
    bool take_event(FsmEvent& out) {
        if (ev_n_ == 0) return false;
        out = ev_[ev_tail_];
        ev_tail_ = (ev_tail_ + 1) % EVENT_QUEUE;
        --ev_n_;
        return true;
    }

    wave::State state() const { return state_; }
    uint8_t     faults() const { return faults_; }
    bool        console_owns() const { return console_owns_; }
    bool        camera_on(int64_t now) const {
        return last_tick_us_ >= 0 && now - last_tick_us_ <= static_cast<int64_t>(CAMERA_ON_TICK_WINDOW_MS) * 1000;
    }
    bool intent_fresh(int64_t now) const {
        return have_intent_ && now - last_intent_us_ < static_cast<int64_t>(INTENT_TIMEOUT_MS) * 1000;
    }
    bool     have_intent() const { return have_intent_; }
    bool     desired() const { return desired_; }
    bool     uart_seen() const { return uart_seen_; }
    bool     pad_low() const { return pad_low_; }
    void     set_pad_low(bool low) { pad_low_ = low; }
    uint8_t  last_seq() const { return last_seq_; }
    uint8_t  acted_seq() const { return acted_seq_; }
    uint32_t session_id() const { return session_id_; }
    uint8_t  recordings_session() const { return recordings_session_; }
    uint8_t  recording_seq() const { return recording_seq_; }
    uint32_t intents() const { return intents_; }
    uint32_t events_dropped() const { return ev_dropped_; }
    int64_t  last_intent_us() const { return last_intent_us_; }
    int64_t  start_us() const { return t_start_; }
    int64_t  last_tick_to_clip_us() const { return last_tick_to_clip_us_; }
    bool     recording_now() const {
        return (state_ == wave::State::Recording || state_ == wave::State::Stopping) && t_start_ >= 0;
    }
    int64_t elapsed_us(int64_t now) const { return recording_now() ? now - t_start_ : 0; }

private:
    static constexpr size_t TICK_HIST = 8;

    void emit(wave::EventKind k, wave::Reason r, bool tick, int64_t t, uint8_t seq) {
        if (ev_n_ == EVENT_QUEUE) {
            ++ev_dropped_;
            return;
        }
        ev_[(ev_tail_ + ev_n_) % EVENT_QUEUE] = FsmEvent{k, r, tick, t, seq, recording_seq_};
        ++ev_n_;
    }
    void refuse(wave::Reason r, uint8_t seq, int64_t now) { emit(wave::EventKind::Refused, r, false, now, seq); }

    void try_start(uint8_t seq, int64_t now) {
        want_start_ = false;
        if (console_owns_) return refuse(wave::Reason::ConsoleOwns, seq, now);
        if (!camera_on(now)) return refuse(wave::Reason::CameraOff, seq, now);
        if (!io_.drive_low()) {
            faults_ |= wave::F_GPI2_REFUSED;
            return refuse(wave::Reason::Gpi2Fault, seq, now);
        }
        state_ = wave::State::Starting;
        t_req_ = now;
        t_fall_ = t_start_ = t_clip_ = -1;
        start_tick_stamped_ = false;
        start_seq_ = seq;
    }

    void begin_stop(wave::EventKind kind, wave::Reason reason, uint8_t seq, int64_t now) {
        want_start_ = false;
        const ReleaseResult r = io_.release();
        if (r == ReleaseResult::Refused) {
            faults_ |= wave::F_GPI2_REFUSED;
            emit(kind, wave::Reason::Gpi2Fault, false, now, seq);
            state_ = wave::State::Fault;
            t_start_ = -1;
            return;
        }
        if (r == ReleaseResult::NotDriven) {
            emit(kind, reason, false, now, seq);
            state_ = wave::State::Idle;
            t_start_ = -1;
            return;
        }
        state_ = wave::State::Stopping;
        stop_kind_ = kind;
        stop_reason_ = reason;
        stop_seq_ = seq;
        t_release_req_ = now;
        t_rise_ = t_stop_ = -1;
        stop_tick_stamped_ = false;
    }

    bool intent_timed_out(int64_t now) const {
        return have_intent_ && now - last_intent_us_ >= static_cast<int64_t>(INTENT_TIMEOUT_MS) * 1000;
    }

    void poll_starting(int64_t now) {
        if (intent_timed_out(now)) {
            faults_ |= wave::F_INTENT_TIMEOUT;
            return begin_stop(wave::EventKind::AutoStop, wave::Reason::IntentTimeout, last_seq_, now);
        }
        if (t_fall_ < 0) {
            if (now - t_req_ > static_cast<int64_t>(START_EDGE_TIMEOUT_MS) * 1000) {
                // The pad never went low: release whatever is pending and refuse.
                io_.release();
                faults_ |= wave::F_GPI2_REFUSED;
                refuse(wave::Reason::Gpi2Fault, start_seq_, now);
                state_ = wave::State::Idle;
            }
            return;
        }
        if (t_start_ < 0) {
            if (now - t_fall_ <= static_cast<int64_t>(TICK_AFTER_EDGE_MS) * 1000) return;
            t_start_ = t_fall_;  // camera tick absent: stamp the edge itself
            start_tick_stamped_ = false;
        }
        if (t_clip_ >= 0) {
            cross_check_start_tick();
            enter_recording(wave::Reason::None, now);
        } else if (now - t_start_ > static_cast<int64_t>(CLIP_WAIT_MS) * 1000) {
            faults_ |= wave::F_CLIP_MISSING;
            enter_recording(wave::Reason::ClipMissing, now);
        }
    }

    // The clip line must begin CLIP_LATENCY_MIN..MAX after the start tick. If
    // it does not, use the neighbouring recorded tick that satisfies it.
    void cross_check_start_tick() {
        const int64_t lo = CLIP_LATENCY_MIN_US, hi = CLIP_LATENCY_MAX_US;
        if (start_tick_stamped_) {
            const int64_t lat = t_clip_ - t_start_;
            if (lat < lo || lat > hi) {
                int64_t best = -1;
                for (size_t i = 0; i < tick_n_; ++i) {
                    const int64_t t = tick_hist_[i];
                    const int64_t l = t_clip_ - t;
                    if (l < lo || l > hi) continue;
                    const int64_t d = t > t_start_ ? t - t_start_ : t_start_ - t;
                    const int64_t bd = best > t_start_ ? best - t_start_ : t_start_ - best;
                    if (best < 0 || d < bd) best = t;
                }
                if (best >= 0) t_start_ = best;
            }
        }
        last_tick_to_clip_us_ = t_clip_ - t_start_;
    }

    void enter_recording(wave::Reason reason, int64_t now) {
        state_ = wave::State::Recording;
        ++recordings_session_;
        ++recording_seq_;
        emit(wave::EventKind::Start, reason, start_tick_stamped_, t_start_, start_seq_);
        if (!desired_) begin_stop(wave::EventKind::Stop, wave::Reason::None, last_seq_, now);
    }

    void poll_recording(int64_t now) {
        if (intent_timed_out(now)) {
            faults_ |= wave::F_INTENT_TIMEOUT;
            return begin_stop(wave::EventKind::AutoStop, wave::Reason::IntentTimeout, last_seq_, now);
        }
        if (last_tick_us_ < 0 || now - last_tick_us_ > static_cast<int64_t>(TICK_LOST_MS) * 1000) {
            faults_ |= wave::F_TICK_LOST_MID_REC;
            return begin_stop(wave::EventKind::AutoStop, wave::Reason::TickLost, last_seq_, now);
        }
    }

    void poll_stopping(int64_t now) {
        if (t_rise_ < 0) {
            if (now - t_release_req_ > static_cast<int64_t>(STOP_EDGE_TIMEOUT_MS) * 1000) {
                faults_ |= wave::F_GPI2_REFUSED;
                emit(stop_kind_, wave::Reason::Gpi2Fault, false, now, stop_seq_);
                state_ = wave::State::Fault;
                t_start_ = -1;
            }
            return;
        }
        if (t_stop_ < 0) {
            if (now - t_rise_ <= static_cast<int64_t>(TICK_AFTER_EDGE_MS) * 1000) return;
            t_stop_ = t_rise_;
            stop_tick_stamped_ = false;
        }
        emit(stop_kind_, stop_reason_, stop_tick_stamped_, t_stop_, stop_seq_);
        state_ = wave::State::Idle;
        t_start_ = -1;
        if (want_start_ && desired_) try_start(want_start_seq_, now);
    }

    RecordIo&   io_;
    wave::State state_ = wave::State::Boot;
    uint8_t     faults_ = 0;
    bool        console_owns_ = false;
    bool        uart_seen_ = false;
    bool        pad_low_ = false;

    bool     have_intent_ = false;
    bool     desired_ = false;
    int64_t  last_intent_us_ = 0;
    uint8_t  last_seq_ = 0;
    uint8_t  acted_seq_ = 0;
    bool     have_session_ = false;
    uint32_t session_id_ = 0;
    uint32_t intents_ = 0;
    bool     want_start_ = false;
    uint8_t  want_start_seq_ = 0;

    uint8_t recordings_session_ = 0;
    uint8_t recording_seq_ = 0;

    // STARTING
    int64_t t_req_ = -1;
    int64_t t_fall_ = -1;
    int64_t t_start_ = -1;
    int64_t t_clip_ = -1;
    bool    start_tick_stamped_ = false;
    uint8_t start_seq_ = 0;
    int64_t last_tick_to_clip_us_ = -1;

    // STOPPING
    wave::EventKind stop_kind_ = wave::EventKind::Stop;
    wave::Reason    stop_reason_ = wave::Reason::None;
    uint8_t         stop_seq_ = 0;
    int64_t         t_release_req_ = -1;
    int64_t         t_rise_ = -1;
    int64_t         t_stop_ = -1;
    bool            stop_tick_stamped_ = false;

    // Ticks
    int64_t last_tick_us_ = -1;
    int64_t tick_hist_[TICK_HIST] = {};
    size_t  tick_head_ = 0;
    size_t  tick_n_ = 0;

    // Event queue
    FsmEvent ev_[EVENT_QUEUE] = {};
    size_t   ev_tail_ = 0;
    size_t   ev_n_ = 0;
    uint32_t ev_dropped_ = 0;
};
