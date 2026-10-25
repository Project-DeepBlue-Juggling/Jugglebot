#include "record_ctrl.h"

#include "console.h"
#include "edge_capture.h"
#include "gpi2_drive.h"
#include "latency_test.h"
#include "record_fsm.h"
#include "time_slave.h"

namespace {

// The FSM's view of GPI2, on top of gpi2_drive (which enforces the 500 ms
// minimum states by stretching).
class Gpi2Io : public RecordIo {
public:
    bool drive_low() override { return gpi2_drive::request_start(); }
    ReleaseResult release() override {
        if (!gpi2_drive::pad_held_low()) {
            // Never driven: drop a start still waiting out its stretch.
            if (gpi2_drive::cancel_pending()) console::out("REC", "pending GPI2 start cancelled");
            return ReleaseResult::NotDriven;
        }
        if (gpi2_drive::busy()) return ReleaseResult::Pending;  // a stop is already scheduled
        return gpi2_drive::request_stop() ? ReleaseResult::Pending : ReleaseResult::Refused;
    }
};

Gpi2Io       s_io;
RecordFsm    s_fsm(s_io);
TickDetector s_tick_detect;
TickTracker  s_ticks;
wave::State  s_last_state = wave::State::Boot;

uint32_t s_intents_rx = 0;
uint32_t s_cmd_bad = 0;      // desired > 1, DLC != 8, extended / RTR
uint8_t  s_last_flags = 0;

// Latched faults that do not come from the FSM, with the counter values at
// the last CLEAR_FAULTS so a clear really clears.
uint8_t  s_extra_faults = 0;
uint32_t s_bus_off_base = 0;
uint32_t s_steps_base = 0;
uint32_t s_overflow_base = 0;

bool     s_sim = false;
uint8_t  s_sim_seq = 200;
int64_t  s_sim_next_us = 0;

uint32_t s_events_sent = 0;
uint32_t s_events_queue_full = 0;

void clear_extra_faults() {
    s_extra_faults = 0;
    s_bus_off_base = can_link::stats().bus_off_events;
    s_steps_base = time_slave::steps();
    s_overflow_base = edge_capture::overflows();
}

void update_extra_faults() {
    if (can_link::stats().bus_off_events != s_bus_off_base) s_extra_faults |= wave::F_BUS_OFF_SEEN;
    if (time_slave::steps() != s_steps_base) s_extra_faults |= wave::F_SYNC_STEP;
    if (edge_capture::overflows() != s_overflow_base) s_extra_faults |= wave::F_EDGE_RING_OVERFLOW;
}

void send_event(const FsmEvent& e, int64_t now) {
    wave::RecEvent r{};
    r.kind = e.kind;
    r.reason = e.reason;
    r.tick_stamped = e.tick_stamped;
    r.cmd_seq = e.cmd_seq;
    r.recording_seq = e.recording_seq;
    int64_t wall = 0;
    r.synced = time_slave::synced(now) && time_slave::wall_from_local(e.t_local_us, wall);
    r.t40 = r.synced ? wave::t40_from_wall(wall) : wave::t40_from_age(now - e.t_local_us);
    uint8_t d[8];
    wave::encode_rec_event(r, d);
    // Unsynced: the TX task rewrites the age at each attempt.
    const bool queued = can_link::send(wave::ID_REC_EVENT, d, REC_EVENT_TX_RETRIES, r.synced ? -1 : e.t_local_us);
    if (queued) ++s_events_sent;
    else ++s_events_queue_full;
    char wall_txt[24];
    if (r.synced) snprintf(wall_txt, sizeof(wall_txt), "%lld", (long long)wall);
    else snprintf(wall_txt, sizeof(wall_txt), "-");
    console::out("REC", "CAN event=%s reason=%s seq=%u recording_seq=%u t_local_us=%lld wall_us=%s synced=%d "
                        "tick_stamped=%d tick_to_clip_us=%lld queued=%d",
                 wave::kind_name(e.kind), wave::reason_name(e.reason), e.cmd_seq, e.recording_seq,
                 (long long)e.t_local_us, wall_txt, r.synced, e.tick_stamped, (long long)s_fsm.last_tick_to_clip_us(),
                 queued);
}

}  // namespace

namespace record_ctrl {

void begin() {
    gpi2_drive::set_mode(RecMode::Level);
    s_fsm.set_pad_low(pin_io::read_level(PIN_GPI2) == 0);
    clear_extra_faults();
}

void poll(int64_t now) {
    if (s_sim && now >= s_sim_next_us) {
        s_fsm.on_intent(1, s_sim_seq, 0, s_fsm.session_id(), now);
        s_sim_next_us += static_cast<int64_t>(INTENT_PERIOD_MS) * 1000;
        if (now - s_sim_next_us > static_cast<int64_t>(SCHEDULE_RESYNC_PERIODS * INTENT_PERIOD_MS) * 1000)
            s_sim_next_us = now + static_cast<int64_t>(INTENT_PERIOD_MS) * 1000;
    }
    s_fsm.poll(now);
    FsmEvent e;
    while (s_fsm.take_event(e)) send_event(e, now);
    update_extra_faults();
    if (s_fsm.state() != s_last_state) {
        console::out("REC", "CAN state %s -> %s", wave::state_name(s_last_state), wave::state_name(s_fsm.state()));
        s_last_state = s_fsm.state();
    }
}

void on_edge(const EdgeEvent& ev) {
    if (ev.gpio == PIN_GPI2) {
        s_fsm.on_gpi2_edge(ev.level, ev.t_us);
    } else if (ev.gpio == PIN_CAM_P4) {
        if (s_tick_detect.is_tick(ev.t_us)) {
            s_fsm.on_tick(ev.t_us);
            s_ticks.on_tick(ev.t_us);
        }
    }
}

void on_clip_line(int64_t t_first_us) { s_fsm.on_clip_line(t_first_us); }

void on_cmd(const can_link::RxFrame& f) {
    wave::Cmd c{};
    if (wave::decode_cmd(f.data, f.dlc, f.extd, f.rtr, c) != wave::CmdDecode::Ok) {
        ++s_cmd_bad;
        return;
    }
    ++s_intents_rx;
    s_last_flags = c.flags;
    if (c.flags & wave::CMD_FLAG_CLEAR_FAULTS) clear_extra_faults();
    s_fsm.on_intent(c.desired, c.cmd_seq, c.flags, c.session_id, f.t_rx_us);
}

bool ctl_console() {
    if (s_fsm.console_owns()) {
        console::out("INFO", "ctl: console already owns gpi2");
        return true;
    }
    if (!s_fsm.set_console_owns(true)) {
        console::out("ERR", "ctl console refused: CAN state is %s (must be IDLE)", wave::state_name(s_fsm.state()));
        return false;
    }
    console::out("INFO", "ctl: console owns gpi2; CAN intents are REFUSED CONSOLE_OWNS. `ctl can` hands back");
    return true;
}

bool ctl_can() {
    if (!s_fsm.console_owns()) {
        console::out("INFO", "ctl: CAN already owns gpi2");
        return true;
    }
    if (latency_test::running()) {
        console::out("ERR", "ctl can refused: lat running; `lat abort` first");
        return false;
    }
    if (gpi2_drive::busy() || gpi2_drive::pad_held_low()) {
        console::out("ERR", "ctl can refused: GPI2 busy or held low; `rec stop` first");
        return false;
    }
    if (gpi2_drive::mode() != RecMode::Level || !gpi2_drive::mode_confirmed()) gpi2_drive::set_mode(RecMode::Level);
    gpi2_drive::assume(RecState::Idle);
    s_fsm.set_console_owns(false);
    console::out("INFO", "ctl: CAN owns gpi2 (level mode)");
    return true;
}

bool console_owns() { return s_fsm.console_owns(); }

bool clear_fault_state() { return s_fsm.clear_fault_state(); }

void sim_intent(bool on) {
    if (on) {
        ++s_sim_seq;
        s_sim = true;
        s_sim_next_us = esp_timer_get_time();
        console::out("INFO", "sim intent ON (BENCH ONLY): desired=1 seq=%u at %lu Hz, session_id=%lu", s_sim_seq,
                     (unsigned long)INTENT_RATE_HZ, (unsigned long)s_fsm.session_id());
    } else {
        if (s_sim) console::out("INFO", "sim intent OFF: no more local intents (AUTO_STOP after %lu ms if recording)",
                                (unsigned long)INTENT_TIMEOUT_MS);
        s_sim = false;
    }
}
bool sim_intent_on() { return s_sim; }

wave::Heartbeat heartbeat(int64_t now) {
    wave::Heartbeat h{};
    h.state = s_fsm.state();
    uint8_t st = 0;
    if (s_fsm.camera_on(now)) st |= wave::ST_CAMERA_ON;
    if (s_fsm.uart_seen()) st |= wave::ST_UART_SEEN;
    if (time_slave::synced(now)) st |= wave::ST_TIME_SYNCED;
    if (s_fsm.intent_fresh(now)) st |= wave::ST_INTENT_FRESH;
    if (s_fsm.desired()) st |= wave::ST_DESIRED;
    if (s_fsm.console_owns()) st |= wave::ST_CONSOLE_OWNS;
    if (s_fsm.pad_low()) st |= wave::ST_GPI2_LOW;
    h.status = st;
    h.faults = static_cast<uint8_t>(s_fsm.faults() | s_extra_faults);
    h.elapsed_100ms = wave::elapsed_100ms(s_fsm.elapsed_us(now));
    h.sync_age_100ms = time_slave::sync_age_100ms(now);
    h.recordings = s_fsm.recordings_session();
    h.cmd_seq_echo = s_fsm.acted_seq();
    return h;
}

uint32_t tick_period_ns() { return s_ticks.period_ns(); }
uint32_t tick_jitter_us() { return s_ticks.jitter_pp_us(); }
uint32_t ticks_missed_total() { return s_ticks.missed_total(); }
int64_t  last_tick_to_clip_us() { return s_fsm.last_tick_to_clip_us(); }
uint32_t intents_rx() { return s_intents_rx; }

void print_status(int64_t now) {
    const wave::Heartbeat h = heartbeat(now);
    const int64_t li = s_fsm.last_intent_us();
    char age[24];
    if (s_fsm.have_intent()) snprintf(age, sizeof(age), "%lld", (long long)((now - li) / 1000));
    else snprintf(age, sizeof(age), "-");
    console::out("INFO",
                 "can rec state=%s owner=%s sim_intent=%d camera_on=%d uart_seen=%d gpi2_low=%d faults=0x%02X "
                 "elapsed_s=%.1f recordings=%u recording_seq=%u session_id=%lu",
                 wave::state_name(h.state), s_fsm.console_owns() ? "console" : "can", s_sim,
                 (h.status & wave::ST_CAMERA_ON) != 0, (h.status & wave::ST_UART_SEEN) != 0,
                 (h.status & wave::ST_GPI2_LOW) != 0, h.faults, h.elapsed_100ms / 10.0, h.recordings,
                 s_fsm.recording_seq(), (unsigned long)s_fsm.session_id());
    console::out("INFO",
                 "can intent last_desired=%u last_seq=%u acted_seq=%u last_flags=0x%02X age_ms=%s fresh=%d rx=%lu "
                 "bad=%lu events_sent=%lu events_queue_full=%lu fsm_events_dropped=%lu",
                 s_fsm.desired(), s_fsm.last_seq(), s_fsm.acted_seq(), s_last_flags, age,
                 (h.status & wave::ST_INTENT_FRESH) != 0, (unsigned long)s_intents_rx, (unsigned long)s_cmd_bad,
                 (unsigned long)s_events_sent, (unsigned long)s_events_queue_full,
                 (unsigned long)s_fsm.events_dropped());
    console::out("INFO", "can tick period_ns=%lu jitter_pp_us=%lu missed_total=%lu last_tick_to_clip_us=%lld",
                 (unsigned long)s_ticks.period_ns(), (unsigned long)s_ticks.jitter_pp_us(),
                 (unsigned long)s_ticks.missed_total(), (long long)s_fsm.last_tick_to_clip_us());
}

}  // namespace record_ctrl
