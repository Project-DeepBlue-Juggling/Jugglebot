// Recording state machine (src/record_fsm.h), wave_camera_can_adapter.md s5.2-s5.4.
// All time is synthetic: edges, ticks and the clip line carry their own
// stamps exactly as the ISR / UART stamping would deliver them.

#include <unity.h>

#include "record_fsm.h"

using namespace wave;

namespace {

constexpr int64_t MS = 1000;
constexpr int64_t TICK = TICK_NOMINAL_PERIOD_US;

struct FakeIo : RecordIo {
    bool          allow_drive = true;
    ReleaseResult release_result = ReleaseResult::Pending;
    int           drives = 0;
    int           releases = 0;
    bool          low = false;  // commanded level
    bool drive_low() override {
        ++drives;
        if (!allow_drive) return false;
        low = true;
        return true;
    }
    ReleaseResult release() override {
        ++releases;
        if (release_result == ReleaseResult::Pending) low = false;
        return release_result;
    }
};

// A bench harness: camera ticks every TICK from t=0 while camera_on, GPI2
// edges follow the commanded level after `edge_lag`, and the clip line
// follows the first tick after a falling edge by `clip_after_tick`.
struct Bench {
    FakeIo    io;
    RecordFsm fsm{io};
    int64_t   now = 0;
    int64_t   next_tick = 0;
    bool      camera_on = true;
    bool      clip_enabled = true;
    int64_t   clip_after_tick = 15500;
    int64_t   edge_lag = 20;
    bool      pad_low = false;
    int64_t   pending_edge_at = -1;
    int64_t   pending_clip_at = -1;
    int64_t   last_fall = -1;

    // Advance in 1 ms steps, delivering ticks/edges/clip lines in time order.
    void run_until(int64_t t_end, bool intents = false, uint8_t desired = 0, uint8_t seq = 0, int64_t intent_period = 200 * MS) {
        int64_t next_intent = now;
        while (now < t_end) {
            now += MS;
            if (io.low != pad_low && pending_edge_at < 0) pending_edge_at = now - MS + edge_lag;
            const bool edge_now = pending_edge_at >= 0 && pending_edge_at <= now;
            deliver_ticks(edge_now ? pending_edge_at - 1 : now);  // time order: ticks before the edge first
            if (edge_now) {
                pad_low = io.low;
                fsm.on_gpi2_edge(pad_low ? 0 : 1, pending_edge_at);
                if (pad_low) last_fall = pending_edge_at;
                pending_edge_at = -1;
            }
            deliver_ticks(now);
            if (pending_clip_at >= 0 && pending_clip_at <= now) {
                fsm.on_clip_line(pending_clip_at);
                pending_clip_at = -1;
            }
            if (intents && now >= next_intent) {
                fsm.on_intent(desired, seq, 0, 1000, now);
                next_intent += intent_period;
            }
            fsm.poll(now);
        }
    }
    void deliver_ticks(int64_t upto) {
        if (!camera_on) {
            next_tick = now + TICK;
            return;
        }
        while (next_tick <= upto) {
            fsm.on_tick(next_tick);
            if (last_fall >= 0 && next_tick > last_fall) {
                if (pending_clip_at < 0 && clip_enabled) pending_clip_at = next_tick + clip_after_tick;
                last_fall = -1;
            }
            next_tick += TICK;
        }
    }
    void intent(uint8_t desired, uint8_t seq, uint8_t flags = 0, uint32_t session = 1000) {
        fsm.on_intent(desired, seq, flags, session, now);
    }
    bool event(FsmEvent& e) { return fsm.take_event(e); }
};

void expect_event(Bench& b, EventKind k, Reason r, uint8_t seq) {
    FsmEvent e{};
    TEST_ASSERT_TRUE_MESSAGE(b.event(e), "expected an event");
    TEST_ASSERT_EQUAL_STRING(kind_name(k), kind_name(e.kind));
    TEST_ASSERT_EQUAL_STRING(reason_name(r), reason_name(e.reason));
    TEST_ASSERT_EQUAL_UINT8(seq, e.cmd_seq);
}

void expect_no_event(Bench& b) {
    FsmEvent e{};
    TEST_ASSERT_FALSE_MESSAGE(b.event(e), "unexpected event");
}

}  // namespace

void setUp() {}
void tearDown() {}

static void test_boot_to_idle() {
    Bench b;
    TEST_ASSERT_EQUAL(static_cast<int>(State::Boot), static_cast<int>(b.fsm.state()));
    b.run_until(5 * MS);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
}

static void test_happy_path_start_and_stop() {
    Bench b;
    b.run_until(1000 * MS);
    TEST_ASSERT_TRUE(b.fsm.camera_on(b.now));
    b.intent(1, 7);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Starting), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_EQUAL(1, b.io.drives);
    const int64_t t_cmd = b.now;
    b.run_until(t_cmd + 300 * MS, true, 1, 7);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Recording), static_cast<int>(b.fsm.state()));
    FsmEvent e{};
    TEST_ASSERT_TRUE(b.event(e));
    TEST_ASSERT_EQUAL(static_cast<int>(EventKind::Start), static_cast<int>(e.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(Reason::None), static_cast<int>(e.reason));
    TEST_ASSERT_TRUE(e.tick_stamped);
    TEST_ASSERT_EQUAL_UINT8(7, e.cmd_seq);
    TEST_ASSERT_EQUAL_UINT8(1, e.recording_seq);
    // Start stamp = the first tick after the falling edge (t_cmd + 20 us).
    const int64_t fall = t_cmd + 20;
    const int64_t first_tick = ((fall / TICK) + 1) * TICK;
    TEST_ASSERT_EQUAL_INT64(first_tick, e.t_local_us);
    TEST_ASSERT_EQUAL_INT64(15500, b.fsm.last_tick_to_clip_us());
    TEST_ASSERT_EQUAL_UINT8(1, b.fsm.recordings_session());
    TEST_ASSERT_EQUAL_UINT8(0, b.fsm.faults());
    TEST_ASSERT_TRUE(b.io.low);
    // Elapsed counts from the start tick.
    TEST_ASSERT_EQUAL_INT64(b.now - first_tick, b.fsm.elapsed_us(b.now));

    // Keepalives: no new events.
    b.run_until(b.now + 2000 * MS, true, 1, 7);
    expect_no_event(b);

    // Stop.
    b.intent(0, 8);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Stopping), static_cast<int>(b.fsm.state()));
    const int64_t t_stop_cmd = b.now;
    b.run_until(b.now + 200 * MS, true, 0, 8);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_TRUE(b.event(e));
    TEST_ASSERT_EQUAL(static_cast<int>(EventKind::Stop), static_cast<int>(e.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(Reason::None), static_cast<int>(e.reason));
    TEST_ASSERT_TRUE(e.tick_stamped);
    TEST_ASSERT_EQUAL_UINT8(8, e.cmd_seq);
    const int64_t rise = t_stop_cmd + 20;
    TEST_ASSERT_EQUAL_INT64(((rise / TICK) + 1) * TICK, e.t_local_us);
    TEST_ASSERT_FALSE(b.io.low);
    TEST_ASSERT_EQUAL_INT64(0, b.fsm.elapsed_us(b.now));
    TEST_ASSERT_EQUAL_UINT8(8, b.fsm.acted_seq());
    expect_no_event(b);
}

static void test_clip_missing_keeps_gpi2_low() {
    Bench b;
    b.clip_enabled = false;
    b.run_until(500 * MS);
    b.intent(1, 3);
    b.run_until(b.now + 400 * MS, true, 1, 3);
    FsmEvent e{};
    TEST_ASSERT_TRUE(b.event(e));
    TEST_ASSERT_EQUAL(static_cast<int>(EventKind::Start), static_cast<int>(e.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(Reason::ClipMissing), static_cast<int>(e.reason));
    TEST_ASSERT_TRUE(e.tick_stamped);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Recording), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_TRUE(b.io.low);  // owner decision: GPI2 stays low
    TEST_ASSERT_EQUAL_UINT8(F_CLIP_MISSING, b.fsm.faults());
    TEST_ASSERT_EQUAL(0, b.io.releases);
}

static void test_intent_timeout_auto_stop_at_3s() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 9);
    b.run_until(b.now + 400 * MS, true, 1, 9);
    FsmEvent e{};
    TEST_ASSERT_TRUE(b.event(e));  // START
    // The last intent arrives at t_last; nothing after it.
    b.intent(1, 9);
    const int64_t t_last = b.now;
    b.run_until(t_last + 2999 * MS);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Recording), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_EQUAL(0, b.io.releases);
    b.run_until(t_last + 3000 * MS);
    TEST_ASSERT_EQUAL(1, b.io.releases);  // released at exactly 3.000 s
    TEST_ASSERT_EQUAL(static_cast<int>(State::Stopping), static_cast<int>(b.fsm.state()));
    b.run_until(b.now + 200 * MS);
    TEST_ASSERT_TRUE(b.event(e));
    TEST_ASSERT_EQUAL(static_cast<int>(EventKind::AutoStop), static_cast<int>(e.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(Reason::IntentTimeout), static_cast<int>(e.reason));
    TEST_ASSERT_EQUAL_UINT8(9, e.cmd_seq);  // last seen
    TEST_ASSERT_EQUAL_UINT8(F_INTENT_TIMEOUT, b.fsm.faults());
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_FALSE(b.io.low);
}

static void test_tick_lost_mid_recording_releases_gpi2() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 4);
    b.run_until(b.now + 400 * MS, true, 1, 4);
    FsmEvent e{};
    TEST_ASSERT_TRUE(b.event(e));  // START
    b.camera_on = false;
    const int64_t t_off = b.now;
    b.run_until(t_off + 500 * MS, true, 1, 4);
    TEST_ASSERT_EQUAL(1, b.io.releases);
    TEST_ASSERT_FALSE(b.io.low);
    TEST_ASSERT_TRUE(b.event(e));
    TEST_ASSERT_EQUAL(static_cast<int>(EventKind::AutoStop), static_cast<int>(e.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(Reason::TickLost), static_cast<int>(e.reason));
    TEST_ASSERT_FALSE(e.tick_stamped);  // no tick after the rising edge: stamped from the edge
    TEST_ASSERT_TRUE(b.fsm.faults() & F_TICK_LOST_MID_REC);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    // The stream still says desired=1 with the same seq: no restart, no event.
    b.camera_on = true;
    b.run_until(b.now + 500 * MS, true, 1, 4);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    expect_no_event(b);
}

static void test_console_owns_refusal() {
    Bench b;
    b.run_until(500 * MS);
    TEST_ASSERT_TRUE(b.fsm.set_console_owns(true));
    b.intent(1, 1);
    expect_event(b, EventKind::Refused, Reason::ConsoleOwns, 1);
    TEST_ASSERT_EQUAL(0, b.io.drives);
    b.intent(0, 2);
    expect_event(b, EventKind::Refused, Reason::ConsoleOwns, 2);
    TEST_ASSERT_TRUE(b.fsm.set_console_owns(false));
    // Taking ownership is refused outside IDLE.
    b.intent(1, 3);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Starting), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_FALSE(b.fsm.set_console_owns(true));
    TEST_ASSERT_FALSE(b.fsm.console_owns());
}

static void test_no_change_refusals_and_seq_echo() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(0, 20);  // stop while idle
    expect_event(b, EventKind::Refused, Reason::NoChange, 20);
    TEST_ASSERT_EQUAL_UINT8(20, b.fsm.acted_seq());
    b.intent(0, 20);  // keepalive
    expect_no_event(b);
    b.intent(1, 21);
    b.run_until(b.now + 300 * MS, true, 1, 21);
    expect_event(b, EventKind::Start, Reason::None, 21);
    b.intent(1, 22);  // start while recording
    expect_event(b, EventKind::Refused, Reason::NoChange, 22);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Recording), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_EQUAL_UINT8(22, b.fsm.acted_seq());
    b.intent(1, 22);
    expect_no_event(b);
}

static void test_camera_off_refusal() {
    Bench b;
    b.camera_on = false;
    b.run_until(500 * MS);
    b.intent(1, 1);
    expect_event(b, EventKind::Refused, Reason::CameraOff, 1);
    TEST_ASSERT_EQUAL(0, b.io.drives);
}

static void test_drive_refused_is_gpi2_fault() {
    Bench b;
    b.run_until(500 * MS);
    b.io.allow_drive = false;
    b.intent(1, 1);
    expect_event(b, EventKind::Refused, Reason::Gpi2Fault, 1);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_EQUAL_UINT8(F_GPI2_REFUSED, b.fsm.faults());
    b.intent(0, 2, CMD_FLAG_CLEAR_FAULTS);
    TEST_ASSERT_EQUAL_UINT8(0, b.fsm.faults());
}

static void test_no_falling_edge_times_out() {
    Bench b;
    b.edge_lag = 5000 * MS;  // pad never follows within the window
    b.run_until(500 * MS);
    b.intent(1, 1);
    const int64_t t0 = b.now;
    b.run_until(t0 + START_EDGE_TIMEOUT_MS * MS + 5 * MS, true, 1, 1);
    expect_event(b, EventKind::Refused, Reason::Gpi2Fault, 1);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_EQUAL(1, b.io.releases);
}

static void test_start_stamped_from_edge_without_tick() {
    // Ticks stop right after the start request (camera_on was true at the
    // request): stamp the falling edge, TICK_STAMPED clear.
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 1);
    const int64_t t0 = b.now;
    b.camera_on = false;
    b.run_until(t0 + 230 * MS, true, 1, 1);
    FsmEvent e{};
    TEST_ASSERT_TRUE(b.event(e));
    TEST_ASSERT_EQUAL(static_cast<int>(EventKind::Start), static_cast<int>(e.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(Reason::ClipMissing), static_cast<int>(e.reason));
    TEST_ASSERT_FALSE(e.tick_stamped);
    TEST_ASSERT_EQUAL_INT64(t0 + 20, e.t_local_us);
}

static void test_stop_during_starting_completes_then_stops() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 1);
    b.run_until(b.now + 2 * MS);
    b.intent(0, 2);  // node gave up (e.g. its own timeout)
    b.run_until(b.now + 400 * MS, true, 0, 2);
    expect_event(b, EventKind::Start, Reason::None, 1);
    expect_event(b, EventKind::Stop, Reason::None, 2);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
}

static void test_session_change_resets_recordings() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 1);
    b.run_until(b.now + 300 * MS, true, 1, 1);
    b.intent(0, 2);
    b.run_until(b.now + 700 * MS, true, 0, 2);
    TEST_ASSERT_EQUAL_UINT8(1, b.fsm.recordings_session());
    TEST_ASSERT_EQUAL_UINT8(1, b.fsm.recording_seq());
    b.fsm.on_intent(0, 2, 0, 2000, b.now);  // new ROS launch
    TEST_ASSERT_EQUAL_UINT8(0, b.fsm.recordings_session());
    TEST_ASSERT_EQUAL_UINT8(1, b.fsm.recording_seq());  // since boot, not reset
}

static void test_clip_cross_check_moves_to_neighbour_tick() {
    // The clip line appears 48.8 ms after the first tick following the
    // falling edge, outside the 10-25 ms window: the cross-check moves the
    // start to the next recorded tick, 15.5 ms before the line.
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 1);
    b.clip_after_tick = 15500 + TICK;  // 48.8 ms after the first tick
    b.run_until(b.now + 400 * MS, true, 1, 1);
    FsmEvent e{};
    TEST_ASSERT_TRUE(b.event(e));
    TEST_ASSERT_EQUAL(static_cast<int>(Reason::None), static_cast<int>(e.reason));
    TEST_ASSERT_EQUAL_INT64(15500, b.fsm.last_tick_to_clip_us());
}

static void test_tick_detector_and_tracker() {
    TickDetector d;
    TEST_ASSERT_TRUE(d.is_tick(0));
    TEST_ASSERT_FALSE(d.is_tick(8));  // trailing edge of the 8 us pulse
    TEST_ASSERT_TRUE(d.is_tick(33334));
    TEST_ASSERT_FALSE(d.is_tick(33343));
    TickTracker t;
    for (int i = 0; i < 40; ++i) t.on_tick(i * 33334LL + (i % 2));
    TEST_ASSERT_UINT32_WITHIN(1000, 33334000u, t.period_ns());
    TEST_ASSERT_EQUAL_UINT32(2, t.jitter_pp_us());
    t.on_tick(39 * 33334LL + 3 * 33334LL);  // two missed
    TEST_ASSERT_EQUAL_UINT32(2, t.missed_total());
}

// Review fix 1: the new-seq stop frame is lost; repeats that carry the old
// seq byte with desired=0 still stop the recording.
static void test_lost_stop_frame_repaired_by_same_seq_repeats() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 7);
    b.run_until(b.now + 400 * MS, true, 1, 7);
    expect_event(b, EventKind::Start, Reason::None, 7);
    b.intent(0, 7);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Stopping), static_cast<int>(b.fsm.state()));
    b.run_until(b.now + 200 * MS, true, 0, 7);
    expect_event(b, EventKind::Stop, Reason::None, 7);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_FALSE(b.io.low);
    expect_no_event(b);
}

// Review fix 2: a second start seq while STARTING is acked NO_CHANGE; the
// original keeps its START.
static void test_new_seq_during_starting_is_no_change() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 1);
    b.run_until(b.now + 2 * MS, true, 1, 1);
    b.intent(1, 2);
    expect_event(b, EventKind::Refused, Reason::NoChange, 2);
    b.run_until(b.now + 400 * MS, true, 1, 2);
    expect_event(b, EventKind::Start, Reason::None, 1);
    expect_no_event(b);
}

static void test_new_seq_during_stopping_is_no_change() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 1);
    b.run_until(b.now + 400 * MS, true, 1, 1);
    expect_event(b, EventKind::Start, Reason::None, 1);
    b.intent(0, 2);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Stopping), static_cast<int>(b.fsm.state()));
    b.intent(0, 3);
    expect_event(b, EventKind::Refused, Reason::NoChange, 3);
    b.run_until(b.now + 200 * MS, true, 0, 3);
    expect_event(b, EventKind::Stop, Reason::None, 2);
    expect_no_event(b);
}

static void test_stop_after_intent_timeout_is_prompt_no_change() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 9);
    b.run_until(b.now + 400 * MS, true, 1, 9);
    expect_event(b, EventKind::Start, Reason::None, 9);
    b.intent(1, 9);
    while (b.fsm.state() == State::Recording) b.run_until(b.now + MS);  // AUTO_STOP fires
    TEST_ASSERT_EQUAL(static_cast<int>(State::Stopping), static_cast<int>(b.fsm.state()));
    b.intent(0, 10);  // the node's stop arrives while the auto-stop is in flight
    expect_event(b, EventKind::Refused, Reason::NoChange, 10);
    b.run_until(b.now + 200 * MS, true, 0, 10);
    expect_event(b, EventKind::AutoStop, Reason::IntentTimeout, 9);
    b.intent(0, 11);  // and once IDLE
    expect_event(b, EventKind::Refused, Reason::NoChange, 11);
}

static void test_session_change_mid_recording() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 5);
    b.run_until(b.now + 400 * MS, true, 1, 5);
    expect_event(b, EventKind::Start, Reason::None, 5);
    TEST_ASSERT_EQUAL_UINT8(1, b.fsm.recordings_session());
    // ROS relaunch: same seq byte, desired=1, new session -> treated as a new seq.
    b.intent(1, 5, 0, 2000);
    expect_event(b, EventKind::Refused, Reason::NoChange, 5);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Recording), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_EQUAL_UINT8(0, b.fsm.recordings_session());
    TEST_ASSERT_EQUAL_UINT8(1, b.fsm.recording_seq());
    TEST_ASSERT_EQUAL_UINT32(2000, b.fsm.session_id());
}

static void test_desired1_keepalive_during_stopping_does_not_restart() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 4);
    b.run_until(b.now + 400 * MS, true, 1, 4);
    expect_event(b, EventKind::Start, Reason::None, 4);
    b.camera_on = false;
    b.run_until(b.now + 215 * MS, true, 1, 4);  // TICK_LOST -> STOPPING
    TEST_ASSERT_EQUAL(static_cast<int>(State::Stopping), static_cast<int>(b.fsm.state()));
    b.intent(1, 4);
    b.camera_on = true;
    b.run_until(b.now + 300 * MS, true, 1, 4);
    expect_event(b, EventKind::AutoStop, Reason::TickLost, 4);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_EQUAL(1, b.io.drives);
    expect_no_event(b);
}

static void test_fault_entry_and_exit() {
    Bench b;
    b.run_until(500 * MS);
    b.intent(1, 1);
    b.run_until(b.now + 400 * MS, true, 1, 1);
    expect_event(b, EventKind::Start, Reason::None, 1);
    b.io.release_result = ReleaseResult::Refused;
    b.intent(0, 2);
    expect_event(b, EventKind::Stop, Reason::Gpi2Fault, 2);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Fault), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_TRUE(b.fsm.faults() & F_GPI2_REFUSED);
    b.intent(1, 3);
    expect_event(b, EventKind::Refused, Reason::Gpi2Fault, 3);
    b.intent(1, 3);
    expect_no_event(b);
    TEST_ASSERT_FALSE(b.fsm.set_console_owns(true));
    // Exit 1: CLEAR_FAULTS.
    b.intent(0, 4, CMD_FLAG_CLEAR_FAULTS);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_EQUAL_UINT8(0, b.fsm.faults());
    expect_event(b, EventKind::Refused, Reason::NoChange, 4);
    // Exit 2: the console's `rec assume idle`, after a stop-edge timeout.
    b.io.release_result = ReleaseResult::Pending;
    b.io.low = false;  // the operator freed the pad
    b.run_until(b.now + 600 * MS);
    b.intent(1, 5);
    b.run_until(b.now + 400 * MS, true, 1, 5);
    expect_event(b, EventKind::Start, Reason::None, 5);
    b.edge_lag = 5000 * MS;
    b.intent(0, 6);
    b.run_until(b.now + STOP_EDGE_TIMEOUT_MS * MS + 5 * MS, true, 0, 6);
    expect_event(b, EventKind::Stop, Reason::Gpi2Fault, 6);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Fault), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_TRUE(b.fsm.clear_fault_state());
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_FALSE(b.fsm.clear_fault_state());
}

static void test_intent_timeout_while_starting() {
    // The intent that starts the take is already 3 s old at the first poll
    // (stamped by the RX task long before loop() handled it).
    Bench b;
    b.run_until(500 * MS);
    b.fsm.on_intent(1, 5, 0, 1000, b.now - 3000 * MS);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Starting), static_cast<int>(b.fsm.state()));
    b.io.release_result = ReleaseResult::NotDriven;  // start still waiting out its stretch
    b.fsm.poll(b.now);
    expect_event(b, EventKind::AutoStop, Reason::IntentTimeout, 5);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Idle), static_cast<int>(b.fsm.state()));
    TEST_ASSERT_EQUAL_UINT8(F_INTENT_TIMEOUT, b.fsm.faults());
    TEST_ASSERT_EQUAL(1, b.io.releases);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_boot_to_idle);
    RUN_TEST(test_happy_path_start_and_stop);
    RUN_TEST(test_clip_missing_keeps_gpi2_low);
    RUN_TEST(test_intent_timeout_auto_stop_at_3s);
    RUN_TEST(test_tick_lost_mid_recording_releases_gpi2);
    RUN_TEST(test_console_owns_refusal);
    RUN_TEST(test_no_change_refusals_and_seq_echo);
    RUN_TEST(test_camera_off_refusal);
    RUN_TEST(test_drive_refused_is_gpi2_fault);
    RUN_TEST(test_no_falling_edge_times_out);
    RUN_TEST(test_start_stamped_from_edge_without_tick);
    RUN_TEST(test_stop_during_starting_completes_then_stops);
    RUN_TEST(test_session_change_resets_recordings);
    RUN_TEST(test_clip_cross_check_moves_to_neighbour_tick);
    RUN_TEST(test_tick_detector_and_tracker);
    RUN_TEST(test_lost_stop_frame_repaired_by_same_seq_repeats);
    RUN_TEST(test_new_seq_during_starting_is_no_change);
    RUN_TEST(test_new_seq_during_stopping_is_no_change);
    RUN_TEST(test_stop_after_intent_timeout_is_prompt_no_change);
    RUN_TEST(test_session_change_mid_recording);
    RUN_TEST(test_desired1_keepalive_during_stopping_does_not_restart);
    RUN_TEST(test_fault_entry_and_exit);
    RUN_TEST(test_intent_timeout_while_starting);
    return UNITY_END();
}
