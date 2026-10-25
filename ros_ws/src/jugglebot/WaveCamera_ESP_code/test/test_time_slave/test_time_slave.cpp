// 0x7DD time-slave estimator (src/time_slave_math.h), wave_camera_can_adapter.md s5.5.

#include <unity.h>

#include "time_slave_math.h"

namespace {

constexpr int64_t MS = 1000;
constexpr int64_t S = 1000000;
constexpr int64_t WALL0 = 1792368000LL * S;  // 2026-10-19
constexpr int64_t OFFSET = WALL0 - 5 * S;    // wall - local, local starts at 5 s

using R = TimeSlaveEstimator::Result;

// Feed 100 Hz beacons in [t0, t1) local time, offset `off(t)` minus a delay.
template <typename Delay>
void feed(TimeSlaveEstimator& est, int64_t t0, int64_t t1, int64_t off, Delay delay) {
    int k = 0;
    for (int64_t t = t0; t < t1; t += 10 * MS, ++k) est.on_beacon(t + off - delay(k), t);
}
int64_t no_delay(int) { return 0; }

uint8_t lcg_state = 1;
int64_t pseudo_delay_us(int k) {
    // Every 10th frame on time; the rest 0..4 ms late.
    if (k % 10 == 0) return 0;
    lcg_state = static_cast<uint8_t>(lcg_state * 37 + 11);
    return static_cast<int64_t>(lcg_state) * 4000 / 255;
}

void frame_bytes(uint32_t sec, uint32_t usec, uint8_t out[8]) {
    wave::put_u32(out, sec);
    wave::put_u32(out + 4, usec);
}

}  // namespace

void setUp() {}
void tearDown() {}

static void test_constant_offset() {
    TimeSlaveEstimator est;
    TEST_ASSERT_FALSE(est.anchored());
    int64_t w;
    TEST_ASSERT_FALSE(est.wall_from_local(1 * S, w));
    for (int i = 0; i < 9; ++i) TEST_ASSERT_EQUAL(static_cast<int>(R::Accepted),
                                                  static_cast<int>(est.on_beacon(5 * S + i * 10 * MS + OFFSET, 5 * S + i * 10 * MS)));
    TEST_ASSERT_EQUAL(static_cast<int>(R::Anchored), static_cast<int>(est.on_beacon(5 * S + 90 * MS + OFFSET, 5 * S + 90 * MS)));
    feed(est, 5 * S + 100 * MS, 15 * S, OFFSET, no_delay);
    TEST_ASSERT_TRUE(est.anchored());
    TEST_ASSERT_TRUE(est.synced(15 * S));
    TEST_ASSERT_TRUE(est.wall_from_local(20 * S, w));
    TEST_ASSERT_EQUAL_INT64(20 * S + OFFSET, w);
    TEST_ASSERT_EQUAL_INT64(0, est.offset_spread_us());
    TEST_ASSERT_EQUAL_INT64(0, est.track_err_us());
    TEST_ASSERT_EQUAL_UINT32(0, est.steps());
    TEST_ASSERT_EQUAL_UINT32(1000, est.accepted());
}

static void test_delayed_frames_never_pull_offset_down() {
    TimeSlaveEstimator est;
    feed(est, 5 * S, 15 * S, OFFSET, pseudo_delay_us);
    int64_t w;
    TEST_ASSERT_TRUE(est.wall_from_local(15 * S, w));
    TEST_ASSERT_INT64_WITHIN(1, 15 * S + OFFSET, w);
    TEST_ASSERT_TRUE(est.offset_spread_us() > 2000);  // the spread is seen ...
    // ... a full second in which every frame is 1.5 ms late (a busy bus) is
    // not tracked: the estimate does not move.
    feed(est, 15 * S, 16 * S + 10 * MS, OFFSET, [](int) { return int64_t(1500); });
    feed(est, 16 * S + 10 * MS, 17 * S, OFFSET, no_delay);
    TEST_ASSERT_TRUE(est.wall_from_local(17 * S, w));
    TEST_ASSERT_INT64_WITHIN(1, 17 * S + OFFSET, w);
    TEST_ASSERT_EQUAL_UINT32(0, est.steps());
}

static void test_skew_is_tracked() {
    // Local crystal 30 ppm slow against the wall: offset grows 30 us/s.
    TimeSlaveEstimator est;
    for (int64_t t = 5 * S; t < 25 * S; t += 10 * MS) est.on_beacon(t + OFFSET + (t - 5 * S) * 30 / S, t);
    int64_t w;
    TEST_ASSERT_TRUE(est.wall_from_local(26 * S, w));  // 1 s of extrapolation
    TEST_ASSERT_INT64_WITHIN(5, 26 * S + OFFSET + 21 * 30, w);
    TEST_ASSERT_TRUE(est.track_err_us() < 20);
}

static void test_single_early_frame_rejected() {
    TimeSlaveEstimator est;
    feed(est, 5 * S, 10 * S, OFFSET, no_delay);
    TEST_ASSERT_EQUAL(static_cast<int>(R::RejectedEarly),
                      static_cast<int>(est.on_beacon(10 * S + OFFSET + 1500, 10 * S)));
    feed(est, 10 * S + 10 * MS, 14 * S, OFFSET, no_delay);
    int64_t w;
    TEST_ASSERT_TRUE(est.wall_from_local(14 * S, w));
    TEST_ASSERT_EQUAL_INT64(14 * S + OFFSET, w);
    TEST_ASSERT_EQUAL_UINT32(1, est.rejected_early());
    TEST_ASSERT_EQUAL_UINT32(0, est.steps());
}

static void test_step_detection_reanchors() {
    TimeSlaveEstimator est;
    feed(est, 5 * S, 10 * S, OFFSET, no_delay);
    const int64_t STEP = 50 * MS;  // bridge wall clock jumps forward 50 ms
    feed(est, 10 * S, 14 * S, OFFSET + STEP, no_delay);
    TEST_ASSERT_EQUAL_UINT32(1, est.steps());
    int64_t w;
    TEST_ASSERT_TRUE(est.wall_from_local(14 * S, w));
    TEST_ASSERT_EQUAL_INT64(14 * S + OFFSET + STEP, w);
    TEST_ASSERT_TRUE(est.synced(14 * S));
    // And backwards.
    feed(est, 14 * S, 18 * S, OFFSET - STEP, no_delay);
    TEST_ASSERT_EQUAL_UINT32(2, est.steps());
    TEST_ASSERT_TRUE(est.wall_from_local(18 * S, w));
    TEST_ASSERT_EQUAL_INT64(18 * S + OFFSET - STEP, w);
}

static void test_one_deviant_window_is_not_a_step() {
    TimeSlaveEstimator est;
    feed(est, 5 * S, 10 * S, OFFSET, no_delay);
    feed(est, 10 * S, 11 * S, OFFSET - 3000, no_delay);  // one window 3 ms late
    feed(est, 11 * S, 14 * S, OFFSET, no_delay);
    TEST_ASSERT_EQUAL_UINT32(0, est.steps());
    int64_t w;
    TEST_ASSERT_TRUE(est.wall_from_local(14 * S, w));
    TEST_ASSERT_EQUAL_INT64(14 * S + OFFSET, w);
}

static void test_sync_age_and_staleness() {
    TimeSlaveEstimator est;
    TEST_ASSERT_EQUAL_INT64(-1, est.sync_age_us(1 * S));
    TEST_ASSERT_EQUAL_UINT8(255, wave::sync_age_100ms(est.ever_accepted(), est.sync_age_us(1 * S)));
    feed(est, 5 * S, 6 * S, OFFSET, no_delay);
    const int64_t last = 6 * S - 10 * MS;
    TEST_ASSERT_EQUAL_INT64(250 * MS, est.sync_age_us(last + 250 * MS));
    TEST_ASSERT_EQUAL_UINT8(2, wave::sync_age_100ms(true, est.sync_age_us(last + 250 * MS)));
    TEST_ASSERT_TRUE(est.synced(last + 4999 * MS));
    TEST_ASSERT_FALSE(est.synced(last + 5000 * MS));  // TIME_SYNCED drops after 5 s
    int64_t w;
    TEST_ASSERT_TRUE(est.wall_from_local(last + 10 * S, w));  // still coasts
    TEST_ASSERT_EQUAL_UINT8(254, wave::sync_age_100ms(true, est.sync_age_us(last + 25499 * MS)));
    TEST_ASSERT_EQUAL_UINT8(255, wave::sync_age_100ms(true, est.sync_age_us(last + 25500 * MS)));
    TEST_ASSERT_EQUAL_UINT8(255, wave::sync_age_100ms(true, est.sync_age_us(last + 600 * S)));
}

static void test_frame_validation() {
    TimeSlaveEstimator est;
    uint8_t d[8];
    frame_bytes(1792368000u, 1000000u, d);  // usec >= 1e6
    TEST_ASSERT_EQUAL(static_cast<int>(R::RejectedFormat), static_cast<int>(est.on_frame(d, 8, false, false, S)));
    frame_bytes(1792368000u, 5u, d);
    TEST_ASSERT_EQUAL(static_cast<int>(R::RejectedFormat), static_cast<int>(est.on_frame(d, 7, false, false, S)));
    TEST_ASSERT_EQUAL(static_cast<int>(R::RejectedFormat), static_cast<int>(est.on_frame(d, 8, true, false, S)));
    TEST_ASSERT_EQUAL(static_cast<int>(R::RejectedFormat), static_cast<int>(est.on_frame(d, 8, false, true, S)));
    frame_bytes(1700000000u, 5u, d);  // 2023: before 2026-01-01
    TEST_ASSERT_EQUAL(static_cast<int>(R::RejectedFormat), static_cast<int>(est.on_frame(d, 8, false, false, S)));
    TEST_ASSERT_EQUAL_UINT32(5, est.rejected());
    TEST_ASSERT_EQUAL_UINT32(0, est.accepted());
    TEST_ASSERT_FALSE(est.ever_accepted());
    frame_bytes(1792368000u, 999999u, d);
    TEST_ASSERT_EQUAL(static_cast<int>(R::Accepted), static_cast<int>(est.on_frame(d, 8, false, false, S)));
    TEST_ASSERT_EQUAL_UINT32(1, est.accepted());
}

// Review fix 3: a long silence restarts the first-anchor path; not a step.
static void test_stale_gap_reanchors_without_step() {
    TimeSlaveEstimator est;
    feed(est, 5 * S, 10 * S, OFFSET, no_delay);
    TEST_ASSERT_TRUE(est.anchored());
    const int64_t t = 10 * S + 95 * S;  // 95 s of silence
    TEST_ASSERT_FALSE(est.synced(t));
    TEST_ASSERT_EQUAL(static_cast<int>(R::Accepted), static_cast<int>(est.on_beacon(t + OFFSET + 1500, t)));
    feed(est, t + 10 * MS, t + 2 * S, OFFSET + 1500, no_delay);
    TEST_ASSERT_EQUAL_UINT32(0, est.steps());
    TEST_ASSERT_EQUAL_UINT32(0, est.rejected_early());
    TEST_ASSERT_EQUAL_UINT32(1, est.stale_restarts());
    int64_t w;
    TEST_ASSERT_TRUE(est.wall_from_local(t + 2 * S, w));
    TEST_ASSERT_EQUAL_INT64(t + 2 * S + OFFSET + 1500, w);
    TEST_ASSERT_TRUE(est.synced(t + 2 * S));
}

// Review fix 4: one early outlier per window, several windows running, is
// not a step (each window still has accepted frames).
static void test_early_outliers_in_consecutive_windows_are_not_a_step() {
    TimeSlaveEstimator est;
    feed(est, 5 * S, 10 * S, OFFSET, no_delay);
    for (int64_t t = 10 * S; t < 14 * S; t += 10 * MS) {
        const bool outlier = (t % S) == 500 * MS;
        est.on_beacon(t + OFFSET + (outlier ? 1500 : 0), t);
    }
    TEST_ASSERT_EQUAL_UINT32(0, est.steps());
    TEST_ASSERT_EQUAL_UINT32(4, est.rejected_early());
    int64_t w;
    TEST_ASSERT_TRUE(est.wall_from_local(14 * S, w));
    TEST_ASSERT_EQUAL_INT64(14 * S + OFFSET, w);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_constant_offset);
    RUN_TEST(test_delayed_frames_never_pull_offset_down);
    RUN_TEST(test_skew_is_tracked);
    RUN_TEST(test_single_early_frame_rejected);
    RUN_TEST(test_step_detection_reanchors);
    RUN_TEST(test_one_deviant_window_is_not_a_step);
    RUN_TEST(test_sync_age_and_staleness);
    RUN_TEST(test_frame_validation);
    RUN_TEST(test_stale_gap_reanchors_without_step);
    RUN_TEST(test_early_outliers_in_consecutive_windows_are_not_a_step);
    return UNITY_END();
}
