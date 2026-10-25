// Pulse / period statistics (src/stats_math.h).

#include <unity.h>

#include "stats_math.h"

void setUp() {}
void tearDown() {}

static void test_running_stats_basic() {
    RunningStats s;
    const double xs[] = {2, 4, 4, 4, 5, 5, 7, 9};
    for (double x : xs) s.add(x);
    TEST_ASSERT_EQUAL_UINT32(8, s.n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 5.0, s.mean);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0, s.min);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 9.0, s.max);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 2.138089935, s.stdev());  // sample stdev
}

static void test_running_stats_single_sample() {
    RunningStats s;
    s.add(42);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, s.stdev());
}

static void test_negative_pulse_train() {
    EdgeRecord e[40];
    size_t n = 0;
    int64_t t = 500;
    for (int i = 0; i < 20; ++i) {
        e[n++] = EdgeRecord{t, 0};
        e[n++] = EdgeRecord{t + 200, 1};
        t += (i % 2) ? 33343 : 33323;  // +-10 us jitter around 33333
    }
    const PulseReport r = analyse_pulse(e, n);
    TEST_ASSERT_EQUAL(static_cast<int>(PulsePolarity::Negative), static_cast<int>(r.polarity));
    TEST_ASSERT_EQUAL_UINT32(19, r.period_us.n);
    TEST_ASSERT_DOUBLE_WITHIN(15.0, 33333.0, r.period_us.mean);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 20.0, r.period_us.max - r.period_us.min);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 200.0, r.width_us.mean);
    TEST_ASSERT_EQUAL_INT64(e[38].t_us, r.last_leading_us);
}

static void test_positive_pulse_and_missed_edge() {
    EdgeRecord e[] = {{0, 1}, {50, 0}, {1000, 1}, {1050, 0}, {2000, 1}, {3000, 1}, {3050, 0}, {4000, 1}, {4050, 0}};
    const PulseReport r = analyse_pulse(e, sizeof(e) / sizeof(e[0]));
    TEST_ASSERT_EQUAL(static_cast<int>(PulsePolarity::Positive), static_cast<int>(r.polarity));
    TEST_ASSERT_EQUAL_UINT32(1, r.repeated_levels);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 50.0, r.width_us.mean);
    // Periods 0->1000, 1000->2000, 3000->4000; 2000->3000 spans the missed edge.
    TEST_ASSERT_EQUAL_UINT32(3, r.period_us.n);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1000.0, r.period_us.mean);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_running_stats_basic);
    RUN_TEST(test_running_stats_single_sample);
    RUN_TEST(test_negative_pulse_train);
    RUN_TEST(test_positive_pulse_and_missed_edge);
    return UNITY_END();
}
