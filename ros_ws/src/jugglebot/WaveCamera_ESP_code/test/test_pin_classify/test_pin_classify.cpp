// Pin classifier (src/pin_classify.h): PULSE / UART / STATIC / ACTIVE.

#include <unity.h>

#include "pin_classify.h"

void setUp() {}
void tearDown() {}

static void test_no_edges_is_static() {
    ProbeAccumulator a;
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::StaticHigh), static_cast<int>(classify_pin(a, 1)));
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::StaticLow), static_cast<int>(classify_pin(a, 0)));
}

// 30 Hz negative pulse, 100 us wide, +-20 us jitter.
static void test_30hz_pulse_train_is_pulse() {
    ProbeAccumulator a;
    int64_t t = 1000;
    for (int i = 0; i < 60; ++i) {
        const int64_t jitter = (i % 3) * 10 - 10;
        a.on_edge(t + jitter, 0);
        a.on_edge(t + jitter + 100, 1);
        t += 33333;
    }
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::Pulse), static_cast<int>(classify_pin(a, 1)));
    TEST_ASSERT_TRUE(role_is_camera_output(PinRole::Pulse));
}

static void test_wrong_rate_is_not_pulse() {
    ProbeAccumulator a;
    int64_t t = 0;
    for (int i = 0; i < 60; ++i) {  // 50 Hz
        a.on_edge(t, 0);
        a.on_edge(t + 100, 1);
        t += 20000;
    }
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::Active), static_cast<int>(classify_pin(a, 1)));
}

// Push one 8-N-1 byte at 115200 (bit ~8.68 us) as edges.
static void push_byte(ProbeAccumulator& a, int64_t& t, uint8_t b) {
    uint8_t bits[10];
    bits[0] = 0;
    for (int i = 0; i < 8; ++i) bits[1 + i] = (b >> i) & 1;
    bits[9] = 1;
    uint8_t level = 1;
    for (int i = 0; i < 10; ++i) {
        if (bits[i] != level) {
            a.on_edge(t, bits[i]);
            level = bits[i];
        }
        t += 9;  // round 8.68 us up; still a one-bit width
    }
}

static void test_uart_burst_is_uart() {
    ProbeAccumulator a;
    int64_t t = 10000;
    const char* msg = "Created new clip.\r\n";
    for (const char* p = msg; *p; ++p) push_byte(a, t, static_cast<uint8_t>(*p));
    TEST_ASSERT_TRUE(a.min_width_us() >= 6 && a.min_width_us() <= 12);
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::Uart), static_cast<int>(classify_pin(a, 1)));
}

static void test_slow_toggles_are_active() {
    ProbeAccumulator a;
    a.on_edge(0, 0);
    a.on_edge(20000, 1);
    a.on_edge(700000, 0);
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::Active), static_cast<int>(classify_pin(a, 0)));
}

// --- Boundary cases -----------------------------------------------------------

// Negative 100 us pulses; `edges` total edges; periods alternate p+dev, p-dev.
static PinRole classify_train(int64_t period_us, int edges, int64_t dev_us = 0) {
    ProbeAccumulator a;
    int64_t t = 1000;
    for (int i = 0; i < edges; ++i) {
        if (i % 2 == 0) a.on_edge(t, 0);
        else {
            a.on_edge(t + 100, 1);
            t += period_us + ((i / 2) % 2 ? -dev_us : dev_us);
        }
    }
    return classify_pin(a, 1);
}

static void test_pulse_period_bounds() {
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::Pulse), static_cast<int>(classify_train(30000, 40)));
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::Pulse), static_cast<int>(classify_train(36000, 40)));
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(PinRole::Pulse), static_cast<int>(classify_train(29000, 40)));
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(PinRole::Pulse), static_cast<int>(classify_train(37000, 40)));
}

static void test_pulse_edge_count_bound() {
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::Pulse), static_cast<int>(classify_train(33333, 20)));
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(PinRole::Pulse), static_cast<int>(classify_train(33333, 19)));
}

static void test_pulse_cv_bound() {
    // Alternating +-dev gives a sample stdev slightly above dev, so dev = 5.0 %
    // of the mean lands just over the 5 % CV limit; 4.5 % stays under it.
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(PinRole::Pulse), static_cast<int>(classify_train(33333, 40, 1667)));
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::Pulse), static_cast<int>(classify_train(33333, 40, 1500)));
}

// One burst: low for w, then alternating 20 us states (all gaps < 1 ms).
static PinRole classify_burst_min_width(int64_t w) {
    ProbeAccumulator a;
    int64_t t = 5000;
    a.on_edge(t, 0);
    a.on_edge(t += w, 1);
    for (int i = 0; i < 6; ++i) a.on_edge(t += 20, static_cast<uint8_t>(i % 2 == 0 ? 0 : 1));
    return classify_pin(a, 1);
}

static void test_uart_width_bounds() {
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::Uart), static_cast<int>(classify_burst_min_width(6)));
    TEST_ASSERT_EQUAL(static_cast<int>(PinRole::Uart), static_cast<int>(classify_burst_min_width(12)));
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(PinRole::Uart), static_cast<int>(classify_burst_min_width(5)));
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(PinRole::Uart), static_cast<int>(classify_burst_min_width(13)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_pulse_period_bounds);
    RUN_TEST(test_pulse_edge_count_bound);
    RUN_TEST(test_pulse_cv_bound);
    RUN_TEST(test_uart_width_bounds);
    RUN_TEST(test_no_edges_is_static);
    RUN_TEST(test_30hz_pulse_train_is_pulse);
    RUN_TEST(test_wrong_rate_is_not_pulse);
    RUN_TEST(test_uart_burst_is_uart);
    RUN_TEST(test_slow_toggles_are_active);
    return UNITY_END();
}
