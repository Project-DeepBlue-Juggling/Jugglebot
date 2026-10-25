// Edge event ring (src/event_ring.h) and UART line assembler
// (src/line_assembler.h).

#include <unity.h>

#include <cstring>

#include "event_ring.h"
#include "line_assembler.h"

void setUp() {}
void tearDown() {}

static void test_ring_fifo_and_overflow() {
    static EventRing<8> ring;
    for (int i = 0; i < 10; ++i) ring.push(EdgeEvent{i, 26, static_cast<uint8_t>(i & 1)});
    TEST_ASSERT_EQUAL_UINT32(8, ring.size());
    TEST_ASSERT_EQUAL_UINT32(2, ring.overflows());
    TEST_ASSERT_EQUAL_UINT32(8, ring.high_water());
    EdgeEvent ev;
    for (int i = 0; i < 8; ++i) {
        TEST_ASSERT_TRUE(ring.pop(ev));
        TEST_ASSERT_EQUAL_INT64(i, ev.t_us);  // oldest kept, newest dropped
    }
    TEST_ASSERT_FALSE(ring.pop(ev));
}

static void test_ring_wraps() {
    static EventRing<4> ring;
    EdgeEvent ev;
    for (int i = 0; i < 1000; ++i) {
        TEST_ASSERT_TRUE(ring.push(EdgeEvent{i, 32, 0}));
        TEST_ASSERT_TRUE(ring.pop(ev));
        TEST_ASSERT_EQUAL_INT64(i, ev.t_us);
    }
    TEST_ASSERT_EQUAL_UINT32(0, ring.overflows());
}

static void test_line_crlf_and_escape() {
    static LineAssembler a;
    static UartLine line;
    const char* in = "Created new clip.\r\n";
    int done = 0;
    for (const char* p = in; *p; ++p)
        if (a.feed(static_cast<uint8_t>(*p), 100 + (p - in), line)) ++done;
    TEST_ASSERT_EQUAL(1, done);
    TEST_ASSERT_EQUAL_STRING("Created new clip.", line.text);
    TEST_ASSERT_EQUAL(static_cast<int>(LineEnd::Eol), static_cast<int>(line.end));
    TEST_ASSERT_EQUAL_INT64(100, line.t_first_us);
    TEST_ASSERT_EQUAL_INT64(117, line.t_last_us);

    a.feed(0x01, 200, line);
    a.feed('A', 201, line);
    TEST_ASSERT_TRUE(a.feed('\n', 202, line));
    TEST_ASSERT_EQUAL_STRING("\\x01A", line.text);
}

static void test_line_partial_flushes() {
    static LineAssembler a;
    static UartLine line;
    bool flushed = false;
    for (size_t i = 0; i < UART_LINE_MAX_BYTES; ++i) flushed = a.feed('x', 0, line);
    TEST_ASSERT_TRUE(flushed);
    TEST_ASSERT_EQUAL(static_cast<int>(LineEnd::Overflow), static_cast<int>(line.end));
    TEST_ASSERT_EQUAL(UART_LINE_MAX_BYTES, line.raw_len);

    a.feed('y', 1000, line);
    TEST_ASSERT_FALSE(a.poll_timeout(1000 + UART_LINE_IDLE_FLUSH_MS * 1000 - 1, line));
    TEST_ASSERT_TRUE(a.poll_timeout(1000 + UART_LINE_IDLE_FLUSH_MS * 1000, line));
    TEST_ASSERT_EQUAL(static_cast<int>(LineEnd::Timeout), static_cast<int>(line.end));
    TEST_ASSERT_EQUAL_STRING("y", line.text);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_ring_fifo_and_overflow);
    RUN_TEST(test_ring_wraps);
    RUN_TEST(test_line_crlf_and_escape);
    RUN_TEST(test_line_partial_flushes);
    return UNITY_END();
}
