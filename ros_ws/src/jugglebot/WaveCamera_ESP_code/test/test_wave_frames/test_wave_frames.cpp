// Wave CAN frame codecs (src/wave_frames.h) against wave_camera_can_adapter.md s4.

#include <unity.h>

#include <cstring>

#include "wave_frames.h"

using namespace wave;

void setUp() {}
void tearDown() {}

static void assert_bytes(const uint8_t* exp, const uint8_t* got) { TEST_ASSERT_EQUAL_HEX8_ARRAY(exp, got, 8); }

static void test_ids() {
    TEST_ASSERT_EQUAL_HEX32(0x7DD, ID_TIME_SYNC);
    TEST_ASSERT_EQUAL_HEX32(0x7E2, ID_CMD);
    TEST_ASSERT_EQUAL_HEX32(0x7E3, ID_HEARTBEAT);
    TEST_ASSERT_EQUAL_HEX32(0x7E4, ID_REC_EVENT);
    TEST_ASSERT_EQUAL_HEX32(0x7E5, ID_INFO);
    TEST_ASSERT_EQUAL_HEX32(0x7E2, ID_BLOCK_FIRST);
    TEST_ASSERT_EQUAL_HEX32(0x7E7, ID_BLOCK_LAST);
}

static void test_cmd_decode_exact_and_round_trip() {
    // desired=1 seq=0x2A flags=CLEAR_FAULTS reserved=0 session=0x6712_3456
    const uint8_t d[8] = {0x01, 0x2A, 0x01, 0x00, 0x56, 0x34, 0x12, 0x67};
    Cmd c{};
    TEST_ASSERT_EQUAL(static_cast<int>(CmdDecode::Ok), static_cast<int>(decode_cmd(d, 8, false, false, c)));
    TEST_ASSERT_EQUAL_UINT8(1, c.desired);
    TEST_ASSERT_EQUAL_UINT8(0x2A, c.cmd_seq);
    TEST_ASSERT_EQUAL_UINT8(CMD_FLAG_CLEAR_FAULTS, c.flags);
    TEST_ASSERT_EQUAL_HEX32(0x67123456, c.session_id);
    uint8_t back[8];
    encode_cmd(c, back);
    assert_bytes(d, back);
}

static void test_cmd_decode_rejects() {
    uint8_t d[8] = {0x02, 1, 0, 0, 0, 0, 0, 0};
    Cmd c{};
    TEST_ASSERT_EQUAL(static_cast<int>(CmdDecode::BadDesired), static_cast<int>(decode_cmd(d, 8, false, false, c)));
    d[0] = 0;
    TEST_ASSERT_EQUAL(static_cast<int>(CmdDecode::BadDlc), static_cast<int>(decode_cmd(d, 7, false, false, c)));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdDecode::BadFrame), static_cast<int>(decode_cmd(d, 8, true, false, c)));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdDecode::BadFrame), static_cast<int>(decode_cmd(d, 8, false, true, c)));
}

static void test_heartbeat_exact_bytes() {
    Heartbeat h{};
    h.state = State::Recording;
    h.status = ST_CAMERA_ON | ST_UART_SEEN | ST_TIME_SYNCED | ST_INTENT_FRESH | ST_DESIRED | ST_GPI2_LOW;
    h.faults = F_CLIP_MISSING | F_SYNC_STEP;
    h.elapsed_100ms = elapsed_100ms(1234567890);  // 1234.56789 s -> 12345
    h.sync_age_100ms = sync_age_100ms(true, 150000);  // 0.15 s -> 1
    h.recordings = 7;
    h.cmd_seq_echo = 0x99;
    uint8_t out[8];
    encode_heartbeat(h, out);
    const uint8_t exp[8] = {0x03, 0x5F, 0x24, 0x39, 0x30, 0x01, 0x07, 0x99};
    assert_bytes(exp, out);
}

static void test_heartbeat_reserved_bits_zero() {
    Heartbeat h{};
    h.state = State::Boot;
    h.status = 0xFF;
    h.faults = 0xFF;
    uint8_t out[8];
    encode_heartbeat(h, out);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x7F, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x7F, out[2]);
}

static void test_elapsed_and_sync_age_saturation() {
    TEST_ASSERT_EQUAL_UINT16(0, elapsed_100ms(-5));
    TEST_ASSERT_EQUAL_UINT16(0, elapsed_100ms(99999));
    TEST_ASSERT_EQUAL_UINT16(1, elapsed_100ms(100000));
    TEST_ASSERT_EQUAL_UINT16(65534, elapsed_100ms(6553400000LL));
    TEST_ASSERT_EQUAL_UINT16(65534, elapsed_100ms(10000000000LL));  // saturates at 6553.4 s
    TEST_ASSERT_EQUAL_UINT8(255, sync_age_100ms(false, 0));         // never
    TEST_ASSERT_EQUAL_UINT8(0, sync_age_100ms(true, 99999));
    TEST_ASSERT_EQUAL_UINT8(254, sync_age_100ms(true, 25499999));
    TEST_ASSERT_EQUAL_UINT8(255, sync_age_100ms(true, 25500000));   // >= 25.5 s
    TEST_ASSERT_EQUAL_UINT8(255, sync_age_100ms(true, 3600000000LL));
}

static void test_rec_event_synced_exact_bytes() {
    // Wall 2026-10-19T00:00:00.123456Z = 1792368000.123456 s.
    const int64_t wall = 1792368000LL * 1000000LL + 123456;
    RecEvent e{};
    e.t40 = t40_from_wall(wall);
    e.kind = EventKind::Start;
    e.reason = Reason::None;
    e.synced = true;
    e.tick_stamped = true;
    e.cmd_seq = 0x11;
    e.recording_seq = 0x22;
    uint8_t out[8];
    encode_rec_event(e, out);
    const uint64_t low = static_cast<uint64_t>(wall) & 0xFFFFFFFFFFULL;
    const uint8_t exp[8] = {static_cast<uint8_t>(low), static_cast<uint8_t>(low >> 8), static_cast<uint8_t>(low >> 16),
                            static_cast<uint8_t>(low >> 24), static_cast<uint8_t>(low >> 32),
                            0x19 /* kind 1 | SYNCED | TICK */, 0x11, 0x22};
    assert_bytes(exp, out);
    TEST_ASSERT_EQUAL_HEX64(low, get_u40(out));
    // Literal check of the 40-bit field for a hand-computed value.
    e.t40 = t40_from_wall(0x123456789ABCLL);  // high byte 0x12 dropped
    encode_rec_event(e, out);
    const uint8_t exp2[5] = {0xBC, 0x9A, 0x78, 0x56, 0x34};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp2, out, 5);
}

static void test_rec_event_unsynced_age_path() {
    RecEvent e{};
    e.t40 = t40_from_age(48123);  // 48.123 ms old at TX
    e.kind = EventKind::AutoStop;
    e.reason = Reason::IntentTimeout;
    e.synced = false;
    e.tick_stamped = false;
    e.cmd_seq = 5;
    e.recording_seq = 255;
    uint8_t out[8];
    encode_rec_event(e, out);
    const uint8_t exp[8] = {0xFB, 0xBB, 0x00, 0x00, 0x00, 0x43 /* 3 | 2<<5 */, 0x05, 0xFF};
    assert_bytes(exp, out);
    TEST_ASSERT_EQUAL_UINT64(0, t40_from_age(-10));
    TEST_ASSERT_EQUAL_HEX64(T40_MASK, t40_from_age(1LL << 45));
}

static void test_rec_event_reason_bits() {
    TEST_ASSERT_EQUAL_HEX8(0xE4, rec_event_kind_byte(EventKind::Refused, Reason::Gpi2Fault, false, false));
    TEST_ASSERT_EQUAL_HEX8(0xA1, rec_event_kind_byte(EventKind::Start, Reason::ClipMissing, false, false));
    TEST_ASSERT_EQUAL_HEX8(0x2C, rec_event_kind_byte(EventKind::Refused, Reason::NoChange, true, false));
    TEST_ASSERT_EQUAL_HEX8(0xC4, rec_event_kind_byte(EventKind::Refused, Reason::ConsoleOwns, false, false));
}

static void test_info_pages_exact_bytes() {
    uint8_t out[8];
    encode_info_identity(0x0102, 1, 0x00A1B2C3, out);
    const uint8_t p0[8] = {0, 0x02, 0x01, 0x01, 0xC3, 0xB2, 0xA1, 0x00};
    assert_bytes(p0, out);

    encode_info_tick(33334300, 5, 2, out);  // 0x01FCA41C
    const uint8_t p1[8] = {1, 0x1C, 0xA4, 0xFC, 0x01, 0x05, 0x00, 0x02};
    assert_bytes(p1, out);

    encode_info_sync(150, 42, 100, 3, 1, out);
    const uint8_t p2[8] = {2, 0x96, 0x00, 0x2A, 0x00, 100, 3, 1};
    assert_bytes(p2, out);

    encode_info_link(15500, 0x1234, 9, 5, out);
    const uint8_t p3[8] = {3, 0x8C, 0x3C, 0x34, 0x12, 9, 5, 0};
    assert_bytes(p3, out);
}

static void test_time_sync_decode() {
    uint8_t d[8];
    put_u32(d, 1792368000u);
    put_u32(d + 4, 999999u);
    TimeSync ts{};
    TEST_ASSERT_TRUE(decode_time_sync(d, 8, false, false, ts));
    TEST_ASSERT_EQUAL_INT64(1792368000LL * 1000000LL + 999999, time_sync_wall_us(ts));
    put_u32(d + 4, 1000000u);
    TEST_ASSERT_FALSE(decode_time_sync(d, 8, false, false, ts));  // usec >= 1e6
    put_u32(d + 4, 0);
    TEST_ASSERT_FALSE(decode_time_sync(d, 7, false, false, ts));  // DLC != 8
    TEST_ASSERT_FALSE(decode_time_sync(d, 8, true, false, ts));   // extended
    TEST_ASSERT_FALSE(decode_time_sync(d, 8, false, true, ts));   // RTR
    put_u32(d, SYNC_MIN_UNIX_S - 1);
    TEST_ASSERT_FALSE(decode_time_sync(d, 8, false, false, ts));  // before 2026
}

static void test_saturating_helpers() {
    TEST_ASSERT_EQUAL_UINT8(255, sat_u8(1000));
    TEST_ASSERT_EQUAL_UINT8(0, sat_u8(-1));
    TEST_ASSERT_EQUAL_UINT16(65535, sat_u16(70000));
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, sat_u32(1LL << 40));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_ids);
    RUN_TEST(test_cmd_decode_exact_and_round_trip);
    RUN_TEST(test_cmd_decode_rejects);
    RUN_TEST(test_heartbeat_exact_bytes);
    RUN_TEST(test_heartbeat_reserved_bits_zero);
    RUN_TEST(test_elapsed_and_sync_age_saturation);
    RUN_TEST(test_rec_event_synced_exact_bytes);
    RUN_TEST(test_rec_event_unsynced_age_path);
    RUN_TEST(test_rec_event_reason_bits);
    RUN_TEST(test_info_pages_exact_bytes);
    RUN_TEST(test_time_sync_decode);
    RUN_TEST(test_saturating_helpers);
    return UNITY_END();
}
