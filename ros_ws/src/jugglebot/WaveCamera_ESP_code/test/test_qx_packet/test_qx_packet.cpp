// QX10001 packet builder + checksum (src/qx_packet.h).

#include <unity.h>

#include "qx_packet.h"

void setUp() {}
void tearDown() {}

static void test_idle_packet_exact_bytes() {
    uint8_t pkt[qx::PACKET_BYTES];
    qx::build(qx::REC_IDLE, 0, 0, pkt);
    const uint8_t expected[qx::PACKET_BYTES] = {0x51, 0x58, 0x14, 0x91, 0x4E, 0x62, 0x00, 0x00, 0x00, 0x00, 0, 0,
                                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 0xBE};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, pkt, qx::PACKET_BYTES);
}

static void test_record_pressed_ends_in_bc() {
    uint8_t pkt[qx::PACKET_BYTES];
    qx::build(qx::REC_PRESSED, 0, 0, pkt);
    TEST_ASSERT_EQUAL_HEX8(0x02, pkt[7]);
    TEST_ASSERT_EQUAL_HEX8(0xBC, pkt[23]);
}

static void test_checksum_sums_to_ff_for_any_fields() {
    uint8_t pkt[qx::PACKET_BYTES];
    for (int rec = 0; rec < 4; ++rec)
        for (int jog = 0; jog < 256; jog += 37)
            for (int sel = 0; sel < 256; sel += 51) {
                qx::build(static_cast<uint8_t>(rec), static_cast<uint8_t>(jog), static_cast<uint8_t>(sel), pkt);
                uint32_t sum = 0;
                for (int i = 3; i <= 23; ++i) sum += pkt[i];
                TEST_ASSERT_EQUAL_HEX8(0xFF, sum & 0xFF);
                TEST_ASSERT_TRUE(qx::checksum_ok(pkt));
            }
}

static void test_checksum_ok_rejects_corruption() {
    uint8_t pkt[qx::PACKET_BYTES];
    qx::build(qx::REC_IDLE, 0, 0, pkt);
    pkt[12] ^= 0x01;
    TEST_ASSERT_FALSE(qx::checksum_ok(pkt));
}

static void test_parse_hex_roundtrip() {
    uint8_t pkt[qx::PACKET_BYTES];
    TEST_ASSERT_TRUE(qx::parse_hex("51 58 14 91 4E 62 00 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 BC", pkt));
    TEST_ASSERT_TRUE(qx::checksum_ok(pkt));
    TEST_ASSERT_TRUE(qx::header_ok(pkt));
    char hex[qx::PACKET_BYTES * 3];
    TEST_ASSERT_EQUAL_STRING("51 58 14 91 4E 62 00 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 BC",
                             qx::to_hex(pkt, hex, sizeof(hex)));
    TEST_ASSERT_TRUE(qx::parse_hex("515814914e6200000000000000000000000000000000be", pkt) == false);  // 23 bytes
    TEST_ASSERT_FALSE(qx::parse_hex("51 58 zz", pkt));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_idle_packet_exact_bytes);
    RUN_TEST(test_record_pressed_ends_in_bc);
    RUN_TEST(test_checksum_sums_to_ff_for_any_fields);
    RUN_TEST(test_checksum_ok_rejects_corruption);
    RUN_TEST(test_parse_hex_roundtrip);
    return UNITY_END();
}
