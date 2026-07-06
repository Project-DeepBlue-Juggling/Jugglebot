// =============================================================================
//  test_udp_framing.cpp — compiled test of the REAL C++ UDP framing layer
// =============================================================================
//  The hand-written C++ framing (crc16_ccitt / encode_frame / decode_frame in the
//  generated udp_protocol.h) is the NETWORK TRUST BOUNDARY — every byte the Teensy
//  accepts off the wire passes through decode_frame. Yet no test ever EXECUTED it
//  The cross-language xref only compared struct sizes + the
//  committed-vs-generated text, never ran the C++ codec. This runs it.
//
//  The CRC is pinned to the canonical CRC-16/CCITT-FALSE check value (0x29B1 for
//  "123456789") — the SAME anchor tests/firmware/test_udp_protocol_xlang.py pins
//  the Python crc16 to, so agreement on the canonical value is cross-language
//  parity by transitivity. Then: encode→decode round-trip, and every rejection the
//  firmware relies on (short buffer, bad magic, bad version, length mismatch, CRC
//  corruption). SCOPE: the pure framing codec (no HAL, no firmware TU).
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "udp_protocol.h"

using namespace JbUdp;

// A representative payload (an arbitrary 12-byte blob).
static const uint8_t PAYLOAD[12] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                                    0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5};

TEST_CASE("C++ crc16_ccitt matches the canonical CRC-16/CCITT-FALSE check value") {
  // 0x29B1 for "123456789" — the same anchor the Python crc16 is pinned to, so
  // both languages agree on the canonical value (cross-language parity).
  const uint8_t s[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc16_ccitt(s, 9) == 0x29B1);
  // Empty input → the 0xFFFF init value (no bytes folded in).
  CHECK(crc16_ccitt(nullptr, 0) == 0xFFFF);
}

TEST_CASE("encode_frame → decode_frame round-trips header + payload") {
  uint8_t out[64];
  const uint16_t total = encode_frame((uint8_t)MsgType::TELEMETRY, 0x1234,
                                      PAYLOAD, sizeof(PAYLOAD), out, sizeof(out));
  REQUIRE(total == HEADER_SIZE + sizeof(PAYLOAD) + CRC_SIZE);

  Header hdr{};
  const uint8_t* pl = nullptr;
  REQUIRE(decode_frame(out, total, &hdr, &pl));
  CHECK(hdr.magic == MAGIC);
  CHECK(hdr.version == PROTOCOL_VERSION);
  CHECK(hdr.msg_type == (uint8_t)MsgType::TELEMETRY);
  CHECK(hdr.seq == 0x1234);
  CHECK(hdr.length == sizeof(PAYLOAD));
  REQUIRE(pl != nullptr);
  CHECK(memcmp(pl, PAYLOAD, sizeof(PAYLOAD)) == 0);
}

TEST_CASE("encode_frame refuses an oversized payload and a too-small out buffer") {
  uint8_t out[64];
  // Payload over MAX_PAYLOAD → 0 (never encodes a frame the peer would reject).
  CHECK(encode_frame((uint8_t)MsgType::SETPOINT, 0, PAYLOAD, MAX_PAYLOAD + 1,
                     out, sizeof(out)) == 0);
  // Out buffer too small for header+payload+crc → 0.
  CHECK(encode_frame((uint8_t)MsgType::SETPOINT, 0, PAYLOAD, sizeof(PAYLOAD),
                     out, HEADER_SIZE) == 0);
}

TEST_CASE("encode_frame refuses a null payload with a nonzero length") {
  uint8_t out[64];
  // len>0 with a null payload would else skip the memcpy but still CRC over the
  // uninitialized out[] and return a 'valid' garbage frame → guard returns 0.
  CHECK(encode_frame((uint8_t)MsgType::TELEMETRY, 0, nullptr, sizeof(PAYLOAD),
                     out, sizeof(out)) == 0);
  // A zero-length frame with a null payload is still legitimate (header + CRC only)
  // and must round-trip — the guard only fires when len>0.
  const uint16_t n = encode_frame((uint8_t)MsgType::HEARTBEAT_T2J, 0, nullptr, 0,
                                  out, sizeof(out));
  REQUIRE(n == HEADER_SIZE + CRC_SIZE);
  Header hdr{}; const uint8_t* pl = nullptr;
  REQUIRE(decode_frame(out, n, &hdr, &pl));
  CHECK(hdr.length == 0);
}

TEST_CASE("decode_frame rejects a runt (shorter than header+crc)") {
  uint8_t buf[4] = {0};
  Header hdr{}; const uint8_t* pl = nullptr;
  CHECK_FALSE(decode_frame(buf, sizeof(buf), &hdr, &pl));
}

TEST_CASE("decode_frame rejects bad magic / bad version / length mismatch / CRC corruption") {
  uint8_t out[64];
  const uint16_t total = encode_frame((uint8_t)MsgType::HEARTBEAT_T2J, 7,
                                      PAYLOAD, sizeof(PAYLOAD), out, sizeof(out));
  REQUIRE(total > 0);
  Header hdr{}; const uint8_t* pl = nullptr;

  SUBCASE("bad magic") {
    uint8_t bad[64]; memcpy(bad, out, total);
    bad[0] ^= 0xFF;
    CHECK_FALSE(decode_frame(bad, total, &hdr, &pl));
  }
  SUBCASE("bad version") {
    uint8_t bad[64]; memcpy(bad, out, total);
    bad[2] = (uint8_t)(PROTOCOL_VERSION + 1);   // version byte (after 2-byte magic)
    CHECK_FALSE(decode_frame(bad, total, &hdr, &pl));
  }
  SUBCASE("length mismatch (truncated by one)") {
    CHECK_FALSE(decode_frame(out, (uint16_t)(total - 1), &hdr, &pl));
  }
  SUBCASE("CRC corruption (flip a payload bit)") {
    uint8_t bad[64]; memcpy(bad, out, total);
    bad[HEADER_SIZE + 3] ^= 0x01;
    CHECK_FALSE(decode_frame(bad, total, &hdr, &pl));
  }
  // The untouched frame still decodes (guards against a broken fixture).
  CHECK(decode_frame(out, total, &hdr, &pl));
}
