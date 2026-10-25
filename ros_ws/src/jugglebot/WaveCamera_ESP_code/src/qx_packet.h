#pragma once

// QX10001 remote packet builder (pure, header-only).
//
// Layout (wave_camera_findings.md / wave_camera_gpio_explorer.md), 24 bytes:
//   [0..2]   51 58 14        header
//   [3..6]   91 4E 62 00     fixed
//   [7]      rec             00 idle, 02 record pressed
//   [8]      jog             signed jog delta
//   [9]      sel             select button
//   [10..22] 13 x 00
//   [23]     checksum: makes sum(bytes[3..23]) == 0xFF (mod 256)
// Idle packet ends in BE, record-pressed in BC.

#include <cstddef>
#include <cstdint>

namespace qx {

constexpr size_t  PACKET_BYTES     = 24;
constexpr size_t  CHECKSUM_FIRST   = 3;   // first byte covered by the checksum
constexpr size_t  CHECKSUM_INDEX   = 23;
constexpr uint8_t REC_IDLE         = 0x00;
constexpr uint8_t REC_PRESSED      = 0x02;
constexpr uint8_t HEADER[7]        = {0x51, 0x58, 0x14, 0x91, 0x4E, 0x62, 0x00};

// Checksum over bytes[3..22] such that bytes[3..23] sum to 0xFF.
inline uint8_t checksum(const uint8_t* pkt) {
    uint32_t sum = 0;
    for (size_t i = CHECKSUM_FIRST; i < CHECKSUM_INDEX; ++i) sum += pkt[i];
    return static_cast<uint8_t>((0xFFu - (sum & 0xFFu)) & 0xFFu);
}

inline bool checksum_ok(const uint8_t* pkt) {
    uint32_t sum = 0;
    for (size_t i = CHECKSUM_FIRST; i <= CHECKSUM_INDEX; ++i) sum += pkt[i];
    return (sum & 0xFFu) == 0xFFu;
}

inline bool header_ok(const uint8_t* pkt) {
    for (size_t i = 0; i < sizeof(HEADER); ++i)
        if (pkt[i] != HEADER[i]) return false;
    return true;
}

inline void build(uint8_t rec, uint8_t jog, uint8_t sel, uint8_t out[PACKET_BYTES]) {
    for (size_t i = 0; i < PACKET_BYTES; ++i) out[i] = 0;
    for (size_t i = 0; i < sizeof(HEADER); ++i) out[i] = HEADER[i];
    out[7] = rec;
    out[8] = jog;
    out[9] = sel;
    out[CHECKSUM_INDEX] = checksum(out);
}

inline int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Parse exactly PACKET_BYTES hex bytes. Whitespace, ':' and ',' between
// bytes are ignored; an optional "0x" prefix per byte is accepted.
// Returns false on any malformed input or wrong byte count.
inline bool parse_hex(const char* s, uint8_t out[PACKET_BYTES]) {
    size_t n = 0;
    while (*s) {
        if (*s == ' ' || *s == '\t' || *s == ':' || *s == ',') { ++s; continue; }
        if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
        const int hi = hex_nibble(s[0]);
        const int lo = hi < 0 ? -1 : hex_nibble(s[1]);
        if (hi < 0 || lo < 0) return false;
        if (n >= PACKET_BYTES) return false;
        out[n++] = static_cast<uint8_t>((hi << 4) | lo);
        s += 2;
    }
    return n == PACKET_BYTES;
}

// "51 58 14 ..." into buf (needs PACKET_BYTES*3 bytes). Returns buf.
inline char* to_hex(const uint8_t* pkt, char* buf, size_t buf_len) {
    static const char HEX_DIGITS[] = "0123456789ABCDEF";
    size_t w = 0;
    for (size_t i = 0; i < PACKET_BYTES && w + 3 <= buf_len; ++i) {
        buf[w++] = HEX_DIGITS[pkt[i] >> 4];
        buf[w++] = HEX_DIGITS[pkt[i] & 0xF];
        buf[w++] = (i + 1 < PACKET_BYTES) ? ' ' : '\0';
    }
    if (w > 0 && w <= buf_len) buf[w - 1] = '\0';
    else if (buf_len > 0) buf[0] = '\0';
    return buf;
}

}  // namespace qx
