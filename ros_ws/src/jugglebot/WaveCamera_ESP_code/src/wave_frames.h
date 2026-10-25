#pragma once

// Wave camera CAN frames (wave_camera_can_adapter.md s4): pure, header-only, Arduino-free.
//
// Classic CAN, 11-bit IDs, DLC always 8, unused bytes 0, little-endian.
// Decoders take the DLC and refuse anything but 8 so a short frame never
// reads stale buffer bytes. Encoders always write all 8 bytes.

#include <cstdint>
#include <cstring>

#include "config.h"

namespace wave {

// --- IDs ---------------------------------------------------------------------
constexpr uint32_t ID_TIME_SYNC   = TIME_SYNC_CAN_ID;  // 0x7DD, bridge -> all
constexpr uint32_t ID_CMD         = 0x7E2;             // Jetson -> ESP, 5 Hz
constexpr uint32_t ID_HEARTBEAT   = 0x7E3;             // ESP -> Jetson, 10 Hz
constexpr uint32_t ID_REC_EVENT   = 0x7E4;             // ESP -> Jetson, on event
constexpr uint32_t ID_INFO        = 0x7E5;             // ESP -> Jetson, page rotation
constexpr uint32_t ID_BLOCK_FIRST = 0x7E2;
constexpr uint32_t ID_BLOCK_LAST  = 0x7E7;             // 0x7E6-0x7E7 reserved
static_assert(ID_CMD > ID_TIME_SYNC && ID_INFO > ID_TIME_SYNC, "0x7DD must win arbitration");

// --- Enumerations ----------------------------------------------------------------
enum class State : uint8_t { Boot = 0, Idle = 1, Starting = 2, Recording = 3, Stopping = 4, Fault = 5 };
enum class EventKind : uint8_t { Start = 1, Stop = 2, AutoStop = 3, Refused = 4 };
enum class Reason : uint8_t {
    None = 0,
    NoChange = 1,
    IntentTimeout = 2,
    TickLost = 3,
    CameraOff = 4,
    ClipMissing = 5,
    ConsoleOwns = 6,
    Gpi2Fault = 7,
};

inline const char* state_name(State s) {
    switch (s) {
        case State::Boot: return "BOOT";
        case State::Idle: return "IDLE";
        case State::Starting: return "STARTING";
        case State::Recording: return "RECORDING";
        case State::Stopping: return "STOPPING";
        case State::Fault: return "FAULT";
    }
    return "?";
}
inline const char* kind_name(EventKind k) {
    switch (k) {
        case EventKind::Start: return "START";
        case EventKind::Stop: return "STOP";
        case EventKind::AutoStop: return "AUTO_STOP";
        case EventKind::Refused: return "REFUSED";
    }
    return "?";
}
inline const char* reason_name(Reason r) {
    static const char* const NAMES[] = {"NONE",        "NO_CHANGE",    "INTENT_TIMEOUT", "TICK_LOST",
                                        "CAMERA_OFF",  "CLIP_MISSING", "CONSOLE_OWNS",   "GPI2_FAULT"};
    const unsigned i = static_cast<unsigned>(r);
    return i < 8 ? NAMES[i] : "?";
}

// Heartbeat byte 1 (status).
constexpr uint8_t ST_CAMERA_ON    = 1u << 0;
constexpr uint8_t ST_UART_SEEN    = 1u << 1;
constexpr uint8_t ST_TIME_SYNCED  = 1u << 2;
constexpr uint8_t ST_INTENT_FRESH = 1u << 3;
constexpr uint8_t ST_DESIRED      = 1u << 4;
constexpr uint8_t ST_CONSOLE_OWNS = 1u << 5;
constexpr uint8_t ST_GPI2_LOW     = 1u << 6;

// Heartbeat byte 2 (latched faults).
constexpr uint8_t F_GPI2_REFUSED       = 1u << 0;
constexpr uint8_t F_TICK_LOST_MID_REC  = 1u << 1;
constexpr uint8_t F_CLIP_MISSING       = 1u << 2;
constexpr uint8_t F_INTENT_TIMEOUT     = 1u << 3;
constexpr uint8_t F_BUS_OFF_SEEN       = 1u << 4;
constexpr uint8_t F_SYNC_STEP          = 1u << 5;
constexpr uint8_t F_EDGE_RING_OVERFLOW = 1u << 6;

// WAVE_CMD byte 2.
constexpr uint8_t CMD_FLAG_CLEAR_FAULTS = 1u << 0;

// WAVE_REC_EVENT byte 5.
constexpr uint8_t EV_KIND_MASK      = 0x07;
constexpr uint8_t EV_SYNCED         = 1u << 3;
constexpr uint8_t EV_TICK_STAMPED   = 1u << 4;
constexpr uint8_t EV_REASON_SHIFT   = 5;

// WAVE_INFO pages.
constexpr uint8_t INFO_PAGE_IDENTITY = 0;
constexpr uint8_t INFO_PAGE_TICK     = 1;
constexpr uint8_t INFO_PAGE_SYNC     = 2;
constexpr uint8_t INFO_PAGE_LINK     = 3;
static_assert(INFO_NUM_PAGES == 4, "page count and the page table must agree");

// --- Little-endian helpers ---------------------------------------------------------
inline void put_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put_u32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
inline void put_u40(uint8_t* p, uint64_t v) {
    for (int i = 0; i < 5; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
inline uint16_t get_u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t get_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}
inline uint64_t get_u40(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 4; i >= 0; --i) v = (v << 8) | p[i];
    return v;
}

// --- Saturating field conversions ----------------------------------------------------
constexpr uint64_t T40_MASK = (1ULL << 40) - 1;
constexpr uint16_t ELAPSED_MAX_100MS = 65534;  // 6553.4 s
constexpr uint8_t  SYNC_AGE_NEVER = 255;       // never, or >= 25.5 s

inline uint8_t sat_u8(int64_t v) { return v < 0 ? 0 : v > 255 ? 255 : static_cast<uint8_t>(v); }
inline uint16_t sat_u16(int64_t v) { return v < 0 ? 0 : v > 65535 ? 65535 : static_cast<uint16_t>(v); }
inline uint32_t sat_u32(int64_t v) {
    return v < 0 ? 0 : v > 0xFFFFFFFFLL ? 0xFFFFFFFFu : static_cast<uint32_t>(v);
}

// Elapsed since the start tick in 100 ms units, saturating at 6553.4 s.
inline uint16_t elapsed_100ms(int64_t elapsed_us) {
    if (elapsed_us <= 0) return 0;
    const int64_t u = elapsed_us / 100000;
    return u > ELAPSED_MAX_100MS ? ELAPSED_MAX_100MS : static_cast<uint16_t>(u);
}

// Sync age in 100 ms units; 255 = never, or 25.5 s and older.
inline uint8_t sync_age_100ms(bool ever, int64_t age_us) {
    if (!ever || age_us < 0) return SYNC_AGE_NEVER;
    const int64_t u = age_us / 100000;
    return u >= SYNC_AGE_NEVER ? SYNC_AGE_NEVER : static_cast<uint8_t>(u);
}

// REC_EVENT time field: synced -> low 40 bits of the wall time in us;
// unsynced -> the event's age at TX in us, clamped to [0, 2^40 - 1].
inline uint64_t t40_from_wall(int64_t wall_us) { return static_cast<uint64_t>(wall_us) & T40_MASK; }
inline uint64_t t40_from_age(int64_t age_us) {
    if (age_us <= 0) return 0;
    return static_cast<uint64_t>(age_us) > T40_MASK ? T40_MASK : static_cast<uint64_t>(age_us);
}

// --- WAVE_CMD (decode) -----------------------------------------------------------------
struct Cmd {
    uint8_t  desired;  // 0 or 1
    uint8_t  cmd_seq;
    uint8_t  flags;
    uint32_t session_id;
};
enum class CmdDecode : uint8_t { Ok, BadDlc, BadDesired, BadFrame };

inline CmdDecode decode_cmd(const uint8_t* d, uint8_t dlc, bool extd, bool rtr, Cmd& out) {
    if (extd || rtr) return CmdDecode::BadFrame;
    if (dlc != CAN_DLC) return CmdDecode::BadDlc;
    if (d[0] > 1) return CmdDecode::BadDesired;  // ignored and counted by the caller
    out.desired = d[0];
    out.cmd_seq = d[1];
    out.flags = d[2];
    out.session_id = get_u32(d + 4);
    return CmdDecode::Ok;
}

// Test / bench helper: the node's encoding of a WAVE_CMD.
inline void encode_cmd(const Cmd& c, uint8_t out[8]) {
    memset(out, 0, 8);
    out[0] = c.desired;
    out[1] = c.cmd_seq;
    out[2] = c.flags;
    put_u32(out + 4, c.session_id);
}

// --- 0x7DD TIME_SYNC (decode) ----------------------------------------------------------
struct TimeSync {
    uint32_t unix_s;
    uint32_t usec;
};

// `<II` sec, usec. Refuses extended/RTR frames, DLC != 8, usec >= 1e6 and
// any time before 2026-01-01.
inline bool decode_time_sync(const uint8_t* d, uint8_t dlc, bool extd, bool rtr, TimeSync& out) {
    if (extd || rtr || dlc != CAN_DLC) return false;
    const uint32_t s = get_u32(d);
    const uint32_t us = get_u32(d + 4);
    if (us >= 1000000u || s < SYNC_MIN_UNIX_S) return false;
    out.unix_s = s;
    out.usec = us;
    return true;
}
inline int64_t time_sync_wall_us(const TimeSync& t) {
    return static_cast<int64_t>(t.unix_s) * 1000000LL + static_cast<int64_t>(t.usec);
}

// --- WAVE_HEARTBEAT (encode) -------------------------------------------------------------
struct Heartbeat {
    State    state;
    uint8_t  status;
    uint8_t  faults;
    uint16_t elapsed_100ms;
    uint8_t  sync_age_100ms;
    uint8_t  recordings;
    uint8_t  cmd_seq_echo;
};

inline void encode_heartbeat(const Heartbeat& h, uint8_t out[8]) {
    out[0] = static_cast<uint8_t>(h.state);
    out[1] = static_cast<uint8_t>(h.status & 0x7F);  // bit7 reserved 0
    out[2] = static_cast<uint8_t>(h.faults & 0x7F);
    put_u16(out + 3, h.elapsed_100ms);
    out[5] = h.sync_age_100ms;
    out[6] = h.recordings;
    out[7] = h.cmd_seq_echo;
}

// --- WAVE_REC_EVENT (encode) ---------------------------------------------------------------
struct RecEvent {
    uint64_t  t40;  // wall us (synced) or age us (unsynced), low 40 bits
    EventKind kind;
    Reason    reason;
    bool      synced;
    bool      tick_stamped;
    uint8_t   cmd_seq;
    uint8_t   recording_seq;
};

inline uint8_t rec_event_kind_byte(EventKind k, Reason r, bool synced, bool tick) {
    return static_cast<uint8_t>((static_cast<uint8_t>(k) & EV_KIND_MASK) | (synced ? EV_SYNCED : 0) |
                                (tick ? EV_TICK_STAMPED : 0) | ((static_cast<uint8_t>(r) & 0x07) << EV_REASON_SHIFT));
}

inline void encode_rec_event(const RecEvent& e, uint8_t out[8]) {
    put_u40(out, e.t40 & T40_MASK);
    out[5] = rec_event_kind_byte(e.kind, e.reason, e.synced, e.tick_stamped);
    out[6] = e.cmd_seq;
    out[7] = e.recording_seq;
}

// --- WAVE_INFO (encode) ----------------------------------------------------------------------
inline void encode_info_identity(uint16_t fw_version, uint8_t wire_version, uint32_t uptime_s, uint8_t out[8]) {
    out[0] = INFO_PAGE_IDENTITY;
    put_u16(out + 1, fw_version);
    out[3] = wire_version;
    put_u32(out + 4, uptime_s);
}

inline void encode_info_tick(uint32_t tick_period_ns, uint16_t tick_jitter_us, uint8_t ticks_missed, uint8_t out[8]) {
    out[0] = INFO_PAGE_TICK;
    put_u32(out + 1, tick_period_ns);
    put_u16(out + 5, tick_jitter_us);
    out[7] = ticks_missed;
}

inline void encode_info_sync(uint16_t offset_spread_us, uint16_t track_err_us, uint8_t accepted_per_s,
                             uint8_t rejected_per_s, uint8_t steps, uint8_t out[8]) {
    out[0] = INFO_PAGE_SYNC;
    put_u16(out + 1, offset_spread_us);
    put_u16(out + 3, track_err_us);
    out[5] = accepted_per_s;
    out[6] = rejected_per_s;
    out[7] = steps;
}

inline void encode_info_link(uint16_t last_tick_to_clip_us, uint16_t can_tx_errors, uint8_t bus_off_events,
                             uint8_t intent_rx_per_s, uint8_t out[8]) {
    out[0] = INFO_PAGE_LINK;
    put_u16(out + 1, last_tick_to_clip_us);
    put_u16(out + 3, can_tx_errors);
    out[5] = bus_off_events;
    out[6] = intent_rx_per_s;
    out[7] = 0;
}

}  // namespace wave
