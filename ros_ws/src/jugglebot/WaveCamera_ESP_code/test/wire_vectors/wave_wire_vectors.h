// GENERATED from tests/fixtures/wave_camera/wire_vectors.json by
// tests/fixtures/wave_camera/gen_wave_vectors_header.py. Do not hand-edit;
// tests/firmware/test_wave_vectors_header.py fails when this is stale.
#pragma once
#include <cstdint>

namespace wave_vectors {

struct CmdVec { const char* name; uint8_t bytes[8]; uint8_t desired; uint8_t cmd_seq; uint8_t flags; uint32_t session_id; };
static const CmdVec CMD[] = {
    {"clear_faults_session", {0x01, 0x2A, 0x01, 0x00, 0x3D, 0x2C, 0x1B, 0x6A}, 1, 42, 1, 1780165693u},
    {"idle_all_zero", {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0, 0, 0, 0u},
    {"max_values", {0x01, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF}, 1, 255, 255, 4294967295u},
};

struct HeartbeatVec { const char* name; uint8_t bytes[8]; uint8_t state; uint8_t status; uint8_t faults; uint16_t elapsed_100ms; uint8_t sync_age_100ms; uint8_t recordings; uint8_t cmd_seq_echo; };
static const HeartbeatVec HEARTBEAT[] = {
    {"recording_all_status", {0x03, 0x5F, 0x04, 0xD2, 0x04, 0x03, 0x09, 0x2A}, 3, 95, 4, 1234, 3, 9, 42},
    {"idle_console_owns_never_synced", {0x01, 0x20, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00}, 1, 32, 0, 0, 255, 0, 0},
    {"starting_camera_on_intent", {0x02, 0x19, 0x00, 0x00, 0x00, 0x07, 0x03, 0x11}, 2, 25, 0, 0, 7, 3, 17},
    {"boot_zero", {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0, 0, 0, 0, 0, 0, 0},
    {"fault_every_latch", {0x05, 0x7F, 0x7F, 0xFE, 0xFF, 0xFE, 0xFF, 0xFF}, 5, 127, 127, 65534, 254, 255, 255},
    {"stopping_elapsed_saturated", {0x04, 0x0D, 0x22, 0xFE, 0xFF, 0x01, 0x07, 0x99}, 4, 13, 34, 65534, 1, 7, 153},
};

struct RecEventVec { const char* name; uint8_t bytes[8]; uint64_t t40; uint8_t kind; uint8_t reason; bool synced; bool tick_stamped; uint8_t cmd_seq_echo; uint8_t recording_seq; };
static const RecEventVec REC_EVENT[] = {
    {"start_synced_tick_clip_missing", {0x05, 0x04, 0x03, 0x02, 0x01, 0xB9, 0x2A, 0xFE}, 4328719365ULL, 1, 5, true, true, 42, 254},
    {"refused_console_owns_unsynced", {0x10, 0x27, 0x00, 0x00, 0x00, 0xC4, 0x03, 0x00}, 10000u, 4, 6, false, false, 3, 0},
    {"auto_stop_intent_timeout_age", {0xFB, 0xBB, 0x00, 0x00, 0x00, 0x43, 0x05, 0xFF}, 48123u, 3, 2, false, false, 5, 255},
    {"stop_none_max_t40", {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0A, 0x00, 0x01}, 1099511627775ULL, 2, 0, true, false, 0, 1},
    {"refused_gpi2_fault_both_flags", {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x80}, 0u, 4, 7, true, true, 255, 128},
    {"refused_no_change_synced", {0x9A, 0x78, 0x56, 0x34, 0x12, 0x2C, 0x09, 0x02}, 78187493530ULL, 4, 1, true, false, 9, 2},
    {"start_tick_lost_reason", {0x01, 0x00, 0x00, 0x00, 0x00, 0x71, 0x01, 0x01}, 1u, 1, 3, false, true, 1, 1},
    {"start_camera_off_reason", {0x02, 0x00, 0x00, 0x00, 0x00, 0x81, 0x01, 0x01}, 2u, 1, 4, false, false, 1, 1},
};

// Info pages: f[] holds the page's fields in wire order (see the JSON `fields` keys).
struct InfoVec { const char* name; uint8_t bytes[8]; uint8_t page; uint32_t f[5]; };
static const InfoVec INFO[] = {
    {"page0_identity", {0x00, 0x0C, 0x00, 0x01, 0x10, 0x0E, 0x00, 0x00}, 0, {12u, 1u, 3600u, 0u, 0u}},
    {"page0_identity_max", {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 0, {65535u, 255u, 4294967295u, 0u, 0u}},
    {"page1_tick", {0x01, 0x36, 0x82, 0x00, 0x00, 0x2C, 0x01, 0x02}, 1, {33334u, 300u, 2u, 0u, 0u}},
    {"page2_sync", {0x02, 0x96, 0x00, 0x28, 0x00, 0x64, 0x01, 0x00}, 2, {150u, 40u, 100u, 1u, 0u}},
    {"page3_link", {0x03, 0x8C, 0x3C, 0x34, 0x12, 0x09, 0x05, 0x00}, 3, {15500u, 4660u, 9u, 5u, 0u}},
};

}  // namespace wave_vectors
