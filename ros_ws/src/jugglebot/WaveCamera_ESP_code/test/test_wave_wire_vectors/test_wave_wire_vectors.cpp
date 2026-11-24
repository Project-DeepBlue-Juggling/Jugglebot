// The shared wire vectors (FEAT-13): tests/fixtures/wave_camera/wire_vectors.json, rendered to
// ../wire_vectors/wave_wire_vectors.h by tests/fixtures/wave_camera/gen_wave_vectors_header.py.
// The node's Python tests (tests/ros/test_wave_camera.py) consume the same JSON, so the two
// halves of the wire cannot drift apart. Every vector is checked in the direction the adapter
// owns: encode fields -> the exact bytes (heartbeat, rec event, info pages, and the node's cmd
// encoding), and decode_cmd bytes -> fields (the one frame the adapter decodes).

#include <unity.h>

#include <cstdio>
#include <cstring>

#include "../wire_vectors/wave_wire_vectors.h"
#include "wave_frames.h"

using namespace wave;

void setUp() {}
void tearDown() {}

static void expect_bytes(const char* name, const uint8_t* exp, const uint8_t* got) {
    char msg[96];
    snprintf(msg, sizeof msg, "vector %s", name);
    TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(exp, got, 8, msg);
}

static void test_cmd_vectors() {
    for (const auto& v : wave_vectors::CMD) {
        Cmd c{};
        c.desired = v.desired;
        c.cmd_seq = v.cmd_seq;
        c.flags = v.flags;
        c.session_id = v.session_id;
        uint8_t out[8];
        encode_cmd(c, out);
        expect_bytes(v.name, v.bytes, out);

        Cmd d{};
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(CmdDecode::Ok),
                                  static_cast<int>(decode_cmd(v.bytes, 8, false, false, d)), v.name);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(v.desired, d.desired, v.name);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(v.cmd_seq, d.cmd_seq, v.name);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(v.flags, d.flags, v.name);
        TEST_ASSERT_EQUAL_HEX32_MESSAGE(v.session_id, d.session_id, v.name);
    }
}

static void test_heartbeat_vectors() {
    for (const auto& v : wave_vectors::HEARTBEAT) {
        Heartbeat h{};
        h.state = static_cast<State>(v.state);
        h.status = v.status;
        h.faults = v.faults;
        h.elapsed_100ms = v.elapsed_100ms;
        h.sync_age_100ms = v.sync_age_100ms;
        h.recordings = v.recordings;
        h.cmd_seq_echo = v.cmd_seq_echo;
        uint8_t out[8];
        encode_heartbeat(h, out);
        expect_bytes(v.name, v.bytes, out);
    }
}

static void test_rec_event_vectors() {
    for (const auto& v : wave_vectors::REC_EVENT) {
        RecEvent e{};
        e.t40 = v.t40;
        e.kind = static_cast<EventKind>(v.kind);
        e.reason = static_cast<Reason>(v.reason);
        e.synced = v.synced;
        e.tick_stamped = v.tick_stamped;
        e.cmd_seq = v.cmd_seq_echo;
        e.recording_seq = v.recording_seq;
        uint8_t out[8];
        encode_rec_event(e, out);
        expect_bytes(v.name, v.bytes, out);
        TEST_ASSERT_EQUAL_HEX64_MESSAGE(v.t40, get_u40(out), v.name);
    }
}

static void test_info_vectors() {
    for (const auto& v : wave_vectors::INFO) {
        uint8_t out[8];
        memset(out, 0xEE, sizeof out);  // the encoders must write every byte they own, incl. padding
        switch (v.page) {
            case INFO_PAGE_IDENTITY:
                encode_info_identity(static_cast<uint16_t>(v.f[0]), static_cast<uint8_t>(v.f[1]), v.f[2], out);
                break;
            case INFO_PAGE_TICK:
                encode_info_tick(v.f[0], static_cast<uint16_t>(v.f[1]), static_cast<uint8_t>(v.f[2]), out);
                break;
            case INFO_PAGE_SYNC:
                encode_info_sync(static_cast<uint16_t>(v.f[0]), static_cast<uint16_t>(v.f[1]),
                                 static_cast<uint8_t>(v.f[2]), static_cast<uint8_t>(v.f[3]),
                                 static_cast<uint8_t>(v.f[4]), out);
                break;
            case INFO_PAGE_LINK:
                encode_info_link(static_cast<uint16_t>(v.f[0]), static_cast<uint16_t>(v.f[1]),
                                 static_cast<uint8_t>(v.f[2]), static_cast<uint8_t>(v.f[3]), out);
                break;
            default:
                TEST_FAIL_MESSAGE(v.name);
        }
        // Every page writes all 8 bytes (identity ends on uptime_s, the link page zeroes byte 7).
        expect_bytes(v.name, v.bytes, out);
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_cmd_vectors);
    RUN_TEST(test_heartbeat_vectors);
    RUN_TEST(test_rec_event_vectors);
    RUN_TEST(test_info_vectors);
    return UNITY_END();
}
