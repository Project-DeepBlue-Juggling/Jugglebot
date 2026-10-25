#pragma once

// GPI2 (Wave pin 6, GPIO26) open-drain drive with minimum-time enforcement.
//
// Edge mode: `toggle` = low for GPI2_EDGE_LOW_MS, then release. The low pulse
// starts no sooner than GPI2_EDGE_MIN_HIGH_MS after the previous rising edge.
// Level mode: `start` pulls low, `stop` releases; each state is held for at
// least GPI2_LEVEL_MIN_STATE_MS. Requests that come too early are STRETCHED
// (scheduled later, logged as WARN), never dropped.
//
// All timing is a non-blocking state machine polled from loop(). Edge times
// come from the GPIO26 ISR (the level the pad actually reached), and both the
// command time and the ISR stamp are logged.

#include "pin_io.h"

#include "event_ring.h"

enum class RecMode : uint8_t { Edge, Level };
enum class RecState : uint8_t { Unknown, Idle, Recording };

inline const char* rec_mode_name(RecMode m) { return m == RecMode::Edge ? "edge" : "level"; }
inline const char* rec_state_name(RecState s) {
    return s == RecState::Idle ? "idle" : s == RecState::Recording ? "recording" : "unknown";
}

namespace gpi2_drive {

void begin();  // pin already claimed open-drain by edge_capture::begin()

void     set_mode(RecMode m);
RecMode  mode();
bool     mode_confirmed();  // false until `rec mode` has been issued

// Each returns false and prints ERR if refused (mode not confirmed, wrong
// mode, busy, or already in the requested level).
bool request_toggle();
bool request_start();
bool request_stop();

void poll(int64_t now_us);
void on_edge(const EdgeEvent& ev);  // GPIO26 edges only

bool     busy();
// Drop an operation still waiting for its start time (ToggleWait / StartWait /
// StopWait). An operation already driving (edge-mode low pulse) is left to
// finish so the pad is always released. Returns true if one was cancelled.
bool     cancel_pending();
RecState state();
void     assume(RecState s);
void     note_clip_created();  // camera said "Created new clip."

// Most recent ISR-observed edges on GPIO26 (-1 if none yet).
int64_t last_rise_us();
int64_t last_fall_us();
bool    pad_held_low();

}  // namespace gpi2_drive
