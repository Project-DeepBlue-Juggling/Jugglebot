#pragma once

// Recording controller (wave_camera_can_adapter.md s5.2-s5.4): the CAN-side owner of GPI2.
//
// Wraps RecordFsm (record_fsm.h) with the real inputs (ISR edge stamps from
// the edge ring, the camera's "Created new clip." line, WAVE_CMD frames) and
// outputs (gpi2_drive requests, WAVE_REC_EVENT frames). In CAN mode this
// module and latency_test are the only callers of gpi2_drive::request_*;
// latency_test only runs while the console owns GPI2 (`ctl console`).

#include "pin_io.h"

#include "can_link.h"
#include "event_ring.h"
#include "wave_frames.h"

namespace record_ctrl {

void begin();  // sets level mode; CAN owns GPI2 from boot
void poll(int64_t now_us);

void on_edge(const EdgeEvent& ev);   // GPIO26 and GPIO27 edges
void on_clip_line(int64_t t_first_us);
void on_cmd(const can_link::RxFrame& f);

// Console ownership. Each prints its own INFO / ERR line.
bool ctl_console();
bool ctl_can();
bool console_owns();
bool clear_fault_state();  // console `rec assume idle` while CAN owns and FAULT

// Bench-only local intent stream (desired=1 at INTENT_RATE_HZ). Caller checks arm.
void sim_intent(bool on);
bool sim_intent_on();

// Heartbeat contents and INFO page inputs.
wave::Heartbeat heartbeat(int64_t now_us);
uint32_t        tick_period_ns();
uint32_t        tick_jitter_us();
uint32_t        ticks_missed_total();
int64_t         last_tick_to_clip_us();
uint32_t        intents_rx();  // valid WAVE_CMD frames (sim excluded)

void print_status(int64_t now_us);  // `can` lines

}  // namespace record_ctrl
