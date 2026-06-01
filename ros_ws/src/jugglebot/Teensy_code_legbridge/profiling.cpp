// =============================================================================
//  profiling.cpp — firmware instrumentation → 1 Hz PROFILE frame
// =============================================================================
#include "profiling.h"

#include <Arduino.h>
#include <cstring>
#include "freertos_shim.h"
#include "legbridge_config.h"
#include "protocol_config.h"   // CanBus::BITS_PER_FRAME_APPROX
#include "udp_protocol.h"
#include "udp_link.h"
#include "can_buses.h"
#include "leg_interp.h"
#include "time_base.h"

namespace LegBridge {

// Fixed PROFILE slot order — must match profiling.h and the Jetson consumer.
static const char* kTaskSlots[JbUdp::PROFILE_NUM_TASKS] = {
  "canrx", "tsync", "net", "fault", "telem", "hb", "diag", "IDLE", "other"
};

// Per-window baselines.
static uint32_t s_prev_can1_rx = 0, s_prev_can1_tx = 0, s_prev_can2_rx = 0, s_prev_can2_tx = 0;
static uint64_t s_prev_us = 0;

#if (configGENERATE_RUN_TIME_STATS == 1) && (configUSE_TRACE_FACILITY == 1)
// Per-task cumulative run-time counter from the previous window, keyed by slot.
static uint32_t s_prev_task_rt[JbUdp::PROFILE_NUM_TASKS] = {0};
static uint32_t s_prev_total_rt = 0;

static int slot_for(const char* name) {
  for (int i = 0; i < (int)JbUdp::PROFILE_NUM_TASKS - 1; ++i)
    if (strcmp(name, kTaskSlots[i]) == 0) return i;
  return JbUdp::PROFILE_NUM_TASKS - 1;   // "other"
}

static void fill_cpu(uint16_t cpu_pct_x100[]) {
  // Snapshot all tasks.
  const UBaseType_t n = uxTaskGetNumberOfTasks();
  static TaskStatus_t status[24];
  if (n == 0 || n > 24) { for (int i = 0; i < JbUdp::PROFILE_NUM_TASKS; ++i) cpu_pct_x100[i] = 0; return; }
  uint32_t total_rt = 0;
  const UBaseType_t got = uxTaskGetSystemState(status, 24, &total_rt);

  uint32_t cur_rt[JbUdp::PROFILE_NUM_TASKS] = {0};
  for (UBaseType_t i = 0; i < got; ++i)
    cur_rt[slot_for(status[i].pcTaskName)] += status[i].ulRunTimeCounter;

  const uint32_t dtot = total_rt - s_prev_total_rt;
  for (int s = 0; s < (int)JbUdp::PROFILE_NUM_TASKS; ++s) {
    const uint32_t d = cur_rt[s] - s_prev_task_rt[s];
    cpu_pct_x100[s] = (dtot > 0) ? (uint16_t)((uint64_t)d * 10000ULL / dtot) : 0;
    s_prev_task_rt[s] = cur_rt[s];
  }
  s_prev_total_rt = total_rt;
}
#else
static void fill_cpu(uint16_t cpu_pct_x100[]) {
  for (int i = 0; i < (int)JbUdp::PROFILE_NUM_TASKS; ++i) cpu_pct_x100[i] = 0;  // stats disabled
}
#endif

void profiling_init() {
  const CanStats c = can_buses_stats();
  s_prev_can1_rx = c.can1_rx; s_prev_can1_tx = c.can1_tx;
  s_prev_can2_rx = c.can2_rx; s_prev_can2_tx = c.can2_tx;
  s_prev_us = micros64();
}

// Bus utilisation %·100 over the window from a frame-count delta.
static uint16_t util_x100(uint32_t frames, uint32_t window_us) {
  if (window_us == 0) return 0;
  // bits = frames * BITS_PER_FRAME; capacity_bits = baud * window_s.
  const double bits = (double)frames * (double)CanBus::BITS_PER_FRAME_APPROX;
  const double cap  = (double)CAN_BITRATE * ((double)window_us * 1e-6);
  const double pct = (cap > 0) ? (bits / cap) * 100.0 : 0.0;
  const double v = pct * 100.0;
  return (v < 0) ? 0 : (v > 65535) ? 65535 : (uint16_t)v;
}

void profiling_step() {
  const uint64_t now = micros64();
  const uint32_t win = (uint32_t)(now - s_prev_us);
  const CanStats c = can_buses_stats();

  JbUdp::ProfilePayload p{};
  p.t_teensy_us = now_wall_us();
  fill_cpu(p.cpu_pct_x100);

  p.can1_rx = c.can1_rx - s_prev_can1_rx;
  p.can1_tx = c.can1_tx - s_prev_can1_tx;
  p.can2_rx = c.can2_rx - s_prev_can2_rx;
  p.can2_tx = c.can2_tx - s_prev_can2_tx;
  p.can1_util_x100 = util_x100(p.can1_rx + p.can1_tx, win);
  p.can2_util_x100 = util_x100(p.can2_rx + p.can2_tx, win);
  p.udp_rtt_us = udp_last_rtt_us();
  p.udp_jitter_us = udp_rtt_jitter_us();
  p.interp_deadline_misses = interp_deadline_misses();
  p.interp_max_jitter_us = interp_max_jitter_us();
  p.free_heap_bytes = (uint32_t)xPortGetFreeHeapSize();

  udp_send_stream(JbUdp::MsgType::PROFILE, (const uint8_t*)&p, sizeof(p));

  // Reset per-window baselines.
  s_prev_can1_rx = c.can1_rx; s_prev_can1_tx = c.can1_tx;
  s_prev_can2_rx = c.can2_rx; s_prev_can2_tx = c.can2_tx;
  s_prev_us = now;
  interp_reset_jitter();          // max-jitter is per-window
}

}  // namespace LegBridge
